#include "Car.h"
#include "VehicleSegment.h"
#include "Utill/DebugConsole.h"
#include "Nav/ReedsShepp.h"
#include "Nav/VehicleCollision.h"
#include "Nav/SimulationState.h"
#include "Utill/Assert.h"
#include "Utill/PerfLog.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace
{
    // road의 밴드 중 횡오프셋 d에 가장 가까운 밴드의 centerOffset. 밴드 없으면 d 그대로.
    float NearestBandOffset(const shared_ptr<Road> &road, float d)
    {
        if (!road)
            return d;
        const LaneSection *sec = RoadDataManager::Get().GetLateralProfile(road, 0.0f);
        if (sec == nullptr || sec->bands.empty())
            return d;
        float best = sec->bands.front().centerOffset;
        float bestDelta = std::abs(d - best);
        for (const LaneBand &b : sec->bands)
        {
            float delta = std::abs(d - b.centerOffset);
            if (delta < bestDelta)
            {
                bestDelta = delta;
                best = b.centerOffset;
            }
        }
        return best;
    }

    std::vector<std::unique_ptr<VehicleSegment>> BuildReedSheppSegments(const ReedsShepp::Path &path, const Vec3 &startPos,
                                                                        float startAngleRad, float turningRadius)
    {
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        std::vector<ReedsShepp::Leg> legs = ReedsShepp::GetLegs(path, startPos, startAngleRad, turningRadius);
        segments.reserve(legs.size());
        for (size_t i = 0; i < legs.size(); ++i)
        {
            ReedsShepp::Leg &leg = legs[i];
            bool isFinalLeg = (i + 1 == legs.size());
            segments.push_back(
                std::make_unique<RSFollowSegment>(std::move(leg.points), leg.gear, leg.endIndex, isFinalLeg));
        }
        return segments;
    }

    std::vector<std::unique_ptr<VehicleSegment>> BuildExactSegments(const ReedsShepp::Path &path, float maxSteerAngle)
    {
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        segments.reserve(path.size());
        for (const ReedsShepp::PathElement &element : path)
        {
            float steerAngle = 0.0f;
            if (element.steering == ReedsShepp::Steering::Left)
                steerAngle = -maxSteerAngle;
            else if (element.steering == ReedsShepp::Steering::Right)
                steerAngle = maxSteerAngle;
            segments.push_back(std::make_unique<RSExactSegment>(element, steerAngle));
        }
        return segments;
    }

    float DirectionToAngleRad(const Vec3 &direction)
    {
        return atan2f(direction.GetZ(), direction.GetX());
    }

    // 후보 선택(BehaviorPlan) 구간 성능 계측. 끄려면 false로. 모든 차의 plan을 합산해 EMIT_EVERY plan마다 평균/최대를 찍는다.
    constexpr bool BEHAVIOR_PLAN_PERF = true;
    constexpr int BEHAVIOR_PLAN_PERF_EMIT_EVERY = 200;

    struct BehaviorPlanPerf
    {
        int plans = 0;
        long long candidateSum = 0;
        double buildMsSum = 0.0, buildMsMax = 0.0;   // 후보 생성(BuildCandidate 루프)
        double selectMsSum = 0.0, selectMsMax = 0.0; // 안전판정+비용평가로 best 고르는 루프
        double planMsMax = 0.0;                      // 한 plan의 후보 선택 전체(build+select) 최대

        void Record(int candidates, double buildMs, double selectMs)
        {
            ++plans;
            candidateSum += candidates;
            buildMsSum += buildMs;
            buildMsMax = std::max(buildMsMax, buildMs);
            selectMsSum += selectMs;
            selectMsMax = std::max(selectMsMax, selectMs);
            planMsMax = std::max(planMsMax, buildMs + selectMs);
            if (plans >= BEHAVIOR_PLAN_PERF_EMIT_EVERY)
                Emit();
        }

        void Emit()
        {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "[perf] behaviorPlan plans=%d candAvg=%.1f | total avg=%.3fms max=%.3fms | build avg=%.3fms max=%.3fms | select avg=%.3fms max=%.3fms",
                          plans, static_cast<double>(candidateSum) / plans, (buildMsSum + selectMsSum) / plans, planMsMax,
                          buildMsSum / plans, buildMsMax, selectMsSum / plans, selectMsMax);
            PerfLog::Emit(buf);
            *this = BehaviorPlanPerf{};
        }
    };
    BehaviorPlanPerf g_behaviorPlanPerf;

    inline double MsSince(std::chrono::steady_clock::time_point start)
    {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    }
}

#pragma region Common
void Car::UpdateMode()
{
    const char *reason = "";
    UpdateFindPath();
    Mode next = DecideNextMode(&reason);
    if (next != m_mode)
    {
        DebugConsole::Log(GetName() + ": " + StateToString(m_mode) + " -> " + StateToString(next) + " (" + reason + ")");
        OnModeExit(next);
        Mode prev = m_mode;
        m_mode = next;
        OnModeEnter(prev);
    }
}

void Car::UpdateFindPath()
{
    if (m_roaming)
    {
        EnsureRoamingPath();
        return;
    }

    if (m_destRoad == nullptr || m_currentRoad != nullptr)
        return;

    Vec3 position = GetPosition();
    RoadPose pose = RoadDataManager::Get().GetClosestRoad(position);
    SetCurrentRoad(pose.road, NearestBandOffset(pose.road, pose.d));
    TryFindPathAndSetRoad();
}

Car::Mode Car::DecideNextMode(const char **reason) const
{

    if (m_mode == Mode::Stop)
    {
        if (m_roaming)
        {
            *reason = "roaming";
            return Mode::Drive; // 배회 모드는 출차(Park) 없이 바로 주행 시작
        }
        if (m_destRoad == nullptr)
        {
            return Mode::Stop;
        }
        *reason = "go to Dest";
        return Mode::Drive; // 주차 비활성: 출차(Park) 없이 바로 주행
    }
    else if (m_mode == Mode::Park)
    {
        if (m_parkPlanPending || m_parkSequenceActive || !m_vehicleController.IsFinished())
        {
            *reason = "parking in progress";
            return Mode::Park;
        }
        if (m_subMode == SubMode::P_EXIT)
        {
            // 출차 끝났으면 Drive로 전환
            *reason = "normal driving";
            return Mode::Drive;
        }
        // 그 외(P_ENTER_LEG1/LEG2/ALIGN 등 입차 계열)가 여기 왔다는 건 이미 다 끝났다는 뜻 -> Stop
        *reason = "done parking";
        return Mode::Stop;
    }
    else if (m_mode == Mode::Drive)
    {
        if (m_roaming)
            return Mode::Drive; // 배회 모드: 목적지/도착 판정 없이 계속 주행

        bool arrived = false;
        if (m_destRoad != nullptr)
        {
            const std::vector<Vec3> &destPts = m_destRoad->GetReferenceLine().GetSplinePoints();
            Vec3 destEnd = destPts.empty() ? GetPosition() : destPts.back();
            Vec3 projectedPosition = m_destRoad->GetReferenceLine().GetLookaheadPoint(GetPosition(), 0.0f);
            arrived = (destEnd - projectedPosition).Length() < ARRIVE_DISTANCE;
        }

        // 주차 비활성: 도착하면 그냥 Stop.
        if (m_destRoad == nullptr || arrived)
        {
            *reason = m_destRoad == nullptr ? "no destination road" : "arrived at destination";
            return Mode::Stop;
        }
        return Mode::Drive;
    }

    *reason = "unreachable";
    return m_mode;
}

void Car::OnModeEnter(Mode prev)
{
    if (m_mode == Mode::Drive)
    {
        SetSubMode(SubMode::D_Normal);
        m_emergencyBrake = false;   // 직전 Drive의 비상 상태가 새 주행에 새지 않게 리셋
        m_laneChangeActive = false; // 새 주행 시작 -- 이전 차선변경 매뉴버 상태가 남지 않게 리셋
        m_laneChangeFromOffset = 0.0f;
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        segments.push_back(std::make_unique<SplineFollowSegment>());
        m_vehicleController.BeginPlan(std::move(segments));
    }
    else if (m_mode == Mode::Park)
    {
        SetSubMode(prev == Mode::Stop ? SubMode::P_EXIT : SubMode::P_ENTER_LEG1);
        m_parkPlanPending = true;    // 도착 즉시 RS를 계산하지 않고, 완전히 멈출 때까지 기다린다 (UpdatePark에서 처리).
        m_parkSequenceActive = true; // 주차 시퀀스 시작 — 완료(UpdatePark)까지 Park 유지.
        DebugConsole::Log(GetName() + ": Park plan pending: waiting for full stop before planning");
    }
    else if (m_mode == Mode::Stop)
    {
        // 입차 완료 후 Stop으로 오는 경우도 RS 매뉴버로 꺾여있던 조향을 중앙으로 되돌린다.
        if (prev == Mode::Park && m_subMode == SubMode::P_ENTER_ALIGN)
        {
            std::vector<std::unique_ptr<VehicleSegment>> segments;
            segments.push_back(std::make_unique<CenterSteerSegment>());
            m_vehicleController.BeginPlan(std::move(segments));
        }
        SetSubMode(SubMode::None);
    }
}

void Car::OnModeExit(Mode next)
{
    if (!m_vehicleController.IsFinished())
        m_vehicleController.Abort();
}

void Car::SetSubMode(SubMode next)
{
    if (next == m_subMode)
        return;
    DebugConsole::Log(GetName() + ": " + SubStateToString(m_subMode) + " -> " + SubStateToString(next));
    m_subMode = next;
}

VehicleCollision::VehicleShape Car::BuildVehicleShape() const
{
    VehicleCollision::VehicleShape shape;
    shape.pivotToCenter = m_colliderOffset.z;
    shape.halfWidth = m_halfExtents.GetX();
    shape.halfLength = m_halfExtents.GetZ();
    return shape;
}

bool Car::TryFindPathAndSetRoad()
{
    m_path = RoadDataManager::Get().FindPath(m_currentRoad, m_destRoad);
    m_pathIndex = 0;
    if (m_path.empty())
    {
        m_destRoad = nullptr;
        SetCurrentRoad(nullptr, 0.0f);
        return false;
    }

    return true;
}

shared_ptr<Road> Car::PickRandomSuccessor(const shared_ptr<Road> &road) const
{
    if (road == nullptr)
        return nullptr;

    const std::vector<shared_ptr<Road>> &successors = RoadDataManager::Get().GetRoadSuccessors(road->GetId());
    if (successors.empty())
        return nullptr;
    return successors[rand() % successors.size()];
}

vector<shared_ptr<Road>> Car::BuildRoamingPath(const shared_ptr<Road> &startRoad) const
{
    vector<shared_ptr<Road>> path;
    if (startRoad == nullptr)
        return path;

    path.push_back(startRoad);
    shared_ptr<Road> current = startRoad;
    for (size_t i = 0; i < ROAMING_MIN_AHEAD; ++i)
    {
        shared_ptr<Road> next = PickRandomSuccessor(current);
        if (next == nullptr)
            break; // 막다른 road
        path.push_back(next);
        current = next;
    }
    return path;
}

void Car::EnsureRoamingPath()
{
    if (m_currentRoad == nullptr)
    {
        RoadPose pose = RoadDataManager::Get().GetClosestRoad(GetPosition());
        SetCurrentRoad(pose.road, NearestBandOffset(pose.road, pose.d));
        m_path = BuildRoamingPath(m_currentRoad);
        m_pathIndex = 0;
        return;
    }
    MaintainRoamingPath();
}

void Car::MaintainRoamingPath()
{
    constexpr size_t KEEP_BEHIND = 1; // 메모리 상한용: 지나온 road는 이만큼만 남기고 앞부분을 버린다

    while (m_pathIndex > KEEP_BEHIND)
    {
        m_path.erase(m_path.begin());
        --m_pathIndex;
    }

    // 현재 road 앞으로 항상 ROAMING_MIN_AHEAD개의 road가 남아 있도록 랜덤 후속으로 채운다.
    while (!m_path.empty() && m_path.size() - m_pathIndex <= ROAMING_MIN_AHEAD)
    {
        shared_ptr<Road> next = PickRandomSuccessor(m_path.back());
        if (next == nullptr)
            break; // 막다른 road
        m_path.push_back(next);
    }
}
#pragma endregion

#pragma region Park

void Car::UpdatePark()
{
    // 주차 비활성(stub). Park 모드는 진입하지 않는다.
    Accelerate(0.0f);
}

void Car::BeginParkPlan() {}

const Spline *Car::FindBestParkingSpline() const { return nullptr; }

bool Car::ComputeParkPrePose(Vec3 &outPos, float &outAngleRad) const
{
    (void)outPos;
    (void)outAngleRad;
    return false;
}

bool Car::PlanEnterForCurrentSpot() { return false; }

bool Car::ReserveNextParkSpot() { return false; }

bool Car::BeginParkEnterOrRetry() { return false; }

bool Car::PlanParkLegTo(const Vec3 &targetPos, float targetAngleRad, bool exact)
{
    (void)targetPos;
    (void)targetAngleRad;
    (void)exact;
    return false;
}

void Car::BeginParkSpotLeg() {}

#pragma endregion

#pragma region Stop
void Car::UpdateStop()
{
    // Drive에서 "도착"으로 넘어왔으면 그 목적지는 여기서 소모한다(m_destLane 해제). 안 지우면
    // destLane이 남아 다음 프레임 DecideNextMode(Stop)가 바로 Drive를 돌려주고 Drive는 다시
    // "도착"으로 Stop을 돌려줘 매 프레임 Stop<->Drive 진동이 생긴다 -- 그동안 제동(Stop 틱)과
    // 유지(Drive 틱)가 번갈아 걸려 목적지 앞에서 기어가기만 하고 멈추지 못한다.
    if (m_destRoad != nullptr)
    {
        const std::vector<Vec3> &destPts = m_destRoad->GetReferenceLine().GetSplinePoints();
        Vec3 destEnd = destPts.empty() ? GetPosition() : destPts.back();
        Vec3 projectedPosition = m_destRoad->GetReferenceLine().GetLookaheadPoint(GetPosition(), 0.0f);
        if ((destEnd - projectedPosition).Length() < ARRIVE_DISTANCE)
            m_destRoad = nullptr;
    }

    if (!m_vehicleController.IsFinished())
    {
        m_wantSegmentTick = true;
        return;
    }

    if (m_speed > 0.01f)
    {
        if (m_currentRoad != nullptr)
            Steer(Stanley(m_currentSpline));
        Accelerate(0.0f);
        return;
    }

    std::shared_ptr<RoadNode> dest = RoadDataManager::Get().GetRandomDestNode();
    if (!dest)
        return;
    // GetRandomDestNode는 지금 위치와 무관하게 뽑으므로, 이미 도착 판정 거리 안인 노드(방금 왔던 곳과
    // 같거나 바로 근처)가 나오면 그냥 이번 틱은 건너뛴다 -- 안 그러면 Drive 진입하자마자 그 자리서 바로
    // "도착"으로 잡혀 Stop<->Drive가 실제 주행 없이 계속 반복된다.
    if ((dest->position - GetPosition()).Length() < ARRIVE_DISTANCE)
        return;

    SetDestination(dest);
}
#pragma endregion

#pragma region Drive
void Car::UpdateDrive()
{
    if (!CheckPath())
        return;
    UpdateBehaviorPlan();

    m_wantSegmentTick = true;
}

bool Car::CheckPath()
{
    if (m_currentRoad == nullptr)
    {
        m_destRoad = nullptr;
        return false;
    }

    // path find
    Vec3 position = GetPosition();
    auto roadEnd = [&]() -> Vec3
    {
        const std::vector<Vec3> &pts = m_currentSpline.GetSplinePoints();
        return pts.empty() ? position : pts.back();
    };

    // 현재 road의 끝에 다가가면 경로상 다음 road로 넘어간다.
    Vec3 projectedPosition = m_currentSpline.GetLookaheadPoint(position, 0.0f);
    float roadEndDistance = (roadEnd() - projectedPosition).Length();
    while (roadEndDistance < LANE_TRANSITION_THRESHOLD)
    {
        // 신호로 서야 하면 road를 안 넘긴다
        if (ShouldStopForSignal(m_currentRoad))
            break;

        if (m_pathIndex + 1 >= m_path.size())
        {
            if (m_roaming)
            {
                MaintainRoamingPath(); // 랜덤 후속 road로 버퍼를 채운다
                if (m_pathIndex + 1 >= m_path.size())
                    break; // 후속 road가 없는 막다른 road -- 현재 road 끝에서 멈춘다
            }
            else
            {
                m_destRoad = nullptr;
                SetCurrentRoad(nullptr, 0.0f);
                return false;
            }
        }
        ++m_pathIndex;
        SetCurrentRoad(m_path[m_pathIndex], m_currentOffset);
        projectedPosition = m_currentSpline.GetLookaheadPoint(position, 0.0f);
        roadEndDistance = (roadEnd() - projectedPosition).Length();
    }
    return true;
}

float Car::ComputeLookaheadDistance() const
{
    float minSafeLookahead = 2.0f * m_wheelbase / tanf(m_maxSteerAngle);
    constexpr float LOOKAHEAD_TIME = 1.5f; // 몇 초 앞을 볼지
    return max(m_speed * LOOKAHEAD_TIME, minSafeLookahead);
}

void Car::DriveControl()
{
    const Spline &spline = m_currentSpline;
    float targetSteer = Stanley(spline);
    Steer(targetSteer);

    // speed control: 목표 속도는 행동 계획(UpdateBehaviorPlan, 0.2초 주기)이 정해두고, 여기선 이번
    // 프레임의 조향각이 물리적으로 허용하는 한계 속도로 한 번 더 클램프만 한다.
    // 직전 플랜에서 안전한 후보가 없었으면(m_emergencyBrake) 조향은 유지한 채 비상 제동만 밟는다.
    if (m_emergencyBrake)
    {
        EmergBrake();
    }
    else
    {
        // 목표는 계획부가 계산한 실제 속도캡(desiredSpeed=커브/제한속도/신호/앞차 반영)을 그대로 쓰고,
        // 감속 프로파일은 새 비례+저크제한 Accelerate가 스스로 만든다. (0.2초 앞 targetSpeed를 쓰면
        // 이미 한 스텝 낮춰진 값이라 비례 오차가 작아 제동이 약해진다.)
        float steerSpeedCap = CalcMaxSpeed(targetSteer);
        Accelerate(std::min(steerSpeedCap, m_lastDesiredSpeed), m_lastAccelFF);
    }

    // Debug: 스탠리 기준점(경로 최근접점) 표시
    Vec3 pathPoint = spline.GetPositionAt(spline.GetSplinePosition(GetPosition()));
    DirectX::XMFLOAT3 targetMarkerPos = ToXMFLOAT3(pathPoint);
    targetMarkerPos.y = GetPosition().GetY() + 0.2f;
    m_targetMarker.GetTransform().SetPosition(targetMarkerPos);
}

#pragma endregion

#pragma region BehaviorPlan

std::vector<Car::RoadSpeedSample> Car::ScanRoadSpeedConstraints(float lookDistance) const
{
    constexpr float ROAD_SAMPLE_SPACING = 5.0f; // 도로 스캔 샘플 간격 (m)

    Vec3 calPosition = GetPosition();
    const std::vector<VehicleCollision::Obstacle> &obstacles = RoadDataManager::Get().GetObstacles();

    // m_currentLane->GetSpline() -> (필요시) path상의 다음 노드들의 lane spline 순으로 lookDistance까지 훑으며
    // 커브 지점(로컬 곡률 기반 최대속도)과 노드 지점(제한속도)의 샘플을 모은다. 각 샘플이 곧
    // ComputeSpeedCapFromSamples가 쓰는 "가상 리더"의 (위치, 거리, 요구 속도) 후보가 된다.
    std::vector<RoadSpeedSample> samples;
    samples.reserve(static_cast<size_t>(lookDistance / ROAD_SAMPLE_SPACING) + 4);
    auto splineEnd = [](const Spline *sp) -> Vec3
    {
        const std::vector<Vec3> &p = sp->GetSplinePoints();
        return p.empty() ? Vec3::sZero() : p.back();
    };
    {
        float currentNodeT = m_currentSpline.GetSplinePosition(calPosition);
        Assert(currentNodeT >= 0.0f); // ScanRoadSpeedConstraints 호출 전엔 항상 m_currentSpline이 세팅되어 있어야 함
        float currentNodeDistance = m_currentSpline.GetLength() * (1.0f - currentNodeT);
        float currentNodeSpeed = (m_currentRoad == m_destRoad) ? 0.0f : std::min(m_currentRoad->GetSpeedLimit(), m_maxSpeed);
        samples.push_back({splineEnd(&m_currentSpline), currentNodeDistance, currentNodeSpeed});
    }
    {
        const Spline *spline = &m_currentSpline;
        shared_ptr<Road> segmentRoad = m_currentRoad;
        size_t pathIndex = m_pathIndex;
        Vec3 segmentStart = calPosition;
        float traveledDistance = 0.0f;
        float remainingDistance = lookDistance;

        while (remainingDistance > 0.0f && spline)
        {
            float startT = spline->GetSplinePosition(segmentStart);
            float splineLength = spline->GetLength();
            float segmentDistance = splineLength > 0.0f ? (1.0f - startT) * splineLength : 0.0f;
            float walkDistance = std::min(segmentDistance, remainingDistance);

            // nodeT < startT면 이미 지나온 신호라 건너뛴다 (안 그러면 통과 직후 급제동).
            if (shared_ptr<RoadNode> signalNode = RoadDataManager::Get().GetSignalNodeForRoad(segmentRoad->GetId()))
            {
                if (splineLength > 0.0f && ShouldStopForSignal(segmentRoad))
                {
                    float nodeT = spline->GetSplinePosition(signalNode->position);
                    if (nodeT >= startT)
                    {
                        float nodeDistance = (segmentRoad == m_currentRoad)
                                                 ? (signalNode->position - calPosition).Length()
                                                 : traveledDistance + (nodeT - startT) * splineLength;
                        // 정지선 앞 MIN_SAFE_GAP만큼 여유를 두고 선다 (거리<0이면 캡이 0으로 내려가 정지선을 넘어 크리핑하지 않는다).
                        samples.push_back({signalNode->position, nodeDistance - MIN_SAFE_GAP, 0.0f});
                    }
                }
            }

            const std::vector<Vec3> &points = spline->GetSplinePoints();
            if (!points.empty() && splineLength > 0.0f)
            {
                size_t lastIndex = points.size() - 1;

                float apexRadius = spline->GetMinRadiusAhead();
                float apexT = spline->GetApexT();

                size_t sampleCount = static_cast<size_t>(walkDistance / ROAD_SAMPLE_SPACING) + 1;
                for (size_t s = 1; s <= sampleCount; ++s)
                {
                    float localDistance = std::min(walkDistance, s * ROAD_SAMPLE_SPACING);
                    float t = startT + localDistance / splineLength;
                    size_t index = static_cast<size_t>(std::clamp(t, 0.0f, 1.0f) * lastIndex);

                    // apex 이전은 정점 반경으로 캡(정점 대비 감속), apex 통과 후엔 그 위치 국소 곡률로 캡(코너 탈출 가속)
                    float maxSpeed = m_maxSpeed;
                    float curveRadius = (t <= apexT) ? apexRadius : spline->GetRadiusAt(t);
                    if (curveRadius < std::numeric_limits<float>::max())
                        maxSpeed = CURVE_SPEED_COEFF * std::sqrt(curveRadius);
                    samples.push_back({points[index], traveledDistance + localDistance, maxSpeed});

                    // 경로 박스와 겹치는 정적 장애물이 있으면 그 앞에 0속도 샘플(가상 정지선)을 세운다
                    if (!obstacles.empty())
                    {
                        constexpr float OBSTACLE_PROBE_MARGIN = 0.25f;
                        Vec3 probeDir = spline->GetDirectionAt(std::clamp(t, 0.0f, 1.0f));
                        VehicleCollision::VehicleShape probeShape;
                        probeShape.pivotToCenter = 0.0f;
                        probeShape.halfLength = ROAD_SAMPLE_SPACING * 0.5f;
                        probeShape.halfWidth = m_halfExtents.GetX() + OBSTACLE_PROBE_MARGIN;
                        float probeHeading = atan2f(probeDir.GetZ(), probeDir.GetX());
                        if (VehicleCollision::IsColliding(points[index], probeHeading, obstacles, probeShape))
                        {
                            // 프로브 안 어디에 걸렸는지는 모르니 프로브 반길이 + 안전마진만큼 앞에서 선다.
                            float stopDistance = traveledDistance + localDistance - ROAD_SAMPLE_SPACING * 0.5f - MIN_SAFE_GAP;
                            samples.push_back({points[index], stopDistance, 0.0f}); // 거리<0이면 캡이 0으로 내려가 장애물 앞에서 크리핑하지 않는다
                        }
                    }
                }
            }

            traveledDistance += walkDistance;
            remainingDistance -= walkDistance;
            if (remainingDistance <= 0.0f)
                break;

            shared_ptr<Road> nextRoad = (pathIndex + 1 < m_path.size()) ? m_path[pathIndex + 1] : nullptr;
            if (!nextRoad)
            {
                // 경로가 여기서 끝난다는 것은 이 road가 destRoad라는 뜻: 진짜 정지 지점인 road 끝에 0속도를 박는다.
                if (segmentRoad == m_destRoad)
                    samples.push_back({splineEnd(spline), traveledDistance, 0.0f});
                break;
            }

            float nextNodeSpeed = std::min(nextRoad->GetSpeedLimit(), m_maxSpeed);
            const Spline &nextRef = nextRoad->GetReferenceLine();
            Vec3 nextStart = nextRef.GetSplinePoints().empty() ? segmentStart : nextRef.GetSplinePoints().front();
            samples.push_back({nextStart, traveledDistance, nextNodeSpeed});

            segmentRoad = nextRoad;
            segmentStart = nextStart;
            spline = &nextRef;
            ++pathIndex;
        }
    }
    std::sort(samples.begin(), samples.end(), [](const RoadSpeedSample &a, const RoadSpeedSample &b)
              { return a.distance < b.distance; });
    return samples;
}

// samples 각각을, distanceOffset만큼 이미 다가간 지점 기준으로 다시 평가한다 -- distanceOffset=0이면
// "지금 이 순간" 기준(=예전 ComputeDesiredCruiseSpeed)과 같고, distanceOffset>0이면 궤적을 따라 그만큼
// 전진한 미래 시점 기준의 국소 안전속도 상한이 된다 (desiredSpeed는 계속 낮아지는데 그 사실을
// "지금 시점 값 하나"로만 비교하면 접근 구간에서 감속 판단이 늦어지는 문제를 피하기 위함).
float Car::ComputeSpeedCapFromSamples(const std::vector<RoadSpeedSample> &samples, float distanceOffset,
                                      const RoadSpeedSample **binding) const
{
    float speedCap = m_maxSpeed;
    if (binding != nullptr)
        *binding = nullptr;
    for (const RoadSpeedSample &sample : samples)
    {
        float remaining = sample.distance - distanceOffset;
        // 리더/정지선(speed==0)은 표준간격을 침범(remaining<0)해도 캡을 0쪽으로 낮춰 크리핑을 막는다. 반면 커브/제한속도
        // 같은 speed>0 정적 제약은 지나고 나면(remaining<=0) 더는 감속 대상이 아니라 스킵한다.
        bool hardStop = sample.leader != nullptr || sample.speed <= 0.0f;
        if (!hardStop && remaining <= 0.0f)
            continue;
        float allowedSpeed = std::sqrt(std::max(0.0f, sample.speed * sample.speed + 2.0f * m_planBrake * remaining));
        if (allowedSpeed < speedCap)
        {
            speedCap = allowedSpeed;
            if (binding != nullptr)
                *binding = &sample;
        }
    }
    return speedCap;
}

// 레인 등록 없이 위치만으로 주변 차를 모은다. 컬링 반경은 "3초(BEHAVIOR_SAFETY_HORIZON) 동안
// 나와 상대가 정면으로 마주 달려도 닿을 수 없는 거리"로 차마다 계산한다 -- 이 밖의 차는 어떤
// 후보 궤적으로도 시뮬레이션 중 겹칠 수 없으므로 EvaluateTrajectorySafety에 넘길 필요가 없다.
std::vector<Car::NearbyCar> Car::CollectNearbyCars() const
{
    std::vector<NearbyCar> nearby;
    Vec3 egoPosition = GetPosition();
    Vec3 egoFwd = GetForwardAxis();
    constexpr float SAME_DIRECTION_COS = 0.7071f; // 45도
    constexpr float VERTICAL_SEPARATION = 3.0f;   // 고가도로/지하차도 등 다른 층의 차 컷오프(m)
    for (Car *other : m_SimState->GetCars())
    {
        if (other == this)
            continue;
        // OBB 충돌검사가 x,z 평면뿐이라 머리 위/아래 차를 겹친 걸로 오판한다 -- 높이차로 먼저 거른다.
        if (std::fabs(other->GetPosition().GetY() - egoPosition.GetY()) > VERTICAL_SEPARATION)
            continue;
        // 가속 후보(+m_maxAccel)로 3초 내 더 갈 수 있는 거리(0.5*a*t^2)까지 더해 안전측으로 잡는다.
        float reachDistance = (m_speed + other->GetSpeed()) * BEHAVIOR_SAFETY_HORIZON +
                              0.5f * m_maxAccel * BEHAVIOR_SAFETY_HORIZON * BEHAVIOR_SAFETY_HORIZON +
                              GetLength() * 0.5f + other->GetLength() * 0.5f + MIN_SAFE_GAP;
        if ((other->GetPosition() - egoPosition).Length() > reachDistance)
            continue;

        // 교차/합류 방향(진행방향 차 45도 초과) 차 중 내가 우선권을 가진 차만 "양보 예정"으로 본다.
        // 같은 방향 차(앞차/뒷차)는 우선순위 대상이 아니다 -- 코리도 리더 샘플로 차간을 유지한다.
        bool crossing = egoFwd.Dot(other->GetForwardAxis()) < SAME_DIRECTION_COS;
        nearby.push_back({other, crossing && HasPriorityOver(other)});
    }
    return nearby;
}

// 전방 약 15m 경로(현재 레인 잔여 + 다음 레인)의 최소 곡률반경이 회전 매뉴버 수준(<20m)인가.
// Todo: GetMinRadiusAhead 함수 수정으로 전방 15m가 아니라 속한 레인의 전체 스플라인을 확인하게됨 수정필요
bool Car::IsTurningAhead() const
{
    constexpr float TURN_LOOK_DISTANCE = 15.0f;
    constexpr float TURN_RADIUS_THRESHOLD = 20.0f;
    if (m_currentRoad == nullptr)
        return false;

    const Spline &spline = m_currentSpline;
    float length = spline.GetLength();
    float t0 = spline.GetSplinePosition(GetPosition());
    float t1 = (length > 0.0f) ? std::min(1.0f, t0 + TURN_LOOK_DISTANCE / length) : 1.0f;
    float minRadius = spline.GetMinRadiusAhead();

    float covered = (t1 - t0) * length;
    if (covered < TURN_LOOK_DISTANCE && m_pathIndex + 1 < m_path.size())
    {
        const Spline &next = m_path[m_pathIndex + 1]->GetReferenceLine();
        float nextLength = next.GetLength();
        float nextT1 = (nextLength > 0.0f) ? std::min(1.0f, (TURN_LOOK_DISTANCE - covered) / nextLength) : 1.0f;
        minRadius = std::min(minRadius, next.GetMinRadiusAhead());
    }
    return minRadius < TURN_RADIUS_THRESHOLD;
}

// 교차 상황의 통행 우선권: 직진 > 회전(우회전/좌회전 등 작은 곡률 매뉴버), 같으면 이름 비교로
// 결정적 tie-break -- 규칙이 상보적(내가 우선이면 상대는 양보)이어야 교착이 안 생긴다.
bool Car::HasPriorityOver(const Car *other) const
{
    bool meTurning = IsTurningAhead();
    bool otherTurning = other->IsTurningAhead();
    if (meTurning != otherTurning)
        return !meTurning;
    return GetName() < other->GetName();
}

// 내 예정 경로(현재 레인 -> path의 다음 레인들)를 걸으며, 각 차를 그 세그먼트 스플라인에 투영해
// 횡 오프셋이 코리도 폭(차선폭/2 + 상대 반폭) 안이면 가상 리더 샘플로 추가한다.
// - 속도는 내 경로 방향 성분만 쓴다(dot). 교차/역방향 차는 0으로 클램프되어 그 지점의 정지
//   장애물처럼 취급되고, 코리도를 벗어나면 다음 플랜(0.2초)에서 샘플이 자연히 사라진다.
// - 옆 차선을 평행하게 달리는 차는 코리도 밖이라 여기서 걸러진다 -- 그런 차와의 간섭(차선변경 등)은
//   EvaluateTrajectorySafety의 OBB 시뮬이 담당한다.
void Car::AppendCarConstraintSamples(std::vector<RoadSpeedSample> &samples,
                                     const std::vector<NearbyCar> &nearbyCars, float lookDistance) const
{
    const Spline *spline = &m_currentSpline;
    size_t pathIndex = m_pathIndex;
    float baseDistance = 0.0f; // 내 위치에서 이 세그먼트 시작(startT)까지의 누적 경로거리
    float startT = spline->GetSplinePosition(GetPosition());

    while (spline != nullptr && baseDistance <= lookDistance)
    {
        float splineLength = spline->GetLength();
        for (const NearbyCar &nearbyCar : nearbyCars)
        {
            // 나에게 양보할 차는 가상 리더로 세우지 않는다 -- 세우면 서로가 서로 앞에 정지선을
            // 만들어 교차로에서 대칭 교착이 생긴다. 실제 차체 회피는 OBB 검사가 담당.
            Car *other = nearbyCar.car;
            float otherT = spline->GetSplinePosition(other->GetPosition());
            Vec3 projected = spline->GetPositionAt(otherT);
            float lateralOffset = (other->GetPosition() - projected).Length();
            float corridor = RoadDataManager::ROAD_WIDTH * 0.5f + other->GetHalfWidth();
            Vec3 pathDir = spline->GetDirectionAt(otherT);

            // yields(교차차 양보) 스킵은 "내 경로를 가로지르는" 차만. 커브 도는 앞차는 pathDir과 정렬돼 있으니 리더로 유지.
            const char *skip = (otherT < startT)                                                           ? "behindOnSeg"
                               : (lateralOffset > corridor)                                                ? "outCorridor"
                               : (nearbyCar.yieldsToMe && pathDir.Dot(other->GetForwardAxis()) <= 0.7071f) ? "yields"
                                                                                                           : nullptr;
            if (skip != nullptr)
                continue;

            float alongSpeed = std::max(0.0f, pathDir.Dot(other->GetForwardAxis()) * other->GetSpeed());

            float arcGap = baseDistance + (otherT - startT) * splineLength - other->GetLength() - MIN_SAFE_GAP;
            float worldGap = (other->GetPosition() - GetPosition()).Length() - other->GetLength() - MIN_SAFE_GAP;
            float gap = std::min(arcGap, worldGap); // gap<0(표준간격 침범)이면 캡이 0으로 내려가 크리핑/돌진 방지

            samples.push_back({other->GetPosition(), gap, alongSpeed, other});
        }

        baseDistance += (1.0f - startT) * splineLength;
        shared_ptr<Road> nextRoad = (pathIndex + 1 < m_path.size()) ? m_path[pathIndex + 1] : nullptr;
        if (nextRoad == nullptr)
            break;
        spline = &nextRoad->GetReferenceLine();
        startT = 0.0f;
        ++pathIndex;
    }
}

// 주행 스플라인을 따라 Pure Pursuit + 자전거 모델로 시뮬레이션
std::vector<Car::TrajectorySample> Car::SimulateEgoTrajectory(const Spline &drivingSpline, float simAccel,
                                                              const std::vector<RoadSpeedSample> &roadSamples) const
{
    std::vector<TrajectorySample> samples;
    if (drivingSpline.GetSplinePoints().size() < 2)
        return samples;

    const Spline &spline = drivingSpline;
    Vec3 pos = GetRigidbodyPosition();
    Vec3 dir = GetForwardAxis();
    float speed = m_speed;
    float lookaheadDistance = ComputeLookaheadDistance();
    float distanceTraveled = 0.0f;

    size_t stepCount = static_cast<size_t>(BEHAVIOR_SAFETY_HORIZON / BEHAVIOR_SIM_STEP + 0.5f);
    samples.reserve(stepCount);

    for (size_t i = 0; i < stepCount; ++i)
    {
        Vec3 targetPos = spline.GetLookaheadPoint(pos, lookaheadDistance);
        Vec3 toTarget = targetPos - pos;
        float distance = toTarget.Length();
        float maxSteer = CalcMaxSteerAngle(speed);
        float steerAngle = 0.0f;
        if (distance > 0.001f)
        {
            // Car::PurePursuit와 같은 조향각 공식을, carRight 없이 dir/toTarget만으로 부호까지 나오는
            // atan2 기반 signed heading error로 다시 쓴 것 (결과는 동일).
            float headingError = atan2f(dir.GetX() * toTarget.GetZ() - dir.GetZ() * toTarget.GetX(), dir.Dot(toTarget));
            steerAngle = std::clamp(-atanf(2.0f * m_wheelbase * sinf(headingError) / distance), -maxSteer, maxSteer);
        }

        float nextSpeed = speed + simAccel * BEHAVIOR_SIM_STEP;
        if (simAccel > 0.0f)
        {
            // 가속 후보는 국소 cap을 "추종"한다: cap 아래면 가속, cap을 넘으면 maxBrake 한도 내에서
            // cap까지 감속 (실제 DriveControl이 하는 일과 같다). 이래야 정지 제약이 지평선 안에 있어도
            // "다가가서 그 앞에 선다"가 가속 후보로 표현된다 -- 없으면 제약 앞 수십 m에서 얼어붙는다.
            // cap이 maxBrake보다 빨리 떨어지면(진짜 못 멈추는 상황) 초과분이 남아 overshoot로 잡힌다.
            float localCap = ComputeSpeedCapFromSamples(roadSamples, distanceTraveled);
            if (nextSpeed > localCap)
                nextSpeed = std::max(localCap, speed - m_maxBrake * BEHAVIOR_SIM_STEP);
        }
        speed = std::clamp(nextSpeed, 0.0f, m_maxSpeed);

        // ApplyMotion의 자전거 모델과 동일: angularVelocity = speed*tan(steerAngle)/wheelbase.
        float currentAngle = atan2f(dir.GetZ(), dir.GetX());
        float nextAngle = currentAngle - BEHAVIOR_SIM_STEP * speed * tanf(steerAngle) / m_wheelbase;
        pos = pos + dir * (speed * BEHAVIOR_SIM_STEP);
        dir = Vec3(cosf(nextAngle), 0.0f, sinf(nextAngle));
        distanceTraveled += speed * BEHAVIOR_SIM_STEP;

        samples.push_back({pos, dir, speed, distanceTraveled});
    }
    return samples;
}

// 상대차 스플레인 궤적 시뮬레이션
Car::OtherPrediction Car::BuildOtherPrediction(const Car *other) const
{
    OtherPrediction pred;
    pred.basePos = other->GetRigidbodyPosition();
    pred.baseFwd = other->GetForwardAxis();

    if (other->m_currentRoad == nullptr || other->m_currentSpline.GetSplinePoints().size() < 2)
        return pred; // road 없음(도착 등) -> 직진 폴백

    const Spline &spline = other->m_currentSpline;
    float t0 = spline.GetSplinePosition(pred.basePos);
    Vec3 projected = spline.GetPositionAt(t0);
    Vec3 dir0 = spline.GetDirectionAt(t0);

    // 주행선 중심에서 차선폭 이상 벗어났거나 진행방향과 반대를 보고 있으면 "주행선을 따르는 중" 가정이
    // 안 맞으니 직진 폴백.
    if ((pred.basePos - projected).Length() > RoadDataManager::ROAD_WIDTH || dir0.Dot(pred.baseFwd) < 0.0f)
        return pred;

    Vec3 left0(-dir0.GetZ(), 0.0f, dir0.GetX());
    pred.lateralOffset = (pred.basePos - projected).Dot(left0);

    // 지평선(3초) 동안 갈 수 있는 거리만큼 현재 주행선 + 상대 path의 다음 road 참조선을 이어 붙인다.
    float needDistance = other->GetSpeed() * BEHAVIOR_SAFETY_HORIZON + 5.0f;
    bool pathAligned = other->m_pathIndex < other->m_path.size() &&
                       other->m_path[other->m_pathIndex] == other->m_currentRoad;
    const Spline *segSpline = &spline;
    float segStartT = t0;
    size_t pathIndex = other->m_pathIndex;
    float covered = 0.0f;
    while (covered < needDistance)
    {
        float remainingArc = (1.0f - segStartT) * segSpline->GetLength();
        pred.segments.push_back({segSpline, segStartT, remainingArc});
        covered += remainingArc;
        if (!pathAligned || pathIndex + 1 >= other->m_path.size())
            break;
        ++pathIndex;
        segSpline = &other->m_path[pathIndex]->GetReferenceLine();
        segStartT = 0.0f;
    }
    return pred;
}

// pred를 따라 distance만큼 전진했을 때의 예상 (뒷축 위치, 진행방향). 세그먼트가 바닥나면 마지막
// 지점에서 직진 외삽.
void Car::PredictOtherPose(const OtherPrediction &pred, float distance, Vec3 &outPos, Vec3 &outFwd)
{
    if (pred.segments.empty())
    {
        outPos = pred.basePos + pred.baseFwd * distance;
        outFwd = pred.baseFwd;
        return;
    }

    float remaining = distance;
    for (size_t i = 0; i < pred.segments.size(); ++i)
    {
        const OtherPrediction::Segment &seg = pred.segments[i];
        bool isLast = (i + 1 == pred.segments.size());
        if (remaining > seg.arcLength && !isLast)
        {
            remaining -= seg.arcLength;
            continue;
        }
        if (remaining > seg.arcLength) // 마지막 세그먼트도 넘어감 -> 레인 끝에서 직진 외삽
        {
            outFwd = seg.spline->GetDirectionAt(1.0f);
            Vec3 left(-outFwd.GetZ(), 0.0f, outFwd.GetX());
            outPos = seg.spline->GetPositionAt(1.0f) + left * pred.lateralOffset + outFwd * (remaining - seg.arcLength);
            return;
        }
        float length = seg.spline->GetLength();
        float t = (length > 0.0f) ? seg.startT + remaining / length : seg.startT;
        outFwd = seg.spline->GetDirectionAt(t);
        Vec3 left(-outFwd.GetZ(), 0.0f, outFwd.GetX());
        outPos = seg.spline->GetPositionAt(t) + left * pred.lateralOffset;
        return;
    }
}

// others는 각자 자기 레인 스플라인을 따라 등속 전진한다고 예측해(BuildOtherPrediction), 매 스텝
// ego 궤적과 겹치는지 OBB로 검사한다. 레인을 벗어난 차(주차 매뉴버 등)는 직진 외삽으로 폴백.
Car::TrajectorySafety Car::EvaluateTrajectorySafety(const std::vector<TrajectorySample> &trajectory,
                                                    const std::vector<NearbyCar> &others) const
{
    TrajectorySafety result;
    const std::vector<VehicleCollision::Obstacle> &staticObstacles = RoadDataManager::Get().GetObstacles();
    if (others.empty() && staticObstacles.empty())
        return result;

    VehicleCollision::VehicleShape egoShape = BuildVehicleShape();

    std::vector<OtherPrediction> predictions;
    predictions.reserve(others.size());
    for (const NearbyCar &nearbyCar : others)
        predictions.push_back(BuildOtherPrediction(nearbyCar.car));

    // 아래 두 경우의 차는 내가 회피할 수 없으므로 충돌검사에서 뺀다(안 빼면 전 후보가 unsafe가 돼 비상제동으로 얼어붙는다):
    //  1) 뒤에서 같은 방향으로 따라오는 차 -- 추돌은 뒷차 몫.
    //  2) 지금 이미 내 차체와 겹쳐있는 차(정체로 바짝 붙음) -- 이 후보가 새로 만든 충돌이 아니라 이미 벌어진 상태라, veto하면
    //     정지해 있어도 계속 비상제동만 하게 된다. 접근 중(아직 안 겹침)인 앞차는 그대로 검사되어 실제 추돌은 막는다.
    Vec3 egoPosNow = GetPosition();
    Vec3 egoFwdNow = GetForwardAxis();
    Vec3 egoPivotNow = GetRigidbodyPosition();
    float egoHeadingNow = atan2f(egoFwdNow.GetZ(), egoFwdNow.GetX());
    std::vector<char> ignore(others.size(), 0);
    for (size_t k = 0; k < others.size(); ++k)
    {
        Car *o = others[k].car;
        bool sameDir = o->GetForwardAxis().Dot(egoFwdNow) > 0.7071f;
        if (sameDir && (o->GetPosition() - egoPosNow).Dot(egoFwdNow) < 0.0f)
        {
            ignore[k] = 1; // (1) 뒤에서 따라오는 차
            continue;
        }
        VehicleCollision::VehicleShape os = o->BuildVehicleShape();
        Vec3 oc = o->GetRigidbodyPosition() + o->GetForwardAxis() * os.pivotToCenter;
        float oh = atan2f(o->GetForwardAxis().GetZ(), o->GetForwardAxis().GetX());
        std::vector<VehicleCollision::Obstacle> one{{oc, os.halfLength, os.halfWidth, oh, 0.0f}};
        if (VehicleCollision::IsColliding(egoPivotNow, egoHeadingNow, one, egoShape))
            ignore[k] = 1; // (2) 이미 겹쳐있는 차
    }

    for (size_t i = 0; i < trajectory.size(); ++i)
    {
        float t = (i + 1) * BEHAVIOR_SIM_STEP;
        const TrajectorySample &ego = trajectory[i];
        float egoHeadingRad = atan2f(ego.direction.GetZ(), ego.direction.GetX());
        Vec3 egoCenter = ego.position + ego.direction * egoShape.pivotToCenter;

        std::vector<VehicleCollision::Obstacle> predicted;
        predicted.reserve(others.size());
        for (size_t k = 0; k < others.size(); ++k)
        {
            if (ignore[k])
                continue;
            Car *other = others[k].car;
            float travel = other->GetSpeed() * t;
            if (others[k].yieldsToMe)
            {
                // 나에게 양보할 차: 지금부터 maxBrake로 세운다고 보고 이동량을 제동거리로 캡.
                // (완전히 무시하면 이미 길을 막고 선 차를 뚫으므로, 멈춘 차체는 그대로 장애물로 남긴다)
                float stopDistance = (other->GetSpeed() * other->GetSpeed()) / (2.0f * other->m_maxBrake);
                travel = std::min(travel, stopDistance);
            }
            // 예측 피벗은 뒷축(rigidbody) 기준 -- pivotToCenter(colliderOffset.z)의 기준점과 맞춘다.
            Vec3 otherPivot;
            Vec3 otherFwd;
            PredictOtherPose(predictions[k], travel, otherPivot, otherFwd);
            VehicleCollision::VehicleShape otherShape = other->BuildVehicleShape();
            float otherHeadingRad = atan2f(otherFwd.GetZ(), otherFwd.GetX());
            Vec3 otherCenter = otherPivot + otherFwd * otherShape.pivotToCenter;

            predicted.push_back({otherCenter, otherShape.halfLength, otherShape.halfWidth, otherHeadingRad, 0.0f});

            // gap/headway는 "리더"(전방 + 동방향 45도 이내)만 집계한다. 교차/대향차의 근접은 아래
            // OBB(collisionFree)가 판정하므로, 옆 차선을 스쳐 지나가는 정상 상황이 범퍼 gap 기준으로
            // 잘못 unsafe 처리되지 않게 한다.
            Vec3 toOther = otherCenter - egoCenter;
            float aheadDot = toOther.Dot(ego.direction);
            float sameDirDot = otherFwd.Dot(ego.direction);
            float centerDist = toOther.Length();
            if (aheadDot > 0.0f && sameDirDot > 0.7071f && centerDist > 1e-4f)
            {
                // 두 OBB 중심을 잇는 축 기준 범퍼 gap(SAT support 투영). 겹치면 0으로 클램프.
                Vec3 axis = toOther / centerDist;
                Vec3 egoRight(-ego.direction.GetZ(), 0.0f, ego.direction.GetX());
                Vec3 otherRight(-otherFwd.GetZ(), 0.0f, otherFwd.GetX());
                float egoExtent = egoShape.halfLength * std::fabs(ego.direction.Dot(axis)) +
                                  egoShape.halfWidth * std::fabs(egoRight.Dot(axis));
                float otherExtent = otherShape.halfLength * std::fabs(otherFwd.Dot(axis)) +
                                    otherShape.halfWidth * std::fabs(otherRight.Dot(axis));
                float bumperGap = std::max(0.0f, centerDist - egoExtent - otherExtent);
                result.minGap = std::min(result.minGap, bumperGap);
                if (ego.speed > 0.1f) // 정지 상태(gap/0)는 헤드웨이 무한대 -- 바짝 붙은 게 아니므로 집계 제외
                    result.minTimeHeadway = std::min(result.minTimeHeadway, bumperGap / ego.speed);
            }
        }

        // 정적 장애물은 minGap 집계에서는 뺀다 -- 작은 장애물 옆을 지나가는 정상 궤적이
        // 중심거리 기준(MIN_SAFE_GAP)으로 unsafe 처리되는 걸 막기 위해 겹침 여부만 본다.
        if (VehicleCollision::IsColliding(ego.position, egoHeadingRad, predicted, egoShape) ||
            VehicleCollision::IsColliding(ego.position, egoHeadingRad, staticObstacles, egoShape))
        {
            result.collisionFree = false;
            break;
        }
    }
    return result;
}

// lane에 신호가 있고 지금 서야 하는 상황(ShouldStopForSignal)인데, trajectory 상 어느 시점엔가 정지선
// (신호 노드 위치)의 스플라인 t를 넘어버렸는데도 그 시점 속도가 거의 0이 아니면 "못 멈추고 통과" =
// 신호위반으로 본다.
bool Car::ViolatesSignal(const shared_ptr<Road> &road, const std::vector<TrajectorySample> &trajectory) const
{
    shared_ptr<RoadNode> signalNode = road ? RoadDataManager::Get().GetSignalNodeForRoad(road->GetId()) : nullptr;
    if (signalNode == nullptr || !ShouldStopForSignal(road))
        return false;

    constexpr float MOVING_EPSILON = 0.3f; // m/s -- 이 이상 속도로 정지선을 넘으면 "통과"로 본다
    const Spline &spline = road->GetReferenceLine();
    float signalT = spline.GetSplinePosition(signalNode->position);

    for (const TrajectorySample &sample : trajectory)
    {
        float sampleT = spline.GetSplinePosition(sample.position);
        if (sampleT >= signalT && sample.speed > MOVING_EPSILON)
            return true;
    }
    return false;
}

// laneChoice/speedAction 조합 하나를 궤적 시뮬레이션까지 돌려 완전히 채운 후보로 만든다. 차선변경
// 후보는 목적지까지의 새 경로도 같이 탐색해두고(newPath), 못 찾으면 targetLane을 비워 무효 후보로
// 표시한다. roadSamples는 UpdateBehaviorPlan이 한 번만 스캔해 넘긴 도로제약 샘플(제한속도/커브/신호) --
// 후보마다 다시 스캔할 필요 없이, 궤적의 각 지점에서 ComputeSpeedCapFromSamples로 국소 상한과 비교한다.
Car::BehaviorCandidate Car::BuildCandidate(LaneChoice laneChoice, SpeedAction speedAction,
                                           const shared_ptr<Road> &road, float targetOffset,
                                           const std::vector<RoadSpeedSample> &roadSamples,
                                           const std::vector<NearbyCar> &nearbyCars) const
{
    BehaviorCandidate candidate;
    candidate.laneChoice = laneChoice;
    candidate.speedAction = speedAction;
    candidate.targetRoad = road;
    candidate.targetOffset = targetOffset;
    if (road == nullptr)
        return candidate;

    // 차선변경은 road 시퀀스를 안 바꾸고 같은 road 내 오프셋만 옮긴다(경로 재탐색 불필요).
    Spline drivingSpline = RoadDataManager::Get().BuildOffsetSpline(road, targetOffset);

    float simAccel = 0.0f;
    switch (speedAction)
    {
    case SpeedAction::Accelerate:
        simAccel = m_maxAccel;
        break;
    case SpeedAction::AccelerateHalf:
        simAccel = m_maxAccel * 0.5f;
        break;
    case SpeedAction::Maintain:
        simAccel = 0.0f;
        break;
    case SpeedAction::DecelerateHalf:
        simAccel = -m_maxBrake * 0.5f;
        break;
    case SpeedAction::Decelerate:
        simAccel = -m_maxBrake;
        break;
    }
    std::vector<TrajectorySample> trajectory = SimulateEgoTrajectory(drivingSpline, simAccel, roadSamples);
    if (trajectory.empty())
    {
        candidate.targetSpeed = m_speed;
        candidate.horizonEndSpeed = m_speed;
        return candidate;
    }

    // 차선유지든 변경이든 같은 반경 기반 주변 차 목록으로 궤적과 겹치는지 확인한다 (차선유지도 예외 없음).
    TrajectorySafety safety = EvaluateTrajectorySafety(trajectory, nearbyCars);
    candidate.collisionFree = safety.collisionFree;
    candidate.minApproachGap = safety.minGap;
    candidate.minTimeHeadway = safety.minTimeHeadway;
    candidate.signalViolation = ViolatesSignal(road, trajectory);

    // 궤적의 각 지점마다 "그 지점 기준" 국소 안전속도 상한과 비교해, 가장 심하게 넘어선 값을 찾는다.
    // 지금 시점의 desiredSpeed 하나로만 3초 뒤 속도를 비교하면, desiredSpeed 자체가 접근하면서 계속
    // 낮아지는 걸 못 따라가서 감속 판단이 늦어진다 (커브/정지선에 실제로 다다르는 지점 기준으로 봐야 함).
    // 겸사겸사 이웃 샘플의 진행방향 변화(요레이트)로 그 지점의 횡가속(v*yawRate)도 구해 최댓값을 잡는다.
    float maxOvershoot = 0.0f;
    float maxLatAccel = 0.0f;
    for (size_t i = 0; i < trajectory.size(); ++i)
    {
        const TrajectorySample &sample = trajectory[i];
        float localCap = ComputeSpeedCapFromSamples(roadSamples, sample.distanceTraveled);
        maxOvershoot = std::max(maxOvershoot, sample.speed - localCap);

        if (i > 0)
        {
            const Vec3 &d0 = trajectory[i - 1].direction;
            const Vec3 &d1 = sample.direction;
            float cross = d0.GetX() * d1.GetZ() - d0.GetZ() * d1.GetX();
            float dot = d0.Dot(d1);
            float yawRate = std::fabs(atan2f(cross, dot)) / BEHAVIOR_SIM_STEP;
            maxLatAccel = std::max(maxLatAccel, sample.speed * yawRate);
        }
    }
    candidate.maxSpeedOvershoot = maxOvershoot;
    candidate.maxLateralAccel = maxLatAccel;

    size_t planStepIndex = std::min(trajectory.size(),
                                    static_cast<size_t>(BEHAVIOR_PLAN_INTERVAL / BEHAVIOR_SIM_STEP)) -
                           1;
    candidate.targetSpeed = trajectory[planStepIndex].speed;
    candidate.horizonEndSpeed = trajectory.back().speed;
    return candidate;
}

// 충돌하는(또는 목적지 도달이 불가능해진) 후보는 평가 이전에 걸러낸다.
bool Car::IsCandidateSafe(const BehaviorCandidate &candidate) const
{
    if (candidate.targetRoad == nullptr)
        return false; // BuildCandidate에서 이미 무효 처리된 후보(road 없음)
    return candidate.collisionFree;
}

// cost = w1*(속도 오차) + w2*(차선변경했으면 고정값) + w3*(궤적 최대 횡가속)
//      + w4*(직전에 고른 후보와 차선/속도결정이 달라졌으면 고정값) + w5*(신호위반이면 고정값)
//      + w6*(앞차 시간헤드웨이 부족분)
float Car::EvaluateCandidateCost(const BehaviorCandidate &candidate, float desiredSpeed) const
{
    // 속도 오차 = 목표속도에 못 미친 만큼(1배, 그냥 아쉬운 정도) + BuildCandidate가 궤적 전체를 훑어
    // 계산해둔 maxSpeedOvershoot(4배) -- 궤적 중 어느 지점에서든 그 지점 기준 안전속도를 넘어섰다는
    // 뜻이라, 신호/커브/목적지를 못 멈추고 지나칠 뻔했다는 것이므로 훨씬 나쁘게 취급한다.
    constexpr float OVERSHOOT_PENALTY = 4.0f;
    float undershoot = std::max(0.0f, desiredSpeed - candidate.horizonEndSpeed);
    float speedCost = undershoot + candidate.maxSpeedOvershoot * OVERSHOOT_PENALTY;
    float laneChangeCost = (candidate.laneChoice != LaneChoice::Keep) ? 1.0f : 0.0f;

    // 관성: 직전 틱에 고른 후보(m_currentBehaviorPlan)와 차선 결정 또는 속도 결정이 달라졌으면 물어서,
    // 매 0.2초 후보 간 근소한 비용 차이로 결정이 왔다갔다(플립플롭)하는 걸 억제한다.
    float inertiaCost = (candidate.laneChoice != m_currentBehaviorPlan.laneChoice ? 1.0f : 0.0f) +
                        (candidate.speedAction != m_currentBehaviorPlan.speedAction ? 1.0f : 0.0f);

    float signalViolationCost = candidate.signalViolation ? 1.0f : 0.0f;

    // 횡가속(승차감): 커브를 빨리 돌거나 급하게 차선변경하는 후보일수록 크다.
    float lateralAccelCost = candidate.maxLateralAccel;

    // 거리유지: 안 부딪혀도(collisionFree) 앞차에 바짝 붙는(헤드웨이가 목표보다 짧은) 후보에 소프트 비용.
    // 리더가 없으면 minTimeHeadway=max라 부족분 0.
    float headwayDeficit = (candidate.minTimeHeadway < DESIRED_HEADWAY)
                               ? (DESIRED_HEADWAY - candidate.minTimeHeadway)
                               : 0.0f;

    return m_behaviorWeights.speed * speedCost + m_behaviorWeights.laneChange * laneChangeCost +
           m_behaviorWeights.lateralAccel * lateralAccelCost + m_behaviorWeights.inertia * inertiaCost +
           m_behaviorWeights.signalViolation * signalViolationCost + m_behaviorWeights.following * headwayDeficit;
}

// BEHAVIOR_PLAN_INTERVAL(0.2초)마다 (차선유지/좌변경/우변경) x (가속/유지/감속) 후보를 만들어 평가하고,
// 가장 비용이 낮은 유효 후보를 채택한다. 차선변경 후보를 고르면 그 자리에서 바로 레인/경로를 전환하고,
// 이후 BEHAVIOR_PLAN_INTERVAL 동안은 DriveControl이 그 결과(m_currentBehaviorPlan)를 그대로 따라간다.
void Car::UpdateBehaviorPlan()
{
    if (m_currentRoad == nullptr)
        return;
    if (m_currentTime - m_lastBehaviorPlanTime < BEHAVIOR_PLAN_INTERVAL)
        return;
    m_lastBehaviorPlanTime = m_currentTime;

    // 차선변경 완료 판정: 목표(=현재) 밴드 중심에 충분히 정착했으면 매뉴버 종료(복귀 후보 생성 중단).
    if (m_laneChangeActive)
    {
        const Spline &laneSpline = m_currentSpline;
        float t = laneSpline.GetSplinePosition(GetPosition());
        float lateral = (GetPosition() - laneSpline.GetPositionAt(t)).Length();
        if (lateral < LANE_CHANGE_DONE_LATERAL)
        {
            m_laneChangeActive = false;
            m_laneChangeFromOffset = m_currentOffset;
        }
    }

    constexpr float MIN_LOOK_DISTANCE = 20.0f; // 정지 상태에서도 바로 앞 신호/제한속도는 보이게 하는 최소치

    float lookDistance = std::max(MIN_LOOK_DISTANCE, m_speed / 2 * (m_speed / m_maxBrake));
    std::vector<RoadSpeedSample> roadSamples = ScanRoadSpeedConstraints(lookDistance);

    std::vector<NearbyCar> nearbyCars = CollectNearbyCars();
    AppendCarConstraintSamples(roadSamples, nearbyCars, lookDistance);

    const RoadSpeedSample *binding = nullptr;
    float desiredSpeed = ComputeSpeedCapFromSamples(roadSamples, 0.0f, &binding);
    m_lastDesiredSpeed = desiredSpeed; // UpdateDebugWindow 표시용 캐시

    // CAH: 상한을 묶은 게 앞차면 피드포워드로 "앞차의 가속도"만 쓴다(감속만, ≤0) -- 등속 앞차엔 FF=0이라 추종 평형에서
    // −planBrake 펄스가 안 생겨 멈칫거림이 사라진다. 정적 제약(신호/커브/정지선)은 프로파일 기울기(−planBrake) 그대로.
    if (binding == nullptr)
        m_lastAccelFF = 0.0f;
    else if (binding->leader != nullptr)
        m_lastAccelFF = std::min(0.0f, binding->leader->GetAcceleration());
    else
        m_lastAccelFF = -m_planBrake;

    constexpr LaneChoice laneChoices[] = {LaneChoice::Keep, LaneChoice::ChangeLeft, LaneChoice::ChangeRight};
    constexpr SpeedAction speedActions[] = {SpeedAction::Accelerate, SpeedAction::AccelerateHalf, SpeedAction::Maintain,
                                            SpeedAction::DecelerateHalf, SpeedAction::Decelerate};

    // 현재 밴드 기준 좌/우 인접 밴드의 오프셋. 없으면 false(그 후보는 skip).
    auto adjacentBandOffset = [&](int dirSign, float &out) -> bool
    {
        const LaneSection *sec = RoadDataManager::Get().GetLateralProfile(m_currentRoad, 0.0f);
        if (sec == nullptr)
            return false;
        bool found = false;
        float best = 0.0f;
        for (const LaneBand &b : sec->bands)
        {
            if (dirSign < 0 && b.centerOffset < m_currentOffset - 1e-3f) // 왼쪽 = 더 작은 d
            {
                if (!found || b.centerOffset > best)
                    best = b.centerOffset, found = true;
            }
            else if (dirSign > 0 && b.centerOffset > m_currentOffset + 1e-3f) // 오른쪽 = 더 큰 d
            {
                if (!found || b.centerOffset < best)
                    best = b.centerOffset, found = true;
            }
        }
        if (found)
            out = best;
        return found;
    };

    auto buildStart = std::chrono::steady_clock::now();
    std::vector<BehaviorCandidate> candidates;
    for (LaneChoice laneChoice : laneChoices)
    {
        float targetOffset = m_currentOffset;
        if (laneChoice == LaneChoice::ChangeLeft && !adjacentBandOffset(-1, targetOffset))
            continue;
        if (laneChoice == LaneChoice::ChangeRight && !adjacentBandOffset(+1, targetOffset))
            continue;

        for (SpeedAction speedAction : speedActions)
            candidates.push_back(BuildCandidate(laneChoice, speedAction, m_currentRoad, targetOffset, roadSamples, nearbyCars));
    }

    // 차선변경 매뉴버 중이면 원 밴드 복귀(Abort) 후보도 만든다. 목표 밴드가 도중에 unsafe로 바뀌면
    // 위 Keep 후보들이 전부 걸러지고 이 후보들만 safe로 남아 채택된다.
    if (m_laneChangeActive && m_laneChangeFromOffset != m_currentOffset)
    {
        for (SpeedAction speedAction : speedActions)
            candidates.push_back(BuildCandidate(LaneChoice::Abort, speedAction, m_currentRoad, m_laneChangeFromOffset, roadSamples, nearbyCars));
    }

    [[maybe_unused]] double buildMs = MsSince(buildStart);

    [[maybe_unused]] auto selectStart = std::chrono::steady_clock::now();
    const BehaviorCandidate *best = nullptr;
    float bestCost = std::numeric_limits<float>::max();
    for (const BehaviorCandidate &candidate : candidates)
    {
        if (!IsCandidateSafe(candidate))
            continue;
        float cost = EvaluateCandidateCost(candidate, desiredSpeed);
        if (cost < bestCost)
        {
            bestCost = cost;
            best = &candidate;
        }
    }
    if constexpr (BEHAVIOR_PLAN_PERF)
        g_behaviorPlanPerf.Record(static_cast<int>(candidates.size()), buildMs, MsSince(selectStart));

    // 안전한 후보가 하나도 없으면(주변이 꽉 막힌 극단적 상황) 차선유지 + 감속으로 되돌아가되,
    // 일반 제동으론 못 피한다는 뜻이므로 다음 플랜까지 DriveControl이 비상 제동을 밟게 한다.
    m_emergencyBrake = (best == nullptr);
    if (m_emergencyBrake)
        DebugConsole::Log(GetName() + ": [plan] no safe candidate -> EMERGENCY BRAKE");
    BehaviorCandidate chosen = (best != nullptr)
                                   ? *best
                                   : BuildCandidate(LaneChoice::Keep, SpeedAction::Decelerate, m_currentRoad, m_currentOffset,
                                                    roadSamples, nearbyCars);

    if (chosen.laneChoice != LaneChoice::Keep && chosen.targetRoad != nullptr)
    {
        // 차선변경/복귀 커밋: 지금 밴드 오프셋을 복귀 지점으로 저장하고 매뉴버 진행 상태로 들어간다.
        m_laneChangeFromOffset = m_currentOffset;
        m_laneChangeActive = true;

        // road 시퀀스(m_path)는 그대로 -- 차선변경은 같은 road 내 오프셋 이동일 뿐이다.
        SetCurrentRoad(chosen.targetRoad, chosen.targetOffset); // 내부에서 RebuildSplineRender()까지 처리한다.

        // 다음 틱의 관성 기준선은 "새 밴드 유지".
        chosen.laneChoice = LaneChoice::Keep;
    }

    m_currentBehaviorPlan = std::move(chosen);
}

#pragma endregion
