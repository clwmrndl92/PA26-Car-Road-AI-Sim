#include "Car.h"
#include "VehicleSegment.h"
#include "Utill/DebugConsole.h"
#include "Nav/ReedsShepp.h"
#include "Nav/VehicleCollision.h"
#include "Nav/SimulationState.h"
#include "Utill/Assert.h"
#include <algorithm>
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

    // 위치를 road 참조선에 투영해 얻는 signed lateral offset d(+오른쪽). RoadDataManager::GetClosestRoad와 같은 부호규약.
    float ComputeReferenceOffset(const Spline &referenceLine, const Vec3 &position)
    {
        float t = referenceLine.GetSplinePosition(position);
        Vec3 onRef = referenceLine.GetPositionAt(t);
        Vec3 dir = referenceLine.GetDirectionAt(t);
        Vec3 rightN(dir.GetZ(), 0.0f, -dir.GetX());
        return (position - onRef).Dot(rightN);
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

    // 후축(rearAxle)이 headingRad를 보고 있을 때 target을 겨냥하는 Pure Pursuit 조향각(+ = 우조향).
    // Car::PurePursuit의 순수함수 버전 -- 궤적 시뮬레이션은 '가상의' 위치/방향으로 여러 번 굴려봐야 해서
    // 실제 리지드바디를 읽는 멤버 함수를 못 쓴다.
    float PurePursuitSteerAt(const Vec3 &rearAxle, float headingRad, const Vec3 &target, float wheelbase)
    {
        Vec3 forward(cosf(headingRad), 0.0f, sinf(headingRad));
        Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());
        float dx = target.GetX() - rearAxle.GetX();
        float dz = target.GetZ() - rearAxle.GetZ();
        float distance = std::sqrt(dx * dx + dz * dz);
        if (distance < 0.001f)
            return 0.0f;

        float cosAngle = std::clamp((forward.GetX() * dx + forward.GetZ() * dz) / distance, -1.0f, 1.0f);
        float headingError = acosf(cosAngle);
        float directionSign = (right.GetX() * dx + right.GetZ() * dz > 0.0f) ? 1.0f : -1.0f;
        return atanf((2.0f * wheelbase * sinf(headingError)) / distance) * directionSign;
    }

    // referenceLine을 오른쪽으로 d만큼 민 곡선 위에서, from을 투영해 lookahead만큼 앞선 점.
    // BuildOffsetSpline을 후보 오프셋마다 새로 만들지 않고 참조선에서 바로 뽑아 쓴다(같은 우법선 규약).
    Vec3 OffsetPathPoint(const Spline &referenceLine, const Vec3 &from, float lookahead, float d)
    {
        Vec3 onRef = referenceLine.GetLookaheadPoint(from, lookahead);
        Vec3 dir = referenceLine.GetDirectionAt(referenceLine.GetSplinePosition(onRef));
        Vec3 rightN(dir.GetZ(), 0.0f, -dir.GetX());
        return onRef + rightN * d;
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
    float targetOffset = NearestBandOffset(pose.road, pose.d);
    // 도로 밖(스폰 등)에서 처음 도로로 들어갈 때도 MOBIL 안전기준으로: 안전해질 때까지 Stop 유지.
    if (pose.road != nullptr && !IsSafeLaneEntry(pose.road, targetOffset, CollectNearbyCars()))
        return;
    SetCurrentRoad(pose.road, targetOffset);
    TryFindPathAndSetRoad();
}

Car::Mode Car::DecideNextMode(const char **reason) const
{

    if (m_mode == Mode::Stop)
    {
        if (m_roaming)
        {
            if (m_currentRoad == nullptr)
                return Mode::Stop; // 안전한 진입 지점을 기다리는 중(EnsureRoamingPath)
            *reason = "roaming";
            return Mode::Drive; // 배회 모드는 출차(Park) 없이 바로 주행 시작
        }
        if (m_destRoad == nullptr || m_currentRoad == nullptr)
        {
            return Mode::Stop; // m_currentRoad==nullptr: 목적지는 있지만 안전한 진입 지점을 기다리는 중(UpdateFindPath)
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
        m_planAccelDebug = 0.0f; // 직전 Drive의 목표가속도가 새 주행에 새지 않게 리셋
        // 회피 상태도 같이 리셋한다 -- 특히 backingUp을 들고 나가면 다음 Drive에서 ReverseSegment가
        // 이미 Abort된 채라 영영 끝나지 않는 후진 대기에 갇힌다.
        m_avoid = AvoidState{};
        m_sensor = SensorScan{};
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
        float targetOffset = NearestBandOffset(pose.road, pose.d);
        // 도로 밖(스폰 등)에서 처음 도로로 들어갈 때도 MOBIL 안전기준으로: 안전해질 때까지 Stop 유지.
        if (pose.road != nullptr && !IsSafeLaneEntry(pose.road, targetOffset, CollectNearbyCars()))
            return;
        SetCurrentRoad(pose.road, targetOffset);
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
    AccelerateVel(0.0f);
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
            Steer(PurePursuit(m_currentSpline.GetLookaheadPoint(GetRigidbodyPosition(), ComputeLookaheadDistance())));
        AccelerateVel(0.0f);
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
    // 회피 판단이 먼저: UpdateBehaviorPlan의 횡목표(MOBIL vs 회피 오프셋)와 IDM 제약(레이 히트)이
    // 이번 프레임 스캔 결과를 그대로 쓰도록 한다.
    UpdateAvoidance();
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

        // 다음 road로 합류(끼어들기)해도 뒤차에 안전한지 MOBIL 안전기준으로 판정 -- 안전하지 않으면 road 끝에서 대기.
        if (ShouldHoldForMerge(m_path[m_pathIndex + 1]))
            break;

        ++m_pathIndex;
        SetCurrentRoad(m_path[m_pathIndex], m_currentOffset);
        projectedPosition = m_currentSpline.GetLookaheadPoint(position, 0.0f);
        roadEndDistance = (roadEnd() - projectedPosition).Length();
    }
    return true;
}

float Car::ComputeLookaheadDistance() const
{
    if (!m_currentSpline.IsStraight())
    {
        return 5;
    }
    constexpr float LOOKAHEAD_TIME = 1.5f;
    return m_speed * LOOKAHEAD_TIME;
}

void Car::DriveControl()
{
    const Spline &spline = m_currentSpline;
    Vec3 target = spline.GetLookaheadPoint(GetRigidbodyPosition(), ComputeLookaheadDistance());
    float targetSteer = PurePursuit(target);
    Steer(targetSteer);

    bool IsEmergeBrake = false;
    if (IsEmergeBrake)
    {
        // // 레이/차체 스윕이 잡은 전방 위험이 상용제동으로는 못 서는 거리까지 들어오면 IDM을 건너뛰고 바로
        // // 비상제동. (IDM 제약 목록은 0.2초 주기 스캔이라, 방금 끼어든 차처럼 프레임 단위 반응이 필요한 경우가 남는다.)
        // float hazardDistance = m_sensor.frontDistance;
        // if (m_sensor.bodyContactDistance >= 0.0f && (hazardDistance < 0.0f || m_sensor.bodyContactDistance < hazardDistance))
        //     hazardDistance = m_sensor.bodyContactDistance;
        // if (m_speed > 0.1f && hazardDistance >= 0.0f &&
        //     hazardDistance < m_speed * m_speed / (2.0f * m_maxEmergBrake) + MIN_SAFE_GAP * 0.5f)
        // {
        //     EmergBrake();
        //     return;
        // }
        EmergBrake();
        return;
    }

    // 종방향: 리더/제약 목록은 행동 계획(UpdateBehaviorPlan, 0.2초 주기)이 스캔해두지만, IDM 가속도 자체는
    // 매프레임 다시 계산한다(앞차 속도/가속도/gap을 그때그때 최신값으로) -- ComputeIdmAcceleration 참고.
    float distanceOffset = (GetPosition() - m_planScanPosition).Length(); // 스캔 이후 이동거리(정적 제약 gap 보정용)
    float accelIDM = ComputeIdmAcceleration(m_lastRoadSamples, m_lastIdmParams, distanceOffset);
    float steerSpeedCap = CalcMaxSpeed(targetSteer); // 이번 프레임 조향각이 물리적으로 허용하는 한계속도(커브 안에서 반응형 유지)
    if (m_speed > steerSpeedCap)
        accelIDM = std::min(accelIDM, -m_maxBrake);

    // IDM이 뱉은 목표가속도를 저크제한으로만 수렴
    Accelerate(accelIDM);

    // Debug : Pure Pursuit 목표점(전방주시 지점) 표시
    DirectX::XMFLOAT3 targetMarkerPos = ToXMFLOAT3(target);
    targetMarkerPos.y = GetPosition().GetY() + 0.2f;
    m_targetMarker.GetTransform().SetPosition(targetMarkerPos);
}

#pragma endregion

#pragma region BehaviorPlan

std::vector<Car::RoadSpeedSample> Car::ScanRoadSpeedConstraints(float lookDistance) const
{
    constexpr float ROAD_SAMPLE_SPACING = 5.0f; // 도로 스캔 샘플 간격 (m)

    Vec3 calPosition = GetPosition();

    // 정적 장애물(맵 고정): 룩어헤드 범위 밖은 미리 걸러서 아래 프로브 판정 비용을 줄인다.
    std::vector<VehicleCollision::Obstacle> obstacles;
    {
        float reach = lookDistance + GetLength();
        for (const VehicleCollision::Obstacle &obstacle : RoadDataManager::Get().GetObstacles())
        {
            if ((obstacle.center - calPosition).Length() > reach + obstacle.halfLength + obstacle.halfWidth)
                continue;
            obstacles.push_back(obstacle);
        }
    }

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
                        samples.push_back({signalNode->position, nodeDistance - MIN_SAFE_GAP, 0.0f});
                    }
                }
            }

            // 정적 장애물: ROAD_SAMPLE_SPACING 간격으로 경로 위에 프로브 박스를 두고, 겹치는 장애물을 만나면
            // 그 앞에 0속도 샘플(가상 정지선)을 세운다. 이 세그먼트에서 가장 가까운 것 하나면 충분해 찾는
            // 즉시 멈춘다(더 먼 장애물은 다음 프로브 지점, 또는 다음 세그먼트 순회에서 처리됨).
            if (!obstacles.empty() && splineLength > 0.0f)
            {
                constexpr float OBSTACLE_PROBE_MARGIN = 0.25f;
                int sampleCount = static_cast<int>(walkDistance / ROAD_SAMPLE_SPACING) + 1;
                for (int index = 0; index <= sampleCount; ++index)
                {
                    float localDistance = std::min(static_cast<float>(index) * ROAD_SAMPLE_SPACING, walkDistance);
                    float t = std::clamp(startT + localDistance / splineLength, 0.0f, 1.0f);
                    Vec3 probePos = spline->GetPositionAt(t);
                    Vec3 probeDir = spline->GetDirectionAt(t);
                    VehicleCollision::VehicleShape probeShape;
                    probeShape.pivotToCenter = 0.0f;
                    probeShape.halfLength = ROAD_SAMPLE_SPACING * 0.5f;
                    probeShape.halfWidth = m_halfExtents.GetX() + OBSTACLE_PROBE_MARGIN;
                    float probeHeading = atan2f(probeDir.GetZ(), probeDir.GetX());
                    if (VehicleCollision::IsColliding(probePos, probeHeading, obstacles, probeShape))
                    {
                        // 프로브 안 어디에 걸렸는지는 모르니 프로브 반길이 + 안전마진만큼 앞에서 선다.
                        float stopDistance = traveledDistance + localDistance - ROAD_SAMPLE_SPACING * 0.5f - MIN_SAFE_GAP;
                        samples.push_back({probePos, stopDistance, 0.0f}); // 거리<0이면 캡이 0으로 내려가 장애물 앞에서 크리핑하지 않는다
                        break;
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

            // CheckPath와 같은 기준(MOBIL 안전판정)으로 합류 대기 중이면, 실제로 넘어가지 않을 이 경계에서 멈춘다.
            if (ShouldHoldForMerge(nextRoad))
            {
                samples.push_back({splineEnd(spline), traveledDistance - MIN_SAFE_GAP, 0.0f});
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
        // 높이로 먼저 거르기
        if (std::fabs(other->GetPosition().GetY() - egoPosition.GetY()) > VERTICAL_SEPARATION)
            continue;
        // 가속 후보(+m_maxAccel)로 3초 내 더 갈 수 있는 거리(0.5*a*t^2)까지 더해 안전측으로 잡는다.
        float reachDistance = (m_speed + other->GetSpeed()) * BEHAVIOR_SAFETY_HORIZON +
                              0.5f * m_maxAccel * BEHAVIOR_SAFETY_HORIZON * BEHAVIOR_SAFETY_HORIZON +
                              GetLength() * 0.5f + other->GetLength() * 0.5f + MIN_SAFE_GAP;
        if ((other->GetPosition() - egoPosition).Length() > reachDistance)
            continue;

        // 교차/합류 방향(진행방향 차 45도 초과) 차에 대해 우선권을 가짐
        // 같은 방향 차(앞차/뒷차)는 우선순위 대상 X
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

// 교차 상황의 통행 우선권: 직진 > 회전
bool Car::HasPriorityOver(const Car *other) const
{
    bool meTurning = IsTurningAhead();
    bool otherTurning = other->IsTurningAhead();
    if (meTurning != otherTurning)
        return !meTurning;
    return GetName() < other->GetName(); // todo : car ID 로 변경
}

void Car::AppendCarConstraintSamples(std::vector<RoadSpeedSample> &samples,
                                     const std::vector<NearbyCar> &nearbyCars, float lookDistance) const
{
    const Spline *spline = &m_currentSpline;
    shared_ptr<Road> segmentRoad = m_currentRoad; // leaderLateralOffset 계산용 -- spline은 첫 세그먼트에서 offset 주행경로라 참조선이 따로 필요
    size_t pathIndex = m_pathIndex;
    float baseDistance = 0.0f; // 내 위치에서 이 세그먼트 시작(startT)까지의 누적 경로거리
    float startT = spline->GetSplinePosition(GetPosition());

    while (spline != nullptr && baseDistance <= lookDistance)
    {
        float splineLength = spline->GetLength();
        const Spline &segRef = segmentRoad->GetReferenceLine();
        for (const NearbyCar &nearbyCar : nearbyCars)
        {
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

            RoadSpeedSample sample{other->GetPosition(), gap, alongSpeed, other};
            sample.leaderLateralOffset = ComputeReferenceOffset(segRef, other->GetPosition());
            samples.push_back(sample);
        }

        baseDistance += (1.0f - startT) * splineLength;
        shared_ptr<Road> nextRoad = (pathIndex + 1 < m_path.size()) ? m_path[pathIndex + 1] : nullptr;
        if (nextRoad == nullptr)
            break;
        spline = &nextRoad->GetReferenceLine();
        segmentRoad = nextRoad;
        startT = 0.0f;
        ++pathIndex;
    }
}

void Car::ComputeDrivableRange(const shared_ptr<Road> &road, float &outMin, float &outMax) const
{
    float halfW = GetHalfWidth();
    outMin = -halfW;
    outMax = halfW; // 밴드 없으면 참조선 중심 좁은 범위
    if (const LaneSection *sec = RoadDataManager::Get().GetLateralProfile(road, 0.0f);
        sec != nullptr && !sec->bands.empty())
    {
        outMin = std::numeric_limits<float>::max();
        outMax = -std::numeric_limits<float>::max();
        for (const LaneBand &b : sec->bands)
        {
            outMin = std::min(outMin, b.centerOffset - b.width * 0.5f);
            outMax = std::max(outMax, b.centerOffset + b.width * 0.5f);
        }
        outMin += halfW; // 차체가 도로 밖으로 안 나가게 안쪽으로 조인다
        outMax -= halfW;
        if (outMin > outMax)
            outMin = outMax = (outMin + outMax) * 0.5f;
    }
}

IDM::Params Car::BuildIdmParams(const shared_ptr<Road> &road) const
{
    IDM::Params idm;
    idm.v0 = std::min(road->GetSpeedLimit() * m_personality.speedFactor, m_maxSpeed); // A. 순항
    idm.T = IDM_TIME_HEADWAY * m_personality.headwayFactor;                           // B. 추종
    idm.s0 = MIN_SAFE_GAP * m_personality.headwayFactor;                              // B. 추종
    idm.a = m_maxAccel;
    idm.b = m_maxBrake * m_personality.brakeFactor; // C. 정지
    idm.delta = 4.0f;
    idm.coolness = 1.0f;
    return idm;
}

// samples를 IDM 가상 리더로 보고(앞차/정지선/커브속도 모두), 각 리더의 IDM 가속도 중 최솟값을 반환.
// 실제 앞차(leader!=nullptr)는 gap/속도/가속도를 그 차의 지금 상태에서 새로 읽는다(0.2초 전 스캔값 안 씀) --
// 리더가 '누구인지'만 0.2초 주기 스캔 결과를 쓰고, 값 자체는 매프레임 최신이어야 하므로. 정적 제약(신호/커브 등)은
// 살아있는 대상이 없어 sample.speed를 그대로 쓰되, gap만 distanceOffset(스캔 이후 이동거리)만큼 보정한다.
float Car::ComputeIdmAcceleration(const std::vector<RoadSpeedSample> &samples, const IDM::Params &params,
                                  float distanceOffset) const
{
    // 리더가 하나도 없으면 자유주행 가속(a_free) -- IDM 자유흐름식과 동일.
    float speedRatio = std::min(1.0f, m_speed / std::max(0.1f, params.v0));
    float bestAccel = params.a * (1.0f - std::pow(speedRatio, params.delta));

    for (const RoadSpeedSample &sample : samples)
    {
        float leaderSpeed;
        float leaderAccel;
        float gap;
        if (sample.leader != nullptr)
        {
            // 앞차는 지금 이 순간의 실제 위치/속도/가속도로 gap과 IDM 입력을 다시 잰다.
            leaderSpeed = sample.leader->GetSpeed();
            leaderAccel = sample.leader->GetAcceleration();
            gap = (sample.leader->GetPosition() - GetPosition()).Length() - sample.leader->GetLength();
        }
        else
        {
            // 정적 제약(신호/정지선/커브속도)은 스캔 당시 값 그대로, gap만 그 이후 이동거리로 보정.
            float remaining = sample.distance - distanceOffset;
            bool hardStop = sample.speed <= 0.0f; // 정지선/장애물은 gap<0(표준간격 침범)이어도 강제 제동
            if (!hardStop && remaining <= 0.0f)   // 이미 지나간 커브/제한속도는 스킵
                continue;
            // 샘플 거리는 MIN_SAFE_GAP 마진이 이미 빠져있으므로, IDM엔 실제 gap을 되돌려 넘긴다(정지거리는 IDM s0가 관장).
            leaderSpeed = sample.speed;
            leaderAccel = 0.0f;
            gap = remaining + MIN_SAFE_GAP;
        }
        float a = IDM::CalculateAcceleration(m_speed, m_acceleration, leaderSpeed, leaderAccel, gap, params);
        bestAccel = std::min(bestAccel, a);
    }
    return bestAccel;
}

// nearby 중 refLine 기준 bandCenter 밴드(±bandHalfWidth)에 속한 차들에서 egoS 앞/뒤 가장 가까운 차를
// MOBIL 상태로 뽑는다. position은 참조선 호길이 s.
Car::LaneNeighbors Car::GatherLaneNeighbors(const std::vector<NearbyCar> &nearby, const Spline &refLine,
                                            float bandCenter, float bandHalfWidth, float egoS) const
{
    LaneNeighbors out;
    float refLen = refLine.GetLength();
    float bestLeaderGap = std::numeric_limits<float>::max();
    float bestFollowerGap = std::numeric_limits<float>::max();
    for (const NearbyCar &nb : nearby)
    {
        Car *other = nb.car;
        float d = ComputeReferenceOffset(refLine, other->GetPosition());
        if (std::fabs(d - bandCenter) > bandHalfWidth)
            continue; // 이 밴드 차로가 아님

        Mobil::VehicleState st;
        st.speed = other->GetSpeed();
        st.accel = other->GetAcceleration();
        st.position = refLine.GetSplinePosition(other->GetPosition()) * refLen;
        st.length = other->GetLength();

        if (st.position >= egoS)
        {
            float gap = st.position - egoS;
            if (gap < bestLeaderGap)
            {
                bestLeaderGap = gap;
                out.leader = st;
                out.hasLeader = true;
            }
        }
        else
        {
            float gap = egoS - st.position;
            if (gap < bestFollowerGap)
            {
                bestFollowerGap = gap;
                out.follower = st;
                out.hasFollower = true;
            }
        }
    }
    return out;
}

// road의 targetOffset 근처 밴드로 (신규/강제) 진입해도 되는지 MOBIL 안전기준(유인기준 없이)으로 판정.
// 도로 밖에서 처음 들어갈 때(UpdateFindPath/EnsureRoamingPath)와 다음 road로 합류할 때(ShouldHoldForMerge)가 공유.
bool Car::IsSafeLaneEntry(const shared_ptr<Road> &road, float targetOffset, const std::vector<NearbyCar> &nearby) const
{
    const LaneSection *sec = RoadDataManager::Get().GetLateralProfile(road, 0.0f);
    if (sec == nullptr || sec->bands.empty())
        return true;

    const LaneBand *target = &sec->bands.front();
    for (const LaneBand &b : sec->bands)
        if (std::fabs(targetOffset - b.centerOffset) < std::fabs(targetOffset - target->centerOffset))
            target = &b;

    const Spline &ref = road->GetReferenceLine();
    float egoS = ref.GetSplinePosition(GetPosition()) * ref.GetLength();
    LaneNeighbors nbr = GatherLaneNeighbors(nearby, ref, target->centerOffset, target->width * 0.5f, egoS);
    if (!nbr.hasFollower)
        return true; // 뒤에 아무도 없으면 항상 진입 가능

    Mobil::VehicleState ego;
    ego.speed = m_speed;
    ego.accel = m_acceleration;
    ego.position = egoS;
    ego.length = GetLength();

    Mobil::Params mobil{MOBIL_B_SAFE, m_personality.politeness, MOBIL_A_THR};
    return Mobil::IsSafeLaneChange(ego, &nbr.follower, mobil, BuildIdmParams(road));
}

// nextRoad로 합류(끼어들기)해도 되는지 판정. 경로를 따라가려면 반드시 넘어가야 하는 전이라 '이득'은 안 따지고
// (Mobil::IsSafeLaneChange), 뒤차에 b_safe보다 가혹한 감속을 강요하는지만 본다.
bool Car::ShouldHoldForMerge(const shared_ptr<Road> &nextRoad) const
{
    if (nextRoad == nullptr)
        return false;
    return !IsSafeLaneEntry(nextRoad, m_currentOffset, m_lastNearbyCars);
}

// BEHAVIOR_PLAN_INTERVAL(0.2초)마다 IDM(종방향)로 목표가속도를, MOBIL(횡방향)로 차선변경을 정한다.
void Car::UpdateBehaviorPlan()
{
    if (m_currentRoad == nullptr)
        return;
    if (m_currentTime - m_lastBehaviorPlanTime < BEHAVIOR_PLAN_INTERVAL)
        return;
    m_lastBehaviorPlanTime = m_currentTime;

    constexpr float MIN_LOOK_DISTANCE = 20.0f; // 정지 상태에서도 바로 앞 신호/제한속도는 보이게 하는 최소치
    float lookDistance = std::max(MIN_LOOK_DISTANCE, m_speed / 2 * (m_speed / m_maxBrake));

    // ScanRoadSpeedConstraints가 내부에서 ShouldHoldForMerge(→ m_lastNearbyCars)를 참조하므로 먼저 갱신해둔다.
    m_lastNearbyCars = CollectNearbyCars();
    m_lastRoadSamples = ScanRoadSpeedConstraints(lookDistance);
    AppendCarConstraintSamples(m_lastRoadSamples, m_lastNearbyCars, lookDistance);
    AppendSensorConstraintSample(m_lastRoadSamples);
    m_lastIdmParams = BuildIdmParams(m_currentRoad);
    m_planScanPosition = GetPosition(); // DriveControl이 매프레임 여기 대비 이동거리로 정적 제약 gap을 보정

    // ---- 횡방향: MOBIL 차선변경 판정 + Lerp ----
    // (실제 종방향 IDM 가속도는 DriveControl이 위 캐시로 매프레임 다시 계산한다.)
    // 회피 중에는 MOBIL을 끄고 UpdateAvoidance가 정한 오프셋(복귀 중이면 원래 차로)으로만 수렴한다.
    float targetOffset = m_avoid.active ? (m_avoid.returning ? m_avoid.laneOffset : m_avoid.avoidOffset)
                                        : ComputeLateralTarget(m_lastNearbyCars, m_lastIdmParams);
    m_currentOffset += (targetOffset - m_currentOffset) * m_personality.laneChangeLerpAlpha;
    m_currentSpline = RoadDataManager::Get().BuildOffsetSpline(m_currentRoad, m_currentOffset);
    RebuildSplineRender();
}

// 현재 밴드를 유지할지, MOBIL이 유인+안전 판정한 인접 밴드로 변경할지 정해 목표 횡오프셋을 돌려준다.
float Car::ComputeLateralTarget(const std::vector<NearbyCar> &nearbyCars, const IDM::Params &idm,
                                float *outLaneCenter) const
{
    if (outLaneCenter != nullptr)
        *outLaneCenter = m_currentOffset; // 밴드 정보가 없는 경로에서도 항상 값이 채워져 있게

    const LaneSection *sec = RoadDataManager::Get().GetLateralProfile(m_currentRoad, 0.0f);
    if (sec == nullptr || sec->bands.empty())
        return m_currentOffset;

    // driving 밴드만 centerOffset 오름차순으로 모은다.
    std::vector<const LaneBand *> bands;
    for (const LaneBand &b : sec->bands)
        if (b.type == LaneType::Driving)
            bands.push_back(&b);
    if (bands.empty())
        return m_currentOffset;
    std::sort(bands.begin(), bands.end(), [](const LaneBand *a, const LaneBand *b)
              { return a->centerOffset < b->centerOffset; });

    // 현재 오프셋에 가장 가까운 밴드를 현재 차로로.
    size_t curIdx = 0;
    for (size_t i = 1; i < bands.size(); ++i)
        if (std::fabs(m_currentOffset - bands[i]->centerOffset) < std::fabs(m_currentOffset - bands[curIdx]->centerOffset))
            curIdx = i;
    const LaneBand &curBand = *bands[curIdx];
    if (outLaneCenter != nullptr)
        *outLaneCenter = curBand.centerOffset;

    const Spline &refLine = m_currentRoad->GetReferenceLine();
    float egoS = refLine.GetSplinePosition(GetPosition()) * refLine.GetLength();

    Mobil::VehicleState ego;
    ego.speed = m_speed;
    ego.accel = m_acceleration;
    ego.position = egoS;
    ego.length = GetLength();

    // 리더가 없을 때 넘길 아주 먼 가상 리더 (egoLeader/newLeader는 항상 존재해야 함).
    Mobil::VehicleState farLeader;
    farLeader.speed = idm.v0;
    farLeader.accel = 0.0f;
    farLeader.position = egoS + 1000.0f;
    farLeader.length = 0.0f;

    LaneNeighbors cur = GatherLaneNeighbors(nearbyCars, refLine, curBand.centerOffset, curBand.width * 0.5f, egoS);
    const Mobil::VehicleState *curLeader = cur.hasLeader ? &cur.leader : &farLeader;
    const Mobil::VehicleState *oldFollower = cur.hasFollower ? &cur.follower : nullptr;

    // 레이(정면 중앙, farLength)에 잡힌 전방 장애물도 현재 차로의 리더로 넣는다. GatherLaneNeighbors는
    // Car만 훑기 때문에, 이게 없으면 정적 장애물 앞에서 MOBIL이 "앞이 뻥 뚫렸다"고 보고 차선변경 유인을
    // 아예 못 느껴 -- 결국 장애물 코앞까지 가서 회피(차로 걸침)로만 빠져나가게 된다.
    Mobil::VehicleState sensorLeader;
    if (m_sensor.frontDistance >= 0.0f)
    {
        sensorLeader.speed = m_sensor.frontHitSpeed;
        sensorLeader.accel = 0.0f;
        // Mobil::GetGap이 leader.position - ego.position - leader.length로 재므로, length를 0으로 두고
        // 앞범퍼 기준 실측 거리를 egoS에 더하면 gap이 그대로 frontDistance가 된다.
        sensorLeader.position = egoS + m_sensor.frontDistance;
        sensorLeader.length = 0.0f;
        if (sensorLeader.position < curLeader->position)
            curLeader = &sensorLeader;
    }
    const Mobil::VehicleState &egoLeader = *curLeader;

    Mobil::Params mobil{MOBIL_B_SAFE, m_personality.politeness, MOBIL_A_THR};

    // 좌/우 인접 밴드를 MOBIL로 검사해 통과하면 그 밴드 중심으로.
    for (int di : {-1, 1})
    {
        long adjIdx = static_cast<long>(curIdx) + di;
        if (adjIdx < 0 || adjIdx >= static_cast<long>(bands.size()))
            continue;

        const LaneBand &adjBand = *bands[adjIdx];
        LaneNeighbors nbr = GatherLaneNeighbors(nearbyCars, refLine, adjBand.centerOffset, adjBand.width * 0.5f, egoS);
        const Mobil::VehicleState &newLeader = nbr.hasLeader ? nbr.leader : farLeader;
        const Mobil::VehicleState *newFollower = nbr.hasFollower ? &nbr.follower : nullptr;
        if (Mobil::EvaluateLaneChange(ego, oldFollower, egoLeader, newLeader, newFollower, mobil, idm))
            return adjBand.centerOffset;
    }
    return curBand.centerOffset; // 차선 유지
}

void Car::AppendSensorConstraintSample(std::vector<RoadSpeedSample> &samples) const
{
    // 레이에 잡힌 전방 장애물을 IDM 가상 리더로 넣어 감속 근거로 삼는다. 도로 데이터의 정적 장애물이나,
    // 차로 코리도 판정(AppendCarConstraintSamples)으로는 안 잡히는 갓 끼어든 차가 여기서 처리된다.
    // frontDistance는 앞범퍼 기준이라 차 길이를 뺄 필요가 없다.
    if (m_sensor.frontDistance >= 0.0f)
        samples.push_back({m_sensor.frontHitPosition, m_sensor.frontDistance - MIN_SAFE_GAP, m_sensor.frontHitSpeed});

    // 차체 스윕이 예고한 접촉. 정지한 대상만 넣어 잰 거리이므로 speed 0(정지 제약)으로 건다.
    // bodyContactDistance는 '차체가 닿을 때까지 더 갈 수 있는 거리' 자체라 그대로 gap으로 쓸 수 있다.
    if (m_sensor.bodyContactDistance >= 0.0f)
        samples.push_back({GetPosition(), m_sensor.bodyContactDistance - MIN_SAFE_GAP, 0.0f});
}

#pragma endregion

#pragma region Avoid

Vec3 Car::GetBodyCenter() const
{
    // 콜라이더 오프셋은 차체 로컬(z=전방, x=오른쪽) 기준. 회전은 Y축 요만 있으므로 전방/오른쪽 축으로만 합성하면 된다.
    Vec3 forward = GetForwardAxis();
    Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());
    return GetRigidbodyPosition() + forward * m_colliderOffset.z + right * m_colliderOffset.x;
}

std::vector<VehicleCollision::Obstacle> Car::BuildSensorObstacles() const
{
    // 레이/OBB 판정 대상: 지도상의 정적 장애물 + 지금 주변에 있는 차들. 차 '목록'은 0.2초 주기 캐시
    // (m_lastNearbyCars)지만 위치/헤딩/속도는 매프레임 그 차에서 새로 읽으므로 기하는 항상 최신이다.
    std::vector<VehicleCollision::Obstacle> obstacles;
    Vec3 egoPosition = GetPosition();
    float reach = AVOID_FRONT_RAY_MAX + GetLength();

    for (const VehicleCollision::Obstacle &obstacle : RoadDataManager::Get().GetObstacles())
    {
        if ((obstacle.center - egoPosition).Length() > reach + obstacle.halfLength + obstacle.halfWidth)
            continue;
        obstacles.push_back(obstacle);
    }

    // 테스트용 왕복 동적 장애물(RoadDataManager::UpdateDynamicObstacles가 매프레임 갱신) -- Car가 아니라
    // m_lastNearbyCars엔 안 잡히므로 여기서 별도로 합친다.
    for (const VehicleCollision::Obstacle &obstacle : RoadDataManager::Get().GetDynamicObstacles())
    {
        if ((obstacle.center - egoPosition).Length() > reach + obstacle.halfLength + obstacle.halfWidth)
            continue;
        obstacles.push_back(obstacle);
    }

    obstacles.reserve(obstacles.size() + m_lastNearbyCars.size());
    for (const NearbyCar &nearbyCar : m_lastNearbyCars)
    {
        const Car *other = nearbyCar.car;
        VehicleCollision::Obstacle obstacle;
        obstacle.center = other->GetBodyCenter();
        obstacle.halfLength = other->m_halfExtents.GetZ();
        obstacle.halfWidth = other->m_halfExtents.GetX();
        obstacle.headingRad = DirectionToAngleRad(other->GetForwardAxis());
        obstacle.speed = other->GetSpeed();
        obstacle.type = VehicleCollision::ObstacleType::Dynamic;
        obstacles.push_back(obstacle);
    }
    return obstacles;
}

Car::SensorScan Car::ScanSensors(const std::vector<VehicleCollision::Obstacle> &obstacles) const
{
    SensorScan scan;

    Vec3 center = GetBodyCenter();
    Vec3 forward = GetForwardAxis();
    Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());
    float headingRad = DirectionToAngleRad(forward);
    float halfLength = m_halfExtents.GetZ();
    float halfWidth = m_halfExtents.GetX();

    Vec3 frontCenter = center + forward * halfLength;
    Vec3 frontLeft = frontCenter - right * halfWidth;
    Vec3 frontRight = frontCenter + right * halfWidth;
    // 대각/측면/후방 레이 기준점 -- UpdateAvoidance의 상태전이가 꺼져 있는 동안은 안 쓴다(아래 2)+3)+4) 참고).
    // Vec3 sideLeft = center - right * halfWidth;
    // Vec3 sideRight = center + right * halfWidth;
    // Vec3 rearCenter = center - forward * halfLength;
    // Vec3 rearLeft = rearCenter - right * halfWidth;
    // Vec3 rearRight = rearCenter + right * halfWidth;

    // 전방 레이 길이 = 지금 속도로 상용제동해서 설 수 있는 거리 + 표준 간격. 정지 중에도 바로 앞은 보게 최소치를 둔다.
    float frontLength = std::clamp(m_speed * m_speed / (2.0f * m_maxBrake) + MIN_SAFE_GAP,
                                   AVOID_FRONT_RAY_MIN, AVOID_FRONT_RAY_MAX);
    // 정면 중앙 레이만 더 멀리 -- 감속/차선변경을 미리 결정하기 위한 예고용(회피 트리거엔 안 쓴다).
    float farLength = std::clamp(m_speed * AVOID_FRONT_RAY_FAR_TIME, frontLength, AVOID_FRONT_RAY_FAR_MAX);

    // rightAngle: 전방 기준 오른쪽이 +. heading은 atan2(z,x) 규약이라 각을 '더하면' 왼쪽으로 돌기 때문에 뺀다.
    auto cast = [&](const Vec3 &origin, float rightAngle, float maxDistance)
        -> std::pair<const VehicleCollision::Obstacle *, float>
    {
        float directionRad = headingRad - rightAngle;
        float distance = -1.0f;
        const VehicleCollision::Obstacle *hit =
            VehicleCollision::RaycastObstaclesHit(origin, directionRad, maxDistance, obstacles, &distance);

        SensorRay ray;
        ray.origin = origin;
        ray.hitDistance = hit != nullptr ? distance : -1.0f;
        ray.end = origin + Vec3(cosf(directionRad), 0.0f, sinf(directionRad)) *
                               (hit != nullptr ? distance : maxDistance);
        scan.rays.push_back(ray);
        return {hit, distance};
    };

    // 히트 지점이 내 주행선에서 얼마나 벗어나 있는가(m). 코너에서는 road 경계를 자주 넘으므로, 현재
    // 스플라인에서 벗어난 점은 경로상 다음 road의 주행선(참조선을 m_currentOffset만큼 민 것)으로 한 번
    // 더 재본다 -- 안 그러면 다음 road 초입의 장애물이 현재 스플라인 끝점으로 투영돼 거리가 크게 나온다.
    float corridorHalfWidth = halfWidth + AVOID_CORRIDOR_MARGIN; // 회피 트리거용(넉넉하게)
    float brakeHalfWidth = halfWidth + AVOID_PASS_CLEARANCE;     // 종방향 제동용(차체가 실제로 쓸 폭)
    auto pathDistance = [&](const Vec3 &point) -> float
    {
        float t = m_currentSpline.GetSplinePosition(point);
        float best = (point - m_currentSpline.GetPositionAt(t)).Length();
        if (best <= corridorHalfWidth || m_pathIndex + 1 >= m_path.size())
            return best;

        const Spline &nextLine = m_path[m_pathIndex + 1]->GetReferenceLine();
        float nextT = nextLine.GetSplinePosition(point);
        Vec3 dir = nextLine.GetDirectionAt(nextT);
        Vec3 rightN(dir.GetZ(), 0.0f, -dir.GetX());
        Vec3 onNextPath = nextLine.GetPositionAt(nextT) + rightN * m_currentOffset;
        return std::min(best, (point - onNextPath).Length());
    };

    // 1) 전방 부채꼴(상시): 앞범퍼 중앙/좌우 꼭지점에서 정면 3개 + 중앙에서 좌우 15/30도 4개.
    //    정면 3개(중앙+좌우 꼭지점)는 farLength로 멀리 봐서 IDM/MOBIL 예고용 감지에 쓰고,
    //    대각 4개는 제동거리(frontLength)까지만 -- 얘넨 회피 트리거 판단용이라 멀리 볼 필요가 없다.
    struct FrontRay
    {
        Vec3 origin;
        float rightAngle;
        float maxDistance;
    };
    const FrontRay frontRays[] = {
        {frontCenter, 0.0f, farLength},
        {frontLeft, 0.0f, farLength},
        {frontRight, 0.0f, farLength},
        {frontRight, ToRadians(15.0f), frontLength},
        {frontLeft, ToRadians(-15.0f), frontLength},
        {frontRight, ToRadians(30.0f), frontLength},
        {frontLeft, ToRadians(-30.0f), frontLength},
    };
    for (const FrontRay &frontRay : frontRays)
    {
        auto [hit, distance] = cast(frontRay.origin, frontRay.rightAngle, frontRay.maxDistance);
        if (hit == nullptr)
            continue;
        scan.anyFrontHit = true;

        // 히트가 정말 '내 진로 위'인지 거른다. 부채꼴 레이는 멀리서 옆차로까지 훑고 커브에서는 정면
        // 레이조차 차로를 벗어나므로, 이게 없으면 옆차로/노변에 헛제동을 한다. 기준을 차체 진행축이
        // 아니라 주행선으로 잡는 이유는 pathDistance 주석 참고(코너에서 안 서던 원인).
        Vec3 hitPosition = scan.rays.back().end;
        float offPath = pathDistance(hitPosition);
        if (offPath > corridorHalfWidth)
            continue; // 내 진로에서 완전히 벗어남(옆차로/노변)

        // 종방향 제약(IDM 가상 리더 / 비상제동)은 차체가 실제로 쓸 폭 안에 든 것만 건다. 트리거 코리도
        // (더 넓다)를 그대로 쓰면, 회피로 옆을 통과하는 도중에 아직 코리도에 걸쳐 있는 그 장애물을
        // 정지 리더로 잡아 스스로 제동해버린다 -- IDM은 s0(표준 gap)만큼 앞에서 서려 하므로 딱 그
        // 지점에 갇혀 영영 못 지나간다.
        if (offPath <= brakeHalfWidth && (scan.frontDistance < 0.0f || distance < scan.frontDistance))
        {
            scan.frontDistance = distance;
            scan.frontHitPosition = hitPosition;
            scan.frontHitSpeed = hit->speed;
        }
        // 그중 거의 멈춰 있고 제동거리 안까지 들어온 것만 '피해 갈 대상'. 정상 주행 중인 앞차는 IDM
        // 추종이 처리하고, 멀리(farLength) 보이는 장애물은 아직 MOBIL 차선변경이 처리할 몫이다.
        if (hit->speed <= AVOID_BLOCK_SPEED && distance <= frontLength)
            scan.frontBlocked = true;
    }

    // 2) 대각선 + 3) 측면 + 4) 후방: leftBlocked/rightBlocked/sideNear/rearDistance는 UpdateAvoidance의
    // 상태전이(회피 진입/복귀/후진 판단) 전용이라, 그게 꺼져 있는 동안은 레이 자체를 쏘지 않는다.
    // 되살릴 때 이 블록과 위 기준점(sideLeft 등) 주석을 함께 해제.
    /*
    // 2) 대각선(앞 꼭지점에서 45/70도) + 3) 측면(앞 꼭지점/측면 중점/뒤 꼭지점에서 90도).
    //    기본은 90도 레이만 막힘으로 본다 -- 대각선 레이는 정면의 장애물 자체를 스치기 쉬워서, 그걸
    //    막힘으로 치면 정면에 뭔가 있을 때 양쪽 다 막힌 걸로 나와 회피를 아예 못 하게 된다.
    //    단, 지금 그쪽으로 실제로 꺾고 있으면(steer) 얘기가 다르다 -- 그 방향 대각선은 더 이상 "정면
    //    장애물을 스치는 중"이 아니라 지금 돌아 들어가는 궤적 위에 있는 것이므로, 막힘으로 승격시켜야
    //    꼭짓점으로 박기 전에 재계획(막힌 쪽 재탐색/정지)이 걸린다.
    bool turningLeft = m_steerAngle < -AVOID_STEER_DEADZONE;
    bool turningRight = m_steerAngle > AVOID_STEER_DEADZONE;
    for (int side = -1; side <= 1; side += 2) // -1 = 왼쪽, +1 = 오른쪽
    {
        const Vec3 &frontCorner = side < 0 ? frontLeft : frontRight;
        const Vec3 &sideMid = side < 0 ? sideLeft : sideRight;
        const Vec3 &rearCorner = side < 0 ? rearLeft : rearRight;
        float sign = static_cast<float>(side);
        bool turningThisWay = side < 0 ? turningLeft : turningRight;

        bool diagonalHit = cast(frontCorner, sign * ToRadians(45.0f), AVOID_SIDE_RAY_LENGTH).first != nullptr;
        diagonalHit |= cast(frontCorner, sign * ToRadians(70.0f), AVOID_SIDE_RAY_LENGTH).first != nullptr;

        bool lateralHit = cast(frontCorner, sign * ToRadians(90.0f), AVOID_SIDE_RAY_LENGTH).first != nullptr;
        lateralHit |= cast(sideMid, sign * ToRadians(90.0f), AVOID_SIDE_RAY_LENGTH).first != nullptr;
        lateralHit |= cast(rearCorner, sign * ToRadians(90.0f), AVOID_SIDE_RAY_LENGTH).first != nullptr;

        bool blocked = lateralHit || (turningThisWay && diagonalHit);
        if (side < 0)
            scan.leftBlocked = blocked;
        else
            scan.rightBlocked = blocked;
        scan.sideNear = scan.sideNear || diagonalHit || lateralHit;
    }

    // 4) 후방: 회피가 막혔을 때 뒤로 물러날 공간이 있는지 확인용.
    const Vec3 rearOrigins[] = {rearLeft, rearCenter, rearRight};
    for (const Vec3 &origin : rearOrigins)
    {
        auto [hit, distance] = cast(origin, ToRadians(180.0f), AVOID_REAR_RAY_LENGTH);
        if (hit != nullptr && (scan.rearDistance < 0.0f || distance < scan.rearDistance))
            scan.rearDistance = distance;
    }
    */

    return scan;
}

// targetOffset으로 Lerp해 들어가는 궤적을 자전거 모델로 굴려, 차체(OBB)가 obstacles와 처음 겹치는
// 지점까지의 주행거리를 반환한다(maxDistance까지 안 겹치면 -1). 이 구간 동안 speed는 일정하다고 본다.
// 거리 기준으로 적분하므로(speed*dt = stepDistance) 저속에서도 스텝 수가 폭발하지 않는다.
float Car::SweepBodyPath(float targetOffset, const std::vector<VehicleCollision::Obstacle> &obstacles,
                         float speed, float maxDistance) const
{
    const Spline &referenceLine = m_currentRoad->GetReferenceLine();
    VehicleCollision::VehicleShape shape = BuildVehicleShape();

    Vec3 position = GetRigidbodyPosition(); // 자전거 모델의 기준점은 후축
    float headingRad = DirectionToAngleRad(GetForwardAxis());
    float maxSteerAngle = CalcMaxSteerAngle(speed);
    float lookahead = std::max(3.0f, m_currentSpline.IsStraight() ? speed * 1.5f : 5.0f);
    float offset = m_currentOffset;

    float stepDistance = maxDistance / AVOID_SWEEP_STEPS;
    // 실제 주행과 같은 Lerp 프로파일(BEHAVIOR_PLAN_INTERVAL마다 alpha)로 오프셋이 수렴한다고 보고 굴린다.
    float stepTime = (speed > 0.01f) ? stepDistance / speed : 0.0f;
    float lerpPerStep = m_personality.laneChangeLerpAlpha * stepTime / BEHAVIOR_PLAN_INTERVAL;

    for (int i = 0; i < AVOID_SWEEP_STEPS; ++i)
    {
        offset += (targetOffset - offset) * lerpPerStep;
        Vec3 aim = OffsetPathPoint(referenceLine, position, lookahead, offset);
        float steerAngle = std::clamp(PurePursuitSteerAt(position, headingRad, aim, m_wheelbase),
                                      -maxSteerAngle, maxSteerAngle);
        // heading은 atan2(z,x) 규약이고 조향각은 +가 우조향이라, 부호를 뒤집어 적분해야 ApplyMotion과
        // 같은 방향으로 돈다. speed*dt가 stepDistance이므로 속도 항이 그대로 소거된다.
        headingRad -= stepDistance * tanf(steerAngle) / m_wheelbase;
        position += Vec3(cosf(headingRad), 0.0f, sinf(headingRad)) * stepDistance;

        if (VehicleCollision::IsColliding(position, headingRad, obstacles, shape))
            return stepDistance * static_cast<float>(i + 1);
    }
    return -1.0f;
}

bool Car::SimulateAvoidPath(float targetOffset, const std::vector<VehicleCollision::Obstacle> &obstacles) const
{
    // 멈춰 있어도 "지금 출발하면"을 봐야 하므로 최소 속도를 가정한다.
    float speed = std::max(m_speed, AVOID_SIM_MIN_SPEED);
    return SweepBodyPath(targetOffset, obstacles, speed, speed * AVOID_SIM_TIME) < 0.0f;
}

// 레이는 '점'이라, 회전 중 바깥쪽 앞 꼭지점이 그리는 궤적(코너에서 대각선 꼭짓점으로 박는 원인)을
// 원리적으로 못 잡는다. 지금 주행선을 그대로 따라갔을 때 실제 차체가 처음 닿는 거리를 따로 재둔다.
float Car::PredictBodyContact(const std::vector<VehicleCollision::Obstacle> &obstacles) const
{
    if (m_speed < 0.1f)
        return -1.0f; // 이미 멈춰 있으면 이 예측으로 더 세울 것이 없다

    // 움직이는 대상은 뺀다 -- 이 스윕은 상대를 정지한 것으로 보므로, 같이 굴러가는 앞차까지 넣으면
    // 항상 충돌로 나와 헛제동한다. 움직이는 앞차는 IDM 추종(AppendCarConstraintSamples)이 맡는다.
    std::vector<VehicleCollision::Obstacle> stationary;
    stationary.reserve(obstacles.size());
    for (const VehicleCollision::Obstacle &obstacle : obstacles)
        if (obstacle.speed <= AVOID_BLOCK_SPEED)
            stationary.push_back(obstacle);
    if (stationary.empty())
        return -1.0f;

    // 회피 중이면 실제로 향하는 목표 오프셋으로 굴려야 한다. 지금 오프셋으로 굴리면 Lerp로 빠져나가는
    // 중인 궤적을 충돌로 잘못 보고 회피 도중에 스스로 제동해버린다.
    float targetOffset = m_avoid.active ? (m_avoid.returning ? m_avoid.laneOffset : m_avoid.avoidOffset)
                                        : m_currentOffset;
    float horizon = std::clamp(m_speed * m_speed / (2.0f * m_maxBrake) + MIN_SAFE_GAP,
                               AVOID_FRONT_RAY_MIN, AVOID_FRONT_RAY_MAX);
    return SweepBodyPath(targetOffset, stationary, m_speed, horizon);
}

bool Car::FindAvoidOffset(const SensorScan &scan, const std::vector<VehicleCollision::Obstacle> &obstacles,
                          float laneCenter, float &outOffset) const
{
    float laneWidth = RoadDataManager::ROAD_WIDTH;
    if (const LaneSection *sec = RoadDataManager::Get().GetLateralProfile(m_currentRoad, 0.0f);
        sec != nullptr && !sec->bands.empty())
    {
        const LaneBand *band = &sec->bands.front();
        for (const LaneBand &b : sec->bands)
            if (std::fabs(laneCenter - b.centerOffset) < std::fabs(laneCenter - band->centerOffset))
                band = &b;
        laneWidth = band->width;
    }

    // 도로 밖으로 나가면 안 된다: 양끝 밴드 가장자리에서 차체 반폭만큼 안쪽까지가 허용 범위.
    float minOffset = 0.0f;
    float maxOffset = 0.0f;
    ComputeDrivableRange(m_currentRoad, minOffset, maxOffset);

    // 가까운 오프셋부터: 차로 반폭 -> 한 폭 -> 두 폭. 같은 크기면 좌/우 둘 다 굴려보고 먼저 통과한 쪽을 쓴다.
    const float magnitudes[] = {laneWidth * 0.5f, laneWidth, laneWidth * 2.0f};
    for (float magnitude : magnitudes)
    {
        for (int side : {-1, 1})
        {
            if (side < 0 ? scan.leftBlocked : scan.rightBlocked)
                continue; // 바로 옆에 차/장애물이 붙어 있는 방향
            float candidate = std::clamp(laneCenter + side * magnitude, minOffset, maxOffset);
            if (std::fabs(candidate - laneCenter) < AVOID_MIN_SHIFT)
                continue; // 도로 경계에 잘려 회피가 되지 않는 후보
            if (!SimulateAvoidPath(candidate, obstacles))
                continue;
            outOffset = candidate;
            return true;
        }
    }
    return false;
}

// 매프레임: 레이를 쏘고 그 결과로 회피 상태(진입 / 오프셋 유지 / 복귀 / 막힘-후진)를 전이시킨다.
// 실제 횡오프셋 Lerp와 종방향 감속은 UpdateBehaviorPlan / DriveControl이 이 상태를 보고 수행한다.
void Car::UpdateAvoidance()
{
    if (m_currentRoad == nullptr)
        return;

    std::vector<VehicleCollision::Obstacle> obstacles = BuildSensorObstacles();
    m_sensor = ScanSensors(obstacles);
    RebuildSensorRender();

    // 회피 재구현 중 -- 레이스캔 -> IDM 제약(AppendSensorConstraintSample)/MOBIL 리더 반영만 남기고
    // 트리거/오프셋 회피/후진 등 나머지는 아래에서 하나씩 다시 켠다.
    /*
    // 레이 스캔이 놓치는 '회전 중 차체가 쓸고 가는 면적'을 OBB 스윕으로 보강한다. 접촉이 예고되면
    // 제동 근거(AppendSensorConstraintSample / DriveControl)이자 회피 트리거로 같이 쓴다.
    m_sensor.bodyContactDistance = PredictBodyContact(obstacles);
    if (m_sensor.bodyContactDistance >= 0.0f)
        m_sensor.frontBlocked = true;

    // 실제 물리 충돌은 레이가 못 본 각도(대각선 꼭짓점 등)로 박은 경우까지 잡는 최후의 안전망이다.
    // 레이 판정과 무관하게 여기서 끊어주지 않으면, 계획상으론 아직 '회피 진행 중'이라 계속 밀어붙인다.
    if (m_contactPending)
    {
        m_contactPending = false;
        if (m_avoid.backingUp)
        {
            // 후진하다 뒤를 박았다. ReverseSegment는 속도로 진행거리를 재는데 충돌로 속도가 0이 되므로,
            // 여기서 안 끊으면 진행거리가 영영 안 늘어 후진이 끝나지 않는다.
            DebugConsole::Log(GetName() + ": backup hit something -> abort backup");
            m_avoid.backingUp = false;
            std::vector<std::unique_ptr<VehicleSegment>> segments;
            segments.push_back(std::make_unique<SplineFollowSegment>());
            m_vehicleController.BeginPlan(std::move(segments));
            return;
        }
        if (m_avoid.active)
        {
            DebugConsole::Log(GetName() + ": avoid contact -> stuck");
            HandleAvoidStuck();
            return;
        }
    }

    // 후진 매뉴버 중에는 아무 판단도 하지 않는다. 끝나면 정속주행 계획으로 되돌리고, 다음 프레임에 회피를 다시 시도한다.
    if (m_avoid.backingUp)
    {
        if (!m_vehicleController.IsFinished())
            return;
        m_avoid.backingUp = false;
        m_avoid.stuck = false;
        m_avoid.blockedTimer = 0.0f;
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        segments.push_back(std::make_unique<SplineFollowSegment>());
        m_vehicleController.BeginPlan(std::move(segments));
        return;
    }

    // ---- 회피 중: 오프셋을 유지하다가 레이가 깨끗해지면 원래 차로로 복귀 ----
    if (m_avoid.active)
    {
        // 재계획: 기본적으로는 산출된 오프셋으로 Lerp만 하지만, 그 계획이 실패했다는 신호가 오면 다시 찾는다.
        //  - 내가 '밀고 들어가는 쪽'이 막혔다: 비우려던 공간에 뭔가 들어왔다.
        //    반대쪽 히트는 지금 지나치는 중인 장애물이라 정상이므로 트리거로 쓰면 안 된다 -- 그걸 쓰면
        //    회피가 성공하는 도중에 매번 스스로 취소해버린다.
        //  - 오프셋에 다 도착했는데 아직도 정면이 막혔다: 이 정도 횡이동으로는 못 피한다(더 크게 잡아야 한다).
        if (!m_avoid.returning && m_currentTime - m_avoid.lastPlanTime >= AVOID_REPLAN_INTERVAL)
        {
            bool towardRight = m_avoid.avoidOffset > m_avoid.laneOffset;
            bool shiftSideBlocked = towardRight ? m_sensor.rightBlocked : m_sensor.leftBlocked;
            bool arrivedButBlocked = m_sensor.frontBlocked &&
                                     std::fabs(m_currentOffset - m_avoid.avoidOffset) < AVOID_RETURN_TOLERANCE;
            if (shiftSideBlocked || arrivedButBlocked)
            {
                m_avoid.lastPlanTime = m_currentTime;
                float replanOffset = 0.0f;
                if (FindAvoidOffset(m_sensor, obstacles, m_avoid.laneOffset, replanOffset))
                {
                    if (std::fabs(replanOffset - m_avoid.avoidOffset) > 0.01f)
                        DebugConsole::Log(GetName() + ": avoid replan d " + ToString(m_avoid.avoidOffset) +
                                          " -> " + ToString(replanOffset));
                    m_avoid.avoidOffset = replanOffset;
                }
                else
                {
                    // 어느 쪽으로도 못 간다. 여기서 회피를 접으면(차로 중심으로 복귀) 지금 옆으로 지나치는
                    // 중인 바로 그 장애물 쪽으로 되돌아가는 꼴이 된다 -- 지금 오프셋을 그대로 지킨 채
                    // 정지(필요하면 후진)하고, 0.5초 뒤 다시 찾아본다.
                    HandleAvoidStuck();
                }
            }
        }

        bool clear = !m_sensor.anyFrontHit && !m_sensor.sideNear;
        m_avoid.clearTimer = clear ? m_avoid.clearTimer + m_deltaTime : 0.0f;

        if (!m_avoid.returning && m_avoid.clearTimer >= AVOID_CLEAR_DELAY)
        {
            m_avoid.returning = true;
            DebugConsole::Log(GetName() + ": avoid -> return to lane d " + ToString(m_avoid.laneOffset));
        }
        else if (m_avoid.returning && !clear)
        {
            m_avoid.returning = false; // 복귀 도중 다시 막히면 회피 오프셋으로 되돌아간다
        }

        if (m_avoid.returning && std::fabs(m_currentOffset - m_avoid.laneOffset) < AVOID_RETURN_TOLERANCE)
        {
            m_avoid = AvoidState{};
            SetSubMode(SubMode::D_Normal);
        }
        return;
    }

    // ---- 평상시: 전방이 계속 막혀 있을 때만 회피 판단 ----
    m_avoid.blockedTimer = m_sensor.frontBlocked ? m_avoid.blockedTimer + m_deltaTime : 0.0f;
    if (m_avoid.blockedTimer < AVOID_TRIGGER_DELAY)
    {
        m_avoid.stuck = false;
        return;
    }

    // MOBIL로 정상 차선변경이 가능하면 차로 사이를 걸치는 회피까지 갈 필요가 없다 -- 그쪽에 맡긴다.
    float laneCenter = m_currentOffset;
    float mobilTarget = ComputeLateralTarget(m_lastNearbyCars, m_lastIdmParams, &laneCenter);
    if (std::fabs(mobilTarget - laneCenter) > 0.01f)
    {
        m_avoid.stuck = false;
        return;
    }

    float avoidOffset = 0.0f;
    if (FindAvoidOffset(m_sensor, obstacles, laneCenter, avoidOffset))
    {
        m_avoid.active = true;
        m_avoid.laneOffset = laneCenter;
        m_avoid.avoidOffset = avoidOffset;
        m_avoid.lastPlanTime = m_currentTime;
        SetSubMode(SubMode::D_Avoid);
        DebugConsole::Log(GetName() + ": avoid d " + ToString(laneCenter) + " -> " + ToString(avoidOffset));
        return;
    }

    // 좌우 어느 쪽으로도 못 피한다.
    HandleAvoidStuck();
    */
}

// 회피가 막혔을 때: 그 자리에 정지하고(경적은 UpdateHorn이 알아서 울린다), 뒤가 비어 있으면 차 길이
// 절반만큼 물러나 여유를 만든다. 처음 탐색이 실패한 경우와 회피 중 재탐색이 실패한 경우가 공유한다.
void Car::HandleAvoidStuck()
{
    m_avoid.stuck = true;
    if (m_avoid.backingUp)
        return;

    float backupDistance = GetLength() * 0.5f;
    bool rearClear = m_sensor.rearDistance < 0.0f || m_sensor.rearDistance > backupDistance + MIN_SAFE_GAP;
    if (m_speed > HORN_STOP_SPEED || !rearClear)
        return; // 아직 굴러가는 중이거나 뒤가 막혔다 -- 정지 유지(IDM/비상제동이 세운다)

    m_avoid.backingUp = true;
    m_avoid.blockedTimer = 0.0f;
    std::vector<std::unique_ptr<VehicleSegment>> segments;
    segments.push_back(std::make_unique<ReverseSegment>(backupDistance));
    m_vehicleController.BeginPlan(std::move(segments));
    DebugConsole::Log(GetName() + ": avoid blocked -> back up " + ToString(backupDistance) + "m");
}

#pragma endregion
