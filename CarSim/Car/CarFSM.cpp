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
        m_planAccel = 0.0f; // 직전 Drive의 목표가속도가 새 주행에 새지 않게 리셋
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
            Steer(PurePursuit(m_currentSpline.GetLookaheadPoint(GetRigidbodyPosition(), ComputeLookaheadDistance())));
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

    // 종방향: 리더/제약 목록은 행동 계획(UpdateBehaviorPlan, 0.2초 주기)이 스캔해두지만, IDM 가속도 자체는
    // 매프레임 다시 계산한다(앞차 속도/가속도/gap을 그때그때 최신값으로) -- ComputeIdmAcceleration 참고.
    float distanceOffset = (GetPosition() - m_planScanPosition).Length(); // 스캔 이후 이동거리(정적 제약 gap 보정용)
    float aCmd = ComputeIdmAcceleration(m_lastRoadSamples, m_lastIdmParams, distanceOffset);
    float steerSpeedCap = CalcMaxSpeed(targetSteer); // 이번 프레임 조향각이 물리적으로 허용하는 한계속도(커브 안에서 반응형 유지)
    if (m_speed > steerSpeedCap)
        aCmd = std::min(aCmd, -m_maxBrake);
    m_planAccel = aCmd; // 디버그 UI 표시용 캐시
    CommandAcceleration(aCmd);

    // Debug: Pure Pursuit 목표점(전방주시 지점) 표시
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
    m_lastIdmParams = BuildIdmParams(m_currentRoad);
    m_planScanPosition = GetPosition(); // DriveControl이 매프레임 여기 대비 이동거리로 정적 제약 gap을 보정

    // ---- 횡방향: MOBIL 차선변경 판정 + Lerp ----
    // (실제 종방향 IDM 가속도는 DriveControl이 위 캐시로 매프레임 다시 계산한다.)
    float targetOffset = ComputeLateralTarget(m_lastNearbyCars, m_lastIdmParams);
    m_currentOffset += (targetOffset - m_currentOffset) * m_personality.laneChangeLerpAlpha;
    m_currentSpline = RoadDataManager::Get().BuildOffsetSpline(m_currentRoad, m_currentOffset);
    RebuildSplineRender();
}

// 현재 밴드를 유지할지, MOBIL이 유인+안전 판정한 인접 밴드로 변경할지 정해 목표 횡오프셋을 돌려준다.
float Car::ComputeLateralTarget(const std::vector<NearbyCar> &nearbyCars, const IDM::Params &idm) const
{
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
    const Mobil::VehicleState &egoLeader = cur.hasLeader ? cur.leader : farLeader;
    const Mobil::VehicleState *oldFollower = cur.hasFollower ? &cur.follower : nullptr;

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

#pragma endregion
