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
    // 두 heading의 내적이 이보다 크면 '같은 방향'(추종 관계), 작으면 '교차'(45도 초과). 우선권/충돌예측이 공유한다.
    constexpr float SAME_DIRECTION_COS = 0.7071f;
    // 고가도로/지하차도 등 다른 층에 있는 대상 컷오프(m). 주변차 수집과 교차충돌 예측이 공유한다.
    constexpr float VERTICAL_SEPARATION = 3.0f;
    constexpr float PARALLEL_SENSOR_HEADING_COS = 0.9659f; // within 15 degrees, including anti-parallel
    constexpr float PARALLEL_SENSOR_LATERAL_CLEARANCE = 0.05f;

    // road의 진행방향 밴드 중 횡오프셋 d에 가장 가까운 밴드의 centerOffset. 밴드 없으면 d 그대로.
    float NearestBandOffset(const RoadRef &road, float d)
    {
        const LaneBand *band = RoadDataManager::Get().FindNearestBand(road.road, d, road.direction);
        return band != nullptr ? band->centerOffset : d;
    }

    // 참조선 호길이 s를 '진행방향으로 증가하는' 부호 있는 값으로. 역주행이어도 앞/뒤 비교가 그대로 성립한다.
    float TravelS(const Spline &referenceLine, const Vec3 &position, float dirSign)
    {
        return referenceLine.GetSplinePosition(position) * referenceLine.GetLength() * dirSign;
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

    // 장애물의 횡오프셋 d와, 도로 우법선 방향으로 본 반폭을 함께 구한다(투영 한 번만 하려고 묶음).
    // 반폭을 외접원 반지름으로 잡으면 안 된다 -- 도로와 나란한 긴 장애물이 옆 차로까지 막은 걸로 나온다.
    void ProjectObstacle(const Spline &referenceLine, const VehicleCollision::Obstacle &obstacle,
                         float &outOffset, float &outHalfExtent)
    {
        float t = referenceLine.GetSplinePosition(obstacle.center);
        Vec3 onRef = referenceLine.GetPositionAt(t);
        Vec3 dir = referenceLine.GetDirectionAt(t);
        Vec3 rightN(dir.GetZ(), 0.0f, -dir.GetX());
        outOffset = (obstacle.center - onRef).Dot(rightN);

        Vec3 forward(cosf(obstacle.headingRad), 0.0f, sinf(obstacle.headingRad));
        Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());
        outHalfExtent = std::fabs(rightN.Dot(right)) * obstacle.halfWidth +
                        std::fabs(rightN.Dot(forward)) * obstacle.halfLength;
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
    RoadPose pose = RoadDataManager::Get().GetClosestRoad(position, GetForwardAxis());
    RoadRef entry{pose.road, pose.direction};
    float targetOffset = NearestBandOffset(entry, pose.d);
    // 도로 밖(스폰 등)에서 처음 도로로 들어갈 때도 MOBIL 안전기준으로: 안전해질 때까지 Stop 유지.
    if (pose.road != nullptr && !IsSafeLaneEntry(entry, targetOffset, CollectNearbyCars()))
        return;
    SetCurrentRoad(entry.road, targetOffset, entry.direction);
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
        // 출발은 항상 Park(P_EXIT)의 출차 판단을 거친다: 도로 진행방향과 정렬돼 있으면(도로 위 정차)
        // RS 없이 즉시 Drive로 넘어가고, 주차칸에 꺾여 있으면 RS 출차 매뉴버를 수행한다.
        return Mode::Park;
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
            Vec3 destEnd = RoadDataManager::Get().GetTravelEnd(m_destRoad, m_destDir);
            Vec3 projectedPosition = m_destRoad->GetReferenceLine().GetLookaheadPoint(GetPosition(), 0.0f);
            arrived = (destEnd - projectedPosition).Length() < ARRIVE_DISTANCE;
            // Park 노드는 도로 끝이 아니라 통로 중간에 있을 수 있어 노드 반경으로도 도착을 인정한다.
            if (m_pendingParkNode != nullptr)
                arrived |= (m_pendingParkNode->position - GetPosition()).Length() < PARK_ARRIVE_DISTANCE;
        }

        if (arrived && GetParkTargetNode() != nullptr)
        {
            *reason = "arrived at destination";
            return Mode::Park; // 입차 시퀀스로
        }
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

void Car::BeginSegment(std::unique_ptr<VehicleSegment> segment)
{
    std::vector<std::unique_ptr<VehicleSegment>> segments;
    segments.push_back(std::move(segment));
    m_vehicleController.BeginPlan(std::move(segments));
}

void Car::OnModeEnter(Mode prev)
{
    if (m_mode == Mode::Drive)
    {
        SetSubMode(SubMode::D_Normal);
        m_planAccelDebug = 0.0f; // 직전 Drive의 목표가속도가 새 주행에 새지 않게 리셋
        // 회피 상태도 같이 리셋한다 -- 특히 backingUp을 들고 나가면 다음 Drive에서 ReverseSegment가
        // 이미 Abort된 채라 영영 끝나지 않는 후진 대기에 갇힌다.
        m_avoidTrigger = AvoidTriggerState{};
        m_stuck = StuckState{};
        m_maneuver = ManeuverState{};
        m_wait = WaitState{};
        m_sensor = SensorScan{};
        m_speedCap = -1.0f;
        BeginSegment(std::make_unique<SplineFollowSegment>());
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
            BeginSegment(std::make_unique<CenterSteerSegment>());
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
    m_path = RoadDataManager::Get().FindPath(CurrentRoadRef(), m_destRoad);
    m_pathIndex = 0;
    if (m_path.empty())
    {
        m_destRoad = nullptr;
        SetCurrentRoad(nullptr, 0.0f, LaneDirection::Forward);
        return false;
    }

    m_destDir = m_path.back().direction; // 도착 판정에 쓸 destRoad 끝(어느 쪽으로 들어가 끝나는가)
    return true;
}

RoadRef Car::PickRandomSuccessor(const RoadRef &road) const
{
    if (road.road == nullptr)
        return RoadRef{};

    const std::vector<RoadRef> &successors = RoadDataManager::Get().GetRoadSuccessors(road.road->GetId(), road.direction);
    if (successors.empty())
        return RoadRef{};
    return successors[rand() % successors.size()];
}

vector<RoadRef> Car::BuildRoamingPath(const RoadRef &startRoad) const
{
    vector<RoadRef> path;
    if (startRoad.road == nullptr)
        return path;

    path.push_back(startRoad);
    RoadRef current = startRoad;
    for (size_t i = 0; i < ROAMING_MIN_AHEAD; ++i)
    {
        RoadRef next = PickRandomSuccessor(current);
        if (next.road == nullptr)
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
        RoadPose pose = RoadDataManager::Get().GetClosestRoad(GetPosition(), GetForwardAxis());
        RoadRef entry{pose.road, pose.direction};
        float targetOffset = NearestBandOffset(entry, pose.d);
        // 도로 밖(스폰 등)에서 처음 도로로 들어갈 때도 MOBIL 안전기준으로: 안전해질 때까지 Stop 유지.
        if (pose.road != nullptr && !IsSafeLaneEntry(entry, targetOffset, CollectNearbyCars()))
            return;
        SetCurrentRoad(entry.road, targetOffset, entry.direction);
        m_path = BuildRoamingPath(CurrentRoadRef());
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
        RoadRef next = PickRandomSuccessor(m_path.back());
        if (next.road == nullptr)
            break; // 막다른 road
        m_path.push_back(next);
    }
}
#pragma endregion

#pragma region Park

void Car::UpdatePark()
{
    if (m_parkPlanPending)
    {
        // 완전히 정지할 때까지는 RS 계획을 세우지 않고 감속만 한다.
        if (m_speed > 0.0f)
        {
            AccelerateVel(0.0f);
            return;
        }
        m_parkPlanPending = false;
        DebugConsole::Log(GetName() + ": Park plan pending resolved: fully stopped, beginning RS plan");
        BeginParkPlan();
    }

    if (!m_vehicleController.IsFinished())
    {
        m_wantSegmentTick = true;
        return;
    }

    if (m_subMode == SubMode::P_EXIT)
    {
        // 출차 완료: 이제 도로 위. 더 이상 이 주차칸에 있는 게 아니므로 예약을 풀고 비운다.
        m_parkSequenceActive = false; // 시퀀스 종료 — 다음 프레임 Drive로 전환 허용.
        if (m_parkSpot != nullptr)
        {
            m_SimState->ReleaseParkSpot(m_parkSpot->id);
            m_parkSpot = nullptr;
        }
        // 안 지우면 다음 도로 정차 후 출발(P_EXIT) 때 이 주차장 기준으로 통로를 다시 찾아버린다.
        m_parkNodeId = -1;

        // TryFindPathAndSetRoad는 실패 시 dest/current를 지우므로, 방금 나온 도로를 잃지 않게 보존해뒀다 복원한다.
        shared_ptr<Road> savedDestRoad = m_destRoad;
        LaneDirection savedDestDir = m_destDir;
        shared_ptr<Road> exitRoad = m_currentRoad;
        float exitOffset = m_currentOffset;
        LaneDirection exitDir = m_travelDir;
        if (!TryFindPathAndSetRoad())
        {
            m_destRoad = savedDestRoad;
            m_destDir = savedDestDir;
            SetCurrentRoad(exitRoad, exitOffset, exitDir);
            m_path.clear();
            m_pathIndex = 0;
        }
        return;
    }

    if (m_parkSpot != nullptr)
    {
        // 입차 leg 1(-> 통로 위 진입점 P)이 끝났으면, 이제 P에서 스팟까지 leg 2를 이어 계획한다.
        // (통로를 못 찾아 바로 스팟으로 간 경우엔 PlanEnterForCurrentSpot이 이미 leg2로 세팅해 건너뛴다.)
        if (m_subMode == SubMode::P_ENTER_LEG1)
        {
            // leg 1처럼 완전히 멈춘 뒤 그 pose에서 leg 2를 계획한다(open-loop RS는 시작 pose 기준). 대기 중
            // 컨트롤러가 finished여도 m_parkSequenceActive가 Park를 유지하므로 Drive로 새지 않는다.
            if (m_speed > 0.0f)
            {
                AccelerateVel(0.0f);
                return;
            }

            // 방금 끝난 leg가 진입점 중간 leg였을 수 있다: 아직 P에서 멀면 다시 P를 겨냥한다(leg 체인).
            // 시도 횟수 상한으로 같은 자리를 맴도는 무한 체인은 끊는다.
            constexpr int PARK_MAX_LEG_TRIES = 4;
            constexpr float P_ARRIVE_DISTANCE = 2.0f;
            Vec3 pPos;
            float pAngleRad;
            if (ComputeParkPrePose(pPos, pAngleRad) &&
                (m_rigidbody.GetPosition() - pPos).Length() > P_ARRIVE_DISTANCE &&
                m_parkLegTries < PARK_MAX_LEG_TRIES && PlanEnterForCurrentSpot())
                return;

            SetSubMode(SubMode::P_ENTER_LEG2);
            BeginParkSpotLeg();
            return;
        }

        // 스팟까지의 leg가 pure pursuit로 끝났으면, 실제로 도착한 pose 기준으로 같은 목표까지 RS를 한 번
        // 더 계획해 추종 잔여 오차(주로 최종 헤딩)를 없앤다. exact=true라 이번엔 오차가 남지 않는다.
        if (m_subMode == SubMode::P_ENTER_LEG2)
        {
            if (m_speed > 0.0f)
            {
                AccelerateVel(0.0f);
                return;
            }
            SetSubMode(SubMode::P_ENTER_ALIGN);
            Vec3 spotTarget = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
            float spotAngleRad = DirectionToAngleRad(m_parkSpot->direction);
            if (PlanParkLegTo(spotTarget, spotAngleRad, /*exact=*/true))
                return;
        }
    }

    if (m_subMode == SubMode::None || m_subMode == SubMode::P_ENTER_ALIGN)
    {
        m_parkSequenceActive = false; // 입차 시퀀스 종료 — 다음 프레임 Stop으로 전환 허용.
        m_destRoad = nullptr;
        SetCurrentRoad(nullptr, 0.0f, LaneDirection::Forward);
        return;
    }
}

void Car::BeginParkPlan()
{
    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);
    Vec3 rigidPosition = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());

    if (m_subMode == SubMode::P_EXIT)
    {
        // CheckPath와 기준 맞추려 앞바퀴 위치로 도로 검색. 통로(주차장 참조)가 있으면 그리로 먼저 나가고,
        // 없으면(통로 없이 스팟에 바로 대둔 차 등) 가장 가까운 일반 도로로 직접 나간다.
        Vec3 frontPos = GetPosition();
        RoadPose exitPose;
        if (m_parkSpot != nullptr && m_parkSpot->id >= 0)
            exitPose = RoadDataManager::Get().GetClosestParkingRoad(frontPos, GetForwardAxis());
        if (exitPose.road == nullptr)
            exitPose = RoadDataManager::Get().GetClosestRoad(frontPos, GetForwardAxis());
        if (exitPose.road == nullptr)
        {
            DebugConsole::Log(GetName() + ": BeginParkPlan: no road found to exit onto, abandoning this park attempt");
            m_destRoad = nullptr;
            SetSubMode(SubMode::None);
            return;
        }

        const LaneBand *exitBand = RoadDataManager::Get().FindNearestBand(exitPose.road, exitPose.d, exitPose.direction);
        SetCurrentRoad(exitPose.road, exitBand != nullptr ? exitBand->centerOffset : exitPose.d, exitPose.direction);

        // 밴드 중심 주행선(역방향이면 이미 뒤집혀 있음)이 곧 출차 목표선이다.
        const Spline &exitSpline = m_currentSpline;
        Vec3 closestDir = exitSpline.GetDirectionAt(exitSpline.GetSplinePosition(frontPos));

        // 진행 방향과 충분히 정렬돼 있으면 RS 출차 매뉴버 없이 바로 주행
        constexpr float EXIT_HEADING_ALIGN_ANGLE = ToRadians(60.0f);
        float headingDot = std::clamp(GetForwardAxis().Dot(closestDir), -1.0f, 1.0f);
        if (std::acos(headingDot) <= EXIT_HEADING_ALIGN_ANGLE)
        {
            m_vehicleController.BeginPlan({});
            return;
        }

        VehicleCollision::VehicleShape shape = BuildVehicleShape();
        const std::vector<VehicleCollision::Obstacle> &obstacles = RoadDataManager::Get().GetObstacles();
        auto isCollisionFree = [&](const ReedsShepp::Path &candidate)
        {
            for (const ReedsShepp::PoseSample &pose : ReedsShepp::GetPoses(candidate, rigidPosition, startAngleRad, turningRadius))
            {
                if (VehicleCollision::IsColliding(pose.position, pose.headingRad, obstacles, shape))
                    return false;
            }
            return true;
        };

        constexpr float EXIT_LEAD_DISTANCES[] = {6.0f, 12.0f, 18.0f};
        for (float leadDistance : EXIT_LEAD_DISTANCES)
        {
            Vec3 targetPos = exitSpline.GetLookaheadPoint(frontPos, leadDistance);
            float targetAngleRad = DirectionToAngleRad(exitSpline.GetDirectionAt(exitSpline.GetSplinePosition(targetPos)));
            ReedsShepp::Path path = ReedsShepp::GetOptimalPath(rigidPosition, startAngleRad, targetPos, targetAngleRad, turningRadius, isCollisionFree);
            if (path.empty())
                continue;
            m_vehicleController.BeginPlan(BuildReedSheppSegments(path, rigidPosition, startAngleRad, turningRadius));
            RebuildRSDebugRender(path, rigidPosition, startAngleRad, turningRadius, targetPos, targetAngleRad);
            return;
        }

        // 모든 목표가 실패: 이미 차가 그 자리를 점유 중이므로 예약은 풀지 않고, 정렬된 셈 치고 주행 시작.
        DebugConsole::Log(GetName() + ": BeginParkPlan: RS exit failed for all lead distances, driving as-is");
        m_vehicleController.BeginPlan({});
        return;
    }

    // ---- 이하 입차 ----
    int parkNodeId = -1;
    if (m_parkSpot == nullptr && m_pendingParkNode != nullptr)
    {
        parkNodeId = m_pendingParkNode->id;
        shared_ptr<RoadNode> pendingNode = m_pendingParkNode;
        m_parkSpot = m_SimState->TryReserveParkSpot(parkNodeId);
        m_pendingParkNode = nullptr;
        if (m_parkSpot == nullptr)
        {
            bool hasAnyParkSpot = std::any_of(pendingNode->children.begin(), pendingNode->children.end(),
                                              [](const weak_ptr<RoadNode> &weakChild)
                                              {
                                                  shared_ptr<RoadNode> child = weakChild.lock();
                                                  return child != nullptr && child->nodeType == RoadNodeType::ParkSpot;
                                              });
            if (hasAnyParkSpot)
            {
                // ParkSpot은 있지만 전부 예약 중
                DebugConsole::Log(GetName() + ": Park spot reservation failed for node " + std::to_string(parkNodeId) +
                                  ": all ParkSpot children reserved, abandoning destination");
                m_parkSequenceActive = false; // 시퀀스 취소 — Park에 갇히지 않도록.
                m_destRoad = nullptr;
                return;
            }
            // 주차장이 아닌 노드(ParkSpot 자식이 아예 없음, 예: 길가 목적지)
            // m_parkSpot을 그대로 이 노드로 취급한다(위치/방향만 쓰므로 실제 예약 스팟과 동일하게 동작).
            m_parkSpot = pendingNode;
        }
        m_parkNodeId = parkNodeId;  // 재시도(다른 빈 자리)용으로 Park 노드 id 보관
        m_triedParkSpotIds.clear(); // 새 입차 시퀀스 — 시도 목록 초기화
    }

    if (m_parkSpot == nullptr)
    {
        m_parkSequenceActive = false; // 스팟 정보가 없는 비정상 입차 -- Park에 갇히지 않게 취소
        return;
    }

    // 주차 (BeginParkPlan은 Park 진입시 딱 한 번만 불리므로 여기 오는 시점의 subMode는 항상 leg1)
    if (m_subMode == SubMode::P_ENTER_LEG1)
    {
        if (!BeginParkEnterOrRetry())
        {
            DebugConsole::Log(GetName() + ": BeginParkPlan: no reachable ParkSpot for node " + std::to_string(parkNodeId) +
                              ", abandoning destination");
            m_destRoad = nullptr;
            m_parkSequenceActive = false;
        }
    }
}

// 옛 주차레인(parking_lanes)을 대신해, 스팟이 접한 주차장 통로(Road::IsParkingRoad)의 참조선을 쓴다.
// GetClosestParkingRoad는 일반 도로를 후보에서 뺀다 -- 안 그러면 통로가 없는 스팟이 근처 큰길을
// 통로로 오인해 엉뚱한 P점을 잡는다.
const Spline *Car::FindBestParkingSpline(LaneDirection *outDirection) const
{
    if (m_parkSpot == nullptr)
        return nullptr;

    RoadPose pose = RoadDataManager::Get().GetClosestParkingRoad(m_parkSpot->position, GetForwardAxis());
    if (pose.road == nullptr)
        return nullptr;

    if (outDirection != nullptr)
    {
        // RS 매뉴버로 heading이 돌아가도 P가 흔들리지 않게, 이미 그 통로 위면 진입 때 정한 진행방향을 쓴다.
        *outDirection = (pose.road == m_currentRoad) ? m_travelDir : pose.direction;
    }
    return &pose.road->GetReferenceLine();
}

// P(스팟 앞 지점) pose: 통로 위 스팟 최근접점에서 진행방향으로 P_LEAD_DISTANCE만큼 더 간 점.
bool Car::ComputeParkPrePose(Vec3 &outPos, float &outAngleRad) const
{
    LaneDirection aisleDirection = LaneDirection::Forward;
    const Spline *bestSpline = FindBestParkingSpline(&aisleDirection);
    if (bestSpline == nullptr)
        return false;

    constexpr float P_LEAD_DISTANCE = 3.0f;
    // 참조선은 항상 forward 프레임이라, 진행방향으로 나아가려면 부호를 곱해 s 증감을 뒤집는다.
    float dirSign = GetTravelSign(aisleDirection);
    outPos = bestSpline->GetLookaheadPoint(m_parkSpot->position, P_LEAD_DISTANCE * dirSign);
    float pParam = bestSpline->GetSplinePosition(outPos);
    outAngleRad = DirectionToAngleRad(bestSpline->GetDirectionAt(pParam) * dirSign);
    return true;
}

// m_parkSpot로의 입차 시작 계획: 통로가 있으면 leg 1(-> 스팟 앞 P), 없으면 스팟으로 직접(leg 2 없음).
// P까지 한 번에 RS가 안 나오면(진입로가 벽으로 막힌 pose 등 -- RS는 장애물을 "피해가는" 게 아니라
// 후보를 "거부"만 하므로 좁은 입구에서 전멸할 수 있다) 통로에서 내 위치와 가장 가까운 점(=진입점)
// 으로 먼저 가는 중간 leg를 계획한다. leg 완료 후 UpdatePark가 다시 P를 겨냥한다(leg 체인).
// 계획을 시작했으면 true(+ m_subMode를 해당 leg로 설정), 이 자리로는 경로를 못 찾으면 false.
bool Car::PlanEnterForCurrentSpot()
{
    LaneDirection aisleDirection = LaneDirection::Forward;
    Vec3 pPos;
    float pAngleRad;
    if (ComputeParkPrePose(pPos, pAngleRad))
    {
        ++m_parkLegTries;
        if (PlanParkLegTo(pPos, pAngleRad))
        {
            SetSubMode(SubMode::P_ENTER_LEG1); // leg 1 진행 중 — P 도착 후 UpdatePark가 leg 2를 잇는다.
            return true;
        }

        // 진입점 중간 leg. 이미 진입점 근처인데도 P를 못 가는 거면 이 스팟은 접근 불가.
        constexpr float ENTRY_MIN_DISTANCE = 2.0f;
        const Spline *bestSpline = FindBestParkingSpline(&aisleDirection);
        if (bestSpline == nullptr)
            return false;
        float dirSign = GetTravelSign(aisleDirection);
        Vec3 rigidPosition = m_rigidbody.GetPosition();
        float entryT = bestSpline->GetSplinePosition(rigidPosition);
        Vec3 entryPos = bestSpline->GetPositionAt(entryT);
        if ((entryPos - rigidPosition).Length() > ENTRY_MIN_DISTANCE)
        {
            float entryAngleRad = DirectionToAngleRad(bestSpline->GetDirectionAt(entryT) * dirSign);
            if (PlanParkLegTo(entryPos, entryAngleRad))
            {
                DebugConsole::Log(GetName() + ": PlanEnterForCurrentSpot: P unreachable, detouring via aisle entry point");
                SetSubMode(SubMode::P_ENTER_LEG1);
                return true;
            }
        }
        return false; // 이 스팟의 P까지 못 감 -> 다음 스팟
    }

    // 통로 없음 -> 스팟으로 직접. 단 RS는 도로를 무시한 직행 매뉴버이므로, 도착 판정이
    // 노드에서 먼 곳에서 났다면(멀리 있는 도로 끝 등) 맵을 가로지르는 긴 직행이 나온다 --
    // 상한을 넘으면 이 목적지는 포기한다 (호출자가 abandoning 처리).
    constexpr float MAX_DIRECT_LEG_DISTANCE = 15.0f;
    Vec3 spotTarget = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
    float spotAngleRad = DirectionToAngleRad(m_parkSpot->direction);
    if ((spotTarget - m_rigidbody.GetPosition()).Length() > MAX_DIRECT_LEG_DISTANCE)
    {
        DebugConsole::Log(GetName() + ": PlanEnterForCurrentSpot: direct park target too far (" +
                          std::to_string((spotTarget - m_rigidbody.GetPosition()).Length()) + "m), giving up");
        return false;
    }
    if (PlanParkLegTo(spotTarget, spotAngleRad))
    {
        SetSubMode(SubMode::P_ENTER_LEG2); // leg 1 없이 바로 leg 2
        return true;
    }
    return false;
}

// 현재 m_parkSpot을 tried에 넣고 예약을 푼 뒤, 같은 Park의 다음 빈 자리를 예약한다. 남으면 true.
bool Car::ReserveNextParkSpot()
{
    if (m_parkSpot != nullptr)
    {
        m_triedParkSpotIds.insert(m_parkSpot->id);
        m_SimState->ReleaseParkSpot(m_parkSpot->id);
    }
    m_parkSpot = m_SimState->TryReserveParkSpot(m_parkNodeId, m_triedParkSpotIds);
    return m_parkSpot != nullptr;
}

// 현재 스팟부터 입차를 시도하고, 실패하면 다음 빈 자리로 넘어가며 다 시도한다. 계획을 시작하면 true,
// 모든 자리가 안 되면 false.
bool Car::BeginParkEnterOrRetry()
{
    while (m_parkSpot != nullptr)
    {
        m_parkLegTries = 0; // 새 스팟 -- 진입 leg 체인 시도 횟수 리셋
        if (PlanEnterForCurrentSpot())
            return true;
        if (!ReserveNextParkSpot())
            return false;
    }
    return false;
}

// 현재 pose -> target 까지 Reeds-Shepp으로 계획해 실행시킨다. 못 찾으면 false.
// exact=true면 Pure Pursuit(BuildReedSheppSegments) 대신 정지-조향-이동-정지 방식의 정밀 실행
// (BuildExactSegments)을 쓴다 -- 짧은 최종 정렬 보정 등 추종 오차를 남기면 안 되는 경우에만 켠다.
bool Car::PlanParkLegTo(const Vec3 &targetPos, float targetAngleRad, bool exact)
{
    Vec3 rigidPosition = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());
    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);

    VehicleCollision::VehicleShape shape = BuildVehicleShape();
    const std::vector<VehicleCollision::Obstacle> &obstacles = RoadDataManager::Get().GetObstacles();
    auto isCollisionFree = [&](const ReedsShepp::Path &candidate)
    {
        for (const ReedsShepp::PoseSample &pose : ReedsShepp::GetPoses(candidate, rigidPosition, startAngleRad, turningRadius))
        {
            if (VehicleCollision::IsColliding(pose.position, pose.headingRad, obstacles, shape))
                return false;
        }
        return true;
    };

    ReedsShepp::Path path = ReedsShepp::GetOptimalPath(rigidPosition, startAngleRad, targetPos, targetAngleRad, turningRadius, isCollisionFree);
    if (path.empty())
        return false;

    if (exact)
        m_vehicleController.BeginPlan(BuildExactSegments(path, m_maxSteerAngle));
    else
        m_vehicleController.BeginPlan(BuildReedSheppSegments(path, rigidPosition, startAngleRad, turningRadius));
    RebuildRSDebugRender(path, rigidPosition, startAngleRad, turningRadius, targetPos, targetAngleRad);
    return true;
}

// 입차 leg 2: 현재 pose(=P)에서 예약된 스팟까지 Reeds-Shepp. UpdatePark가 leg 1 완료 시 호출한다.
// 못 들어가면 다음 빈 자리로 넘어가 leg 1부터 다시 시도하고, 모든 자리가 안 되면 멈춘다.
void Car::BeginParkSpotLeg()
{
    Vec3 spotTarget = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
    float spotAngleRad = DirectionToAngleRad(m_parkSpot->direction);
    if (PlanParkLegTo(spotTarget, spotAngleRad))
        return; // leg 2 성공

    DebugConsole::Log(GetName() + ": BeginParkSpotLeg: can't tuck into ParkSpot " + std::to_string(m_parkSpot->id) +
                      " from P, trying next spot");
    // 이 자리 실패 -> 다음 빈 자리부터 leg 1부터 다시. PlanEnterForCurrentSpot이 새 자리에 맞는
    // subMode(leg1 또는 통로 없으면 leg2)를 알아서 세팅한다.
    if (ReserveNextParkSpot() && BeginParkEnterOrRetry())
        return;

    // 남은 자리 없음 -> 입차 종료(빈 플랜 -> 다음 프레임 UpdatePark 완료 처리, 현재 자리에 멈춤).
    DebugConsole::Log(GetName() + ": BeginParkSpotLeg: no reachable ParkSpot left, stopping");
    m_destRoad = nullptr;
    m_vehicleController.BeginPlan({});
}

#pragma endregion

#pragma region Stop
void Car::UpdateStop()
{
    if (m_destRoad != nullptr)
    {
        Vec3 destEnd = RoadDataManager::Get().GetTravelEnd(m_destRoad, m_destDir);
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
        Steer(0.0f);
        AccelerateVel(0.0f);
        return;
    }

    std::shared_ptr<RoadNode> dest = RoadDataManager::Get().GetRandomParkNode();
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

    UpdateSensors();

    // 물리 충돌 뒷수습과 후진 매뉴버는 어느 서브모드에 있든 먼저 끊어야 한다.
    if (!HandleContactPending() && !UpdateBackupState())
    {
        switch (m_subMode)
        {
        case SubMode::D_WaitObstacle:
            UpdateWaitObstacle();
            break;
        case SubMode::D_Avoid:
            UpdateAvoid();
            break;
        case SubMode::D_LaneChange:
            UpdateLaneChange();
            break;
        default:
            DecideAvoidance(); // D_Normal: 다음 서브모드를 고른다
            break;
        }
    }

    UpdateDrivePlan();

    m_wantSegmentTick = true;
}

bool Car::CanEnterFromCurrentBand(const RoadRef &next) const
{
    std::vector<const LaneBand *> entry = RoadDataManager::Get().GetEntryBands(CurrentRoadRef(), next);
    if (entry.empty())
        return true; // 차로 제약이 없는 전이

    const LaneBand *current = RoadDataManager::Get().FindNearestBand(m_currentRoad, m_currentOffset, m_travelDir);
    for (const LaneBand *band : entry)
        if (band == current)
            return true;
    return false;
}

bool Car::RepathFromCurrentBand()
{
    // 지금 차로에서 실제로 진입 가능한 후속 road만 후보로 남긴다.
    std::vector<RoadRef> candidates;
    for (const RoadRef &succ : RoadDataManager::Get().GetRoadSuccessors(m_currentRoad->GetId(), m_travelDir))
        if (CanEnterFromCurrentBand(succ))
            candidates.push_back(succ);
    if (candidates.empty())
        return false;

    if (m_roaming)
    {
        // 배회는 목적지가 없으니 갈 수 있는 곳 하나만 고르면 끝 -- A*를 다시 돌 필요가 없다.
        m_path.resize(m_pathIndex + 1); // 지금 road까지만 남기고 못 가게 된 뒷부분을 버린다
        m_path.push_back(candidates[rand() % candidates.size()]);
        MaintainRoamingPath();
        DebugConsole::Log(GetName() + ": lane missed -> reroute (roaming) via road " + ToString(m_path[m_pathIndex + 1].road->GetId()));
        return true;
    }

    // 목적지 주행: 후보를 첫 칸으로 강제한 경로들 중 가장 짧은 것(칸 수 기준 근사). 후보가 몇 개 안 돼
    // (보통 2~4) A*를 몇 번 더 돌아도 된다. 실패해도 기존 m_path를 살려둬야 하므로 확정 후에 갈아끼운다.
    std::vector<RoadRef> best;
    for (const RoadRef &candidate : candidates)
    {
        std::vector<RoadRef> tail = RoadDataManager::Get().FindPath(candidate, m_destRoad);
        if (!tail.empty() && (best.empty() || tail.size() < best.size()))
            best = std::move(tail);
    }
    if (best.empty())
        return false; // 지금 차로에서 갈 수 있는 어느 road로도 목적지에 못 간다

    DebugConsole::Log(GetName() + ": lane missed -> reroute via road " + ToString(best.front().road->GetId()));
    m_path.resize(m_pathIndex + 1);
    m_path.insert(m_path.end(), best.begin(), best.end());
    m_destDir = m_path.back().direction;
    return true;
}

bool Car::CheckPath()
{
    if (m_currentRoad == nullptr)
    {
        m_destRoad = nullptr;
        return false;
    }

    constexpr float LANE_TRANSITION_THRESHOLD = 2.0f; // 다음 차선으로 완전히 넘어가는(전환되는) 임계값

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
        // 신호로 서야 하면 road를 안 넘긴다. nextRoadId(다음 road=이동)를 알면 그 이동만 특정해서 gating.
        int nextRoadId = (m_pathIndex + 1 < m_path.size()) ? m_path[m_pathIndex + 1].road->GetId() : -1;
        if (ShouldStopForSignal(m_currentRoad, m_travelDir, nextRoadId))
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
                SetCurrentRoad(nullptr, 0.0f, LaneDirection::Forward);
                return false;
            }
        }

        // 좌/우회전 차로를 못 맞춘 채 도착했으면 억지로 넘어가지 않고, 지금 차로에서 갈 수 있는 road로
        // 경로를 다시 짠다(설계상 '차로를 못 맞추면 그 차로를 따라가고 재탐색'). 갈 데가 없으면 끝에서 정지.
        if (!CanEnterFromCurrentBand(m_path[m_pathIndex + 1]) && !RepathFromCurrentBand())
            break;

        // 다음 road로 합류(끼어들기)해도 뒤차에 안전한지 MOBIL 안전기준으로 판정 -- 안전하지 않으면 road 끝에서 대기.
        if (ShouldHoldForMerge(m_path[m_pathIndex + 1]))
            break;
        // Reserve the junction only when committing this transition, never during look-ahead scanning.
        // Vehicles entering from the same road share it and retain normal IDM headway.
        // Other approaches wait at their own road end.
        if (!TryReserveJunction(m_path[m_pathIndex + 1]))
            break;

        ++m_pathIndex;
        const RoadRef &next = m_path[m_pathIndex];
        bool hasLaneMapping = false;
        float nextOffset = RoadDataManager::Get().ResolveConnectingOffset(CurrentRoadRef(), next, m_currentOffset, &hasLaneMapping);
        // 명시적 lane_links 매핑이 없는 직결(램프 등)은 두 road 참조선이 이 지점에서 기하학적으로 안 맞을 수
        // 있어 fromOffset을 그대로 못 쓴다 -- 실제 위치를 다음 road 참조선에 투영해 물리적 연속성을 보장한다.
        if (!hasLaneMapping)
            nextOffset = ComputeReferenceOffset(next.road->GetReferenceLine(), position);
        SetCurrentRoad(next.road, nextOffset, next.direction);
        projectedPosition = m_currentSpline.GetLookaheadPoint(position, 0.0f);
        roadEndDistance = (roadEnd() - projectedPosition).Length();
    }
    return true;
}

void Car::DriveControl()
{
    const Spline &spline = m_currentSpline;

    constexpr float LOOKAHEAD_TIME = 1.5f;
    float lookaheadDistance = m_currentSpline.IsStraight() ? m_speed * LOOKAHEAD_TIME : 5;
    Vec3 target = spline.GetLookaheadPoint(GetRigidbodyPosition(), lookaheadDistance);
    float targetSteer = PurePursuit(target);
    // Stanley 조향 (다시 쓸 수도 있어 주석으로 남겨둠)
    // float targetSteer = Stanley(spline);
    Steer(targetSteer);

    // 종방향: 리더/제약 목록은 행동 계획(UpdateBehaviorPlan, 0.2초 주기)이 스캔해두지만, IDM 가속도 자체는
    // 매프레임 다시 계산한다(앞차 속도/가속도/gap을 그때그때 최신값으로) -- ComputeIdmAcceleration 참고.
    SpeedLimitDebug prevLimitDebug = m_limitDebug;                        // kind/목표속도가 바뀔 때만 로그를 찍기 위한 이전 프레임 값
    float distanceOffset = (GetPosition() - m_planScanPosition).Length(); // 스캔 이후 이동거리(정적 제약 gap 보정용)
    float elapsedTime = m_currentTime - m_lastBehaviorPlanTime;           // 스캔 이후 지난 시간(정적 제약의 가상 리더 전진 보정용)
    float accelIDM = ComputeIdmAcceleration(m_lastRoadSamples, m_lastIdmParams, distanceOffset, elapsedTime, &m_limitDebug);

    if (m_speedCap >= 0.0f && m_speed > m_speedCap)
    {
        float capGap = std::max(1.0f, (m_speed * m_speed - m_speedCap * m_speedCap) / (2.0f * m_maxBrake) + MIN_SAFE_GAP);
        float capAccel = IDM::CalculateAcceleration(m_speed, m_acceleration, m_speedCap, 0.0f, capGap, m_lastIdmParams);
        if (capAccel < accelIDM)
        {
            accelIDM = capAccel;
            m_limitDebug = SpeedLimitDebug{"speedCap", m_speedCap, capGap};
        }
    }

    float steerSpeedCap = CalcMaxSpeed(targetSteer); // 이번 프레임 조향각이 물리적으로 허용하는 한계속도(커브 안에서 반응형 유지)
    if (m_speed > steerSpeedCap && -m_maxBrake < accelIDM)
    {
        accelIDM = -m_maxBrake;
        m_limitDebug = SpeedLimitDebug{"steerCap", steerSpeedCap, 0.0f};
    }

    // 상대차가 완전히 멈추면 canClaimPriority(우선권 행사 조건)가 영원히 막혀 서로 교착될 수 있다 -- 그
    // 상태로 이만큼 버티면 "상대가 움직이고 있어야 한다"는 조건을 풀어준다. yieldsToMe는 시간과 무관하게
    // 둘 중 한쪽만 true이므로(HasPriorityOver가 만드는 전순서), 둘 다 타임아웃이 걸려도 실제로 진행하는
    // 건 원래 우선권 있던 쪽 하나뿐이다 -- 양쪽이 동시에 끼어들어 부딪히는 경우는 안 생긴다.
    bool stuckOnStoppedCar = m_limitDebug.label.rfind("car:", 0) == 0 &&
                             m_limitDebug.targetSpeed < AVOID_BLOCK_SPEED && m_speed < AVOID_BLOCK_SPEED;
    m_priorityStuckElapsed = stuckOnStoppedCar ? m_priorityStuckElapsed + m_deltaTime : 0.0f;

    // constexpr float LOG_TARGET_SPEED_EPS = 0.5f / 3.6f;
    // if (m_limitDebug.label != prevLimitDebug.label ||
    //     std::fabs(m_limitDebug.targetSpeed - prevLimitDebug.targetSpeed) > LOG_TARGET_SPEED_EPS)
    // {
    //     // v/accel: 이 전환이 실제로 일어난 순간의 차 속도/가속도 -- 라벨이 "free"로 풀렸어도 저크제한 때문에
    //     // 실제 가속도가 아직 음수(관성으로 계속 감속 중)인지 여기서 바로 보인다.
    //     DebugConsole::Log(GetName() + ": limit " + prevLimitDebug.label + "(" + ToString(prevLimitDebug.targetSpeed * 3.6f) +
    //                       "km/h) -> " + m_limitDebug.label + "(" + ToString(m_limitDebug.targetSpeed * 3.6f) +
    //                       "km/h) | v=" + ToString(m_speed * 3.6f) + "km/h accel=" + ToString(accelIDM));
    // }

    m_prevSpeedForStallLog = m_speed;

    // IDM이 뱉은 목표가속도를 저크제한으로만 수렴
    Accelerate(accelIDM);

    // Debug : Stanley 경로투영점(앞축 기준) 표시
    float pathT = spline.GetSplinePosition(GetPosition());
    Vec3 pathPoint = spline.GetPositionAt(pathT);
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
        samples.push_back({splineEnd(&m_currentSpline), currentNodeDistance, currentNodeSpeed, nullptr, 0.0f, "curRoadEnd"});
    }
    {
        // 앞쪽 road는 '진행방향으로 본' 주행선(역방향이면 뒤집힌 것)을 써야 t/거리 계산이 그대로 성립한다.
        Spline segmentLine;
        const Spline *spline = &m_currentSpline;
        RoadRef segment = CurrentRoadRef();
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

            RoadRef nextRoad = (pathIndex + 1 < m_path.size()) ? m_path[pathIndex + 1] : RoadRef{};
            int nextRoadId = nextRoad.road != nullptr ? nextRoad.road->GetId() : -1;

            // 신호로 서면 CheckPath가 이 road 끝을 안 넘긴다 -- mergeWait/junctionWait과 같은 규약으로 정지점도
            // 정지선과 road 끝 중 앞선 쪽으로 잡고 스캔을 끊는다. 신호노드가 road 끝보다 뒤(다음 road 쪽)에 있으면
            // 그 거리를 여유로 보고 road 끝을 지나쳐 크리핑하다 스플라인 끝을 벗어난다.
            shared_ptr<RoadNode> signalNode = RoadDataManager::Get().GetSignalNodeForRoad(segment.road->GetId(), nextRoadId);
            if (signalNode != nullptr && splineLength > 0.0f && ShouldStopForSignal(segment.road, segment.direction, nextRoadId))
            {
                float nodeT = spline->GetSplinePosition(signalNode->position);
                float nodeDistance = (segment.road == m_currentRoad)
                                         ? (signalNode->position - calPosition).Length()
                                         : traveledDistance + (nodeT - startT) * splineLength;
                float stopDistance = std::min(nodeDistance, traveledDistance + segmentDistance);
                samples.push_back({signalNode->position, stopDistance - MIN_SAFE_GAP, 0.0f, nullptr, 0.0f, "signal"});
                break;
            }

            traveledDistance += walkDistance;
            remainingDistance -= walkDistance;
            if (remainingDistance <= 0.0f)
                break;

            if (!nextRoad.road)
            {
                // 경로가 여기서 끝난다는 것은 이 road가 destRoad라는 뜻: 진짜 정지 지점인 road 끝에 0속도를 박는다.
                if (segment.road == m_destRoad)
                    samples.push_back({splineEnd(spline), traveledDistance, 0.0f, nullptr, 0.0f, "destEnd"});
                break;
            }

            // CheckPath와 같은 기준(MOBIL 안전판정)으로 합류 대기 중이면, 실제로 넘어가지 않을 이 경계에서 멈춘다.
            bool holdForMerge = ShouldHoldForMerge(nextRoad);
            bool holdForJunction = segment.road == m_currentRoad && ShouldHoldForJunction(nextRoad);
            if (holdForMerge || holdForJunction)
            {
                const char *kind = holdForJunction ? "junctionWait" : "mergeWait";
                samples.push_back({splineEnd(spline), traveledDistance - MIN_SAFE_GAP, 0.0f, nullptr, 0.0f, kind});
                break;
            }

            float nextNodeSpeed = std::min(nextRoad.road->GetSpeedLimit(), m_maxSpeed);
            segmentLine = RoadDataManager::Get().BuildOffsetSpline(nextRoad.road, 0.0f, nextRoad.direction);
            Vec3 nextStart = segmentLine.GetSplinePoints().empty() ? segmentStart : segmentLine.GetSplinePoints().front();
            samples.push_back({nextStart, traveledDistance, nextNodeSpeed, nullptr, 0.0f, "roadLimit"});

            segment = nextRoad;
            segmentStart = nextStart;
            spline = &segmentLine;
            ++pathIndex;
        }
    }
    std::sort(samples.begin(), samples.end(), [](const RoadSpeedSample &a, const RoadSpeedSample &b)
              { return a.distance < b.distance; });
    return samples;
}

std::vector<Car::NearbyCar> Car::CollectNearbyCars() const
{
    constexpr float BEHAVIOR_SAFETY_HORIZON = 3.0f; // 주변 차 수집 반경 산정용 미래 시야(초, 사람의 3초 룰)

    std::vector<NearbyCar> nearby;
    Vec3 egoPosition = GetPosition();
    Vec3 egoFwd = GetForwardAxis();
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
        // 둘 다 느리면 이 반경이 15m까지 줄어든다 -- 옆차로에 '멈춰 있는' 차가 그 밖에 있으면 MOBIL 눈에
        // 안 보여서 그 차로로 변경해버린다. 레이가 보는 만큼(AVOID_FRONT_RAY_MAX)은 항상 후보로 둔다.
        if ((other->GetPosition() - egoPosition).Length() > std::max(reachDistance, AVOID_FRONT_RAY_MAX))
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
        // 곡률은 진행방향과 무관하므로 참조선을 그대로 본다.
        const Spline &next = m_path[m_pathIndex + 1].road->GetReferenceLine();
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
    // 서로 상대를 리더로 보고 멈춘 교착(둘 다 정지라 canClaimPriority가 안 풀림)을 끊는 타임아웃(s).
    // AVOID_WAIT_TIMEOUT과 별개 -- 그건 센서가 잡은 위협 전용이고, 이건 도로 코리도 IDM 정지 전용.
    // INTERSECTION_YIELD_TIMEOUT과 값은 같지만 의미가 달라 별도 이름을 쓴다(교차로 데드락 타이머 자체는 아님).
    constexpr float PRIORITY_STUCK_TIMEOUT = INTERSECTION_YIELD_TIMEOUT;

    Spline segmentLine; // 앞쪽 road의 진행방향 주행선(역방향이면 뒤집힌 것). spline이 이걸 가리킨다.
    const Spline *spline = &m_currentSpline;
    RoadRef segment = CurrentRoadRef(); // leaderLateralOffset 계산용 -- spline은 offset 주행경로라 참조선이 따로 필요
    size_t pathIndex = m_pathIndex;
    float offset = m_currentOffset; // 뒤쪽 세그먼트 코리도 스플라인을 세울 차로 오프셋(아래에서 세그먼트마다 갱신)
    float baseDistance = 0.0f;      // 내 위치에서 이 세그먼트 시작(startT)까지의 누적 경로거리
    float startT = spline->GetSplinePosition(GetPosition());

    while (spline != nullptr && baseDistance <= lookDistance)
    {
        float splineLength = spline->GetLength();
        const Spline &segRef = segment.road->GetReferenceLine();
        for (const NearbyCar &nearbyCar : nearbyCars)
        {
            if (!m_SimState->IsCarAlive(nearbyCar.car))
                continue; // 캐시된 뒤 삭제된 차(0.2초 주기 갱신 사이 delete됨) -- dangling 포인터
            Car *other = nearbyCar.car;

            // Opposite-direction traffic on this same road uses the other carriageway, not this path's leader lane.
            if (other->m_currentRoad == segment.road && other->m_travelDir != segment.direction)
                continue;

            float otherT = spline->GetSplinePosition(other->GetPosition());
            Vec3 projected = spline->GetPositionAt(otherT);
            float lateralOffset = (other->GetPosition() - projected).Length();
            // 회피 오프셋으로 옆을 스쳐 지나가는 중에는 코리도를 차체가 실제로 쓰는 폭까지 좁힌다 --
            // 평소 폭(차로 반폭)으로 두면 옆으로 다 비켜났는데도 리더로 남아 IDM이 s0 앞에 세워버려서
            // 회피를 걸어놓고도 영영 못 지나간다(정적 장애물의 AVOID_PASS_CLEARANCE와 같은 이유).
            float corridor = (m_subMode == SubMode::D_Avoid)
                                 ? GetHalfWidth() + other->GetHalfWidth() + AVOID_PASS_CLEARANCE
                                 : RoadDataManager::ROAD_WIDTH * 0.5f + other->GetHalfWidth();
            Vec3 pathDir = spline->GetDirectionAt(otherT);

            // 범퍼 대 범퍼 종방향 gap. 위치가 앞축 기준이라 내 앞오버행과 상대 뒤오버행을 뺀다.
            float bumperMargin = FrontOverhang() + other->RearOverhang();
            float arcGap = baseDistance + (otherT - startT) * splineLength - bumperMargin;
            // 직선(현) 항에서 횡성분을 뺀 종방향 성분. 거리(.Length()) 그대로 쓰면 옆으로 벌어진 만큼이
            // gap에 섞여, 교차로에서 90도로 붙은 차의 종방향 여유를 실제보다 크게 본다(안 서고 측면으로
            // 박는 원인). 내 진행방향 투영으로 재면 굽은 길 너머의 차까지 gap이 0으로 눌려 헛제동한다.
            float chord = (other->GetPosition() - GetPosition()).Length();
            float straightGap = std::sqrt(std::max(0.0f, chord * chord - lateralOffset * lateralOffset)) - bumperMargin;
            float gap = std::min(arcGap, straightGap) - MIN_SAFE_GAP; // gap<0(표준간격 침범)이면 캡이 0으로 내려가 크리핑/돌진 방지

            // 우선권은 상대가 실제로 비켜줄 수 있을 때만 행사한다. 이미 제동거리 안이거나 상대가 멈춰
            // 있으면(교차로에 갇혀 못 비킨다) 리더로 유지해야 한다 -- 여기서 지우면 IDM엔 감속 근거가 없고,
            // 회피 쪽은 ThreatKind::Vehicle이라 "IDM이 처리한다"며 손을 떼서 아무도 안 선다.
            // 다만 상대가 완전히 멈춘 채 오래가면(둘 다 서로를 리더로 보는 교착) 이 조건 때문에 영영 못
            // 풀린다 -- 내가 그 상태로 PRIORITY_STUCK_TIMEOUT 이상 버텼으면 정지 조건을 풀어준다.
            float brakingGap = m_speed * m_speed / (2.0f * m_maxBrake) + MIN_SAFE_GAP;
            bool priorityTimedOut = m_priorityStuckElapsed >= PRIORITY_STUCK_TIMEOUT;
            bool canClaimPriority = nearbyCar.yieldsToMe && (other->GetSpeed() > AVOID_BLOCK_SPEED || priorityTimedOut) && gap > brakingGap;

            // yields(교차차 양보) 스킵은 "내 경로를 가로지르는" 차만. 커브 도는 앞차는 pathDir과 정렬돼 있으니 리더로 유지.
            const char *skip = (otherT < startT)            ? "behindOnSeg"
                               : (lateralOffset > corridor) ? "outCorridor"
                               : (canClaimPriority && pathDir.Dot(other->GetForwardAxis()) <= SAME_DIRECTION_COS)
                                   ? "yields"
                                   : nullptr;
            if (skip != nullptr)
                continue;

            float alongSpeed = std::max(0.0f, pathDir.Dot(other->GetForwardAxis()) * other->GetSpeed());

            RoadSpeedSample sample{other->GetPosition(), gap, alongSpeed, other};
            sample.leaderLateralOffset = ComputeReferenceOffset(segRef, other->GetPosition());
            sample.leaderScanPosition = other->GetPosition();
            samples.push_back(sample);
        }

        baseDistance += (1.0f - startT) * splineLength;
        RoadRef nextRoad = (pathIndex + 1 < m_path.size()) ? m_path[pathIndex + 1] : RoadRef{};
        if (nextRoad.road == nullptr)
            break;
        // 참조선(d=0)으로 세우면 반대 차로가 코리도 폭(ROAD_WIDTH/2 남짓) 안에 들어와 반대편 차를 리더로
        // 잡는다 -- 실제로 타게 될 차로 오프셋으로 세워야 반대 차로가 corridor 밖으로 벌어진다.
        offset = RoadDataManager::Get().ResolveConnectingOffset(segment, nextRoad, offset);
        segmentLine = RoadDataManager::Get().BuildOffsetSpline(nextRoad.road, offset, nextRoad.direction);
        spline = &segmentLine;
        segment = nextRoad;
        startT = 0.0f;
        ++pathIndex;
    }
}

void Car::ComputeDrivableRange(const RoadRef &road, float &outMin, float &outMax) const
{
    // float halfW = GetHalfWidth();
    float halfW = GetHalfWidth() * 0.5f;
    // 밴드가 없는 구간(주로 교차로 연결로)의 폴백. 차체 반폭(±1m 남짓)으로 조이면 앞을 막고 선 차를
    // 옆으로 지나갈 여지가 사실상 없어져 데드락이 풀리지 않는다 -- 한 차로 폭 안쪽까지는 허용한다.
    // 실제로 부딪히는지는 SimulateAvoidPath의 OBB 스윕이 따로 본다.
    outMin = -(RoadDataManager::ROAD_WIDTH - halfW);
    outMax = RoadDataManager::ROAD_WIDTH - halfW;

    // 진행방향 차로만 -- 회피/차선변경이 마주 오는 차로까지 후보로 잡으면 정면충돌한다.
    std::vector<const LaneBand *> bands = RoadDataManager::Get().GetDrivingBands(road.road, road.direction);
    if (bands.empty())
        return;

    outMin = std::numeric_limits<float>::max();
    outMax = -std::numeric_limits<float>::max();
    for (const LaneBand *b : bands)
    {
        outMin = std::min(outMin, b->centerOffset - b->width * 0.5f);
        outMax = std::max(outMax, b->centerOffset + b->width * 0.5f);
    }
    outMin += halfW; // 차체가 도로 밖으로 안 나가게 안쪽으로 조인다
    outMax -= halfW;
    if (outMin > outMax)
        outMin = outMax = (outMin + outMax) * 0.5f;
}

IDM::Params Car::BuildIdmParams(const shared_ptr<Road> &road) const
{
    constexpr float IDM_TIME_HEADWAY = 1.5f; // IDM T 기본값: 앞차와 원하는 시간 간격(s). m_personality.headwayFactor를 곱해 씀.

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

// 스캔 주기 0.2초, gap/속도/가속도는 매 프레임 계산
float Car::ComputeIdmAcceleration(const std::vector<RoadSpeedSample> &samples, const IDM::Params &params,
                                  float distanceOffset, float elapsedTime, SpeedLimitDebug *outDebug) const
{
    // 리더가 하나도 없으면 자유주행 가속(a_free) -- IDM 자유흐름식과 동일.
    float speedRatio = std::min(1.0f, m_speed / std::max(0.1f, params.v0));
    float bestAccel = params.a * (1.0f - std::pow(speedRatio, params.delta));
    if (outDebug != nullptr)
        *outDebug = SpeedLimitDebug{"free", params.v0, 0.0f};

    for (const RoadSpeedSample &sample : samples)
    {
        // samples는 0.2초 주기 캐시라 그 사이 delete된 차를 가리킬 수 있다 -- 매프레임 여기서 역참조한다.
        if (sample.leader != nullptr && !m_SimState->IsCarAlive(sample.leader))
            continue;

        float leaderSpeed;
        float leaderAccel;
        float gap;
        if (sample.leader != nullptr)
        {
            // 속도/가속도는 지금 값으로 다시 읽되, gap은 스캔 시점의 '경로상 범퍼 gap'을 기준으로 양쪽이
            // 그동안 전진한 거리만큼만 보정한다(정적 제약 분기와 같은 방식). 매프레임 직선거리로 다시
            // 재면 횡방향 이격이 gap에 섞여 교차 중인 차의 여유를 실제보다 크게 본다.
            leaderSpeed = sample.leader->GetSpeed();
            leaderAccel = sample.leader->GetAcceleration();
            // 후진(회피 백업)까지 맞게 보려면 부호 있는 전진량이어야 한다 -- 거리로 재면 후진도 멀어짐이 된다.
            float leaderTravel = (sample.leader->GetPosition() - sample.leaderScanPosition)
                                     .Dot(sample.leader->GetForwardAxis());
            gap = sample.distance + MIN_SAFE_GAP - distanceOffset + leaderTravel;
        }
        else
        {
            // 정적 제약(신호/정지선/커브속도)의 가상 리더가 스캔 이후 sample.speed로 실제 전진한 것처럼 gap을 보정.
            // 안 그러면 gap이 상대속도가 아니라 내 속도 그대로 줄어들어(=리더가 못 박힌 정지물처럼 보여) 필요
            // 이상으로 세게 감속하다 s0 근처에서 급제동, 저크제한 때문에 그 관성으로 목표속도 밑까지 밀린다.
            // sample.speed==0(신호/정지선)이면 이 항이 0이라 기존과 동일한 고정점 취급.
            float remaining = sample.distance - distanceOffset + sample.speed * elapsedTime;
            bool hardStop = sample.speed <= 0.0f; // 정지선/장애물은 gap<0(표준간격 침범)이어도 강제 제동
            if (!hardStop && (remaining <= 0.0f || m_speed <= sample.speed))
                continue;
            // 샘플 거리는 MIN_SAFE_GAP 마진이 이미 빠져있으므로, IDM엔 실제 gap을 되돌려 넘긴다(정지거리는 IDM s0가 관장).
            leaderSpeed = sample.speed;
            leaderAccel = 0.0f;
            gap = remaining + MIN_SAFE_GAP;
        }
        float a = IDM::CalculateAcceleration(m_speed, m_acceleration, leaderSpeed, leaderAccel, gap, params);
        if (a < bestAccel)
        {
            bestAccel = a;
            if (outDebug != nullptr)
                *outDebug = SpeedLimitDebug{sample.leader != nullptr ? "car:" + sample.leader->GetName() : sample.kind, leaderSpeed, gap};
        }
    }
    return bestAccel;
}

// nearby 중 refLine 기준 bandCenter 밴드(±bandHalfWidth)에 속한 차들 + 그 밴드를 막고 있는 장애물
// (m_sensorObstacles)에서 egoS 앞/뒤 가장 가까운 걸 MOBIL 상태로 뽑는다. position은 참조선 호길이 s.
// 장애물은 리더 후보로만 쓴다(뒤차 역할은 못 함).
Car::LaneNeighbors Car::GatherLaneNeighbors(const std::vector<NearbyCar> &nearby, const shared_ptr<Road> &road,
                                            const Spline &refLine, float bandCenter, float bandHalfWidth, float egoS,
                                            float dirSign) const
{
    LaneNeighbors out;
    float bestLeaderGap = std::numeric_limits<float>::max();
    float bestFollowerGap = std::numeric_limits<float>::max();
    for (const NearbyCar &nb : nearby)
    {
        if (!m_SimState->IsCarAlive(nb.car))
            continue; // 캐시된 뒤 삭제된 차(0.2초 주기 갱신 사이 delete됨) -- dangling 포인터
        Car *other = nb.car;
        if (other->m_currentRoad != road)
            continue; // 아직 이 road에 안 들어온 차 -- 최근접점 스냅으로 여기 있는 것처럼 보여도 후보 아님
        float d = ComputeReferenceOffset(refLine, other->GetPosition());
        if (std::fabs(d - bandCenter) > bandHalfWidth)
            continue; // 이 밴드 차로가 아님

        // 마주 오는 차는 추종/양보 대상이 아니다 -- 차선변경 중 밴드에 걸쳐 있어도 리더로 잡히면 안 된다.
        float otherT = refLine.GetSplinePosition(other->GetPosition());
        if (refLine.GetDirectionAt(otherT).Dot(other->GetForwardAxis()) * dirSign <= 0.0f)
            continue;

        Mobil::VehicleState st;
        st.speed = other->GetSpeed();
        st.accel = other->GetAcceleration();
        st.position = TravelS(refLine, other->GetPosition(), dirSign);
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

    // 센서 사거리 안의 장애물을 리더 후보로 본다. 레이 히트(m_sensor.frontHitObstacles)로 잡으면 안 된다 --
    // 레이 길이가 속도에 비례해서 저속에선 10m도 안 나가고, 그러면 옆 차로 장애물이 MOBIL 눈에 안 보여
    // '텅 빈 차로'로 판정돼 그쪽으로 차선변경해버린다.
    // 장애물은 반응해서 감속/양보하는 주체가 아니라 뒤차(follower)는 못 되므로 리더 쪽에만 넣는다.
    for (const VehicleCollision::Obstacle &obstacle : m_sensorObstacles)
    {
        if (obstacle.isVehicle)
            continue; // 차는 위 nearby 루프가 이미 처리했다

        // 밴드 경계에 걸쳐 있기만 해도 그 차로를 막는다 -- 중심이 밴드 밖이라고 흘려보내면 안 된다.
        float d = 0.0f;
        float halfExtent = 0.0f;
        ProjectObstacle(refLine, obstacle, d, halfExtent);
        if (std::fabs(d - bandCenter) > bandHalfWidth + halfExtent)
            continue; // 이 밴드 차로가 아님

        float s = TravelS(refLine, obstacle.center, dirSign);
        if (s < egoS)
            continue; // 이미 지난 장애물은 리더 후보가 아님

        float gap = s - egoS;
        if (gap < bestLeaderGap)
        {
            bestLeaderGap = gap;
            out.leader.speed = obstacle.speed;
            out.leader.accel = 0.0f;
            out.leader.position = s;
            out.leader.length = obstacle.halfLength * 2.0f;
            out.hasLeader = true;
        }
    }

    return out;
}

// road의 targetOffset 근처 밴드로 (신규/강제) 진입해도 되는지 MOBIL 안전기준(유인기준 없이)으로 판정.
// 도로 밖에서 처음 들어갈 때(UpdateFindPath/EnsureRoamingPath)와 다음 road로 합류할 때(ShouldHoldForMerge)가 공유.
bool Car::IsSafeLaneEntry(const RoadRef &road, float targetOffset, const std::vector<NearbyCar> &nearby) const
{
    const LaneBand *target = RoadDataManager::Get().FindNearestBand(road.road, targetOffset, road.direction);
    if (target == nullptr)
        return true;

    const Spline &ref = road.road->GetReferenceLine();
    float dirSign = GetTravelSign(road.direction);
    float egoS = TravelS(ref, GetPosition(), dirSign);
    LaneNeighbors nbr = GatherLaneNeighbors(nearby, road.road, ref, target->centerOffset, target->width * 0.5f, egoS, dirSign);
    if (!nbr.hasFollower)
        return true; // 뒤에 아무도 없으면 항상 진입 가능

    Mobil::VehicleState ego;
    ego.speed = m_speed;
    ego.accel = m_acceleration;
    ego.position = egoS;
    ego.length = GetLength();

    Mobil::Params mobil{MOBIL_B_SAFE, m_personality.politeness, MOBIL_A_THR};
    return Mobil::IsSafeLaneChange(ego, &nbr.follower, mobil, BuildIdmParams(road.road));
}

// nextRoad로 합류(끼어들기)해도 되는지 판정. 경로를 따라가려면 반드시 넘어가야 하는 전이라 '이득'은 안 따지고
// (Mobil::IsSafeLaneChange), 뒤차에 b_safe보다 가혹한 감속을 강요하는지만 본다.
bool Car::ShouldHoldForMerge(const RoadRef &nextRoad) const
{
    if (nextRoad.road == nullptr)
        return false;
    bool hasLaneMapping = false;
    float targetOffset = RoadDataManager::Get().ResolveConnectingOffset(CurrentRoadRef(), nextRoad, m_currentOffset, &hasLaneMapping);
    if (!hasLaneMapping)
        targetOffset = ComputeReferenceOffset(nextRoad.road->GetReferenceLine(), GetPosition());
    return !IsSafeLaneEntry(nextRoad, targetOffset, m_lastNearbyCars);
}

bool Car::ShouldHoldForJunction(const RoadRef &nextRoad) const
{
    if (nextRoad.road == nullptr || m_currentRoad == nullptr || nextRoad.road->GetJunctionId() < 0 || m_SimState == nullptr)
        return false;
    return !m_SimState->IsJunctionAvailable(nextRoad.road->GetJunctionId(), m_currentRoad->GetId(), this);
}

bool Car::TryReserveJunction(const RoadRef &nextRoad)
{
    if (nextRoad.road == nullptr || m_currentRoad == nullptr || nextRoad.road->GetJunctionId() < 0 || m_SimState == nullptr)
        return true;

    int junctionId = nextRoad.road->GetJunctionId();
    if (m_reservedJunctionId >= 0 && m_reservedJunctionId != junctionId)
        ReleaseJunctionReservation();
    if (!m_SimState->TryReserveJunction(junctionId, m_currentRoad->GetId(), this))
        return false;

    m_reservedJunctionId = junctionId;
    return true;
}

void Car::ReleaseJunctionReservation()
{
    if (m_reservedJunctionId >= 0 && m_SimState != nullptr)
        m_SimState->ReleaseJunction(m_reservedJunctionId, this);
    m_reservedJunctionId = -1;
}

// BEHAVIOR_PLAN_INTERVAL(0.2초)마다 IDM(종방향)로 목표가속도를, MOBIL(횡방향)로 차선변경을 정한다.
void Car::UpdateDrivePlan()
{
    if (m_currentRoad == nullptr)
        return;
    if (m_currentTime - m_lastBehaviorPlanTime < BEHAVIOR_PLAN_INTERVAL)
        return;
    m_lastBehaviorPlanTime = m_currentTime;

    constexpr float MIN_LOOK_DISTANCE = 20.0f; // 정지 상태에서도 바로 앞 신호/제한속도는 보이게 하는 최소치
    m_lastIdmParams = BuildIdmParams(m_currentRoad);

    // 스캔 거리 = IDM 상호작용 거리 s*(정지 대상 최악치) + 다음 스캔까지 갈 거리. 더 늦게 보이면 쾌적감속 구간이 없어 최대제동으로 슬램한다.
    float interactGap = m_lastIdmParams.s0 + m_speed * m_lastIdmParams.T +
                        m_speed * m_speed / (2.0f * std::sqrt(std::max(0.0001f, m_lastIdmParams.a * m_lastIdmParams.b)));
    float lookDistance = std::max(MIN_LOOK_DISTANCE, interactGap + m_speed * BEHAVIOR_PLAN_INTERVAL);

    // ScanRoadSpeedConstraints가 내부에서 ShouldHoldForMerge(→ m_lastNearbyCars)를 참조하므로 먼저 갱신해둔다.
    m_lastNearbyCars = CollectNearbyCars();
    m_lastRoadSamples = ScanRoadSpeedConstraints(lookDistance);
    AppendCarConstraintSamples(m_lastRoadSamples, m_lastNearbyCars, lookDistance);
    AppendSensorConstraintSample(m_lastRoadSamples);
    m_planScanPosition = GetPosition(); // DriveControl이 매프레임 여기 대비 이동거리로 정적 제약 gap을 보정

    // ---- 횡방향: MOBIL 차선변경 판정 + Lerp ----
    // (실제 종방향 IDM 가속도는 DriveControl이 위 캐시로 매프레임 다시 계산한다.)
    // 회피/차선변경/정지대기 서브모드에서는 MOBIL을 끄고 그 서브모드가 정한 오프셋으로만 수렴한다.
    float laneCenter = m_currentOffset;
    float targetOffset;
    if (m_subMode == SubMode::D_Avoid || m_subMode == SubMode::D_LaneChange)
    {
        targetOffset = AvoidTargetOffset(); // PredictBodyContact가 굴려보는 목표와 반드시 같아야 한다
    }
    else if (m_subMode == SubMode::D_WaitObstacle)
    {
        targetOffset = laneCenter = CurrentLaneCenter();
    }
    else
    {
        targetOffset = ComputeLateralTarget(m_lastNearbyCars, m_lastIdmParams, &laneCenter);
        bool isLaneChange = std::fabs(targetOffset - laneCenter) > 0.01f;
        // 오프셋은 참조선 프레임, 레이 스캔은 차체 기준 -- 역주행이면 좌우가 뒤집힌다.
        bool sideBlocked = ((targetOffset - laneCenter) * TravelSign() > 0.0f) ? m_sensor.rightBlocked : m_sensor.leftBlocked;
        if (isLaneChange && sideBlocked)
        {
            isLaneChange = false;
            targetOffset = laneCenter;
        }

        if (isLaneChange)
        {
            m_maneuver.laneOffset = laneCenter;
            m_maneuver.laneChangeTarget = targetOffset;
        }
        SetSubMode(isLaneChange ? SubMode::D_LaneChange : SubMode::D_Normal);
    }
    m_currentOffset += (targetOffset - m_currentOffset) * m_personality.laneChangeLerpAlpha;
    m_currentSpline = RoadDataManager::Get().BuildOffsetSpline(m_currentRoad, m_currentOffset, m_travelDir);
    RebuildSplineRender();
}

float Car::CurrentLaneCenter() const
{
    const LaneBand *closest = RoadDataManager::Get().FindNearestBand(m_currentRoad, m_currentOffset, m_travelDir);
    return closest != nullptr ? closest->centerOffset : m_currentOffset;
}

// 경로상 다음 road의 진입 차로 제약을 MOBIL 유인 기준에 얹을 가중치로 환산한다.
Car::RouteLaneGoal Car::ComputeRouteLaneGoal() const
{
    // 경로상 필요한 차로로 붙기 위해 MOBIL 유인 기준에 얹는 가중치들.
    constexpr float ROUTE_BIAS_MAX = 3.0f;         // urgency=1일 때의 가중치(m/s^2). a_thr보다 훨씬 커야 이득 없이도 옮긴다.
    constexpr float ROUTE_LANE_CHANGE_TIME = 4.0f; // 차로 하나 옮기는 데 잡아두는 여유시간(s)
    constexpr float ROUTE_MIN_PLAN_SPEED = 2.0f;   // 남은 시간 환산 시 속도 하한(m/s) -- 정지 중에도 urgency가 발산하지 않게

    RouteLaneGoal goal;
    if (m_currentRoad == nullptr || m_pathIndex + 1 >= m_path.size())
        return goal;

    std::vector<const LaneBand *> entry = RoadDataManager::Get().GetEntryBands(CurrentRoadRef(), m_path[m_pathIndex + 1]);
    if (entry.empty())
        return goal; // 차로 제약이 없는 전이 -- MOBIL이 평소대로 판단하면 된다

    // 진입 가능한 밴드가 여럿이면 지금 오프셋에서 가장 가까운 쪽으로 -- 불필요하게 많이 안 움직인다.
    const LaneBand *target = entry.front();
    for (const LaneBand *band : entry)
        if (std::fabs(m_currentOffset - band->centerOffset) < std::fabs(m_currentOffset - target->centerOffset))
            target = band;
    goal.active = true;
    goal.targetOffset = target->centerOffset;

    // 건너야 할 차로 수를 밴드 목록상의 인덱스 차로 센다. 3차로 건너야 하면 1차로보다 훨씬 일찍 급해져야 한다.
    std::vector<const LaneBand *> bands = RoadDataManager::Get().GetDrivingBands(m_currentRoad, m_travelDir);
    int curIdx = -1;
    int targetIdx = -1;
    for (size_t i = 0; i < bands.size(); ++i)
    {
        if (curIdx < 0 || std::fabs(m_currentOffset - bands[i]->centerOffset) < std::fabs(m_currentOffset - bands[curIdx]->centerOffset))
            curIdx = static_cast<int>(i);
        if (bands[i] == target)
            targetIdx = static_cast<int>(i);
    }
    int lanesToCross = (curIdx >= 0 && targetIdx >= 0) ? ((targetIdx > curIdx) ? targetIdx - curIdx : curIdx - targetIdx) : 1;
    // 이미 목표 차로면 0이 되지만, 그때도 "벗어나면 한 번은 돌아와야 한다"는 값으로 패널티를 매겨야
    // 교차로 직전에 MOBIL이 이득만 보고 나를 옆 차로로 빼내는 걸 막는다.
    lanesToCross = std::max(lanesToCross, 1);

    // 거리가 아니라 남은 '시간'으로 잰다 -- 같은 거리라도 고속과 저속의 여유가 전혀 다르다.
    float t = m_currentSpline.GetSplinePosition(GetPosition());
    float remainDistance = m_currentSpline.GetLength() * (1.0f - t);
    float timeLeft = remainDistance / std::max(m_speed, ROUTE_MIN_PLAN_SPEED);
    float timeNeeded = lanesToCross * ROUTE_LANE_CHANGE_TIME;
    goal.urgency = std::clamp(timeNeeded / std::max(timeLeft, 0.0001f), 0.0f, 1.0f); // clamp가 없으면 교차로 코앞에서 발산해 진동한다
    goal.bias = ROUTE_BIAS_MAX * goal.urgency;
    return goal;
}

// 현재 밴드를 유지할지, MOBIL이 유인+안전 판정한 인접 밴드로 변경할지 정해 목표 횡오프셋을 돌려준다.
float Car::ComputeLateralTarget(const std::vector<NearbyCar> &nearbyCars, const IDM::Params &idm,
                                float *outLaneCenter) const
{
    constexpr float ROUTE_BSAFE_RELAX = 1.0f; // urgency=1일 때 b_safe를 이 비율만큼 더 완화(1.0=최대 2배)

    if (outLaneCenter != nullptr)
        *outLaneCenter = m_currentOffset; // 밴드 정보가 없는 경로에서도 항상 값이 채워져 있게

    // 진행방향 driving 밴드만 centerOffset 오름차순으로. 마주 오는 차로는 차선변경 후보가 아니다.
    std::vector<const LaneBand *> bands = RoadDataManager::Get().GetDrivingBands(m_currentRoad, m_travelDir);
    if (bands.empty())
        return m_currentOffset;

    // 현재 오프셋에 가장 가까운 밴드를 현재 차로로.
    size_t curIdx = 0;
    for (size_t i = 1; i < bands.size(); ++i)
        if (std::fabs(m_currentOffset - bands[i]->centerOffset) < std::fabs(m_currentOffset - bands[curIdx]->centerOffset))
            curIdx = i;
    const LaneBand &curBand = *bands[curIdx];
    if (outLaneCenter != nullptr)
        *outLaneCenter = curBand.centerOffset;

    const Spline &refLine = m_currentRoad->GetReferenceLine();
    float dirSign = TravelSign();
    float egoS = TravelS(refLine, GetPosition(), dirSign);

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

    LaneNeighbors cur = GatherLaneNeighbors(nearbyCars, m_currentRoad, refLine, curBand.centerOffset, curBand.width * 0.5f, egoS, dirSign);
    const Mobil::VehicleState *curLeader = cur.hasLeader ? &cur.leader : &farLeader;
    const Mobil::VehicleState *oldFollower = cur.hasFollower ? &cur.follower : nullptr;

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

    RouteLaneGoal goal = ComputeRouteLaneGoal();

    // 경로상 가야 할 차로가 있으면 그쪽부터, 없으면 이미 기울어 있는 쪽부터 본다. 첫 통과를 바로 채택하는
    // 구조라 순서를 안 맞추면 급한 상황에서 반대쪽이 먼저 통과해버린다.
    // bands/오프셋 모두 참조선 프레임이라 진행방향과 무관하게 비교하면 된다.
    int preferDir;
    if (goal.active && std::fabs(goal.targetOffset - curBand.centerOffset) > 0.01f)
        preferDir = (goal.targetOffset > curBand.centerOffset) ? 1 : -1;
    else
        preferDir = (m_currentOffset > curBand.centerOffset) ? 1 : -1;

    for (int di : {preferDir, -preferDir})
    {
        long adjIdx = static_cast<long>(curIdx) + di;
        if (adjIdx < 0 || adjIdx >= static_cast<long>(bands.size()))
            continue;

        const LaneBand &adjBand = *bands[adjIdx];
        // 목표 차로에 가까워지는 쪽이면 +bias, 멀어지는 쪽이면 -bias. 부호를 안 나누면 "급해질수록
        // 아무 쪽으로나 차선변경하기 쉬워지는" 반대 효과가 난다.
        Mobil::Params mobil{MOBIL_B_SAFE, m_personality.politeness, MOBIL_A_THR};
        float bias = 0.0f;
        if (goal.active)
        {
            bool towardGoal = std::fabs(adjBand.centerOffset - goal.targetOffset) <
                              std::fabs(curBand.centerOffset - goal.targetOffset);
            bias = towardGoal ? goal.bias : -goal.bias;
            // 안전 기준은 bias로 안 뚫리므로, 급할 때만 뒤차에 감수시킬 감속을 조금 키운다(최대 2배).
            // 완전히 풀면 뒤차에 급제동을 강요하고, 안 풀면 혼잡할 때 영영 못 들어가 재경로만 반복한다.
            if (towardGoal)
                mobil.b_safe *= 1.0f + ROUTE_BSAFE_RELAX * goal.urgency;
        }

        LaneNeighbors nbr = GatherLaneNeighbors(nearbyCars, m_currentRoad, refLine, adjBand.centerOffset, adjBand.width * 0.5f, egoS, dirSign);
        const Mobil::VehicleState &newLeader = nbr.hasLeader ? nbr.leader : farLeader;
        const Mobil::VehicleState *newFollower = nbr.hasFollower ? &nbr.follower : nullptr;
        if (Mobil::EvaluateLaneChange(ego, oldFollower, egoLeader, newLeader, newFollower, mobil, idm, bias))
            return adjBand.centerOffset;
    }
    return curBand.centerOffset; // 차선 유지
}

void Car::AppendSensorConstraintSample(std::vector<RoadSpeedSample> &samples) const
{
    // 레이에 잡힌 전방 장애물을 IDM 가상 리더로 넣어 감속 근거로 삼는다. 도로 데이터의 정적 장애물이나,
    // 차로 코리도 판정(AppendCarConstraintSamples)으로는 안 잡히는 갓 끼어든 차가 여기서 처리된다.
    // frontDistance는 앞범퍼 기준이라 차 길이를 뺄 필요가 없다.
    if (m_sensor.frontDistance >= 0.0f && !IsParallelClearSensorVehicle())
    {
        samples.push_back({m_sensor.frontHitPosition, m_sensor.frontDistance - MIN_SAFE_GAP, m_sensor.frontHitSpeed, nullptr, 0.0f, "sensorFront"});
    }

    // 차체 스윕이 예고한 접촉. 정지한 대상만 넣어 잰 거리이므로 speed 0(정지 제약)으로 건다.
    // bodyContactDistance는 '차체가 닿을 때까지 더 갈 수 있는 거리' 자체라 그대로 gap으로 쓸 수 있다.
    if (m_sensor.bodyContactDistance >= 0.0f)
        samples.push_back({GetPosition(), m_sensor.bodyContactDistance, 0.0f, nullptr, 0.0f, "bodySweep"});
}

#pragma endregion

#pragma region Avoid

bool Car::IsParallelClearSensorVehicle() const
{
    if (!m_sensor.hasFrontHitObstacle || !m_sensor.frontHitObstacle.isVehicle ||
        m_sensor.movingConflict || m_sensor.bodyContactDistance >= 0.0f)
        return false;

    const VehicleCollision::Obstacle &hit = m_sensor.frontHitObstacle;
    if (m_speed >= 0.1f && hit.speed <= AVOID_BLOCK_SPEED)
        return true; // The swept path does not contact this stopped vehicle.

    Vec3 forward = GetForwardAxis();
    Vec3 hitForward(cosf(hit.headingRad), 0.0f, sinf(hit.headingRad));
    if (std::fabs(forward.Dot(hitForward)) < PARALLEL_SENSOR_HEADING_COS)
        return false;

    Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());
    Vec3 hitRight(hitForward.GetZ(), 0.0f, -hitForward.GetX());
    float hitLateralHalfExtent = std::fabs(right.Dot(hitForward)) * hit.halfLength +
                                 std::fabs(right.Dot(hitRight)) * hit.halfWidth;
    float lateralClearance = std::fabs((hit.center - GetBodyCenter()).Dot(right)) -
                             GetHalfWidth() - hitLateralHalfExtent;
    return lateralClearance >= PARALLEL_SENSOR_LATERAL_CLEARANCE;
}
Vec3 Car::GetBodyCenter() const
{
    // 콜라이더 오프셋은 차체 로컬(z=전방, x=오른쪽) 기준. 회전은 Y축 요만 있으므로 전방/오른쪽 축으로만 합성하면 된다.
    Vec3 forward = GetForwardAxis();
    Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());
    return GetRigidbodyPosition() + forward * m_colliderOffset.z + right * m_colliderOffset.x;
}

std::vector<VehicleCollision::Obstacle> Car::CollectMapObstaclesInSensorRange() const
{
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
    return obstacles;
}

std::vector<VehicleCollision::Obstacle> Car::BuildSensorObstacles() const
{
    // 레이/OBB 판정 대상: 지도상의 정적 장애물 + 지금 주변에 있는 차들. 차 '목록'은 0.2초 주기 캐시
    // (m_lastNearbyCars)지만 위치/헤딩/속도는 매프레임 그 차에서 새로 읽으므로 기하는 항상 최신이다.
    std::vector<VehicleCollision::Obstacle> obstacles = CollectMapObstaclesInSensorRange();

    obstacles.reserve(obstacles.size() + m_lastNearbyCars.size());
    for (const NearbyCar &nearbyCar : m_lastNearbyCars)
    {
        if (!m_SimState->IsCarAlive(nearbyCar.car))
            continue; // 캐시된 뒤 삭제된 차(0.2초 주기 갱신 사이 delete됨) -- dangling 포인터
        const Car *other = nearbyCar.car;
        VehicleCollision::Obstacle obstacle;
        obstacle.center = other->GetBodyCenter();
        obstacle.halfLength = other->m_halfExtents.GetZ();
        obstacle.halfWidth = other->m_halfExtents.GetX();
        obstacle.headingRad = DirectionToAngleRad(other->GetForwardAxis());
        obstacle.speed = other->GetSpeed();
        obstacle.type = VehicleCollision::ObstacleType::Dynamic;
        obstacle.isVehicle = true;
        obstacle.yieldsToEgo = nearbyCar.yieldsToMe; // 교차 충돌 예측(PredictMovingConflict)이 우선권을 봐야 한다
        obstacles.push_back(obstacle);
    }
    return obstacles;
}

Car::SensorScan Car::ScanSensors(const std::vector<VehicleCollision::Obstacle> &obstacles) const
{
    // 정면 중앙 레이만 따로 더 멀리 본다. 여기 잡힌 장애물은 IDM 가상 리더 + MOBIL의 현재 차로 리더로
    // 들어가서, 코앞까지 가서 회피로 비트는 대신 한참 전에 정상 차선변경으로 빠져나가게 한다.
    // (회피 트리거 자체는 여전히 AVOID_FRONT_RAY_MAX 이내 히트만 쓴다 -- 60m 밖 장애물에 미리 몸을
    //  틀어버리면 안 되므로.)
    constexpr float AVOID_FRONT_RAY_FAR_TIME = 5.0f; // 이 시간만큼 앞을 미리 본다(s)
    constexpr float AVOID_FRONT_RAY_FAR_MAX = 60.0f; // 그 상한(m)
    // 대각/측면 레이 길이(m). "바로 옆 틈이 비었나"만 보는 값싼 사전 거부용이라 옆차로까지 닿게 잡지
    // 않는다 -- 옆차로 차와 부딪히는지는 SimulateAvoidPath의 OBB 판정이 어차피 본다. 길게 잡으면
    // 옆차로에 차가 있는 내내 sideNear가 서서 회피 오프셋에서 못 빠져나온다.
    constexpr float AVOID_SIDE_RAY_LENGTH = 2.5f;
    // 이 이상 꺾여 있어야 "그쪽으로 돌고 있다"고 보고 같은 쪽 대각선 히트를 막힘으로 승격시킨다.
    // 직진 중 미세한 조향 떨림에 대각선이 계속 막힘으로 잡히는 걸 막는 데드존.
    constexpr float AVOID_STEER_DEADZONE = ToRadians(3.0f);
    constexpr float AVOID_REAR_RAY_LENGTH = 6.0f; // 후방 레이 길이(m)

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
    Vec3 sideLeft = center - right * halfWidth;
    Vec3 sideRight = center + right * halfWidth;
    Vec3 rearCenter = center - forward * halfLength;
    Vec3 rearLeft = rearCenter - right * halfWidth;
    Vec3 rearRight = rearCenter + right * halfWidth;

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

        // d는 참조선 프레임이라 진행방향과 무관하게 참조선 우법선으로 그대로 밀면 된다.
        const Spline &nextLine = m_path[m_pathIndex + 1].road->GetReferenceLine();
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
        // {frontCenter, 0.0f, 0.0f}
        {frontCenter, 0.0f, farLength},
        {frontRight, 0.0f, farLength},
        {frontLeft, 0.0f, farLength},
        {frontRight, ToRadians(5.0f), farLength},
        {frontLeft, ToRadians(-5.0f), farLength},

        {frontRight, ToRadians(15.0f), farLength * 0.5f},
        {frontLeft, ToRadians(-15.0f), farLength * 0.5f},
        {frontRight, ToRadians(30.0f), farLength * 0.5f},
        {frontLeft, ToRadians(-30.0f), farLength * 0.5f},
        {frontRight, ToRadians(45.0f), farLength * 0.5f},
        {frontLeft, ToRadians(-45.0f), farLength * 0.5f},
    };
    for (const FrontRay &frontRay : frontRays)
    {
        auto [hit, distance] = cast(frontRay.origin, frontRay.rightAngle, frontRay.maxDistance);
        if (hit == nullptr)
            continue;

        // 히트가 정말 '내 진로 위'인지 거른다.
        Vec3 hitPosition = scan.rays.back().end;
        float offPath = pathDistance(hitPosition);
        if (offPath > corridorHalfWidth)
            continue; // 내 진로에서 완전히 벗어남(옆차로/노변)

        if (offPath <= brakeHalfWidth && (scan.frontDistance < 0.0f || distance < scan.frontDistance))
        {
            scan.frontDistance = distance;
            scan.frontHitPosition = hitPosition;
            // 스칼라 speed를 내 진행방향에 투영해 '멀어지는 성분'만 남긴다. 마주 오는 대상을 같이 도망가는
            // 리더로 넘기면 IDM이 감속하지 않는다(음수는 0으로 -- 정지한 리더가 가장 보수적).
            Vec3 hitDir(cosf(hit->headingRad), 0.0f, sinf(hit->headingRad));
            scan.frontHitSpeed = std::max(0.0f, forward.Dot(hitDir) * hit->speed);
            scan.frontHitObstacle = *hit;
            scan.hasFrontHitObstacle = true;
        }
        // 그중 거의 멈춰 있고 제동거리 안까지 들어온 것만 '피해 갈 대상'. 정상 주행 중인 앞차는 IDM
        // 추종이 처리하고, 멀리(farLength) 보이는 장애물은 아직 MOBIL 차선변경이 처리할 몫이다.
        if (hit->speed <= AVOID_BLOCK_SPEED && distance <= frontLength)
            scan.frontBlocked = true;
    }

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
        bool rearLateralHit = cast(rearCorner, sign * ToRadians(90.0f), AVOID_SIDE_RAY_LENGTH).first != nullptr;

        // 그쪽으로 '새로' 밀고 들어갈지(FindAvoidOffset)는 뒷범퍼 옆까지 다 봐야 한다.
        bool blocked = lateralHit || rearLateralHit || (turningThisWay && diagonalHit);
        if (side < 0)
            scan.leftBlocked = blocked;
        else
            scan.rightBlocked = blocked;
        // 반면 복귀 판정에는 뒤쪽 히트를 넣지 않는다 -- 방금 지나친 장애물이 뒷범퍼 옆에 남아 있는 동안
        // 계속 막히면 회피 오프셋에서 영영 못 돌아온다. 뒤차라면 알아서 감속한다.
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

    return scan;
}

// targetOffset으로 Lerp해 들어가는 궤적을 자전거 모델로 굴려, 차체(OBB)가 obstacles와 처음 겹치는
// 지점까지의 주행거리를 반환한다(maxDistance까지 안 겹치면 -1). 이 구간 동안 speed는 일정하다고 본다.
// 거리 기준으로 적분하므로(speed*dt = stepDistance) 저속에서도 스텝 수가 폭발하지 않는다.
float Car::SweepBodyPath(float targetOffset, const std::vector<VehicleCollision::Obstacle> &obstacles,
                         float speed, float maxDistance) const
{
    // 차체 스윕 적분 스텝 수(거리 기준으로 등분). 스텝 간격이 차 길이보다 작게 유지되므로 얇은 장애물을
    // 건너뛰지 않고, 저속에서도 스텝 수가 폭발하지 않는다.
    constexpr int AVOID_SWEEP_STEPS = 16;

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

// 회피가 지금 향하고 있는 목표 오프셋. 아무것도 진행 중이 아니면 지금 오프셋 그대로.
float Car::AvoidTargetOffset() const
{
    if (m_subMode == SubMode::D_Avoid)
        return m_maneuver.avoidOffset;
    if (m_subMode == SubMode::D_LaneChange)
        return m_maneuver.laneChangeTarget;
    return m_currentOffset;
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
    float targetOffset = AvoidTargetOffset();
    float horizon = std::clamp(m_speed * m_speed / (2.0f * m_maxBrake) + MIN_SAFE_GAP,
                               AVOID_FRONT_RAY_MIN, AVOID_FRONT_RAY_MAX);
    return SweepBodyPath(targetOffset, stationary, m_speed, horizon);
}

// 나와 장애물을 각각 등속으로 굴려 만나는 순간을 찾는다. 레이는 '지금' 진로 위인 것만 통과시키므로
// 비스듬히 들어오는 대상은 코앞에 올 때까지 안 잡히는데(그땐 이미 늦다), 여기서 미리 잡는다.
float Car::PredictMovingConflict(const std::vector<VehicleCollision::Obstacle> &obstacles,
                                 const VehicleCollision::Obstacle **outObstacle) const
{
    constexpr int CONFLICT_STEPS = 12;

    // 멈춰 있어도 "지금 출발하면"을 봐야 한다(SimulateAvoidPath와 같은 가정) -- 안 그러면 정지한 순간
    // 예측이 사라져서 대기가 풀리고, 출발하면 다시 잡히는 걸 반복한다.
    float speed = std::max(m_speed, AVOID_SIM_MIN_SPEED);
    float stepTime = AVOID_SIM_TIME / CONFLICT_STEPS;
    float myRadius = GetHalfWidth() + AVOID_CORRIDOR_MARGIN;

    // 내 미래 위치는 장애물과 무관하니 한 번만 뽑아둔다(GetLookaheadPoint는 스플라인 탐색이라 싸지 않다).
    Vec3 futurePath[CONFLICT_STEPS];
    for (int i = 0; i < CONFLICT_STEPS; ++i)
        futurePath[i] = m_currentSpline.GetLookaheadPoint(GetPosition(), speed * stepTime * static_cast<float>(i + 1));

    float best = -1.0f;
    for (const VehicleCollision::Obstacle &obstacle : obstacles)
    {
        // 멈춰 있는 것은 레이/차체 스윕(정지물)이 맡는다.
        if (obstacle.speed <= AVOID_BLOCK_SPEED)
            continue;

        Vec3 obstacleFwd(cosf(obstacle.headingRad), 0.0f, sinf(obstacle.headingRad));

        // 차는 원칙적으로 IDM/MOBIL이 맡지만, 내 경로를 '가로지르는' 차는 예외다 -- 레이 코리도는 지금
        // 진로 위인 것만 통과시키고 차체 스윕은 정지물만 보므로, 좌회전으로 90도 들어오는 차는 어느
        // 경로로도 안 잡혀 그대로 측면 충돌한다.
        if (obstacle.isVehicle)
        {
            if (GetForwardAxis().Dot(obstacleFwd) > SAME_DIRECTION_COS)
                continue; // 같은 방향으로 가는 차 -- 추종 관계라 IDM이 맡는다
            if (obstacle.yieldsToEgo)
                continue; // 내가 우선권 -- 저쪽이 선다(못 서고 붙으면 리더 gap 판정이 되살린다)
        }

        Vec3 obstacleRight(obstacleFwd.GetZ(), 0.0f, -obstacleFwd.GetX());
        Vec3 velocity = obstacleFwd * obstacle.speed;
        // 외접원 대신 원(내 차) 대 OBB(상대) 거리로 잰다 -- 차처럼 긴 대상은 외접원이 반지름을 2배 넘게
        // 부풀려서, 옆으로 안전하게 스쳐 지나갈 거리에서도 충돌로 잡혀 불필요하게 선다.
        for (int i = 0; i < CONFLICT_STEPS; ++i)
        {
            float elapsed = stepTime * static_cast<float>(i + 1);
            Vec3 delta = obstacle.center + velocity * elapsed - futurePath[i];
            // 축 투영은 XZ 평면이라 y가 떨어져 나간다 -- 고가도로 위/아래 대상은 따로 걸러야 한다.
            if (std::fabs(delta.GetY()) > VERTICAL_SEPARATION)
                continue;
            float along = std::max(0.0f, std::fabs(delta.Dot(obstacleFwd)) - obstacle.halfLength);
            float lateral = std::max(0.0f, std::fabs(delta.Dot(obstacleRight)) - obstacle.halfWidth);
            if (along * along + lateral * lateral > myRadius * myRadius)
                continue;

            float travel = speed * elapsed;
            if (best < 0.0f || travel < best)
            {
                best = travel;
                if (outObstacle != nullptr)
                    *outObstacle = &obstacle;
            }
            break; // 이 장애물에 대한 첫(가장 이른) 접촉만 쓴다
        }
    }
    return best;
}

bool Car::FindAvoidOffset(const SensorScan &scan, const std::vector<VehicleCollision::Obstacle> &obstacles,
                          float laneCenter, float &outOffset) const
{
    float laneWidth = RoadDataManager::ROAD_WIDTH;
    if (const LaneBand *band = RoadDataManager::Get().FindNearestBand(m_currentRoad, laneCenter, m_travelDir))
        laneWidth = band->width;

    // 도로 밖으로 나가면 안 된다: 양끝 밴드 가장자리에서 차체 반폭만큼 안쪽까지가 허용 범위.
    float minOffset = 0.0f;
    float maxOffset = 0.0f;
    ComputeDrivableRange(CurrentRoadRef(), minOffset, maxOffset);

    // side/오프셋은 참조선 프레임, 레이 스캔(scan)은 차체 기준 -- 역주행이면 둘의 좌우가 뒤집힌다.
    const float magnitudes[] = {laneWidth * 0.5f, laneWidth, laneWidth * 2.0f};
    float dirSign = TravelSign();
    bool leanPositive = m_currentOffset > laneCenter; // 이미 기울어 있는 쪽부터 후보로 본다
    for (float magnitude : magnitudes)
    {
        for (int side : {leanPositive ? 1 : -1, leanPositive ? -1 : 1})
        {
            if ((side * dirSign) > 0.0f ? scan.rightBlocked : scan.leftBlocked)
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

// 전방 최근접 히트를 무엇으로 볼지 한 번만 정한다. 차는 IDM/MOBIL이 이미 다루고, 장애물은 움직이는지에
// 따라 대응이 정반대(정지 vs 회피)라 여기서 갈라야 아래 로직이 단순해진다.
Car::ThreatKind Car::ClassifyFrontThreat() const
{
    // 이 속도 이상이면 '움직이는 장애물'(정지 원칙 대상). AVOID_BLOCK_SPEED와 이 값 사이는 판단 보류라
    // 정지/회피 어느 쪽으로도 안 튄다 -- 임계값 하나면 그 근처에서 분류가 매 프레임 뒤집힌다.
    constexpr float AVOID_DYNAMIC_SPEED = 1.5f;

    if (!m_sensor.hasFrontHitObstacle)
    {
        // 레이엔 안 잡혔지만 차체 스윕이 접촉을 예고한 경우(회전 중 바깥 꼭지점). 스윕 대상은 정지한
        // 것만 넣으므로 정적으로 본다.
        return (m_sensor.bodyContactDistance >= 0.0f) ? ThreatKind::Static : ThreatKind::None;
    }

    const VehicleCollision::Obstacle &hit = m_sensor.frontHitObstacle;
    if (hit.isVehicle)
        return ThreatKind::Vehicle; // 정지한 차(신호대기/정체)도 여기 -- 줄 서서 기다려야지 피해 가면 안 된다

    // 임계값 두 개 사이는 판단 보류. 하나로 자르면 그 근처 속도에서 정적/동적이 매 프레임 뒤집혀
    // 회피와 정지가 번갈아 걸린다. 보류 구간에서도 IDM은 frontDistance 리더로 계속 감속한다.
    if (hit.speed <= AVOID_BLOCK_SPEED)
        return ThreatKind::Static;
    if (hit.speed >= AVOID_DYNAMIC_SPEED)
        return m_wait.timedOut ? ThreatKind::Static : ThreatKind::Dynamic;
    return ThreatKind::None;
}

// 움직이는 장애물이 '내가 닿기 전에' 진로를 비우는가. 종방향 TTC가 아니라 횡방향 이탈 시각으로 잰다 --
// 가로지르는 대상에게 의미 있는 건 충돌까지 시간이 아니라 언제 코리도를 벗어나는가이기 때문.
bool Car::IsDynamicThreatClear() const
{
    constexpr float AVOID_TTC_MARGIN = 1.5f; // 동적 장애물이 진로를 비우는 시각보다 이만큼 더 늦게 닿아야 그냥 통과(s)

    if (m_sensor.movingConflict)
        return false; // 이미 '지나가면 부딪힌다'로 판정났다 -- 비켜줄지 따질 단계가 아니다

    const VehicleCollision::Obstacle &hit = m_sensor.frontHitObstacle;
    Vec3 forward = GetForwardAxis();
    Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());

    Vec3 velocity = Vec3(cosf(hit.headingRad), 0.0f, sinf(hit.headingRad)) * hit.speed;
    float lateralSpeed = velocity.Dot(right);
    if (std::fabs(lateralSpeed) < AVOID_BLOCK_SPEED * 0.5f)
        return false; // 내 진행방향과 나란히 간다 -- 비켜줄 기미가 없으니 선다

    float lateralOffset = (hit.center - GetPosition()).Dot(right);
    float corridorHalf = GetHalfWidth() + AVOID_CORRIDOR_MARGIN + hit.halfWidth;
    // 지금 가는 쪽 코리도 경계까지 남은 횡거리(이미 넘었으면 0).
    float clearDistance = (lateralSpeed > 0.0f) ? (corridorHalf - lateralOffset) : (lateralOffset + corridorHalf);
    float timeToClear = std::max(0.0f, clearDistance) / std::fabs(lateralSpeed);
    // 멈춰 있으면 "지금 출발하면"으로 재야 한다. 실제 속도로 재면 정지 중엔 도달시간이 무한이라 항상
    // 여유 있다고 나와서, 대기를 풀고 출발하자마자 다시 서는 진동이 생긴다(SimulateAvoidPath와 같은 가정).
    float timeToReach = m_sensor.frontDistance / std::max(m_speed, AVOID_SIM_MIN_SPEED);
    return timeToReach > timeToClear + AVOID_TTC_MARGIN;
}

// 그 차로 앞쪽이 비어 있는가. m_stationaryObstacles는 이미 센서 사거리로 잘려 있으므로 따로 룩어헤드를
// 두지 않는다. 장애물이 밴드에 걸치기만 해도 막힌 것으로 본다(ProjectObstacle의 횡방향 반폭 기준).
bool Car::IsBandClearAhead(float bandCenter, float bandHalfWidth) const
{
    const Spline &ref = m_currentRoad->GetReferenceLine();
    float dirSign = TravelSign();
    float egoS = TravelS(ref, GetPosition(), dirSign);

    for (const VehicleCollision::Obstacle &obstacle : m_stationaryObstacles)
    {
        float s = TravelS(ref, obstacle.center, dirSign);
        if (s < egoS)
            continue; // 이미 지나친 것은 이 차로를 막지 않는다

        float d = 0.0f;
        float halfExtent = 0.0f;
        ProjectObstacle(ref, obstacle, d, halfExtent);
        if (std::fabs(d - bandCenter) <= bandHalfWidth + halfExtent)
            return false;
    }
    return true;
}

// 그 차로에 들어가도 되는가: 뒤차(IsSafeLaneEntry) + 앞차(그 뒤에 붙었을 때 내가 밟을 감속)
bool Car::IsLaneEntryClear(float targetOffset) const
{
    const LaneBand *target = RoadDataManager::Get().FindNearestBand(m_currentRoad, targetOffset, m_travelDir);
    if (target == nullptr)
        return true;

    const Spline &ref = m_currentRoad->GetReferenceLine();
    float dirSign = TravelSign();
    float egoS = TravelS(ref, GetPosition(), dirSign);
    LaneNeighbors nbr = GatherLaneNeighbors(m_lastNearbyCars, m_currentRoad, ref, target->centerOffset, target->width * 0.5f, egoS, dirSign);

    if (nbr.hasLeader)
    {
        Mobil::VehicleState ego;
        ego.speed = m_speed;
        ego.accel = m_acceleration;
        ego.position = egoS;
        ego.length = GetLength();
        float accel = IDM::CalculateAcceleration(ego.speed, ego.accel, nbr.leader.speed, nbr.leader.accel,
                                                 Mobil::GetGap(ego, &nbr.leader), BuildIdmParams(m_currentRoad));
        if (accel < -MOBIL_B_SAFE)
            return false; // 그 차 뒤에 붙으려면 안전한계보다 세게 밟아야 한다
    }
    return IsSafeLaneEntry(CurrentRoadRef(), targetOffset, m_lastNearbyCars);
}

// 정지 대기 유지/해제. 해제는 "레이에서 사라짐"이 원칙이지만, 정지하면 전방 레이가 AVOID_FRONT_RAY_MIN까지
// 줄어 그 안에 멈춘 대상은 영영 안 사라진다 -- 타임아웃을 두고 정적 취급(회피 시도)으로 넘긴다.
void Car::UpdateWaitObstacle()
{
    constexpr float AVOID_WAIT_TIMEOUT = 5.0f; // 정지 대기가 이만큼 이어지면 '안 비켜준다'로 보고 정적 취급(회피 시도)

    m_wait.elapsed += m_deltaTime;
    m_speedCap = 0.0f; // 대기 중엔 종방향 상한 0 (IDM이 부드럽게 세운다)

    // 레이에서 사라졌거나(None), 비켜나는 중이거나(Dynamic+TTC 여유), 대상이 바뀌었으면(Static/Vehicle) 대기 해제.
    ThreatKind kind = ClassifyFrontThreat();
    bool stillWaiting = (kind == ThreatKind::Dynamic) && !IsDynamicThreatClear();
    if (!stillWaiting)
    {
        m_wait = WaitState{};
        m_speedCap = -1.0f;
        SetSubMode(SubMode::D_Normal); // 다음 프레임에 DecideAvoidance가 다시 판단한다
        DebugConsole::Log(GetName() + ": wait -> clear");
        return;
    }
    if (m_wait.elapsed >= AVOID_WAIT_TIMEOUT)
    {
        // 안 비켜준다. 이 대상이 사라질 때까지 정적 장애물로 보고 회피/차선변경 쪽으로 넘긴다.
        m_wait.elapsed = 0.0f;
        m_wait.timedOut = true;
        m_speedCap = -1.0f;
        SetSubMode(SubMode::D_Normal);
        DebugConsole::Log(GetName() + ": wait timeout -> treat as static");
    }
}

// 이번 프레임의 인지 결과를 한 번에 만든다: 판정 대상 목록 -> 레이 스캔 -> 레이가 못 보는 두 가지 보강.
void Car::UpdateSensors()
{
    m_sensorObstacles = BuildSensorObstacles();
    m_stationaryObstacles.clear();
    for (const VehicleCollision::Obstacle &obstacle : m_sensorObstacles)
        if (obstacle.speed <= AVOID_BLOCK_SPEED)
            m_stationaryObstacles.push_back(obstacle);

    m_sensor = ScanSensors(m_sensorObstacles);
    RebuildSensorRender();
    m_speedCap = -1.0f; // 매프레임 초기화 -- 아래 상태들이 필요할 때만 다시 건다
    if (!m_sensor.hasFrontHitObstacle)
        m_wait.timedOut = false; // 대상이 시야에서 사라졌다 -- 다음 장애물은 다시 정상 분류로

    // 레이 스캔이 놓치는 '회전 중 차체가 쓸고 가는 면적'을 OBB 스윕으로 보강한다. 접촉이 예고되면
    // 제동 근거(AppendSensorConstraintSample / DriveControl)이자 회피 트리거로 같이 쓴다.
    m_sensor.bodyContactDistance = PredictBodyContact(m_sensorObstacles);
    if (m_sensor.bodyContactDistance >= 0.0f)
        m_sensor.frontBlocked = true;

    // 비스듬히 들어오는 동적 장애물은 레이 코리도 필터를 통과하지 못해 frontDistance에 안 잡힌다.
    // 예측 충돌이 더 가까우면 그걸 전방 위협으로 승격시킨다 -- 그래야 IDM이 세울 근거가 생긴다.
    const VehicleCollision::Obstacle *conflictObstacle = nullptr;
    float conflictDistance = PredictMovingConflict(m_sensorObstacles, &conflictObstacle);
    if (conflictObstacle != nullptr && (m_sensor.frontDistance < 0.0f || conflictDistance < m_sensor.frontDistance))
    {
        m_sensor.frontDistance = conflictDistance;
        m_sensor.frontHitPosition = conflictObstacle->center;
        m_sensor.frontHitSpeed = 0.0f; // 가로지르는 대상 뒤를 따라갈 수는 없다 -- 정지 제약으로 건다
        m_sensor.frontHitObstacle = *conflictObstacle;
        m_sensor.hasFrontHitObstacle = true;
        m_sensor.movingConflict = true;
    }

    if (IsParallelClearSensorVehicle())
        m_sensor.frontBlocked = false;
}

// 실제 물리 충돌은 레이가 못 본 각도(대각선 꼭짓점 등)로 박은 경우까지 잡는 최후의 안전망이다.
// 레이 판정과 무관하게 여기서 끊어주지 않으면, 계획상으론 아직 '회피 진행 중'이라 계속 밀어붙인다.
bool Car::HandleContactPending()
{
    if (!m_contactPending)
        return false;

    m_contactPending = false;
    if (m_stuck.backingUp)
    {
        // 후진하다 뒤를 박았다. ReverseSegment는 속도로 진행거리를 재는데 충돌로 속도가 0이 되므로,
        // 여기서 안 끊으면 진행거리가 영영 안 늘어 후진이 끝나지 않는다.
        DebugConsole::Log(GetName() + ": backup hit something -> abort backup");
        m_stuck.backingUp = false;
        BeginSegment(std::make_unique<SplineFollowSegment>());
        return true;
    }
    if (m_subMode == SubMode::D_Avoid)
    {
        DebugConsole::Log(GetName() + ": avoid contact -> stuck");
        HandleAvoidStuck();
        return true;
    }
    return false; // 회피/후진 중이 아니면 뒷수습할 게 없다
}

// 후진 매뉴버 중에는 아무 판단도 하지 않는다. 끝나면 무조건 정속주행 계획으로 복귀 -- 딱 한 번만
// 물러나 보고, 그래도 막혀 있으면 HandleAvoidStuck이 backupAttempted를 보고 더 후진하지 않는다.
bool Car::UpdateBackupState()
{
    if (!m_stuck.backingUp)
        return false;
    if (!m_vehicleController.IsFinished())
        return true;

    m_stuck.backingUp = false;
    BeginSegment(std::make_unique<SplineFollowSegment>());
    return true;
}

// D_Avoid: 산출된 오프셋을 유지하다가 레이가 깨끗해지면 원래 차로로 복귀한다.
void Car::UpdateAvoid()
{
    constexpr float AVOID_REPLAN_INTERVAL = 0.5f; // 회피 중 오프셋 재탐색 최소 간격(s) -- 좌/우 진동 방지

    m_speedCap = AVOID_LOW_SPEED; // 회피 궤적은 조향 여유가 필요하다 -- 최대 조향 가능한 속도까지 낮춘다

    // 재계획: 기본적으로는 산출된 오프셋으로 Lerp만 하지만, 그 계획이 실패했다는 신호가 오면 다시 찾는다.
    //  - 내가 '밀고 들어가는 쪽'이 막혔다: 비우려던 공간에 뭔가 들어왔다.
    //    반대쪽 히트는 지금 지나치는 중인 장애물이라 정상이므로 트리거로 쓰면 안 된다 -- 그걸 쓰면
    //    회피가 성공하는 도중에 매번 스스로 취소해버린다.
    //  - 오프셋에 다 도착했는데 아직도 정면이 막혔다: 이 정도 횡이동으로는 못 피한다(더 크게 잡아야 한다).
    if (m_currentTime - m_maneuver.lastPlanTime >= AVOID_REPLAN_INTERVAL)
    {
        // 오프셋은 참조선 프레임, 레이 스캔은 차체 기준 -- 역주행이면 좌우가 뒤집힌다.
        bool towardRight = (m_maneuver.avoidOffset - m_maneuver.laneOffset) * TravelSign() > 0.0f;
        bool shiftSideBlocked = towardRight ? m_sensor.rightBlocked : m_sensor.leftBlocked;
        bool arrivedButBlocked = m_sensor.frontBlocked &&
                                 std::fabs(m_currentOffset - m_maneuver.avoidOffset) < AVOID_RETURN_TOLERANCE;
        if (shiftSideBlocked || arrivedButBlocked)
        {
            m_maneuver.lastPlanTime = m_currentTime;
            float replanOffset = 0.0f;
            if (FindAvoidOffset(m_sensor, m_sensorObstacles, m_maneuver.laneOffset, replanOffset))
            {
                if (std::fabs(replanOffset - m_maneuver.avoidOffset) > 0.01f)
                    DebugConsole::Log(GetName() + ": avoid replan d " + ToString(m_maneuver.avoidOffset) +
                                      " -> " + ToString(replanOffset));
                m_maneuver.avoidOffset = replanOffset;
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

    // 종료 기준은 '앞이 막혔는가'(frontBlocked)지 '앞에 뭐라도 보이는가'(anyFrontHit)가 아니다.
    // 정면 레이는 최대 60m를 보므로, 후자로 잡으면 앞에 정상 주행 중인 차만 있어도 회피 오프셋에서
    // 영영 못 나온다. 지금 옆에 붙어 있는 것은 sideNear가 따로 본다.
    bool clear = !m_sensor.frontBlocked && !m_sensor.sideNear;
    m_maneuver.clearTimer = clear ? m_maneuver.clearTimer + m_deltaTime : 0.0f;

    if (m_maneuver.clearTimer >= AVOID_CLEAR_DELAY)
    {
        DebugConsole::Log(GetName() + ": avoid -> clear, resume normal at d " + ToString(m_currentOffset));
        m_avoidTrigger = AvoidTriggerState{};
        m_stuck = StuckState{};
        m_maneuver = ManeuverState{};
        SetSubMode(SubMode::D_Normal);
    }
}

// D_LaneChange: 목표 차로에 닿으면 끝(아직 막혀 있으면 다음 판단에서 다시 트리거).
// MOBIL 자발적 차선변경만 여기로 들어온다(장애물 회피는 D_Avoid) -- 완만한 Lerp라 저속 강제가 필요 없다.
// 급조향이 실제로 필요해지면 DriveControl의 steerSpeedCap이 알아서 감속시킨다.
void Car::UpdateLaneChange()
{
    bool arrived = std::fabs(m_currentOffset - m_maneuver.laneChangeTarget) < AVOID_RETURN_TOLERANCE;
    // 아직 원래 차로 쪽에 더 가까울 때만 취소
    bool canAbort = std::fabs(m_currentOffset - m_maneuver.laneOffset) <
                    std::fabs(m_currentOffset - m_maneuver.laneChangeTarget);
    // 목표 차로가 도중에 막히면(장애물이든 차든) 취소
    bool blocked = canAbort &&
                   (!IsBandClearAhead(m_maneuver.laneChangeTarget, GetHalfWidth() + AVOID_CORRIDOR_MARGIN) ||
                    !IsLaneEntryClear(m_maneuver.laneChangeTarget));
    if (arrived || blocked)
    {
        if (!arrived)
            DebugConsole::Log(GetName() + ": lane change blocked -> cancel");
        m_avoidTrigger = AvoidTriggerState{};
        m_stuck = StuckState{};
        m_maneuver = ManeuverState{};
        m_speedCap = -1.0f;
        SetSubMode(SubMode::D_Normal);
    }
}

// 진행 중인 매뉴버가 없을 때: 전방 최근접 위협이 무엇인지로 다음 행동을 고른다.
void Car::DecideAvoidance()
{
    constexpr float AVOID_TRIGGER_DELAY = 0.6f; // 전방이 이만큼 계속 막혀 있어야 회피 시작(s) -- 순간 오검출로 안 흔들리게

    ThreatKind threat = ClassifyFrontThreat();

    if (threat == ThreatKind::None)
    {
        constexpr float JUNCTION_LEADER_AVOID_LOOK_DISTANCE = 12.0f;
        constexpr float JUNCTION_LEADER_RIGHT_SHIFT = 0.75f;
        bool stoppedJunctionLeader = false;
        if (m_currentRoad != nullptr && m_pathIndex + 1 < m_path.size() &&
            m_path[m_pathIndex + 1].road != nullptr && m_path[m_pathIndex + 1].road->GetJunctionId() >= 0)
        {
            for (const RoadSpeedSample &sample : m_lastRoadSamples)
            {
                if (sample.leader == nullptr || !m_SimState->IsCarAlive(sample.leader))
                    continue;
                Car *leader = sample.leader;
                if (leader->m_currentRoad == m_currentRoad || leader->GetSpeed() > AVOID_BLOCK_SPEED)
                    continue; // Keep normal same-road queues in the IDM following path.
                if (sample.distance > JUNCTION_LEADER_AVOID_LOOK_DISTANCE)
                    continue;
                stoppedJunctionLeader = true;
                break;
            }
        }
        if (stoppedJunctionLeader)
        {
            float laneCenter = CurrentLaneCenter();
            float minOffset = 0.0f;
            float maxOffset = 0.0f;
            ComputeDrivableRange(CurrentRoadRef(), minOffset, maxOffset);
            float rightOffset = std::clamp(laneCenter + TravelSign() * JUNCTION_LEADER_RIGHT_SHIFT, minOffset, maxOffset);
            if (std::fabs(rightOffset - laneCenter) >= AVOID_MIN_SHIFT && SimulateAvoidPath(rightOffset, m_sensorObstacles))
            {
                m_maneuver.laneOffset = laneCenter;
                m_maneuver.avoidOffset = rightOffset;
                m_maneuver.lastPlanTime = m_currentTime;
                m_maneuver.clearTimer = -AVOID_CLEAR_DELAY;
                m_speedCap = AVOID_LOW_SPEED;
                SetSubMode(SubMode::D_Avoid);
                DebugConsole::Log(GetName() + ": junction leader -> right offset " + ToString(rightOffset));
                return;
            }
        }
        m_avoidTrigger.staticBlockTimer = 0.0f; // 트리거는 '같은 대상이 계속' 잡혀야 성립한다
        m_avoidTrigger.deadlockTimer = 0.0f;
        m_stuck.stuck = false;
        m_stuck.backupAttempted = false;
        return;
    }

    if (threat == ThreatKind::Dynamic)
    {
        m_avoidTrigger.staticBlockTimer = 0.0f;
        m_avoidTrigger.deadlockTimer = 0.0f;
        // 원칙은 정지. 판단하는 동안 조향 여유가 남게 최저속까지 먼저 줄인다.
        m_speedCap = AVOID_LOW_SPEED;
        if (IsDynamicThreatClear())
            return; // 내가 닿기 전에 비켜난다 -- 그냥 지나간다(속도 상한은 이번 프레임만 유지)

        m_wait.elapsed = 0.0f;
        m_speedCap = 0.0f;
        SetSubMode(SubMode::D_WaitObstacle);
        DebugConsole::Log(GetName() + ": dynamic obstacle -> wait");
        return;
    }

    if (threat == ThreatKind::Vehicle)
    {
        if (IsParallelClearSensorVehicle())
        {
            m_avoidTrigger.staticBlockTimer = 0.0f;
            m_avoidTrigger.deadlockTimer = 0.0f;
            m_stuck.stuck = false;
            m_stuck.backupAttempted = false;
            return;
        }
        // 정상 주행하는 앞차와 정체/신호대기 줄은 IDM(추종)과 MOBIL(차선변경)이 맡는다 -- 줄을 회피로
        // 빠져나가면 안 된다. 다만 '내 진행방향과 어긋난 채 멈춰서 길을 막은' 차는 줄이 아니라 데드락이다:
        // 교차로 안에는 밴드가 없어 MOBIL이 돌지 않고, D_WaitObstacle은 비차량 전용이라 타임아웃도 없어
        // 양쪽 다 영구 정지한다. 이 경우만 아래 오프셋 회피로 넘긴다.
        Vec3 hitDir(cosf(m_sensor.frontHitObstacle.headingRad), 0.0f, sinf(m_sensor.frontHitObstacle.headingRad));
        bool crossing = GetForwardAxis().Dot(hitDir) <= SAME_DIRECTION_COS;
        bool mustYield = !m_sensor.frontHitObstacle.yieldsToEgo;
        bool deadlocked = crossing && mustYield && m_sensor.frontHitObstacle.speed <= AVOID_BLOCK_SPEED &&
                          m_speed <= HORN_STOP_SPEED && !KnowsRedSignalAhead();
        m_avoidTrigger.deadlockTimer = deadlocked ? m_avoidTrigger.deadlockTimer + m_deltaTime : 0.0f;
        if (m_avoidTrigger.deadlockTimer >= INTERSECTION_YIELD_TIMEOUT)
        {
            DebugConsole::Log(GetName() + ": yielding vehicle blocked -> clear by backing up");
            HandleAvoidStuck();
        }
        else
        {
            m_stuck.stuck = false;
            m_stuck.backupAttempted = false;
        }
        // 성공하면 아래 "avoid d ..." 로그가, 실패하면 HandleAvoidStuck이 알아서 남긴다.
        // Vehicle deadlocks do not fall through to the static-obstacle offset avoidance path.
        return;
    }
    else
    {
        // ---- 정적 장애물 ----
        m_avoidTrigger.staticBlockTimer = m_sensor.frontBlocked ? m_avoidTrigger.staticBlockTimer + m_deltaTime : 0.0f;
        if (m_avoidTrigger.staticBlockTimer < AVOID_TRIGGER_DELAY)
        {
            m_stuck.stuck = false;
            if (!m_sensor.frontBlocked)
                m_stuck.backupAttempted = false; // 전방이 실제로 비었다 -- 새 막힘이면 다시 시도해도 된다
            return;
        }
    }

    // 정적 장애물도 GatherLaneNeighbors가 리더 후보로 잡아주므로, 차선변경 자체는 매 계획 주기
    // ComputeLateralTarget/EvaluateLaneChange(MOBIL)이 이미 시도해봤다 -- 막힌 채로 여기까지 왔다는 건
    // 갈 수 있는 인접 차로가 없거나 안전하지 않다고 MOBIL이 이미 판단했다는 뜻이다. 남은 수단은 차로
    // 사이를 걸치는 부분 오프셋 회피뿐.
    float laneCenter = CurrentLaneCenter();
    float avoidOffset = 0.0f;
    if (FindAvoidOffset(m_sensor, m_sensorObstacles, laneCenter, avoidOffset))
    {
        m_maneuver.laneOffset = laneCenter;
        m_maneuver.avoidOffset = avoidOffset;
        m_maneuver.lastPlanTime = m_currentTime;
        m_speedCap = AVOID_LOW_SPEED;
        SetSubMode(SubMode::D_Avoid);
        DebugConsole::Log(GetName() + ": avoid d " + ToString(laneCenter) + " -> " + ToString(avoidOffset));
        return;
    }

    // 좌우 어느 쪽으로도 못 피한다.
    HandleAvoidStuck();
}

// 회피가 막혔을 때: 뒤가 비어 있으면 차 길이 절반만큼 딱 한 번 물러나 본다. 이미 한 번 시도했는데도
// 여전히 막혀 있으면 더 물러나지 않고 그 자리에 정지 유지(경적은 UpdateHorn이 알아서 울린다) --
// 반복 후진 대신 이번 스턱 사이클에서 한 번만 시도해 무한정 뒤로 빠지는 걸 막는다.
void Car::HandleAvoidStuck()
{
    // 후진해도 되는지 볼 때 물러날 거리 뒤로 더 남아 있어야 하는 여유(m). 저속 직선 후진이라 주행 중
    // 표준 간격(MIN_SAFE_GAP)만큼 잡을 필요는 없다 -- 너무 크면 물러날 자리가 있어도 스턱에서 못 벗어난다.
    constexpr float AVOID_BACKUP_CLEARANCE = 1.0f;

    m_stuck.stuck = true;
    if (m_stuck.backingUp || m_stuck.backupAttempted)
        return;

    float backupDistance = GetLength() * 0.5f;
    bool rearClear = m_sensor.rearDistance < 0.0f || m_sensor.rearDistance > backupDistance + AVOID_BACKUP_CLEARANCE;
    if (m_speed > HORN_STOP_SPEED || !rearClear)
        return; // 아직 굴러가는 중이거나 뒤가 막혔다 -- 정지 유지(IDM/비상제동이 세운다)

    m_stuck.backingUp = true;
    m_stuck.backupAttempted = true;
    BeginSegment(std::make_unique<ReverseSegment>(backupDistance));
    DebugConsole::Log(GetName() + ": avoid blocked -> back up once (" + ToString(backupDistance) + "m)");
}

#pragma endregion
