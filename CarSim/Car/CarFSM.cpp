#include "Car.h"
#include "VehicleSegment.h"
#include "Utill/DebugConsole.h"
#include "Nav/ReedsShepp.h"
#include "Nav/HybridAStar.h"
#include "Nav/Mobil.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace
{
    std::vector<std::unique_ptr<VehicleSegment>> BuildParkSegments(const ReedsShepp::Path &path, const Vec3 &startPos,
                                                                   float startAngleRad, float turningRadius)
    {
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        std::vector<ReedsShepp::Leg> legs = ReedsShepp::SampleLegs(path, startPos, startAngleRad, turningRadius);
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

    // ReedsShepp이 쓰는 각도 규약(atan2(z, x), radian)으로 방향 벡터를 변환.
    float DirectionToAngleRad(const Vec3 &direction)
    {
        return atan2f(direction.GetZ(), direction.GetX());
    }
}

void Car::UpdateMode()
{
    const char *reason = "";
    State next = DecideNextMode(&reason);
    if (next != m_mode)
    {
        DebugConsole::Log(std::string(StateToString(m_mode)) + " -> " + StateToString(next) + " (" + reason + ")");
        OnModeExit(m_mode);
        OnModeEnter(next, m_mode);
        m_mode = next;
    }
}

Car::State Car::DecideNextMode(const char **reason) const
{
    if (m_mode == State::Park)
    {
        // 주차 시퀀스가 진행 중이면(정지 대기 m_parkPlanPending, 다단계 사이 idle 포함) Park를 유지한다.
        // m_parkSequenceActive가 없으면 leg1↔leg2 사이의 잠깐 finished 프레임에 Drive로 새어나간다.
        if (m_parkPlanPending || m_parkSequenceActive || !m_vehicleController.IsFinished())
        {
            *reason = "parking in progress";
            return State::Park;
        }
    }

    if (m_mode == State::Drive && m_subMode == SubState::D_Avoid && !m_vehicleController.IsFinished())
    {
        *reason = "avoid maneuver in progress";
        return State::Drive;
    }

    constexpr float ARRIVE_DISTANCE = 5.0f;
    bool arrived = m_destLane != nullptr && (m_destLane->GetEndPoint() - GetPosition()).Length() < ARRIVE_DISTANCE;

    if (arrived && GetParkTargetNode() != nullptr)
    {
        *reason = "arrived at destination with park target";
        return State::Park;
    }

    if (m_destLane == nullptr || arrived)
    {
        *reason = m_destLane == nullptr ? "no destination lane" : "arrived at destination";
        return State::Stop;
    }

    if (m_parkSpot != nullptr)
    {
        if (m_currentLane == nullptr)
        {
            *reason = "park spot reserved, no current lane (exiting park)";
            return State::Park;
        }

        // 입차: 목적지 레인(주차장 진입 레인)의 끝에 다다르면 RS로 주차칸까지 마무리한다.
        constexpr float PARK_TRIGGER_DISTANCE = 8.0f;
        bool nearFinalLane = m_currentLane == m_destLane &&
                             (m_currentLane->GetEndPoint() - GetPosition()).Length() < PARK_TRIGGER_DISTANCE;
        if (nearFinalLane)
        {
            *reason = "near end of final lane, entering park spot";
            return State::Park;
        }
    }

    *reason = "normal driving";
    return State::Drive;
}

void Car::OnModeEnter(State mode, State previous)
{
    if (mode == State::Drive)
    {
        m_subMode = SubState::D_Normal; // 서브상태는 항상 일반주행에서 새로 시작
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        // Park에서 넘어왔다는 건 출차가 막 끝났다는 뜻(입차 완료는 Stop으로 감) — RS 매뉴버로
        // 꺾여있던 조향을 중앙으로 되돌린 뒤 정속주행을 시작한다.
        if (previous == State::Park)
            segments.push_back(std::make_unique<CenterSteerSegment>());
        segments.push_back(std::make_unique<SplineFollowSegment>());
        m_vehicleController.BeginPlan(std::move(segments));
    }
    else if (mode == State::Park)
    {
        // 도착 즉시 RS를 계산하지 않고, 완전히 멈출 때까지 기다린다 (UpdatePark에서 처리).
        m_parkPlanPending = true;
        m_parkSequenceActive = true; // 주차 시퀀스 시작 — 완료(UpdatePark)까지 Park 유지.
        DebugConsole::Log("Park plan pending: waiting for full stop before planning");
    }
    else if (mode == State::Stop)
    {
        // 입차 완료 후 Stop으로 오는 경우도 RS 매뉴버로 꺾여있던 조향을 중앙으로 되돌린다.
        if (previous == State::Park)
        {
            std::vector<std::unique_ptr<VehicleSegment>> segments;
            segments.push_back(std::make_unique<CenterSteerSegment>());
            m_vehicleController.BeginPlan(std::move(segments));
        }
    }
}

void Car::OnModeExit(State mode)
{
    if (!m_vehicleController.IsFinished())
        m_vehicleController.Abort();
}

void Car::BeginParkPlan()
{
    // 아직 예약 전이고(입차 대기 중) 목표 Park 노드가 있으면, 도착한 지금 실제로 예약을 시도한다.
    // parkNodeId는 입차 경로탐색이 실패했을 때 같은 Park 노드의 다른 빈 자리를 다시 예약하는 데도 쓰이므로,
    // m_pendingParkNode를 비운 뒤에도 계속 참조할 수 있게 별도로 들고 있는다.
    int parkNodeId = -1;
    if (m_parkSpot == nullptr && m_pendingParkNode != nullptr)
    {
        parkNodeId = m_pendingParkNode->id;
        m_parkSpot = m_RoadDataManager->TryReserveParkSpot(parkNodeId);
        m_pendingParkNode = nullptr;
        if (m_parkSpot == nullptr)
        {
            // 빈 자리 없음 — 목적지를 포기한다. m_isExitingPark는 건드리지 않는다(스테일 값일 수
            // 있음) — DecideNextMode/UpdatePark 양쪽 다 m_parkSpot==nullptr이면 이 플래그를
            // 안 보게 돼 있어서, destLane==nullptr만으로 다음 프레임에 Stop에 안착한다.
            DebugConsole::Log("Park spot reservation failed for node " + std::to_string(parkNodeId) +
                              ": no available ParkSpot child, abandoning destination");
            m_parkSequenceActive = false; // 시퀀스 취소 — Park에 갇히지 않도록.
            m_destLane = nullptr;
            return;
        }
        m_parkNodeId = parkNodeId;  // 재시도(다른 빈 자리)용으로 Park 노드 id 보관
        m_triedParkSpotIds.clear(); // 새 입차 시퀀스 — 시도 목록 초기화
    }

    if (m_parkSpot == nullptr)
        return;

    // 호출 시점에 currentLane 유무로 방향을 판정 — 출차 분기에서 currentLane을 채우므로
    // 반드시 그 전에 캡처해둔다.
    m_isExitingPark = (m_currentLane == nullptr);
    m_subMode = m_isExitingPark ? SubState::P_IN : SubState::P_OUT; // fsm.txt 서브상태: 출차/주차

    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);
    Vec3 startPos = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());
    HybridAStar::VehicleShape shape = BuildVehicleShape();
    const std::vector<HybridAStar::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();

    if (m_isExitingPark)
    {
        shared_ptr<Lane> closestLane = m_RoadDataManager->GetClosestLane(startPos);

        float splinePos = closestLane->GetSpline().GetSplinePosition(startPos);
        Vec3 closestPos = closestLane->GetSpline().GetPositionAt(splinePos);
        Vec3 closestDir = closestLane->GetSpline().GetDirectionAt(splinePos);
        SetCurrentLane(closestLane);

        // 이미 레인 진행 방향과 90도 이내로 정렬돼 있으면 RS 출차 매뉴버 없이 바로 주행으로 넘어간다.
        constexpr float EXIT_HEADING_ALIGN_ANGLE = ToRadians(90.0f);
        float headingDot = std::clamp(GetForwardAxis().Dot(closestDir), -1.0f, 1.0f);
        if (std::acos(headingDot) <= EXIT_HEADING_ALIGN_ANGLE)
        {
            m_vehicleController.BeginPlan({});
            return;
        }

        // 출차: 레인 접선에 수직(90도)으로 진입점을 바라보는 위치를 target으로 잡는다.
        // startPos가 있는 쪽의 법선 방향으로, startPos만큼 떨어져 있되 최소 6만큼은 떨어뜨린다.
        constexpr float MIN_EXIT_NORMAL_DISTANCE = 6.0f;
        Vec3 normalDir = Vec3(-closestDir.GetZ(), 0.0f, closestDir.GetX()).Normalized();
        float lateralOffset = (startPos - closestPos).Dot(normalDir);
        if (lateralOffset < 0.0f)
            normalDir = normalDir * -1.0f;
        float distance = std::max(std::fabs(lateralOffset), MIN_EXIT_NORMAL_DISTANCE);

        Vec3 targetPos = closestPos + normalDir * distance;
        float targetAngleRad = DirectionToAngleRad(closestPos - targetPos);

        bool foundPath = false;
        ReedsShepp::Path path = HybridAStar::FindPath(startPos, startAngleRad, targetPos, targetAngleRad, obstacles, shape, foundPath);
        if (!foundPath)
        {
            // 출차 실패는 이미 차가 그 자리를 점유 중이므로 예약을 풀지 않는다.
            DebugConsole::Log("BeginParkPlan: HybridA* failed to find an exit path, abandoning this park attempt");
            m_destLane = nullptr;
            return;
        }
        m_vehicleController.BeginPlan(BuildParkSegments(path, startPos, startAngleRad, turningRadius));
        RebuildParkDebugRender(path, startPos, startAngleRad, turningRadius, targetPos, targetAngleRad);
        return;
    }

    // 입차 (장애물 회피, Hybrid A* 2단계): 예약한 스팟으로 진입을 시도하고, 못 하면 같은 Park의 다른 빈
    // 자리로 넘어가며 모두 시도한다. 각 자리는 leg 1(-> 스팟 앞 P) → leg 2(P -> 스팟)로 나눠 탐색한다
    // (먼 거리 한 방보다 짧게 나뉨). 주차레인은 같은 Park 안에서만 잇는 별도 망.
    if (!BeginParkEnterOrRetry())
    {
        DebugConsole::Log("BeginParkPlan: no reachable ParkSpot for node " + std::to_string(parkNodeId) +
                          ", abandoning destination");
        m_destLane = nullptr;
        m_parkSequenceActive = false;
    }
}

// m_parkSpot로의 입차 시작 계획: 주차레인이 있으면 leg 1(-> 스팟 앞 P), 없으면 스팟으로 직접(leg 2 없음).
// 계획을 시작했으면 true(+ m_parkGoingToSpot 설정), 이 자리로는 경로를 못 찾으면 false.
bool Car::PlanEnterForCurrentSpot()
{
    const std::vector<shared_ptr<Lane>> *parkingLanes =
        (m_parkNodeId >= 0) ? m_RoadDataManager->GetParkingLanes(m_parkNodeId) : nullptr;

    if (parkingLanes != nullptr && !parkingLanes->empty())
    {
        // P: 이 Park의 주차레인들 중 스팟에 가장 가까운 스플라인 점에서, 그 스플라인을 따라 조금
        // 더 앞으로(P_LEAD_DISTANCE) 간 점 = 스팟 앞.
        constexpr float P_LEAD_DISTANCE = 3.0f;

        const Spline *bestSpline = nullptr;
        float bestDist = std::numeric_limits<float>::max();
        for (const shared_ptr<Lane> &lane : *parkingLanes)
        {
            const Spline &spline = lane->GetSpline();
            float param = spline.GetSplinePosition(m_parkSpot->position);
            float dist = (spline.GetPositionAt(param) - m_parkSpot->position).Length();
            if (dist < bestDist)
            {
                bestDist = dist;
                bestSpline = &spline;
            }
        }

        Vec3 pPos = bestSpline->GetLookaheadPoint(m_parkSpot->position, P_LEAD_DISTANCE);
        float pParam = bestSpline->GetSplinePosition(pPos);
        float pAngleRad = DirectionToAngleRad(bestSpline->GetDirectionAt(pParam));

        if (PlanParkLegTo(pPos, pAngleRad))
        {
            m_parkGoingToSpot = false; // leg 1 진행 중 — P 도착 후 UpdatePark가 leg 2를 잇는다.
            return true;
        }
        return false; // 이 스팟의 P까지 못 감 -> 다음 스팟
    }

    // 주차레인 없음 -> 스팟으로 직접 (leg 2 단계 없이 한 번에).
    Vec3 spotTarget = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
    float spotAngleRad = DirectionToAngleRad(m_parkSpot->direction);
    if (PlanParkLegTo(spotTarget, spotAngleRad))
    {
        m_parkGoingToSpot = true;
        m_parkAligned = false;
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
        m_RoadDataManager->ReleaseParkSpot(m_parkSpot->id);
    }
    m_parkSpot = m_RoadDataManager->TryReserveParkSpot(m_parkNodeId, m_triedParkSpotIds);
    return m_parkSpot != nullptr;
}

// 현재 스팟부터 입차를 시도하고, 실패하면 다음 빈 자리로 넘어가며 다 시도한다. 계획을 시작하면 true,
// 모든 자리가 안 되면 false.
bool Car::BeginParkEnterOrRetry()
{
    while (m_parkSpot != nullptr)
    {
        if (PlanEnterForCurrentSpot())
            return true;
        if (!ReserveNextParkSpot())
            return false;
    }
    return false;
}

// 현재 pose -> target 까지 Hybrid A*(장애물 회피)로 계획해 실행시킨다. 못 찾으면 false.
// exact=true면 Pure Pursuit(BuildParkSegments) 대신 정지-조향-이동-정지 방식의 정밀 실행
// (BuildExactSegments)을 쓴다 -- 짧은 최종 정렬 보정 등 추종 오차를 남기면 안 되는 경우에만 켠다.
bool Car::PlanParkLegTo(const Vec3 &targetPos, float targetAngleRad, bool exact)
{
    Vec3 startPos = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());
    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);
    HybridAStar::VehicleShape shape = BuildVehicleShape();
    const std::vector<HybridAStar::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();

    bool foundPath = false;
    ReedsShepp::Path path = HybridAStar::FindPath(startPos, startAngleRad, targetPos, targetAngleRad, obstacles, shape, foundPath);
    if (!foundPath)
        return false;

    if (exact)
        m_vehicleController.BeginPlan(BuildExactSegments(path, m_maxSteerAngle));
    else
        m_vehicleController.BeginPlan(BuildParkSegments(path, startPos, startAngleRad, turningRadius));
    RebuildParkDebugRender(path, startPos, startAngleRad, turningRadius, targetPos, targetAngleRad);
    return true;
}

// 입차 leg 2: 현재 pose(=P)에서 예약된 스팟까지 Hybrid A*. UpdatePark가 leg 1 완료 시 호출한다.
// 못 들어가면 다음 빈 자리로 넘어가 leg 1부터 다시 시도하고, 모든 자리가 안 되면 멈춘다.
void Car::BeginParkSpotLeg()
{
    Vec3 spotTarget = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
    float spotAngleRad = DirectionToAngleRad(m_parkSpot->direction);
    if (PlanParkLegTo(spotTarget, spotAngleRad))
        return; // leg 2 성공

    DebugConsole::Log("BeginParkSpotLeg: can't tuck into ParkSpot " + std::to_string(m_parkSpot->id) +
                      " from P, trying next spot");
    // 이 자리 실패 -> 다음 빈 자리부터 leg 1부터 다시. 성공하면 m_parkGoingToSpot=false로 시퀀스 이어짐.
    if (ReserveNextParkSpot() && BeginParkEnterOrRetry())
        return;

    // 남은 자리 없음 -> 입차 종료(빈 플랜 -> 다음 프레임 UpdatePark 완료 처리, 현재 자리에 멈춤).
    DebugConsole::Log("BeginParkSpotLeg: no reachable ParkSpot left, stopping");
    m_destLane = nullptr;
    m_vehicleController.BeginPlan({});
}

HybridAStar::VehicleShape Car::BuildVehicleShape() const
{
    HybridAStar::VehicleShape shape;
    shape.wheelbase = m_wheelbase;
    shape.maxSteerAngleRad = m_maxSteerAngle;
    shape.pivotToCenter = m_colliderOffset.z;
    shape.halfWidth = m_halfExtents.GetX();
    shape.halfLength = m_halfExtents.GetZ();
    return shape;
}

void Car::BeginAvoidPlan()
{
    DebugConsole::Log("Entering DriveSubMode::Avoiding: fully stopped, beginning HybridA* plan");

    Vec3 startPos = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());

    // 목표: 지금 레인의 스플라인을 따라 장애물을 완전히 지나칠 만큼(코리도어보다 넉넉히) 앞의 지점.
    // 레인을 벗어나지 않고 그 위의 한 점으로 잡아두면, 우회가 끝난 뒤 별도 처리 없이 그냥 Drive로
    // 돌아가 기존 m_currentLane/m_path를 그대로 이어서 쓸 수 있다. 현재 레인 안에서 거리가 안
    // 나오면 경로상 다음 레인까지만 이어서 재보고, 그래도 모자라면 다음 레인 끝점을 목표로 삼는다.
    constexpr float AVOID_GOAL_DISTANCE = 30.0f;
    const std::vector<Vec3> &splinePoints = m_currentSpline.GetSplinePoints();

    Vec3 targetPos = startPos;
    float targetAngleRad = startAngleRad;
    bool foundGoal = false;
    Vec3 prevPoint = startPos;
    float traveled = 0.0f;

    if (splinePoints.size() >= 2)
    {
        size_t startIndex = 0;
        float closestDistance = std::numeric_limits<float>::max();
        for (size_t i = 0; i < splinePoints.size(); ++i)
        {
            float distance = (splinePoints[i] - startPos).Length();
            if (distance < closestDistance)
            {
                closestDistance = distance;
                startIndex = i;
            }
        }

        for (size_t i = startIndex; i < splinePoints.size(); ++i)
        {
            const Vec3 &point = splinePoints[i];
            traveled += (point - prevPoint).Length();
            if (traveled >= AVOID_GOAL_DISTANCE)
            {
                targetPos = point;
                targetAngleRad = DirectionToAngleRad(point - prevPoint);
                foundGoal = true;
                break;
            }
            prevPoint = point;
        }
    }

    if (!foundGoal && m_pathIndex + 1 < m_path.size())
    {
        const std::shared_ptr<Lane> &nextLane = m_path[m_pathIndex + 1].lane;
        for (const Vec3 &point : nextLane->GetSpline().GetSplinePoints())
        {
            traveled += (point - prevPoint).Length();
            if (traveled >= AVOID_GOAL_DISTANCE)
            {
                targetPos = point;
                targetAngleRad = DirectionToAngleRad(point - prevPoint);
                foundGoal = true;
                break;
            }
            prevPoint = point;
        }

        if (!foundGoal)
        {
            targetPos = nextLane->GetEndPoint();
            targetAngleRad = DirectionToAngleRad(nextLane->GetSpline().GetDirectionAt(1.0f));
            foundGoal = true;
        }
    }

    if (!foundGoal)
    {
        // 다음 레인도 없음(경로 끝 코앞) -- 현재 레인 끝점을 최선으로 삼는다.
        targetPos = m_currentLane->GetEndPoint();
        targetAngleRad = DirectionToAngleRad(m_currentLane->GetSpline().GetDirectionAt(1.0f));
    }

    HybridAStar::VehicleShape shape = BuildVehicleShape();
    const std::vector<HybridAStar::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();
    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);

    bool foundPath = false;
    ReedsShepp::Path path = HybridAStar::FindPath(startPos, startAngleRad, targetPos, targetAngleRad, obstacles, shape, foundPath);
    if (!foundPath)
    {
        // 경로를 못 찾음 — 정적 장애물이 완전히 막고 있는 상태. fsm.txt의 "회피 재탐색도 없으면
        // 서브상태:정차로 전환"에 대응 — 정차로 넘어가 쿨다운마다 재시도하게 한다. (예전엔 그냥
        // 실패만 로그로 남기고 리턴해서, 다음 프레임 Normal로 샜다가 같은 장애물에 다시 걸려
        // 무한 반복하는 버그가 있었다 — notes/issue.txt)
        m_subMode = SubState::D_Stop;
        DebugConsole::Log("BeginAvoidPlan: HybridA* failed to find an avoid route, entering D_Stop to retry later");
        return;
    }
    m_vehicleController.BeginPlan(BuildParkSegments(path, startPos, startAngleRad, turningRadius));

    DebugConsole::Log("Avoid path segments: " + to_string(path.size()));
}

void Car::UpdateFindPath()
{
    // 경로 (재)탐색 조건: 목적지가 없거나, 이미 레인이 있고 코스 이탈도 아니면 탐색 불필요.
    bool notSearch = (m_destLane == nullptr) ||
                     (m_currentLane != nullptr && !IsOffCourse());
    if (notSearch)
        return;

    if (m_currentLane != nullptr)
        return;

    Vec3 position = GetPosition();
    if (m_parkSpot != nullptr)
        return;

    SetCurrentLane(m_RoadDataManager->GetClosestLane(position));
    EnterCurrentLane();
}

void Car::EnterCurrentLane()
{
    Vec3 position = m_rigidbody.GetPosition();
    m_path = m_RoadDataManager->FindPath(m_currentLane, m_destLane);
    m_pathIndex = 0; // path[0] == 시작(=현재) 레인
    if (m_path.empty())
    {
        m_destLane = nullptr;
        SetCurrentLane(nullptr);
        return;
    }

    MergeOntoLane(m_currentLane, position);
    RescanRoadSpeedConstraints();
}

void Car::UpdateStop()
{
    // 정지 조건(목적지 없음/도착)은 DecideNextMode가 이미 판단했으므로 여기선 감속/정지 동작만 한다.
    m_destLane = nullptr;
    SetCurrentLane(nullptr);

    // Park에서 넘어온 직후면 조향 원복 세그먼트가 아직 안 끝났을 수 있다.
    if (!m_vehicleController.IsFinished())
    {
        m_wantSegmentTick = true;
        return;
    }

    if (m_speed > 0.0f)
    {
        Accelerate(0.0f);
        return;
    }
}

void Car::UpdatePark()
{
    // TODO(주차 중 장애물 감시): fsm.txt는 상태:주차(출차) 전체에 박스캐스트 감지 -> 5초 대기 ->
    // 이동 여부 판단 -> 재탐색 로직을 요구하지만, 여긴 각 leg를 계획하는 시점에 정적 장애물만
    // 반영하고 실행 중(RS pure pursuit로 이동하는 동안)에는 전혀 재검사하지 않는다. 대기시간을
    // 셀 멤버 변수가 없어(Car.h는 이번엔 안 건드림) 구현하지 않음 — 넣으려면 Car.h에 타이머 값을
    // 추가하면서 여기 같이 채워야 한다.
    if (m_parkPlanPending)
    {
        // 완전히 정지할 때까지는 RS 계획을 세우지 않고 감속만 한다.
        if (m_speed > 0.0f)
        {
            Accelerate(0.0f);
            return;
        }
        m_parkPlanPending = false;
        DebugConsole::Log("Park plan pending resolved: fully stopped, beginning RS plan");
        BeginParkPlan();
    }

    if (!m_vehicleController.IsFinished())
    {
        m_wantSegmentTick = true;
        return;
    }

    // m_parkSpot!=nullptr 체크는 방어코드 — BeginParkPlan이 예약 실패로 조기 리턴하면
    // m_isExitingPark가 이전 사이클의 스테일 값(true)일 수 있는데, 그 상태에서 m_parkSpot은
    // 항상 null이므로 여기서 걸러야 아래 m_parkSpot->id가 널 역참조로 크래시하지 않는다.
    if (m_isExitingPark && m_parkSpot != nullptr)
    {
        // 출차 완료: 이제 레인 위. 더 이상 이 주차칸에 있는 게 아니므로 예약을 풀고 비운다.
        m_parkSequenceActive = false; // 시퀀스 종료 — 다음 프레임 Drive로 전환 허용.
        m_RoadDataManager->ReleaseParkSpot(m_parkSpot->id);
        m_parkSpot = nullptr;
        // m_currentLane은 BeginParkPlan에서 이미 정해둔 상태 — 거기서부터 경로/스플라인을 채운다.
        EnterCurrentLane();
        return;
    }

    // 입차 leg 1(-> 스플라인점 P)이 끝났으면, 이제 P에서 스팟까지 leg 2를 이어 계획한다. (주차레인 없이
    // 바로 스팟으로 간 경우엔 BeginParkPlan에서 m_parkGoingToSpot=true라 이 블록을 건너뛰고 완료로 간다.)
    if (!m_isExitingPark && m_parkSpot != nullptr && !m_parkGoingToSpot)
    {
        // leg 1처럼 완전히 멈춘 뒤 그 pose에서 leg 2를 계획한다(open-loop RS는 시작 pose 기준). 대기 중
        // 컨트롤러가 finished여도 m_parkSequenceActive가 Park를 유지하므로 Drive로 새지 않는다.
        if (m_speed > 0.0f)
        {
            Accelerate(0.0f);
            return;
        }
        m_parkGoingToSpot = true;
        m_parkAligned = false;
        BeginParkSpotLeg();
        return;
    }

    // 스팟까지의 leg가 pure pursuit로 끝났는데 아직 정렬 보정을 안 했으면, 실제로 도착한 pose
    // 기준으로 같은 목표 pose까지 RS를 한 번 더 계획해 pure pursuit의 잔여 정렬 오차(주로 최종
    // 헤딩)를 없앤다. exact=true라 정지-조향-이동-정지로 정밀하게 실행돼 이번엔 추종 오차가
    // 남지 않는다. m_parkAligned를 먼저 true로 세워 재귀적으로 반복되지 않게 한다 -- 이미 목표에
    // 정확히 있으면(혹은 장애물 등으로 경로를 못 찾으면) PlanParkLegTo가 false를 반환하고 그냥
    // 완료 처리로 넘어간다.
    if (!m_isExitingPark && m_parkSpot != nullptr && m_parkGoingToSpot && !m_parkAligned)
    {
        if (m_speed > 0.0f)
        {
            Accelerate(0.0f);
            return;
        }
        m_parkAligned = true;
        Vec3 spotTarget = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
        float spotAngleRad = DirectionToAngleRad(m_parkSpot->direction);
        if (PlanParkLegTo(spotTarget, spotAngleRad, /*exact=*/true))
            return;
    }

    // 입차 완료: m_parkSpot은 "지금 여기 주차 중"을 나타내도록 남겨두고 destLane/currentLane만
    // 비운다. 다음 프레임 DecideNextMode가 destLane==nullptr을 보고 Stop으로 전환한다. 이후 새
    // 목적지가 생기면 DecideNextMode의 일반 로직(parkSpot!=nullptr && currentLane==nullptr)이
    // 다시 Park로 복귀시키며 OnModeEnter가 BeginParkPlan을 호출한다.
    if (m_currentLane != nullptr)
    {
        m_parkSequenceActive = false; // 입차 시퀀스 종료 — 다음 프레임 Stop으로 전환 허용.
        m_destLane = nullptr;
        SetCurrentLane(nullptr);
        return;
    }
}

void Car::UpdateDrive()
{
    // 서브상태:회피 — Hybrid A* + RS 경로를 다 추종할 때까지는 일반주행 로직(CheckPath/TryLaneChange/
    // TryAvoidObstacle)을 건드리지 않는다. 완주하면 일반주행으로 복귀한다.
    // TODO(정차): 회피 경로 추종 "중"에 재장애물 감지는 없다 — notes/fsm.txt의 "2초 대기 후 이동
    // 여부 판단, 안 움직였으면 재탐색" 분기는 안 만듦. 장애물이 전부 정적이라(움직이지 않음)
    // 이미 성공한 RS 경로가 도중에 새로 막힐 일이 없어서(고정 장애물 기준으로 계획했으므로)
    // 실질적으로 발생 안 하는 케이스로 보고 생략함 — 동적 장애물이 생기면 다시 볼 것.
    if (m_subMode == SubState::D_Avoid)
    {
        if (!m_vehicleController.IsFinished())
        {
            m_wantSegmentTick = true;
            return;
        }
        // 회피 RS 경로 완주 — m_mode가 계속 Drive라 OnModeEnter(Drive)가 다시 안 불리므로, 예전에
        // 거기서 하던 일반주행용 SplineFollowSegment 재구성을 여기서 직접 해준다.
        m_subMode = SubState::D_Normal;
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        segments.push_back(std::make_unique<SplineFollowSegment>());
        m_vehicleController.BeginPlan(std::move(segments));
    }
    else if (m_subMode == SubState::D_Stop)
    {
        // 서브상태:정차 — BeginAvoidPlan이 경로를 못 찾아 여기로 왔다. fsm.txt는 "2초 대기 -> 이동
        // 여부 판단 -> 안 움직였으면 재탐색"이지만, 장애물이 전부 정적이라 "움직였는지" 판단 자체가
        // 불가능하다(TODO: 동적 장애물 도입되면 여기 이동 판단을 추가해야 함). 그 대신 쿨다운마다
        // Hybrid A* 재탐색만 반복한다. 재탐색 성공하면 BeginAvoidPlan이 알아서 D_Avoid로 옮겨두고,
        // 실패하면 또 D_Stop으로 되돌려놓으므로(BeginAvoidPlan 참고) 여기선 쿨다운만 관리한다.
        if (m_speed > 0.0f)
            Accelerate(0.0f);

        if (m_avoidReplanCooldown > 0.0f)
            m_avoidReplanCooldown = std::max(0.0f, m_avoidReplanCooldown - m_deltaTime);

        if (m_avoidReplanCooldown > 0.0f)
        {
            m_wantSegmentTick = true;
            return;
        }

        constexpr float STOPPED_REPLAN_COOLDOWN = 2.0f; // fsm.txt 서브상태:정차의 재탐색 대기(2초)
        m_avoidReplanCooldown = STOPPED_REPLAN_COOLDOWN;
        m_subMode = SubState::D_Avoid;
        BeginAvoidPlan();
        m_wantSegmentTick = true;
        return;
    }

    if (!CheckPath())
        return;
    // TODO(신호대기): 전방 정지신호/정지선 감지 자체가 없다 — RoadNodeType에 신호 타입이 없어서
    // fsm.txt의 "정지신호 있으면 감속정지 -> 서브상태:신호대기" 분기를 걸 수 없다. 신호 시스템이
    // 생기면 여기서 검사해 SubState::D_WaitSignal로 전환하고, "초록불이면 Normal로 복귀"하는 분기를
    // 추가해야 한다 (SubState::D_WaitSignal 값 자체는 이미 선언돼 있음).
    TryLaneChange();
    if (TryAvoidObstacle())
        return;
    m_wantSegmentTick = true;
}

bool Car::TryLaneChange()
{
    if (m_currentLane == nullptr || m_currentTime - m_lastLaneChangeTime < MOBIL_EVAL_INTERVAL)
        return false;

    // 다음 경로 스텝이 이미 라우팅상 차선변경이면(목적지 도달을 위해 필요한 변경) MOBIL이 따로
    // 끼어들어 다투지 않게 건너뛴다.
    if (m_pathIndex + 1 < m_path.size() && m_path[m_pathIndex + 1].isLaneChange)
        return false;

    // 평가했다는 사실 자체로 다음 평가까지 쿨다운(성공/실패 무관 — 왕복 진동 방지).
    m_lastLaneChangeTime = m_currentTime;

    Vec3 position = m_rigidbody.GetPosition();
    // position은 "도로를 따라간 거리"라는 하나의 좌표계로 취급한다 — 좌/우 인접 레인은 같은 도로의
    // 평행한 차선이라 서로 같은 좌표계를 공유한다고 근사한다 (레인마다 다시 투영하지 않음).
    float egoPosition = m_currentLane->GetSpline().GetSplinePosition(position) * m_currentLane->GetLength();

    LaneNeighbor egoLeaderN, oldFollowerN;
    FindLaneNeighbors(m_currentLane, egoPosition, egoLeaderN, oldFollowerN);

    float v0 = std::min(m_maxSpeed, m_currentLane->GetLimitSpeed());
    CarFollowing::Params cfParams = BuildIdmParams(v0);
    Mobil::Params mobilParams{MOBIL_SAFE_DECEL, MOBIL_POLITENESS, MOBIL_THRESHOLD};

    constexpr float VIRTUAL_LEADER_GAP = 100000.0f; // 실제 리더가 없을 때(뚫린 도로) 쓰는 가상 리더 거리
    auto virtualLeader = [&](float laneLimitSpeed)
    { return Mobil::VehicleState{laneLimitSpeed, 0.0f, egoPosition + VIRTUAL_LEADER_GAP, 0.0f}; };

    Mobil::VehicleState ego{m_speed, m_acceleration, egoPosition, GetLength()};
    Mobil::VehicleState egoLeaderState = (egoLeaderN.car != nullptr) ? ToVehicleState(egoLeaderN) : virtualLeader(v0);
    Mobil::VehicleState oldFollowerStorage;
    const Mobil::VehicleState *oldFollowerState = nullptr;
    if (oldFollowerN.car != nullptr)
    {
        oldFollowerStorage = ToVehicleState(oldFollowerN);
        oldFollowerState = &oldFollowerStorage;
    }

    for (const weak_ptr<Lane> &candidateWeak : {m_currentLane->GetLeft(), m_currentLane->GetRight()})
    {
        shared_ptr<Lane> candidate = candidateWeak.lock();
        if (candidate == nullptr)
            continue;

        LaneNeighbor newLeaderN, newFollowerN;
        FindLaneNeighbors(candidate, egoPosition, newLeaderN, newFollowerN);

        float candidateV0 = std::min(m_maxSpeed, candidate->GetLimitSpeed());
        Mobil::VehicleState newLeaderState = (newLeaderN.car != nullptr) ? ToVehicleState(newLeaderN) : virtualLeader(candidateV0);
        Mobil::VehicleState newFollowerStorage;
        const Mobil::VehicleState *newFollowerState = nullptr;
        if (newFollowerN.car != nullptr)
        {
            newFollowerStorage = ToVehicleState(newFollowerN);
            newFollowerState = &newFollowerStorage;
        }

        bool approved = Mobil::EvaluateLaneChange(ego, oldFollowerState, egoLeaderState, newLeaderState,
                                                  newFollowerState, mobilParams, cfParams);
        if (!approved)
            continue;

        // 목적지 도달 가능성 확인 없이 그냥 옮기면, 이 레인이 목적지로 못 가는 레인일 때 경로를 잃는다.
        vector<LaneStep> newPath = m_RoadDataManager->FindPath(candidate, m_destLane);
        if (newPath.empty())
            continue; // 이 후보로는 목적지에 못 감 -- 포기하고 다음 후보(오른쪽)로

        m_path = std::move(newPath);
        m_pathIndex = 0;
        MergeOntoLane(candidate, position); // 내부에서 SetCurrentLane까지 처리
        RescanRoadSpeedConstraints();
        DebugConsole::Log("MOBIL lane change: " + GetName() + " -> lane " + std::to_string(candidate->GetId()));
        return true;
    }

    return false;
}

bool Car::TryAvoidObstacle()
{
    // Tier 1: 매 프레임 저비용 감시. grid 없이, 앞으로 갈 코리도어(스플라인 lookahead를 따라
    // 일정 간격으로 샘플링한 차량 pose들)를 RoadDataManager가 들고 있는 장애물 목록과 겹치는지만
    // 검사한다. 막힌 걸 확인하면 우선 멈추고, 완전히 멈추면 Tier 2(Hybrid A* 우회, Avoid 모드)를 켠다.
    if (m_avoidReplanCooldown > 0.0f)
        m_avoidReplanCooldown = std::max(0.0f, m_avoidReplanCooldown - m_deltaTime);

    if (m_RoadDataManager == nullptr || m_currentLane == nullptr)
        return false;

    const std::vector<HybridAStar::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();
    if (obstacles.empty())
        return false;

    const std::vector<Vec3> &splinePoints = m_currentSpline.GetSplinePoints();
    if (splinePoints.size() < 2)
        return false;

    HybridAStar::VehicleShape shape = BuildVehicleShape();

    constexpr float LOOKAHEAD_DISTANCE = 10.0f; // 코리도어 길이(m)
    constexpr float SAMPLE_SPACING = 1.0f;      // 샘플 간격(m)

    Vec3 position = m_rigidbody.GetPosition();

    // 현재 위치에서 가장 가까운 스플라인 점부터 훑기 시작한다.
    size_t startIndex = 0;
    float closestDistance = std::numeric_limits<float>::max();
    for (size_t i = 0; i < splinePoints.size(); ++i)
    {
        float distance = (splinePoints[i] - position).Length();
        if (distance < closestDistance)
        {
            closestDistance = distance;
            startIndex = i;
        }
    }

    Vec3 prevPoint = position;
    float traveled = 0.0f;
    float sinceLastSample = 0.0f;
    for (size_t i = startIndex; i < splinePoints.size() && traveled < LOOKAHEAD_DISTANCE; ++i)
    {
        const Vec3 &point = splinePoints[i];
        float segmentLength = (point - prevPoint).Length();
        traveled += segmentLength;
        sinceLastSample += segmentLength;
        if (sinceLastSample < SAMPLE_SPACING)
        {
            prevPoint = point;
            continue;
        }
        sinceLastSample = 0.0f;

        float headingRad = DirectionToAngleRad(point - prevPoint);
        if (HybridAStar::IsColliding(point, headingRad, obstacles, shape))
        {
            // TODO(회피 전 저수준 회피): fsm.txt는 완전정지+Hybrid A* 전에 먼저 가벼운 회피 경로
            // (Reynolds Obstacle Avoidance 등)를 시도해서, 성공하면 정지하지 않고 그 경로로 계속
            // 주행하라고 되어 있다. 지금은 그 단계가 없어서 바로 감속/정지 -> 서브상태:회피(Hybrid
            // A*)로 간다.
            constexpr float AVOID_REPLAN_COOLDOWN = 1.5f; // Hybrid A* 실패 시 매 프레임 재시도하지 않도록.
            if (m_speed > 0.0f)
            {
                DebugConsole::Log("Obstacle blocking corridor: emergency braking");
                EmergBrake();
            }
            else if (m_avoidReplanCooldown <= 0.0f)
            {
                // 이미 완전히 멈춘 상태 확인됨 — Park의 m_parkPlanPending 같은 별도 대기 없이 바로 진입.
                m_subMode = SubState::D_Avoid;
                m_avoidReplanCooldown = AVOID_REPLAN_COOLDOWN;
                BeginAvoidPlan();
            }
            return true;
        }
        prevPoint = point;
    }
    return false;
}

bool Car::CheckPath()
{
    // path find
    Vec3 position = GetPosition();

    // 다음 레인이 차선변경이면, 레인 끝까지 기다리지 않고 바로 차선변경을 진행한다.
    float laneStartDistance = (m_currentLane->GetStartPoint() - position).Length();
    if (m_pathIndex + 1 < m_path.size() && m_path[m_pathIndex + 1].isLaneChange && laneStartDistance > LANE_ENTRY_THRESHOLD)
    {
        ++m_pathIndex;
        MergeOntoLane(m_path[m_pathIndex].lane, position);
        RescanRoadSpeedConstraints();
        return true;
    }

    // 현재 레인의 끝에 다가가면 경로상 다음 레인으로 넘어간다.
    float laneEndDistance = (m_currentLane->GetEndPoint() - position).Length();
    bool enteredByLaneChange = false;
    while (laneEndDistance < LANE_TRANSITION_THRESHOLD)
    {
        if (m_pathIndex + 1 >= m_path.size())
        {
            m_destLane = nullptr;
            SetCurrentLane(nullptr);
            return false;
        }
        ++m_pathIndex;
        SetCurrentLane(m_path[m_pathIndex].lane);
        enteredByLaneChange = m_path[m_pathIndex].isLaneChange;
        m_currentSpline = m_currentLane->GetSpline();
        laneEndDistance = (m_currentLane->GetEndPoint() - position).Length();
        RebuildSplineRender();
    }
    // 차선변경으로 진입한 레인이면, 현재 위치에서 그 레인 위로 합류하는 연결 스플라인을 만든다.
    if (enteredByLaneChange)
    {
        MergeOntoLane(m_currentLane, position);
        RescanRoadSpeedConstraints();
    }
    return true;
}

void Car::DriveControl()
{
    Vec3 position = m_rigidbody.GetPosition();
    // steering
    constexpr float MIN_LOOKAHEAD_DISTANCE = 5.0f; // 저속/정지 시 최소 lookahead (m)
    constexpr float LOOKAHEAD_TIME = 1.0f;         // 몇 초 앞을 볼지
    float lookaheadDistance = std::max(MIN_LOOKAHEAD_DISTANCE, m_speed * LOOKAHEAD_TIME);
    auto targetPosition = m_currentSpline.GetLookaheadPoint(position, lookaheadDistance);
    float targetSteer = PurePursuit(targetPosition);
    Steer(targetSteer);

    // speed control
    float steerSpeedCap = CalcMaxSpeed(targetSteer) * 0.8f;
    DriveSpeedIDM(steerSpeedCap);

    // Debug
    DirectX::XMFLOAT3 targetMarkerPos = ToXMFLOAT3(targetPosition);
    targetMarkerPos.y = 0.2f;
    m_targetMarker.GetTransform().SetPosition(targetMarkerPos);
}
