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
    State next = DecideNextMode(&reason);
    if (next != m_mode)
    {
        DebugConsole::Log(std::string(StateToString(m_mode)) + " -> " + StateToString(next) + " (" + reason + ")");
        OnModeExit(next);
        State prev = m_mode;
        m_mode = next;
        OnModeEnter(prev);
    }
}

void Car::UpdateFindPath()
{
    if (m_destLane == nullptr || m_currentLane != nullptr)
        return;

    Vec3 position = GetPosition();
    if (m_parkSpot != nullptr)
    {
        auto parkNode = m_RoadDataManager->GetNode(m_parkNodeId);
        if (parkNode != nullptr)
            position = parkNode->position;
    }

    SetCurrentLane(m_RoadDataManager->GetClosestLaneStart(position));
    TryFindPathAndSetLane();
}

Car::State Car::DecideNextMode(const char **reason) const
{

    if (m_mode == State::Stop)
    {
        if (m_destLane == nullptr)
        {
            return State::Stop;
        }
        *reason = "go to Dest";
        return State::Park;
    }
    else if (m_mode == State::Park)
    {
        if (m_parkPlanPending || m_parkSequenceActive || !m_vehicleController.IsFinished())
        {
            *reason = "parking in progress";
            return State::Park;
        }
        if (m_subMode == SubState::P_EXIT)
        {
            // 출차 끝났으면 Drive로 전환
            *reason = "normal driving";
            return State::Drive;
        }
        // 그 외(P_ENTER_LEG1/LEG2/ALIGN 등 입차 계열)가 여기 왔다는 건 이미 다 끝났다는 뜻 -> Stop
        *reason = "done parking";
        return State::Stop;
    }
    else if (m_mode == State::Drive)
    {
        if (m_subMode == SubState::D_Avoid && !m_vehicleController.IsFinished())
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
        return State::Drive;
    }

    *reason = "unreachable";
    return m_mode;
}

void Car::OnModeEnter(State prev)
{
    if (m_mode == State::Drive)
    {
        m_subMode = SubState::D_Normal;
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        segments.push_back(std::make_unique<SplineFollowSegment>());
        m_vehicleController.BeginPlan(std::move(segments));
    }
    else if (m_mode == State::Park)
    {
        m_subMode = prev == State::Stop ? SubState::P_EXIT : SubState::P_ENTER_LEG1;
        m_parkPlanPending = true;    // 도착 즉시 RS를 계산하지 않고, 완전히 멈출 때까지 기다린다 (UpdatePark에서 처리).
        m_parkSequenceActive = true; // 주차 시퀀스 시작 — 완료(UpdatePark)까지 Park 유지.
        DebugConsole::Log("Park plan pending: waiting for full stop before planning");
    }
    else if (m_mode == State::Stop)
    {
        // 입차 완료 후 Stop으로 오는 경우도 RS 매뉴버로 꺾여있던 조향을 중앙으로 되돌린다.
        if (prev == State::Park && m_subMode == SubState::P_ENTER_ALIGN)
        {
            std::vector<std::unique_ptr<VehicleSegment>> segments;
            segments.push_back(std::make_unique<CenterSteerSegment>());
            m_vehicleController.BeginPlan(std::move(segments));
        }
        m_subMode = SubState::None;
    }
}

void Car::OnModeExit(State next)
{
    if (!m_vehicleController.IsFinished())
        m_vehicleController.Abort();
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

bool Car::TryFindPathAndSetLane()
{
    m_path = m_RoadDataManager->FindPath(m_currentLane, m_destLane);
    m_pathIndex = 0;
    if (m_path.empty())
    {
        m_destLane = nullptr;
        SetCurrentLane(nullptr);
        return false;
    }

    m_currentSpline = m_currentLane->GetSpline();
    RescanRoadSpeedConstraints();
    return true;
}
#pragma endregion

#pragma region Park

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

    // m_parkSpot이 있어야 뜻이 있는 분기들만 여기 묶는다. BeginParkPlan이 예약 실패로 조기 리턴하면
    // m_parkSpot이 계속 null일 수 있는데, 그 경우는 여기 아무 분기도 안 타고 아래 m_currentLane
    // 체크(시퀀스 정리용 안전장치)로 내려가야 하므로, 그 폴백은 이 블록 밖에 둔다.
    if (m_parkSpot != nullptr)
    {
        if (m_subMode == SubState::P_EXIT)
        {
            // 출차 완료: 이제 레인 위. 더 이상 이 주차칸에 있는 게 아니므로 예약을 풀고 비운다.
            m_parkSequenceActive = false; // 시퀀스 종료 — 다음 프레임 Drive로 전환 허용.
            m_RoadDataManager->ReleaseParkSpot(m_parkSpot->id);
            m_parkSpot = nullptr;
            // m_currentLane은 BeginParkPlan에서 이미 정해둔 상태 — 거기서부터 경로/스플라인을 채운다.
            TryFindPathAndSetLane();
            return;
        }

        // 입차 leg 1(-> 스플라인점 P)이 끝났으면, 이제 P에서 스팟까지 leg 2를 이어 계획한다. (주차레인
        // 없이 바로 스팟으로 간 경우엔 PlanEnterForCurrentSpot에서 이미 leg2로 세팅해서 건너뛴다.)
        if (m_subMode == SubState::P_ENTER_LEG1)
        {
            // leg 1처럼 완전히 멈춘 뒤 그 pose에서 leg 2를 계획한다(open-loop RS는 시작 pose 기준). 대기 중
            // 컨트롤러가 finished여도 m_parkSequenceActive가 Park를 유지하므로 Drive로 새지 않는다.
            if (m_speed > 0.0f)
            {
                Accelerate(0.0f);
                return;
            }
            m_subMode = SubState::P_ENTER_LEG2;
            BeginParkSpotLeg();
            return;
        }

        // 스팟까지의 leg가 pure pursuit로 끝났는데 아직 정렬 보정을 안 했으면, 실제로 도착한 pose
        // 기준으로 같은 목표 pose까지 RS를 한 번 더 계획해 pure pursuit의 잔여 정렬 오차(주로 최종
        // 헤딩)를 없앤다. exact=true라 정지-조향-이동-정지로 정밀하게 실행돼 이번엔 추종 오차가
        // 남지 않는다. subMode를 먼저 ALIGN으로 세워 재귀적으로 반복되지 않게 한다 -- 이미 목표에
        // 정확히 있으면(혹은 장애물 등으로 경로를 못 찾으면) PlanParkLegTo가 false를 반환하고 그냥
        // 완료 처리로 넘어간다.
        if (m_subMode == SubState::P_ENTER_LEG2)
        {
            if (m_speed > 0.0f)
            {
                Accelerate(0.0f);
                return;
            }
            m_subMode = SubState::P_ENTER_ALIGN;
            Vec3 spotTarget = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
            float spotAngleRad = DirectionToAngleRad(m_parkSpot->direction);
            if (PlanParkLegTo(spotTarget, spotAngleRad, /*exact=*/true))
                return;
        }
    }

    // 입차 완료 (또는 m_parkSpot 자체가 없어 더 할 게 없는 경우 시퀀스 정리): m_parkSpot은 "지금
    // 여기 주차 중"을 나타내도록 남겨두고 destLane/currentLane만
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

void Car::BeginParkPlan()
{
    int parkNodeId = -1;
    if (m_parkSpot == nullptr && m_pendingParkNode != nullptr)
    {
        parkNodeId = m_pendingParkNode->id;
        m_parkSpot = m_RoadDataManager->TryReserveParkSpot(parkNodeId);
        m_pendingParkNode = nullptr;
        if (m_parkSpot == nullptr)
        {
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

    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);
    Vec3 startPos = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());
    HybridAStar::VehicleShape shape = BuildVehicleShape();
    const std::vector<HybridAStar::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();

    // 출차
    if (m_subMode == SubState::P_EXIT)
    {
        Vec3 lookupPos = startPos;
        auto parkNode = m_RoadDataManager->GetNode(m_parkNodeId);
        if (parkNode != nullptr)
            lookupPos = parkNode->position;

        shared_ptr<Lane> closestLane = m_RoadDataManager->GetClosestLaneStart(lookupPos);

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
    // 주차 (BeginParkPlan은 Park 진입시 딱 한 번만 불리므로 여기 오는 시점의 subMode는 항상 leg1)
    else if (m_subMode == SubState::P_ENTER_LEG1)
    {
        if (!BeginParkEnterOrRetry())
        {
            DebugConsole::Log("BeginParkPlan: no reachable ParkSpot for node " + std::to_string(parkNodeId) +
                              ", abandoning destination");
            m_destLane = nullptr;
            m_parkSequenceActive = false;
        }
    }
}

// m_parkSpot로의 입차 시작 계획: 주차레인이 있으면 leg 1(-> 스팟 앞 P), 없으면 스팟으로 직접(leg 2 없음).
// 계획을 시작했으면 true(+ m_subMode를 해당 leg로 설정), 이 자리로는 경로를 못 찾으면 false.
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
            m_subMode = SubState::P_ENTER_LEG1; // leg 1 진행 중 — P 도착 후 UpdatePark가 leg 2를 잇는다.
            return true;
        }
        return false; // 이 스팟의 P까지 못 감 -> 다음 스팟
    }

    // 주차레인 없음 -> 스팟으로 직접
    Vec3 spotTarget = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
    float spotAngleRad = DirectionToAngleRad(m_parkSpot->direction);
    if (PlanParkLegTo(spotTarget, spotAngleRad))
    {
        m_subMode = SubState::P_ENTER_LEG2; // leg 1 없이 바로 leg 2
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
    // 이 자리 실패 -> 다음 빈 자리부터 leg 1부터 다시. PlanEnterForCurrentSpot이 새 자리에 맞는
    // subMode(leg1 또는 주차레인 없으면 leg2)를 알아서 세팅한다.
    if (ReserveNextParkSpot() && BeginParkEnterOrRetry())
        return;

    // 남은 자리 없음 -> 입차 종료(빈 플랜 -> 다음 프레임 UpdatePark 완료 처리, 현재 자리에 멈춤).
    DebugConsole::Log("BeginParkSpotLeg: no reachable ParkSpot left, stopping");
    m_destLane = nullptr;
    m_vehicleController.BeginPlan({});
}

#pragma endregion

#pragma region Stop
void Car::UpdateStop()
{
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

    std::vector<std::shared_ptr<RoadNode>> candidates;
    for (const auto &[id, node] : m_RoadDataManager->GetNodes())
    {
        if (node->nodeType != RoadNodeType::ParkSpot)
            candidates.push_back(node);
    }
    if (candidates.empty())
        return;

    SetDestination(candidates[rand() % candidates.size()]);
}
#pragma endregion

#pragma region Drive
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
        BeginAvoidPlan(); // 성공하면 subMode를 D_Avoid로, 실패하면 도로 D_Stop으로 남겨둔다.
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
        SetCurrentLane(candidate);
        m_currentSpline = candidate->GetSpline();
        RescanRoadSpeedConstraints();
        DebugConsole::Log("MOBIL lane change: " + GetName() + " -> lane " + std::to_string(candidate->GetId()));
        return true;
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
        SetCurrentLane(m_path[m_pathIndex].lane);
        m_currentSpline = m_currentLane->GetSpline();
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
    // 차선변경으로 진입한 레인이면(레인/스플라인은 위 while에서 이미 세팅됨) 도로 제약(IDM용)만
    // 새 레인 기준으로 다시 스캔한다.
    if (enteredByLaneChange)
    {
        RescanRoadSpeedConstraints();
    }
    return true;
}

void Car::DriveControl()
{
    Vec3 position = m_rigidbody.GetPosition();
    // steering
    // Pure Pursuit 공식(PurePursuit()): steeringAngle = atan(2*wheelbase*sin(headingError)/distance).
    // headingError가 최악(90도, sin=1)이어도 steeringAngle이 m_maxSteerAngle을 못 넘게 하려면
    // distance(lookahead) >= 2*wheelbase/tan(maxSteerAngle) = 최소 회전지름이어야 한다. 이 하한을
    // 두면 차선 합류처럼 타겟이 옆으로 크게 벗어나 있어도 조향각이 과도하게 꺾이지 않는다 (예전엔
    // MergeOntoLane이 별도 연결 스플라인으로 이 문제를 우회했는데, 이 하한이 있으면 그냥 새 레인
    // 스플라인으로 바로 스위치해도 됨).
    float minSafeLookahead = 2.0f * m_wheelbase / tanf(m_maxSteerAngle);
    constexpr float LOOKAHEAD_TIME = 1.0f; // 몇 초 앞을 볼지
    float lookaheadDistance = std::max(minSafeLookahead, m_speed * LOOKAHEAD_TIME);
    auto targetPosition = m_currentSpline.GetLookaheadPoint(position, lookaheadDistance);
    // TryAvoidObstacle이 전방 코리도어에서 장애물을 감지하고 옆으로 피할 여지를 찾으면 여기서
    // 조준점 자체를 옆으로 밀어(Reynolds식 측면 회피) 차선 추종을 유지한 채 슬쩍 피해가게 한다.
    if (m_avoidLateralOffset != 0.0f)
    {
        float targetParam = m_currentSpline.GetSplinePosition(targetPosition);
        Vec3 targetDir = m_currentSpline.GetDirectionAt(targetParam);
        Vec3 targetNormal = Vec3(-targetDir.GetZ(), 0.0f, targetDir.GetX()).Normalized();
        targetPosition = targetPosition + targetNormal * m_avoidLateralOffset;
    }
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

// 전방 코리도어(현재 스플라인을 따라가는, 차량 형상 폭의 가상 박스캐스트)에 장애물이 있는지 본다.
// 없으면 그냥 일반주행 유지(false). 있으면 옆으로 살짝 피해서(Reynolds식 측면 오프셋) 여전히 갈 수
// 있는지 찾아보고, 있으면 m_avoidLateralOffset만 세팅한 채 일반주행 유지(false) — DriveControl이
// 다음 틱부터 그 오프셋만큼 조준점을 밀어서 슬쩍 피해간다. 옆으로도 못 피하면 하이브리드 A* 회피
// 기동으로 넘긴다(BeginAvoidPlan) — 성공/실패 어느 쪽이든 이번 프레임의 기본 주행 틱은 건너뛴다(true).
bool Car::TryAvoidObstacle()
{
    const std::vector<HybridAStar::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();
    HybridAStar::VehicleShape shape = BuildVehicleShape();
    if (obstacles.empty())
    {
        m_avoidLateralOffset = 0.0f;
        return false;
    }

    Vec3 position = m_rigidbody.GetPosition();

    // lateralOffset만큼 옆으로 민 코리도어를 따라 샘플을 찍어 하나라도 장애물과 겹치면 false.
    auto sweepClear = [&](float lateralOffset)
    {
        for (float d = 0.0f; d <= AVOID_DETECT_DISTANCE; d += AVOID_SAMPLE_STEP)
        {
            Vec3 samplePos = m_currentSpline.GetLookaheadPoint(position, d);
            float sampleParam = m_currentSpline.GetSplinePosition(samplePos);
            Vec3 sampleDir = m_currentSpline.GetDirectionAt(sampleParam);
            Vec3 sampleNormal = Vec3(-sampleDir.GetZ(), 0.0f, sampleDir.GetX()).Normalized();
            Vec3 testPos = samplePos + sampleNormal * lateralOffset;
            if (HybridAStar::IsColliding(testPos, DirectionToAngleRad(sampleDir), obstacles, shape))
                return false;
        }
        return true;
    };

    if (sweepClear(0.0f))
    {
        m_avoidLateralOffset = 0.0f;
        return false; // 전방 코리도어 깨끗함
    }

    // 코리도어를 막는 장애물들 중 감지거리 안에 있는 것들의 스플라인 기준 좌우 위치(부호 있는 옆
    // 오프셋)를 평균내, 장애물이 몰려있는 반대쪽을 우선 피할 방향으로 삼는다(순수 트라이얼이 아니라
    // 장애물 위치에 반응한다는 점에서 Reynolds 회피의 "반발" 취지를 살린 것).
    float biasSum = 0.0f;
    int biasCount = 0;
    for (const HybridAStar::Obstacle &obstacle : obstacles)
    {
        float obsParam = m_currentSpline.GetSplinePosition(obstacle.center);
        Vec3 obsSplinePos = m_currentSpline.GetPositionAt(obsParam);
        float obsForwardDist = (obsSplinePos - position).Length();
        if (obsForwardDist > AVOID_DETECT_DISTANCE + obstacle.halfLength)
            continue;
        Vec3 obsDir = m_currentSpline.GetDirectionAt(obsParam);
        Vec3 obsNormal = Vec3(-obsDir.GetZ(), 0.0f, obsDir.GetX()).Normalized();
        float lateral = (obstacle.center - obsSplinePos).Dot(obsNormal);
        biasSum += lateral;
        ++biasCount;
    }
    float preferredSign = (biasCount > 0 && biasSum != 0.0f) ? -std::copysign(1.0f, biasSum) : 1.0f;

    constexpr float NUDGE_STEP = 0.5f;
    constexpr float MAX_NUDGE = 2.5f; // 차선 폭 절반 남짓 -- 옆 차선까지 크게 침범하지 않는 한도
    for (float sign : {preferredSign, -preferredSign})
    {
        for (float magnitude = NUDGE_STEP; magnitude <= MAX_NUDGE; magnitude += NUDGE_STEP)
        {
            if (sweepClear(sign * magnitude))
            {
                m_avoidLateralOffset = sign * magnitude;
                return false; // 옆으로 피해서 그대로 일반주행 유지
            }
        }
    }

    // 옆으로도 못 피함 -> 하이브리드 A* 회피 기동으로 전환.
    m_avoidLateralOffset = 0.0f;
    BeginAvoidPlan();
    return true;
}

// 현재 pose에서, 현재 차선을 따라 장애물을 지나칠 만큼 앞선 지점까지 Hybrid A*로 회피 경로를 짜서
// 실행시킨다(RS pure pursuit로 천천히 주행 — BuildParkSegments 재사용). 성공하면 subMode를 D_Avoid로
// 세팅, 실패하면 D_Stop으로 세팅해 UpdateDrive의 쿨다운 재탐색 루프가 이어받게 한다.
void Car::BeginAvoidPlan()
{
    Vec3 startPos = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());
    HybridAStar::VehicleShape shape = BuildVehicleShape();
    const std::vector<HybridAStar::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();

    Vec3 goalPos = m_currentSpline.GetLookaheadPoint(startPos, AVOID_DETECT_DISTANCE);
    float goalParam = m_currentSpline.GetSplinePosition(goalPos);
    float goalAngleRad = DirectionToAngleRad(m_currentSpline.GetDirectionAt(goalParam));

    bool foundPath = false;
    ReedsShepp::Path path =
        HybridAStar::FindPath(startPos, startAngleRad, goalPos, goalAngleRad, obstacles, shape, foundPath);
    if (!foundPath)
    {
        DebugConsole::Log("BeginAvoidPlan: HybridA* failed to find an avoid path, staying in Stop substate");
        m_subMode = SubState::D_Stop;
        return;
    }

    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);
    m_subMode = SubState::D_Avoid;
    m_vehicleController.BeginPlan(BuildParkSegments(path, startPos, startAngleRad, turningRadius));
    RebuildParkDebugRender(path, startPos, startAngleRad, turningRadius, goalPos, goalAngleRad);
}

#pragma endregion
