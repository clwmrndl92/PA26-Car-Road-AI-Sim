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
    std::vector<std::unique_ptr<VehicleSegment>> BuildReedSheppSegments(const ReedsShepp::Path &path, const Vec3 &startPos,
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
    if (m_destLane == nullptr || m_currentLane != nullptr)
        return;

    Vec3 position = GetPosition();
    if (m_parkSpot != nullptr)
    {
        auto parkNode = m_RoadDataManager->GetNode(m_parkNodeId);
        if (parkNode != nullptr)
            position = parkNode->position;
    }

    SetCurrentLane(m_RoadDataManager->GetClosestLane(position));
    TryFindPathAndSetLane();
}

Car::Mode Car::DecideNextMode(const char **reason) const
{

    if (m_mode == Mode::Stop)
    {
        if (m_destLane == nullptr)
        {
            return Mode::Stop;
        }
        *reason = "go to Dest";
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
        if (m_subMode == SubMode::D_Avoid && !m_vehicleController.IsFinished())
        {
            *reason = "avoid maneuver in progress";
            return Mode::Drive;
        }

        constexpr float ARRIVE_DISTANCE = 5.0f;
        bool arrived = false;
        if (m_destLane != nullptr)
        {
            arrived = (m_destLane->GetEndPoint() - GetPosition()).Length() < ARRIVE_DISTANCE;
            if (m_pendingParkNode != nullptr)
            {
                arrived |= (m_pendingParkNode->position - GetPosition()).Length() < ARRIVE_DISTANCE;
            }
        }

        if (arrived && GetParkTargetNode() != nullptr)
        {
            *reason = "arrived at destination";
            return Mode::Park;
        }
        if (m_destLane == nullptr || arrived)
        {
            *reason = m_destLane == nullptr ? "no destination lane" : "arrived at destination";
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
    // TODO(주차 중 장애물 감시): fsm.txt는 상태:주차(출차) 전체에 박스캐스트 감지 -> 5초 대기 -> 이동 여부 판단 -> 재탐색 로직
    if (m_parkPlanPending)
    {
        // 완전히 정지할 때까지는 RS 계획을 세우지 않고 감속만 한다.
        if (m_speed > 0.0f)
        {
            Accelerate(0.0f);
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
        // 출차 완료: 이제 레인 위. 더 이상 이 주차칸에 있는 게 아니므로 예약을 풀고 비운다.
        m_parkSequenceActive = false; // 시퀀스 종료 — 다음 프레임 Drive로 전환 허용.
        if (m_parkSpot != nullptr)
        {
            m_RoadDataManager->ReleaseParkSpot(m_parkSpot->id);
            m_parkSpot = nullptr;
        }
        if (!TryFindPathAndSetLane())
            SetSubMode(SubMode::None);
        return;
    }

    if (m_parkSpot != nullptr)
    {
        // 입차 leg 1(-> 스플라인점 P)이 끝났으면, 이제 P에서 스팟까지 leg 2를 이어 계획한다. (주차레인
        // 없이 바로 스팟으로 간 경우엔 PlanEnterForCurrentSpot에서 이미 leg2로 세팅해서 건너뛴다.)
        if (m_subMode == SubMode::P_ENTER_LEG1)
        {
            // leg 1처럼 완전히 멈춘 뒤 그 pose에서 leg 2를 계획한다(open-loop RS는 시작 pose 기준). 대기 중
            // 컨트롤러가 finished여도 m_parkSequenceActive가 Park를 유지하므로 Drive로 새지 않는다.
            if (m_speed > 0.0f)
            {
                Accelerate(0.0f);
                return;
            }
            SetSubMode(SubMode::P_ENTER_LEG2);
            BeginParkSpotLeg();
            return;
        }

        // 스팟까지의 leg가 pure pursuit로 끝났는데 아직 정렬 보정을 안 했으면, 실제로 도착한 pose
        // 기준으로 같은 목표 pose까지 RS를 한 번 더 계획해 pure pursuit의 잔여 정렬 오차(주로 최종
        // 헤딩)를 없앤다. exact=true라 정지-조향-이동-정지로 정밀하게 실행돼 이번엔 추종 오차가
        // 남지 않는다. subMode를 먼저 ALIGN으로 세워 재귀적으로 반복되지 않게 한다 -- 이미 목표에
        // 정확히 있으면(혹은 장애물 등으로 경로를 못 찾으면) PlanParkLegTo가 false를 반환하고 그냥
        // 완료 처리로 넘어간다.
        if (m_subMode == SubMode::P_ENTER_LEG2)
        {
            if (m_speed > 0.0f)
            {
                Accelerate(0.0f);
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
        m_destLane = nullptr;
        SetCurrentLane(nullptr);
        return;
    }
}

void Car::GetLaneLookaheadPoint(const shared_ptr<Lane> &startLane, const Vec3 &position, float lookaheadDistance,
                                Vec3 &outPosition, float &outAngleRad) const
{
    shared_ptr<Lane> lane = startLane;
    const Spline *spline = &lane->GetSpline();
    float t = spline->GetSplinePosition(position);
    float remaining = lookaheadDistance;

    // successor를 최대 이 정도까지만 타고 넘어간다(사이클/데이터 이상으로 인한 무한루프 방지).
    constexpr int MAX_LANE_HOPS = 16;
    for (int hop = 0; hop < MAX_LANE_HOPS; ++hop)
    {
        float splineLength = spline->GetLength();
        float distanceToEnd = splineLength > 0.0f ? (1.0f - t) * splineLength : 0.0f;
        if (remaining <= distanceToEnd)
        {
            float targetT = splineLength > 0.0f ? std::clamp(t + remaining / splineLength, 0.0f, 1.0f) : 1.0f;
            outPosition = spline->GetPositionAt(targetT);
            outAngleRad = DirectionToAngleRad(spline->GetDirectionAt(targetT));
            return;
        }

        shared_ptr<Lane> next;
        for (const weak_ptr<Lane> &weakSucc : lane->GetSuccessors())
        {
            if ((next = weakSucc.lock()))
                break;
        }
        if (!next)
        {
            outPosition = spline->GetPositionAt(1.0f);
            outAngleRad = DirectionToAngleRad(spline->GetDirectionAt(1.0f));
            return;
        }

        remaining -= distanceToEnd;
        lane = next;
        spline = &lane->GetSpline();
        t = 0.0f;
    }

    outPosition = spline->GetPositionAt(1.0f);
    outAngleRad = DirectionToAngleRad(spline->GetDirectionAt(1.0f));
}

void Car::BeginParkPlan()
{
    int parkNodeId = -1;
    if (m_parkSpot == nullptr && m_pendingParkNode != nullptr)
    {
        parkNodeId = m_pendingParkNode->id;
        shared_ptr<RoadNode> pendingNode = m_pendingParkNode;
        m_parkSpot = m_RoadDataManager->TryReserveParkSpot(parkNodeId);
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
                m_destLane = nullptr;
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
        return;

    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);
    Vec3 rigidPosition = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());
    HybridAStar::VehicleShape shape = BuildVehicleShape();
    const std::vector<HybridAStar::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();

    // 출차
    if (m_subMode == SubMode::P_EXIT)
    {
        // CheckPath와 기준 맞추려 앞바퀴 위치로 레인 검색
        Vec3 frontPos = GetPosition();
        const std::vector<shared_ptr<Lane>> *parkingLanes =
            (m_parkNodeId >= 0) ? m_RoadDataManager->GetParkingLanes(m_parkNodeId) : nullptr;
        shared_ptr<Lane> closestLane = (parkingLanes != nullptr && !parkingLanes->empty())
                                           ? m_RoadDataManager->GetClosestParkLane(frontPos, m_parkNodeId)
                                           : m_RoadDataManager->GetClosestLane(frontPos);
        if (closestLane == nullptr)
        {
            DebugConsole::Log(GetName() + ": BeginParkPlan: no lane found to exit onto, abandoning this park attempt");
            m_destLane = nullptr;
            SetSubMode(SubMode::None);
            return;
        }

        float splinePos = closestLane->GetSpline().GetSplinePosition(frontPos);
        Vec3 closestDir = closestLane->GetSpline().GetDirectionAt(splinePos);
        SetCurrentLane(closestLane);

        // 이미 레인 진행 방향과 90도 이내로 정렬돼 있으면 RS 출차 매뉴버 없이 바로 주행(-> MOBIL 합류)으로 넘어간다.
        constexpr float EXIT_HEADING_ALIGN_ANGLE = ToRadians(90.0f);
        float headingDot = std::clamp(GetForwardAxis().Dot(closestDir), -1.0f, 1.0f);
        if (std::acos(headingDot) <= EXIT_HEADING_ALIGN_ANGLE)
        {
            m_vehicleController.BeginPlan({});
            return;
        }

        // 출차 목표: 레인 위, 현재 위치에서 조금 앞선 지점(lookahead)을 target으로 삼고 그 지점의 레인
        // 진행방향을 목표 heading으로 삼는다 — RS로 레인 위에 정렬해서 올라서면 이후 Drive의 일반 주행/
        // MOBIL이 알아서 합류를 이어받는다.
        constexpr float EXIT_LEAD_DISTANCE = 6.0f;
        Vec3 targetPos;
        float targetAngleRad;
        GetLaneLookaheadPoint(closestLane, rigidPosition, EXIT_LEAD_DISTANCE, targetPos, targetAngleRad);

        bool foundPath = false;
        ReedsShepp::Path path = HybridAStar::FindPath(rigidPosition, startAngleRad, targetPos, targetAngleRad, obstacles, shape, foundPath);
        if (!foundPath)
        {
            // 출차 실패는 이미 차가 그 자리를 점유 중이므로 예약을 풀지 않는다.
            DebugConsole::Log(GetName() + ": BeginParkPlan: HybridA* failed to find an exit path, abandoning this park attempt");
            m_destLane = nullptr;
            SetSubMode(SubMode::None);
            return;
        }
        m_vehicleController.BeginPlan(BuildReedSheppSegments(path, rigidPosition, startAngleRad, turningRadius));
        RebuildRSDebugRender(path, rigidPosition, startAngleRad, turningRadius, targetPos, targetAngleRad);
        return;
    }
    // 주차 (BeginParkPlan은 Park 진입시 딱 한 번만 불리므로 여기 오는 시점의 subMode는 항상 leg1)
    else if (m_subMode == SubMode::P_ENTER_LEG1)
    {
        if (!BeginParkEnterOrRetry())
        {
            DebugConsole::Log(GetName() + ": BeginParkPlan: no reachable ParkSpot for node " + std::to_string(parkNodeId) +
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
        // 주차레인의 방향은 출차 기준으로 잡아뒀으므로, 입차 시 P에서의 목표 heading은 그 반대다.
        float pAngleRad = DirectionToAngleRad(bestSpline->GetDirectionAt(pParam) * -1.0f);

        if (PlanParkLegTo(pPos, pAngleRad))
        {
            SetSubMode(SubMode::P_ENTER_LEG1); // leg 1 진행 중 — P 도착 후 UpdatePark가 leg 2를 잇는다.
            return true;
        }
        return false; // 이 스팟의 P까지 못 감 -> 다음 스팟
    }

    // 주차레인 없음 -> 스팟으로 직접
    Vec3 spotTarget = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
    float spotAngleRad = DirectionToAngleRad(m_parkSpot->direction);
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
    Vec3 rigidPosition = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());
    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);
    HybridAStar::VehicleShape shape = BuildVehicleShape();
    const std::vector<HybridAStar::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();

    bool foundPath = false;
    ReedsShepp::Path path = HybridAStar::FindPath(rigidPosition, startAngleRad, targetPos, targetAngleRad, obstacles, shape, foundPath);
    if (!foundPath)
        return false;

    if (exact)
        m_vehicleController.BeginPlan(BuildExactSegments(path, m_maxSteerAngle));
    else
        m_vehicleController.BeginPlan(BuildReedSheppSegments(path, rigidPosition, startAngleRad, turningRadius));
    RebuildRSDebugRender(path, rigidPosition, startAngleRad, turningRadius, targetPos, targetAngleRad);
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

    DebugConsole::Log(GetName() + ": BeginParkSpotLeg: can't tuck into ParkSpot " + std::to_string(m_parkSpot->id) +
                      " from P, trying next spot");
    // 이 자리 실패 -> 다음 빈 자리부터 leg 1부터 다시. PlanEnterForCurrentSpot이 새 자리에 맞는
    // subMode(leg1 또는 주차레인 없으면 leg2)를 알아서 세팅한다.
    if (ReserveNextParkSpot() && BeginParkEnterOrRetry())
        return;

    // 남은 자리 없음 -> 입차 종료(빈 플랜 -> 다음 프레임 UpdatePark 완료 처리, 현재 자리에 멈춤).
    DebugConsole::Log(GetName() + ": BeginParkSpotLeg: no reachable ParkSpot left, stopping");
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

    std::shared_ptr<RoadNode> dest = m_RoadDataManager->GetRandomDestNode();
    if (!dest)
        return;

    SetDestination(dest);
}
#pragma endregion

#pragma region Drive
void Car::UpdateDrive()
{
    // 서브상태:회피 — Hybrid A* + RS 경로
    // TODO(정차): 2초 대기 후 이동
    if (m_subMode == SubMode::D_Avoid)
    {
        if (!m_vehicleController.IsFinished())
        {
            m_wantSegmentTick = true;
            return;
        }
        // 회피 RS 경로 완주 — m_mode가 계속 Drive라 OnModeEnter(Drive)가 다시 안 불리므로, 예전에
        // 거기서 하던 일반주행용 SplineFollowSegment 재구성을 여기서 직접 해준다.
        SetSubMode(SubMode::D_Normal);
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        segments.push_back(std::make_unique<SplineFollowSegment>());
        m_vehicleController.BeginPlan(std::move(segments));
    }
    else if (m_subMode == SubMode::D_Stop)
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
    TryLaneChange();
    // if (TryAvoidObstacle())
    //     return;
    m_obstacleAheadGap = ScanBBoxObstacleGap();
    m_wantSegmentTick = true;
}

bool Car::TryLaneChange(bool ignoreCooldown)
{
    if (m_currentLane == nullptr)
        return false;

    Vec3 position = GetPosition();
    // position은 "도로를 따라간 거리"라는 하나의 좌표계로 취급한다 — 좌/우 인접 레인은 같은 도로의
    // 평행한 차선이라 서로 같은 좌표계를 공유한다고 근사한다 (레인마다 다시 투영하지 않음).
    float egoPosition = m_currentLane->GetSpline().GetSplinePosition(position) * m_currentLane->GetLength();

    // 연결된 차선까지 넓혀 찾는다(FindGraphNeighbors) -- 결과 position은 egoPosition 기준 상대 거리(u)로
    // 나오므로, 아래 ego/virtualLeader도 전부 그 좌표계(ego=0)로 맞춘다.
    LaneNeighbor egoLeaderN, oldFollowerN;
    FindGraphNeighbors(m_currentLane, egoPosition, egoLeaderN, oldFollowerN);

    float v0 = std::min(m_maxSpeed, m_currentLane->GetLimitSpeed());
    CarFollowing::Params cfParams = BuildIdmParams(v0);
    Mobil::Params mobilParams{MOBIL_SAFE_DECEL, MOBIL_POLITENESS, MOBIL_THRESHOLD};

    constexpr float VIRTUAL_LEADER_GAP = 100000.0f; // 실제 리더가 없을 때(뚫린 도로) 쓰는 가상 리더 거리
    auto virtualLeader = [&](float laneLimitSpeed)
    { return Mobil::VehicleState{laneLimitSpeed, 0.0f, VIRTUAL_LEADER_GAP, 0.0f}; };

    Mobil::VehicleState ego{m_speed, m_acceleration, 0.0f, GetLength()};
    Mobil::VehicleState egoLeaderState = (egoLeaderN.car != nullptr) ? ToVehicleState(egoLeaderN) : virtualLeader(v0);
    Mobil::VehicleState oldFollowerStorage;
    const Mobil::VehicleState *oldFollowerState = nullptr;
    if (oldFollowerN.car != nullptr)
    {
        oldFollowerStorage = ToVehicleState(oldFollowerN);
        oldFollowerState = &oldFollowerStorage;
    }

    // candidate로 실제 전환을 수행. mandatory=true면 유인 기준 없이 안전 기준만 통과하면 되고(라우팅
    // 강제 변경), false면 기존 MOBIL 안전+유인 기준을 그대로 쓴다(임의 추월성 변경).
    auto commitTo = [&](const shared_ptr<Lane> candidate, bool mandatory) -> bool
    {
        if (candidate == nullptr)
            return false;

        LaneNeighbor newLeaderN, newFollowerN;
        FindGraphNeighbors(candidate, egoPosition, newLeaderN, newFollowerN);

        float candidateV0 = std::min(m_maxSpeed, candidate->GetLimitSpeed());
        Mobil::VehicleState newLeaderState = (newLeaderN.car != nullptr) ? ToVehicleState(newLeaderN) : virtualLeader(candidateV0);
        Mobil::VehicleState newFollowerStorage;
        const Mobil::VehicleState *newFollowerState = nullptr;
        if (newFollowerN.car != nullptr)
        {
            newFollowerStorage = ToVehicleState(newFollowerN);
            newFollowerState = &newFollowerStorage;
        }

        bool approved = mandatory
                            ? Mobil::IsSafeLaneChange(ego, newFollowerState, mobilParams, cfParams)
                            : Mobil::EvaluateLaneChange(ego, oldFollowerState, egoLeaderState, newLeaderState,
                                                        newFollowerState, mobilParams, cfParams);
        if (!approved)
            return false;

        // 목적지 도달 가능성 확인 없이 그냥 옮기면, 이 레인이 목적지로 못 가는 레인일 때 경로를 잃는다.
        vector<LaneStep> newPath = m_RoadDataManager->FindPath(candidate, m_destLane);
        if (newPath.empty())
            return false; // 이 후보로는 목적지에 못 감

        m_path = std::move(newPath);
        m_pathIndex = 0;
        SetCurrentLane(candidate);
        m_currentSpline = candidate->GetSpline();
        RescanRoadSpeedConstraints();
        m_lastLaneChangeTime = m_currentTime;
        DebugConsole::Log(GetName() + ": " + (mandatory ? "Mandatory" : "MOBIL") + " lane change -> lane " +
                          std::to_string(candidate->GetId()));
        return true;
    };

    // 다음 경로 스텝이 차선변경이면 안전 기준만 확인
    if (m_pathIndex + 1 < m_path.size() && m_path[m_pathIndex + 1].isLaneChange)
    {
        float laneStartDistance = (m_currentLane->GetStartPoint() - position).Length();
        if (laneStartDistance <= LANE_ENTRY_THRESHOLD)
            return false;
        return commitTo(m_path[m_pathIndex + 1].lane, /*mandatory=*/true);
    }

    if (!ignoreCooldown && m_currentTime - m_lastLaneChangeTime < MOBIL_EVAL_INTERVAL)
        return false;

    // 평가했다는 사실 자체로 다음 평가까지 쿨다운(성공/실패 무관 — 왕복 진동 방지).
    m_lastLaneChangeTime = m_currentTime;

    for (const weak_ptr<Lane> &candidateWeak : {m_currentLane->GetLeft(), m_currentLane->GetRight()})
    {
        if (commitTo(candidateWeak.lock(), /*mandatory=*/false))
            return true;
    }

    return false;
}

bool Car::CheckPath()
{
    if (m_currentLane == nullptr)
    {
        m_destLane = nullptr;
        return false;
    }

    // path find
    Vec3 position = GetPosition();

    // 현재 레인의 끝에 다가가면 경로상 다음 레인으로 넘어간다.
    Vec3 projectedPosition = m_currentSpline.GetLookaheadPoint(position, 0.0f);
    float laneEndDistance = (m_currentLane->GetEndPoint() - projectedPosition).Length();
    bool enteredByLaneChange = false;
    while (laneEndDistance < LANE_TRANSITION_THRESHOLD)
    {
        // 신호로 서야 하면 레인을 안 넘긴다
        if (ShouldStopForSignal(m_currentLane))
            break;

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
        projectedPosition = m_currentSpline.GetLookaheadPoint(position, 0.0f);
        laneEndDistance = (m_currentLane->GetEndPoint() - projectedPosition).Length();
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

float Car::ComputeLookaheadDistance() const
{
    float minSafeLookahead = 2.0f * m_wheelbase / tanf(m_maxSteerAngle);
    constexpr float LOOKAHEAD_TIME = 1.0f; // 몇 초 앞을 볼지
    return std::max(minSafeLookahead, m_speed * LOOKAHEAD_TIME);
}

void Car::DriveControl()
{
    Vec3 rigidPosition = GetRigidbodyPosition();
    float lookaheadDistance = ComputeLookaheadDistance();
    Vec3 targetPosition, targetDir;
    GetLookaheadPose(&m_currentSpline, m_currentLane, m_pathIndex, rigidPosition, lookaheadDistance, targetPosition, targetDir);
    // Reynolds식 측면 회피
    if (m_avoidLateralOffset != 0.0f)
    {
        Vec3 targetNormal = Vec3(-targetDir.GetZ(), 0.0f, targetDir.GetX()).Normalized();
        targetPosition = targetPosition + targetNormal * m_avoidLateralOffset;
    }
    float targetSteer = PurePursuit(targetPosition);
    Steer(targetSteer);

    // speed control
    float steerSpeedCap = CalcMaxSpeed(targetSteer);
    DriveSpeedIDM(steerSpeedCap);

    // Debug
    DirectX::XMFLOAT3 targetMarkerPos = ToXMFLOAT3(targetPosition);
    targetMarkerPos.y = 0.2f;
    m_targetMarker.GetTransform().SetPosition(targetMarkerPos);
}

void Car::GetLookaheadPose(const Spline *startSpline, const shared_ptr<Lane> &startLane, size_t startPathIndex,
                           const Vec3 &fromPosition, float distance, Vec3 &outPosition, Vec3 &outDirection) const
{
    const Spline *spline = startSpline;
    shared_ptr<Lane> segmentLane = startLane;
    size_t pathIndex = startPathIndex;
    Vec3 segmentStart = fromPosition;
    float remainingDistance = distance;

    while (true)
    {
        float startT = spline->GetSplinePosition(segmentStart);
        float splineLength = spline->GetLength();
        float segmentDistance = splineLength > 0.0f ? (1.0f - startT) * splineLength : 0.0f;

        if (remainingDistance <= segmentDistance)
        {
            outPosition = spline->GetLookaheadPoint(segmentStart, remainingDistance);
            outDirection = spline->GetDirectionAt(spline->GetSplinePosition(outPosition));
            return;
        }

        remainingDistance -= segmentDistance;

        shared_ptr<Lane> nextLane = (pathIndex + 1 < m_path.size()) ? m_path[pathIndex + 1].lane : nullptr;
        bool isNextLaneCurved = nextLane && nextLane->GetSpline().GetMinRadiusAhead(0.0f, 1.0f) < std::numeric_limits<float>::max();
        float nextLaneRamain = (fromPosition - segmentLane->GetEndPoint()).Length();
        if (!nextLane || (isNextLaneCurved && nextLaneRamain >= LANE_TRANSITION_THRESHOLD * 1.5f))
        {
            // 경로가 여기서 끝남 -- 기존 스플라인 클램프와 동일하게 마지막 레인 끝점에 멈춘다.
            outPosition = segmentLane->GetEndPoint();
            outDirection = spline->GetDirectionAt(spline->GetSplinePosition(outPosition));
            return;
        }

        segmentLane = nextLane;
        segmentStart = nextLane->GetStartPoint();
        spline = &nextLane->GetSpline();
        ++pathIndex;
    }
}

void Car::SimulateBBoxTrajectory(float lateralOffset, std::vector<Vec3> &outPositions,
                                 std::vector<Vec3> &outDirections) const
{
    outPositions.clear();
    outDirections.clear();

    Vec3 rigidPos = m_rigidbody.GetPosition();
    Vec3 dir = GetForwardAxis();
    float lookaheadDistance = ComputeLookaheadDistance();

    const Spline *posSpline = &m_currentSpline; // pos가 지금 어느 레인 위에 있는지 추적하는 커서
    shared_ptr<Lane> posLane = m_currentLane;
    size_t posPathIndex = m_pathIndex;

    constexpr float TRAJECTORY_SUBSTEP = 0.25f; // 오버슈트 방지용 적분 스텝

    for (float traveled = 0.0f; traveled <= AVOID_DETECT_DISTANCE; traveled += AVOID_SAMPLE_STEP)
    {
        outPositions.push_back(rigidPos);
        outDirections.push_back(dir);

        float remaining = AVOID_SAMPLE_STEP;
        while (remaining > 0.0f)
        {
            float step = std::min(TRAJECTORY_SUBSTEP, remaining);

            // pos가 posSpline 끝에 사실상 도달했으면(가장 가까운 점이 마지막 샘플) 경로상 다음 레인으로
            // 커서를 옮긴다 -- ScanRoadSpeedConstraints/GetLookaheadPose와 같은 판정 방식.
            while (posSpline->GetLength() > 0.0f &&
                   (1.0f - posSpline->GetSplinePosition(rigidPos)) * posSpline->GetLength() < TRAJECTORY_SUBSTEP * 0.5f)
            {
                shared_ptr<Lane> nextLane = (posPathIndex + 1 < m_path.size()) ? m_path[posPathIndex + 1].lane : nullptr;
                if (!nextLane)
                    break; // 경로 끝 -- 이 레인 기준으로 계속(기존 클램프 동작과 동일)
                posLane = nextLane;
                posSpline = &nextLane->GetSpline();
                ++posPathIndex;
            }

            // DriveControl과 동일한 조준점(+lateralOffset) -- pos가 속한 레인(posSpline 등)부터 걸어서
            // 찾는다. "지금 시뮬레이션 중인" pos 기준으로 매 서브스텝 다시 구하는 건 실제 주행도 매
            // 프레임 그 시점 위치에서 다시 구하는 것과 동일하다.
            Vec3 targetPos, targetDir;
            GetLookaheadPose(posSpline, posLane, posPathIndex, rigidPos, lookaheadDistance, targetPos, targetDir);
            if (lateralOffset != 0.0f)
            {
                Vec3 targetNormal = Vec3(-targetDir.GetZ(), 0.0f, targetDir.GetX()).Normalized();
                targetPos = targetPos + targetNormal * lateralOffset;
            }

            // Car::PurePursuit와 동일한 조향각 공식을, atan2(z,x) 부호 규약의 signed heading error로
            // 다시 쓴 것 -- 결과는 같지만 carRight 없이 dir/toTarget만으로 부호까지 바로 나온다.
            Vec3 toTarget = targetPos - rigidPos;
            float distance = toTarget.Length();
            float steerAngle = 0.0f;
            if (distance > 0.001f)
            {
                float headingError = atan2f(dir.GetX() * toTarget.GetZ() - dir.GetZ() * toTarget.GetX(), dir.Dot(toTarget));
                steerAngle = std::clamp(-atanf(2.0f * m_wheelbase * sinf(headingError) / distance),
                                        -m_maxSteerAngle, m_maxSteerAngle);
            }

            // ApplyMotion의 자전거 모델과 동일: angularVelocity = speed*tan(steerAngle)/wheelbase, 부호는
            // 위 steerAngle 유도와 마찬가지로 atan2(z,x) 각도 기준으로 뒤집힌다(HybridAStar::StepPose의
            // Left/kappa 부호 규약과 교차검증됨).
            float currentAngle = atan2f(dir.GetZ(), dir.GetX());
            float nextAngle = currentAngle - step * tanf(steerAngle) / m_wheelbase;
            rigidPos = rigidPos + dir * step;
            dir = Vec3(cosf(nextAngle), 0.0f, sinf(nextAngle));

            remaining -= step;
        }
    }
}

bool Car::TryAvoidObstacle()
{
    const std::vector<HybridAStar::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();
    HybridAStar::VehicleShape shape = BuildVehicleShape();
    if (obstacles.empty())
    {
        m_avoidLateralOffset = 0.0f;
        m_obstacleAheadGap = -1.0f;
        m_bboxDebugRenders.clear();
        return false;
    }

    // lateralOffset 궤적을 시뮬레이션해서 처음 충돌하는 샘플의 인덱스를 반환(없으면 -1).
    auto findCollision = [&](float lateralOffset, std::vector<Vec3> &positions, std::vector<Vec3> &directions)
    {
        SimulateBBoxTrajectory(lateralOffset, positions, directions);
        for (size_t i = 0; i < positions.size(); ++i)
        {
            if (HybridAStar::IsColliding(positions[i], DirectionToAngleRad(directions[i]), obstacles, shape))
                return static_cast<int>(i);
        }
        return -1;
    };
    auto sweepClear = [&](float lateralOffset)
    {
        std::vector<Vec3> positions, directions;
        return findCollision(lateralOffset, positions, directions) < 0;
    };

    std::vector<Vec3> basePositions, baseDirections;
    int collideIndex = findCollision(0.0f, basePositions, baseDirections);
    RebuildBBDebugRender(basePositions, baseDirections, obstacles, shape); // 디버그: 충돌=빨강/통과=초록 박스

    if (collideIndex < 0)
    {
        m_avoidLateralOffset = 0.0f;
        m_obstacleAheadGap = -1.0f;
        return false; // 코리도어 궤적 전체가 깨끗함
    }
    float collisionDistance = collideIndex * AVOID_SAMPLE_STEP;

    // 1) 제동거리 기준 "너무 가까움" — 비상 제동(m_maxEmergBrake)으로도 못 설 거리 안이면 감속/옆으로
    // 피할 여유 없이 바로 재계획.
    float stoppingDistance = (m_speed * m_speed) / (2.0f * m_maxEmergBrake);
    float closeThreshold = stoppingDistance * AVOID_CLOSE_DISTANCE_MARGIN;
    if (collisionDistance <= closeThreshold)
    {
        m_avoidLateralOffset = 0.0f;
        m_obstacleAheadGap = -1.0f;
        BeginAvoidPlan();
        return true;
    }

    // 2) 아직 여유 있음 -- 충돌 지점까지의 거리를 정지한 가상 선행차량 gap으로 노출, DriveSpeedIDM이 감속.
    m_obstacleAheadGap = collisionDistance;

    // 어느 쪽으로 옆 오프셋을 먼저 시도할지: 충돌 지점 근처(차량 크기 범위 안)에 있는 장애물들이 그
    // 지점 기준 좌/우 어느 쪽에 몰려 있는지로 정한다 — 충돌과 무관한 먼 장애물은 편향에 안 섞이게, 충돌
    // 지점의 실제 heading 기준으로 판단한다.
    Vec3 collidePos = basePositions[collideIndex];
    Vec3 collideDir = baseDirections[collideIndex];
    Vec3 collideNormal = Vec3(-collideDir.GetZ(), 0.0f, collideDir.GetX()).Normalized();
    float biasSum = 0.0f;
    int biasCount = 0;
    for (const HybridAStar::Obstacle &obstacle : obstacles)
    {
        float proximity = shape.halfLength + obstacle.halfLength + shape.halfWidth + obstacle.halfWidth;
        if ((obstacle.center - collidePos).Length() > proximity)
            continue;
        biasSum += (obstacle.center - collidePos).Dot(collideNormal);
        ++biasCount;
    }
    float preferredSign = (biasCount > 0 && biasSum != 0.0f) ? -std::copysign(1.0f, biasSum) : 1.0f;

    constexpr float NUDGE_STEP = 0.5f;
    constexpr float MAX_NUDGE = 2.5f; // 차선 폭 절반 남짓 -- 옆 차선까지 크게 침범하지 않는 한도
    bool offsetFound = false;
    for (float sign : {preferredSign, -preferredSign})
    {
        for (float magnitude = NUDGE_STEP; magnitude <= MAX_NUDGE; magnitude += NUDGE_STEP)
        {
            if (sweepClear(sign * magnitude))
            {
                offsetFound = true;
                break;
            }
        }
        if (offsetFound)
            break;
    }

    // 옆 오프셋이 있고 + MOBIL도 차선변경을 승인해야만 회피기동으로 인정한다(쿨다운 무시하고 즉시
    // 평가). 성공하면 TryLaneChange가 이미 레인/스플라인을 새 것으로 바꿔놨으므로, 옛 스플라인 기준
    // 오프셋은 더 의미가 없어 0으로 둔다.
    if (offsetFound && TryLaneChange(/*ignoreCooldown=*/true))
    {
        m_avoidLateralOffset = 0.0f;
        return false;
    }

    // 옆으로도 못 피하거나 MOBIL이 승인하지 않음 -> 정지 후 하이브리드 A*+RS 회피 기동으로 전환.
    m_avoidLateralOffset = 0.0f;
    m_obstacleAheadGap = -1.0f;
    BeginAvoidPlan();
    return true;
}

// TryAvoidObstacle과 달리 회피 기동(옆 오프셋 탐색, BeginAvoidPlan)은 전혀 건드리지 않고 SimulateBBoxTrajectory
// 전방 sweep으로 정적+동적(주변 차량) 장애물까지의 최근접 거리만 반환한다(없으면 -1). DriveSpeedIDM이
// 이 값을 정지한 가상 리더의 gap으로 사용.
float Car::ScanBBoxObstacleGap()
{
    if (m_currentLane == nullptr)
    {
        m_bboxDebugRenders.clear();
        return -1.0f;
    }

    std::vector<HybridAStar::Obstacle> obstacles = m_RoadDataManager->GetObstacles();

    Vec3 position = GetPosition();
    float egoLanePos = m_currentLane->GetSpline().GetSplinePosition(position) * m_currentLane->GetLength();

    for (Car *other : CollectNearbyCars(m_currentLane, egoLanePos))
    {
        // BuildVehicleShape의 pivotToCenter 컨벤션과 동일하게, "pivot"(rigidbody 위치)에서 전방으로
        // pivotToCenter만큼 떨어진 지점을 실제 충돌판정 박스 중심으로 쓴다.
        HybridAStar::VehicleShape otherShape = other->BuildVehicleShape();
        Vec3 otherPivot = other->GetRigidbodyPosition();
        Vec3 otherFwd = other->GetForwardAxis();
        HybridAStar::Obstacle obstacle;
        obstacle.center = otherPivot + otherFwd * otherShape.pivotToCenter;
        obstacle.halfLength = otherShape.halfLength;
        obstacle.halfWidth = otherShape.halfWidth;
        obstacle.headingRad = DirectionToAngleRad(otherFwd);
        obstacles.push_back(obstacle);
    }

    if (obstacles.empty())
    {
        m_bboxDebugRenders.clear();
        return -1.0f;
    }

    HybridAStar::VehicleShape shape = BuildVehicleShape();
    std::vector<Vec3> positions, directions;
    SimulateBBoxTrajectory(0.0f, positions, directions);
    RebuildBBDebugRender(positions, directions, obstacles, shape); // 디버그: 충돌=빨강/통과=초록 박스 (TryAvoidObstacle과 동일)

    for (size_t i = 0; i < positions.size(); ++i)
    {
        if (HybridAStar::IsColliding(positions[i], DirectionToAngleRad(directions[i]), obstacles, shape))
            return static_cast<float>(i) * AVOID_SAMPLE_STEP;
    }
    return -1.0f;
}

// 현재 pose에서, 현재 차선을 따라 장애물을 지나칠 만큼 앞선 지점까지 Hybrid A*로 회피 경로를 짜서
// 실행시킨다(RS pure pursuit로 천천히 주행 — BuildParkSegments 재사용). 성공하면 subMode를 D_Avoid로
// 세팅, 실패하면 D_Stop으로 세팅해 UpdateDrive의 쿨다운 재탐색 루프가 이어받게 한다.
void Car::BeginAvoidPlan()
{
    EmergBrake(); // 회피 재계획에 들어왔다는 것 자체가 이미 위험 신호 -- 경로 탐색/실행 전에 즉시 감속부터 시작.

    Vec3 rigidPosition = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());
    HybridAStar::VehicleShape shape = BuildVehicleShape();
    const std::vector<HybridAStar::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();

    Vec3 goalPos = m_currentSpline.GetLookaheadPoint(rigidPosition, AVOID_DETECT_DISTANCE);
    float goalParam = m_currentSpline.GetSplinePosition(goalPos);
    float goalAngleRad = DirectionToAngleRad(m_currentSpline.GetDirectionAt(goalParam));

    bool foundPath = false;
    ReedsShepp::Path path =
        HybridAStar::FindPath(rigidPosition, startAngleRad, goalPos, goalAngleRad, obstacles, shape, foundPath);
    if (!foundPath)
    {
        DebugConsole::Log(GetName() + ": BeginAvoidPlan: HybridA* failed to find an avoid path, staying in Stop substate");
        SetSubMode(SubMode::D_Stop);
        return;
    }

    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);
    SetSubMode(SubMode::D_Avoid);
    m_vehicleController.BeginPlan(BuildReedSheppSegments(path, rigidPosition, startAngleRad, turningRadius));
    RebuildRSDebugRender(path, rigidPosition, startAngleRad, turningRadius, goalPos, goalAngleRad);
}

#pragma endregion
