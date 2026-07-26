#include "Car.h"
#include "VehicleSegment.h"
#include "Utill/DebugConsole.h"
#include "Nav/ReedsShepp.h"
#include "Nav/VehicleCollision.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
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
        // 출발은 항상 Park(P_EXIT)의 출차 판단을 거친다: 레인 방향과 정렬돼 있으면(도로 위 정차)
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
        bool arrived = false;
        if (m_destLane != nullptr)
        {
            // 직선거리(현재 위치 <-> 끝점)로 재면, 이 속도의 최소회전반경(wheelbase/tan(maxSteerAngle))이
            // ARRIVE_DISTANCE보다 큰 경우 차가 끝점 주변을 궤도처럼 돌면서 영원히 이 거리 안으로 못 들어올
            // 수 있다. 대신 destLane 스플라인 위로 현재 위치를 투영한 지점 기준 "남은 경로 거리"로 재면,
            // 차가 목적지 주변 어디에 있든(옆으로 벗어나 돌고 있어도) 경로상 끝에 가까우면 도착으로 잡힌다.
            Vec3 projectedPosition = m_destLane->GetSpline().GetLookaheadPoint(GetPosition(), 0.0f);
            arrived = (m_destLane->GetEndPoint() - projectedPosition).Length() < ARRIVE_DISTANCE;
            if (m_pendingParkNode != nullptr)
            {
                // 주차 노드는 넓은 반경으로: 노드 "옆을 지나는" 순간 잡아야 그 자리에서 짧은
                // RS로 입차한다 (레인 끝까지 가면 멀리서 RS 직행을 하게 됨).
                arrived |= (m_pendingParkNode->position - GetPosition()).Length() < PARK_ARRIVE_DISTANCE;
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
        m_emergencyBrake = false; // 직전 Drive의 비상 상태가 새 주행에 새지 않게 리셋
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

    return true;
}
#pragma endregion

#pragma region Park

void Car::UpdatePark()
{
    // TODO(주차 중 장애물 감시): fsm.txt는 상태:주차(출차) 전체에 박스캐스트 감지 -> 5초 대기 -> 이동 여부 판단 -> 재탐색
    // 로직인데, 아래 IsParkObstacleAhead로 "감지되면 정지"까지만 구현했다. 5초 대기/이동판단/재탐색 단계는 아직 없어서
    // 장애물이 안 사라지면 계속 정지만 유지한다.
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
        // 안 지우면 다음 도로 정차 후 출발(P_EXIT) 때 BeginParkPlan이 이 주차장의 주차레인을
        // 검색해(GetClosestParkLane) 멀리 떨어진 레인으로 붙어버린다.
        m_parkNodeId = -1;

        // 출차가 주차레인 위에서 끝났으면 FindPath(주차레인 -> 메인망 destLane)는 분리망이라
        // 반드시 실패한다. 이때 "가장 가까운 메인 레인"으로 점프하면 벽 너머 레인이 잡혀 전진
        // 후보가 전부 충돌(safe=N)로 동결될 수 있으므로, 대신 지금 레인(주차레인)을 경로 없이
        // 그대로 따라 로트 출구(레인 끝)까지 주행한다 -- 끝에 도달하면 CheckPath가 목적지를 비워
        // Stop이 되고, 그 자리(출구 근처)에서의 다음 출발은 메인 레인에 정상적으로 붙는다.
        // (TryFindPathAndSetLane이 실패 시 currentLane/destLane을 지우므로 보존해뒀다 복원.)
        shared_ptr<Lane> savedDestLane = m_destLane;
        shared_ptr<Lane> exitLane = m_currentLane;
        if (!TryFindPathAndSetLane())
        {
            m_destLane = savedDestLane;
            SetCurrentLane(exitLane);
            m_path.clear();
            m_pathIndex = 0;
        }
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


void Car::BeginParkPlan()
{
    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);
    Vec3 rigidPosition = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());

    // 출차 -- 아래 입차용 예약 블록보다 반드시 먼저, m_parkSpot 유무와 무관하게 처리한다.
    // 예약 블록을 먼저 타면 "이번 출발의 목적지"가 주차장일 때 m_pendingParkNode(도착지!)의 스팟을
    // 지금 예약하고 m_parkNodeId를 도착지 주차장으로 바꿔, 아래에서 도착지 주차장의 주차레인을
    // 현재 레인으로 잡아버린다(출발 불능 루프).
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

        // 레인 진행 방향과 충분히 정렬돼 있으면 RS 출차 매뉴버 없이 바로 주행. 90°로 잡으면 수직
        // 주차 상태(정확히 90°)가 "정렬됨"으로 새서 매뉴버 없이 주차칸에 박힌 채 출차 완료 처리된다.
        constexpr float EXIT_HEADING_ALIGN_ANGLE = ToRadians(60.0f);
        float headingDot = std::clamp(GetForwardAxis().Dot(closestDir), -1.0f, 1.0f);
        if (std::acos(headingDot) <= EXIT_HEADING_ALIGN_ANGLE)
        {
            m_vehicleController.BeginPlan({});
            return;
        }

        // 출차 목표: 내 투영점에서 레인을 따라 EXIT_LEAD_DISTANCE 앞의 pose. heading은 그 지점의
        // 레인 진행방향(주차레인 방향은 출차 기준으로 저장돼 있음). 출차 레인은 m_path에 없으므로
        // GetLookaheadPose(경로 워킹) 대신 이 레인 스플라인에서 직접 구한다.
        // 짧은 목표는 턴어라운드 공간이 부족해 RS 후보가 전부 장애물에 클립될 수 있으므로,
        // 점점 먼 목표(=더 긴 활주 공간)로 재시도한다.
        VehicleCollision::VehicleShape shape = BuildVehicleShape();
        const std::vector<VehicleCollision::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();
        auto isCollisionFree = [&](const ReedsShepp::Path &candidate)
        {
            for (const ReedsShepp::PoseSample &pose : ReedsShepp::SamplePoses(candidate, rigidPosition, startAngleRad, turningRadius))
            {
                if (VehicleCollision::IsColliding(pose.position, pose.headingRad, obstacles, shape))
                    return false;
            }
            return true;
        };

        const Spline &exitSpline = closestLane->GetSpline();
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
            m_destLane = nullptr;
            m_parkSequenceActive = false;
        }
    }
}

// 이 Park의 주차레인들 중 m_parkSpot에 가장 가까운 스플라인. 주차레인이 없으면 nullptr.
const Spline *Car::FindBestParkingSpline() const
{
    const std::vector<shared_ptr<Lane>> *parkingLanes =
        (m_parkNodeId >= 0) ? m_RoadDataManager->GetParkingLanes(m_parkNodeId) : nullptr;
    if (parkingLanes == nullptr || parkingLanes->empty() || m_parkSpot == nullptr)
        return nullptr;

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
    return bestSpline;
}

// P(스팟 앞 지점) pose: 주차레인 위 스팟 최근접점에서 스플라인을 따라 P_LEAD_DISTANCE만큼 물러난 점.
// 주차레인의 방향은 출차 기준으로 잡아뒀으므로, 입차 시 P에서의 목표 heading은 그 반대다.
bool Car::ComputeParkPrePose(Vec3 &outPos, float &outAngleRad) const
{
    const Spline *bestSpline = FindBestParkingSpline();
    if (bestSpline == nullptr)
        return false;

    constexpr float P_LEAD_DISTANCE = 3.0f;
    outPos = bestSpline->GetLookaheadPoint(m_parkSpot->position, -P_LEAD_DISTANCE);
    float pParam = bestSpline->GetSplinePosition(outPos);
    outAngleRad = DirectionToAngleRad(bestSpline->GetDirectionAt(pParam) * -1.0f);
    return true;
}

// m_parkSpot로의 입차 시작 계획: 주차레인이 있으면 leg 1(-> 스팟 앞 P), 없으면 스팟으로 직접(leg 2 없음).
// P까지 한 번에 RS가 안 나오면(진입로가 벽으로 막힌 pose 등 -- RS는 장애물을 "피해가는" 게 아니라
// 후보를 "거부"만 하므로 좁은 입구에서 전멸할 수 있다) 주차레인에서 내 위치와 가장 가까운 점(=진입점)
// 으로 먼저 가는 중간 leg를 계획한다. leg 완료 후 UpdatePark가 다시 P를 겨냥한다(leg 체인).
// 계획을 시작했으면 true(+ m_subMode를 해당 leg로 설정), 이 자리로는 경로를 못 찾으면 false.
bool Car::PlanEnterForCurrentSpot()
{
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
        const Spline *bestSpline = FindBestParkingSpline();
        Vec3 rigidPosition = m_rigidbody.GetPosition();
        float entryT = bestSpline->GetSplinePosition(rigidPosition);
        Vec3 entryPos = bestSpline->GetPositionAt(entryT);
        if ((entryPos - rigidPosition).Length() > ENTRY_MIN_DISTANCE)
        {
            float entryAngleRad = DirectionToAngleRad(bestSpline->GetDirectionAt(entryT) * -1.0f); // 입차 방향(레인 역방향)
            if (PlanParkLegTo(entryPos, entryAngleRad))
            {
                DebugConsole::Log(GetName() + ": PlanEnterForCurrentSpot: P unreachable, detouring via lane entry point");
                SetSubMode(SubMode::P_ENTER_LEG1);
                return true;
            }
        }
        return false; // 이 스팟의 P까지 못 감 -> 다음 스팟
    }

    // 주차레인 없음 -> 스팟으로 직접. 단 RS는 도로를 무시한 직행 매뉴버이므로, 도착 판정이
    // 노드에서 먼 곳에서 났다면(멀리 있는 레인 끝 등) 맵을 가로지르는 긴 직행이 나온다 --
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
        m_parkLegTries = 0; // 새 스팟 -- 진입 leg 체인 시도 횟수 리셋
        if (PlanEnterForCurrentSpot())
            return true;
        if (!ReserveNextParkSpot())
            return false;
    }
    return false;
}

// 현재 pose -> target 까지 Reeds-Shepp으로 계획해 실행시킨다. 못 찾으면 false.
// exact=true면 Pure Pursuit(BuildParkSegments) 대신 정지-조향-이동-정지 방식의 정밀 실행
// (BuildExactSegments)을 쓴다 -- 짧은 최종 정렬 보정 등 추종 오차를 남기면 안 되는 경우에만 켠다.
bool Car::PlanParkLegTo(const Vec3 &targetPos, float targetAngleRad, bool exact)
{
    Vec3 rigidPosition = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());
    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);

    VehicleCollision::VehicleShape shape = BuildVehicleShape();
    const std::vector<VehicleCollision::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();
    auto isCollisionFree = [&](const ReedsShepp::Path &candidate)
    {
        for (const ReedsShepp::PoseSample &pose : ReedsShepp::SamplePoses(candidate, rigidPosition, startAngleRad, turningRadius))
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
    // Drive에서 "도착"으로 넘어왔으면 그 목적지는 여기서 소모한다(m_destLane 해제). 안 지우면
    // destLane이 남아 다음 프레임 DecideNextMode(Stop)가 바로 Drive를 돌려주고 Drive는 다시
    // "도착"으로 Stop을 돌려줘 매 프레임 Stop<->Drive 진동이 생긴다 -- 그동안 제동(Stop 틱)과
    // 유지(Drive 틱)가 번갈아 걸려 목적지 앞에서 기어가기만 하고 멈추지 못한다.
    if (m_destLane != nullptr)
    {
        Vec3 projectedPosition = m_destLane->GetSpline().GetLookaheadPoint(GetPosition(), 0.0f);
        bool arrived = (m_destLane->GetEndPoint() - projectedPosition).Length() < ARRIVE_DISTANCE;
        if (m_pendingParkNode != nullptr)
            arrived |= (m_pendingParkNode->position - GetPosition()).Length() < PARK_ARRIVE_DISTANCE; // DecideNextMode와 동일 기준
        if (arrived)
            m_destLane = nullptr;
    }

    // Park에서 넘어온 직후면 조향 원복 세그먼트가 아직 안 끝났을 수 있다.
    if (!m_vehicleController.IsFinished())
    {
        m_wantSegmentTick = true;
        return;
    }

    if (m_speed > 0.01f)
    {
        // 방금 "도착"으로 넘어온 경우 조향을 그 순간 각도로 얼어붙힌 채 그냥 굴러가게 두면, 남은
        // 제동거리 동안 차가 똑바로(혹은 마지막 조향이 튼 방향으로) 밀려나가 목적지 지점을 지나쳐버린다.
        // m_currentLane이 아직 유효한 동안은 계속 그 레인(끝점)을 조준해서 감속 중에도 목적지 쪽으로
        // 붙는 방향을 유지한다.
        if (m_currentLane != nullptr)
        {
            Vec3 rigidPosition = GetRigidbodyPosition();
            float lookaheadDistance = ComputeLookaheadDistance();
            Vec3 targetPosition, targetDir;
            GetLookaheadPose(m_currentLane, m_pathIndex, rigidPosition, lookaheadDistance, targetPosition, targetDir);
            Steer(PurePursuit(targetPosition));
        }
        Accelerate(0.0f);
        return;
    }

    std::shared_ptr<RoadNode> dest = m_RoadDataManager->GetRandomDestNode();
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
    if (m_currentLane == nullptr)
    {
        m_destLane = nullptr;
        return false;
    }

    // path find
    Vec3 position = GetPosition();

    // 현재 레인의 끝에 다가가면 경로상 다음 레인으로 넘어간다.
    Vec3 projectedPosition = m_currentLane->GetSpline().GetLookaheadPoint(position, 0.0f);
    float laneEndDistance = (m_currentLane->GetEndPoint() - projectedPosition).Length();
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
        projectedPosition = m_currentLane->GetSpline().GetLookaheadPoint(position, 0.0f);
        laneEndDistance = (m_currentLane->GetEndPoint() - projectedPosition).Length();
        RebuildSplineRender();
    }
    return true;
}

float Car::ComputeLookaheadDistance() const
{
    float minSafeLookahead = 2.0f * m_wheelbase / tanf(m_maxSteerAngle);
    constexpr float LOOKAHEAD_TIME = 1.5f; // 몇 초 앞을 볼지
    return  max(m_speed * LOOKAHEAD_TIME, minSafeLookahead);
}

void Car::DriveControl()
{
    Vec3 rigidPosition = GetRigidbodyPosition();
    float lookaheadDistance = ComputeLookaheadDistance();
    Vec3 targetPosition, targetDir;
    GetLookaheadPose(m_currentLane, m_pathIndex, rigidPosition, lookaheadDistance, targetPosition, targetDir);
    float targetSteer = PurePursuit(targetPosition);
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
        float steerSpeedCap = CalcMaxSpeed(targetSteer);
        Accelerate(std::min(steerSpeedCap, m_currentBehaviorPlan.targetSpeed));
    }

    // Debug
    DirectX::XMFLOAT3 targetMarkerPos = ToXMFLOAT3(targetPosition);
    targetMarkerPos.y = 0.2f;
    m_targetMarker.GetTransform().SetPosition(targetMarkerPos);
}

void Car::GetLookaheadPose(const shared_ptr<Lane> &startLane, size_t startPathIndex,
                           const Vec3 &fromPosition, float distance, Vec3 &outPosition, Vec3 &outDirection) const
{
    shared_ptr<Lane> segmentLane = startLane;
    const Spline *spline = &startLane->GetSpline();
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
        if (!nextLane || (isNextLaneCurved && nextLaneRamain >= LANE_TRANSITION_THRESHOLD * 2.0f))
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

#pragma endregion
