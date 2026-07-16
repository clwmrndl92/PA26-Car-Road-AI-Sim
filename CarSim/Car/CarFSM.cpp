#include "Car.h"
#include "VehicleSegment.h"
#include "Utill/DebugConsole.h"
#include "Nav/ReedsShepp.h"
#include "Nav/HybridAStar.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace
{
    // ReedsShepp::Path의 각 세그먼트를 VehicleController가 실행할 수 있는 세그먼트로 변환.
    std::vector<std::unique_ptr<VehicleSegment>> BuildParkSegments(const ReedsShepp::Path &path, float steerAngle)
    {
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        segments.reserve(path.size());
        for (const ReedsShepp::PathElement &element : path)
            segments.push_back(std::make_unique<ArcMoveSegment>(element.steering, element.gear, element.param, steerAngle));
        return segments;
    }

    // ReedsShepp이 쓰는 각도 규약(atan2(z, x), degree)으로 방향 벡터를 변환.
    float DirectionToAngleDeg(const Vec3 &direction)
    {
        return ToDegrees(atan2f(direction.GetZ(), direction.GetX()));
    }
}

void Car::UpdateMode()
{
    const char *reason = "";
    DriveMode next = DecideNextMode(&reason);
    if (next != m_mode)
    {
        DebugConsole::Log(std::string(DriveModeToString(m_mode)) + " -> " + DriveModeToString(next) + " (" + reason + ")");
        OnModeExit(m_mode);
        OnModeEnter(next, m_mode);
        m_mode = next;
    }
}

Car::DriveMode Car::DecideNextMode(const char **reason) const
{
    if (m_mode == DriveMode::Park)
    {
        // 정지 대기(m_parkPlanPending) 구간에서는 vehicleController가 아직 finished 상태라
        // 아래 avoidPending 체크로 새는 걸 막는다 — 주차 중엔 Avoid로 전환되지 않아야 한다.
        if (m_parkPlanPending || !m_vehicleController.IsFinished())
        {
            *reason = "parking in progress";
            return DriveMode::Park;
        }
    }

    if (m_mode == DriveMode::Avoid)
    {
        if (!m_vehicleController.IsFinished())
        {
            *reason = "avoid maneuver in progress";
            return DriveMode::Avoid;
        }
    }

    if (m_avoidPending)
    {
        *reason = "obstacle detected, avoid pending";
        return DriveMode::Avoid;
    }

    constexpr float ARRIVE_DISTANCE = 5.0f;
    bool arrived = m_destLane != nullptr && (m_destLane->GetEndPoint() - GetPosition()).Length() < ARRIVE_DISTANCE;

    if (arrived && GetParkTargetNode() != nullptr)
    {
        *reason = "arrived at destination with park target";
        return DriveMode::Park;
    }

    if (m_destLane == nullptr || arrived)
    {
        *reason = m_destLane == nullptr ? "no destination lane" : "arrived at destination";
        return DriveMode::Stop;
    }

    if (m_parkSpot != nullptr)
    {
        if (m_currentLane == nullptr)
        {
            *reason = "park spot reserved, no current lane (exiting park)";
            return DriveMode::Park;
        }

        // 입차: 목적지 레인(주차장 진입 레인)의 끝에 다다르면 RS로 주차칸까지 마무리한다.
        constexpr float PARK_TRIGGER_DISTANCE = 8.0f;
        bool nearFinalLane = m_currentLane == m_destLane &&
                             (m_currentLane->GetEndPoint() - GetPosition()).Length() < PARK_TRIGGER_DISTANCE;
        if (nearFinalLane)
        {
            *reason = "near end of final lane, entering park spot";
            return DriveMode::Park;
        }
    }

    *reason = "normal driving";
    return DriveMode::Drive;
}

void Car::OnModeEnter(DriveMode mode, DriveMode previous)
{
    if (mode == DriveMode::Drive)
    {
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        // Park에서 넘어왔다는 건 출차가 막 끝났다는 뜻(입차 완료는 Stop으로 감) — RS 매뉴버로
        // 꺾여있던 조향을 중앙으로 되돌린 뒤 정속주행을 시작한다.
        if (previous == DriveMode::Park)
            segments.push_back(std::make_unique<CenterSteerSegment>());
        segments.push_back(std::make_unique<SplineFollowSegment>());
        m_vehicleController.BeginPlan(std::move(segments));
    }
    else if (mode == DriveMode::Park)
    {
        // 도착 즉시 RS를 계산하지 않고, 완전히 멈출 때까지 기다린다 (UpdatePark에서 처리).
        m_parkPlanPending = true;
        DebugConsole::Log("Park plan pending: waiting for full stop before planning");
    }
    else if (mode == DriveMode::Avoid)
    {
        // Avoid는 TryAvoidObstacle이 이미 차를 완전히 세운 뒤에만 m_avoidPending을 켜므로, Park와
        // 달리 정지 대기 없이 바로 계획을 세운다.
        BeginAvoidPlan();
    }
    else if (mode == DriveMode::Stop)
    {
        // 입차 완료 후 Stop으로 오는 경우도 RS 매뉴버로 꺾여있던 조향을 중앙으로 되돌린다.
        if (previous == DriveMode::Park)
        {
            std::vector<std::unique_ptr<VehicleSegment>> segments;
            segments.push_back(std::make_unique<CenterSteerSegment>());
            m_vehicleController.BeginPlan(std::move(segments));
        }
    }
}

void Car::OnModeExit(DriveMode mode)
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
            m_destLane = nullptr;
            return;
        }
    }

    if (m_parkSpot == nullptr)
        return;

    // 호출 시점에 currentLane 유무로 방향을 판정 — 출차 분기에서 currentLane을 채우므로
    // 반드시 그 전에 캡처해둔다.
    m_isExitingPark = (m_currentLane == nullptr);

    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);
    Vec3 startPos = m_rigidbody.GetPosition();
    float startAngleDeg = DirectionToAngleDeg(GetForwardAxis());
    HybridAStar::VehicleShape shape = BuildVehicleShape();
    const std::vector<HybridAStar::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();

    if (m_isExitingPark)
    {
        shared_ptr<Lane> closestLane = m_RoadDataManager->GetClosestLane(startPos);

        float splinePos = closestLane->GetSpline().GetSplinePosition(startPos);
        Vec3 closestPos = closestLane->GetSpline().GetPositionAt(splinePos);
        Vec3 closestDir = closestLane->GetSpline().GetDirectionAt(splinePos);
        m_currentLane = closestLane;

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
        float targetAngleDeg = DirectionToAngleDeg(closestPos - targetPos);

        bool foundPath = false;
        ReedsShepp::Path path = HybridAStar::FindPath(startPos, startAngleDeg, targetPos, targetAngleDeg, obstacles, shape, foundPath);
        if (!foundPath)
        {
            // 출차 실패는 이미 차가 그 자리를 점유 중이므로 예약을 풀지 않는다.
            DebugConsole::Log("BeginParkPlan: HybridA* failed to find an exit path, abandoning this park attempt");
            m_destLane = nullptr;
            return;
        }
        m_vehicleController.BeginPlan(BuildParkSegments(path, m_maxSteerAngle));
        RebuildParkDebugRender(path, startPos, startAngleDeg, turningRadius, targetPos, targetAngleDeg);
        return;
    }

    // 입차: 현재 레인에서 예약해둔 주차칸으로 들어간다. 경로탐색이 막히면 그 자리를 포기하고
    // 예약을 풀어준 뒤, 같은 Park 노드의 다른 빈 자리를 하나씩 다시 예약해 시도한다.
    // 모든 자리가 막혀 있으면 그제서야 목적지를 포기한다.
    unordered_set<int> triedSpotIds;
    while (true)
    {
        Vec3 targetPos = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
        float targetAngleDeg = DirectionToAngleDeg(m_parkSpot->direction);

        bool foundPath = false;
        ReedsShepp::Path path = HybridAStar::FindPath(startPos, startAngleDeg, targetPos, targetAngleDeg, obstacles, shape, foundPath);
        if (foundPath)
        {
            m_vehicleController.BeginPlan(BuildParkSegments(path, m_maxSteerAngle));
            RebuildParkDebugRender(path, startPos, startAngleDeg, turningRadius, targetPos, targetAngleDeg);
            return;
        }

        DebugConsole::Log("BeginParkPlan: HybridA* failed to reach ParkSpot " + std::to_string(m_parkSpot->id) +
                          ", trying next available spot");
        triedSpotIds.insert(m_parkSpot->id);
        m_RoadDataManager->ReleaseParkSpot(m_parkSpot->id);
        m_parkSpot = m_RoadDataManager->TryReserveParkSpot(parkNodeId, triedSpotIds);
        if (m_parkSpot == nullptr)
        {
            DebugConsole::Log("BeginParkPlan: no reachable ParkSpot left for node " + std::to_string(parkNodeId) +
                              ", abandoning destination");
            m_destLane = nullptr;
            return;
        }
    }
}

HybridAStar::VehicleShape Car::BuildVehicleShape() const
{
    HybridAStar::VehicleShape shape;
    shape.wheelbase = m_wheelbase;
    shape.maxSteerAngleDeg = ToDegrees(m_maxSteerAngle);
    shape.pivotToCenter = m_colliderOffset.z;
    shape.halfWidth = m_halfExtents.GetX();
    shape.halfLength = m_halfExtents.GetZ();
    return shape;
}

void Car::BeginAvoidPlan()
{
    m_avoidPending = false;
    DebugConsole::Log("Avoid pending resolved: fully stopped, beginning HybridA* plan");

    Vec3 startPos = m_rigidbody.GetPosition();
    float startAngleDeg = DirectionToAngleDeg(GetForwardAxis());

    // 목표: 지금 레인의 스플라인을 따라 장애물을 완전히 지나칠 만큼(코리도어보다 넉넉히) 앞의 지점.
    // 레인을 벗어나지 않고 그 위의 한 점으로 잡아두면, 우회가 끝난 뒤 별도 처리 없이 그냥 Drive로
    // 돌아가 기존 m_currentLane/m_path를 그대로 이어서 쓸 수 있다. 현재 레인 안에서 거리가 안
    // 나오면 경로상 다음 레인까지만 이어서 재보고, 그래도 모자라면 다음 레인 끝점을 목표로 삼는다.
    constexpr float AVOID_GOAL_DISTANCE = 30.0f;
    const std::vector<Vec3> &splinePoints = m_currentSpline.GetSplinePoints();

    Vec3 targetPos = startPos;
    float targetAngleDeg = startAngleDeg;
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
                targetAngleDeg = DirectionToAngleDeg(point - prevPoint);
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
                targetAngleDeg = DirectionToAngleDeg(point - prevPoint);
                foundGoal = true;
                break;
            }
            prevPoint = point;
        }

        if (!foundGoal)
        {
            targetPos = nextLane->GetEndPoint();
            targetAngleDeg = DirectionToAngleDeg(nextLane->GetSpline().GetDirectionAt(1.0f));
            foundGoal = true;
        }
    }

    if (!foundGoal)
    {
        // 다음 레인도 없음(경로 끝 코앞) -- 현재 레인 끝점을 최선으로 삼는다.
        targetPos = m_currentLane->GetEndPoint();
        targetAngleDeg = DirectionToAngleDeg(m_currentLane->GetSpline().GetDirectionAt(1.0f));
    }

    HybridAStar::VehicleShape shape = BuildVehicleShape();
    const std::vector<HybridAStar::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();

    bool foundPath = false;
    ReedsShepp::Path path = HybridAStar::FindPath(startPos, startAngleDeg, targetPos, targetAngleDeg, obstacles, shape, foundPath);
    if (!foundPath)
    {
        // 실패해도 releas할 예약은 없다 — 그냥 Drive로 돌아가고, TryAvoidObstacle이 다음 프레임
        // 다시 코리도어를 검사해 막혀있으면 재시도(m_avoidPending)를 건다.
        DebugConsole::Log("BeginAvoidPlan: HybridA* failed to find an avoid route, will retry on next detection");
        return;
    }
    m_vehicleController.BeginPlan(BuildParkSegments(path, m_maxSteerAngle));

    DebugConsole::Log("Avoid path segments: " + to_string(path.size()));
}

void Car::UpdateAvoid()
{
    m_vehicleController.Tick(*this);
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

    m_currentLane = m_RoadDataManager->GetClosestLane(position);
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
        m_currentLane = nullptr;
        return;
    }

    BakePathSpeedProfile();
    MergeOntoLane(m_currentLane, position);
}

void Car::UpdateStop()
{
    // 정지 조건(목적지 없음/도착)은 DecideNextMode가 이미 판단했으므로 여기선 감속/정지 동작만 한다.
    m_destLane = nullptr;
    m_currentLane = nullptr;

    // Park에서 넘어온 직후면 조향 원복 세그먼트가 아직 안 끝났을 수 있다.
    if (!m_vehicleController.IsFinished())
    {
        m_vehicleController.Tick(*this);
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

    m_vehicleController.Tick(*this);
    if (!m_vehicleController.IsFinished())
        return;

    // m_parkSpot!=nullptr 체크는 방어코드 — BeginParkPlan이 예약 실패로 조기 리턴하면
    // m_isExitingPark가 이전 사이클의 스테일 값(true)일 수 있는데, 그 상태에서 m_parkSpot은
    // 항상 null이므로 여기서 걸러야 아래 m_parkSpot->id가 널 역참조로 크래시하지 않는다.
    if (m_isExitingPark && m_parkSpot != nullptr)
    {
        // 출차 완료: 이제 레인 위. 더 이상 이 주차칸에 있는 게 아니므로 예약을 풀고 비운다.
        m_RoadDataManager->ReleaseParkSpot(m_parkSpot->id);
        m_parkSpot = nullptr;
        // m_currentLane은 BeginParkPlan에서 이미 정해둔 상태 — 거기서부터 경로/스플라인을 채운다.
        EnterCurrentLane();
        return;
    }

    // 입차 완료: m_parkSpot은 "지금 여기 주차 중"을 나타내도록 남겨두고 destLane/currentLane만
    // 비운다. 다음 프레임 DecideNextMode가 destLane==nullptr을 보고 Stop으로 전환한다. 이후 새
    // 목적지가 생기면 DecideNextMode의 일반 로직(parkSpot!=nullptr && currentLane==nullptr)이
    // 다시 Park로 복귀시키며 OnModeEnter가 BeginParkPlan을 호출한다.
    if (m_currentLane != nullptr)
    {
        m_destLane = nullptr;
        m_currentLane = nullptr;
        return;
    }
}

void Car::UpdateDrive()
{
    if (!CheckPath())
        return;
    if (TryAvoidObstacle())
        return;
    m_vehicleController.Tick(*this);
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

        float headingDeg = DirectionToAngleDeg(point - prevPoint);
        if (HybridAStar::IsColliding(point, headingDeg, obstacles, shape))
        {
            constexpr float AVOID_REPLAN_COOLDOWN = 1.5f; // Hybrid A* 실패 시 매 프레임 재시도하지 않도록.
            if (m_speed > 0.0f)
            {
                DebugConsole::Log("Obstacle blocking corridor: emergency braking");
                EmergBrake();
            }
            else if (m_avoidReplanCooldown <= 0.0f)
            {
                m_avoidPending = true;
                m_avoidReplanCooldown = AVOID_REPLAN_COOLDOWN;
                DebugConsole::Log("Avoid pending: fully stopped, waiting to plan avoid route");
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
            m_currentLane = nullptr;
            return false;
        }
        ++m_pathIndex;
        m_currentLane = m_path[m_pathIndex].lane;
        enteredByLaneChange = m_path[m_pathIndex].isLaneChange;
        m_currentSpline = m_currentLane->GetSpline();
        laneEndDistance = (m_currentLane->GetEndPoint() - position).Length();
        RebuildSplineRender();
    }
    // 차선변경으로 진입한 레인이면, 현재 위치에서 그 레인 위로 합류하는 연결 스플라인을 만든다.
    if (enteredByLaneChange)
        MergeOntoLane(m_currentLane, position);
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

    // speed control: 현재 위치의 baked 경로 최대속도(코너/제한속도/제동램프 전파가 이미 반영됨)를
    // 그대로 목표 속도로 삼는다. 실제 가속/제동 램프(저크 완화)는 Accelerate() 안에서 처리된다.
    float pathDistance = GetPathDistance(m_pathIndex, position);
    float maxSteerSpeed = CalcMaxSpeed(targetSteer) * 0.8f;
    float targetSpeed = std::min(GetPathMaxSpeed(pathDistance), maxSteerSpeed);

    Accelerate(targetSpeed);

    // Debug
    DirectX::XMFLOAT3 targetMarkerPos = ToXMFLOAT3(targetPosition);
    targetMarkerPos.y = 0.2f;
    m_targetMarker.GetTransform().SetPosition(targetMarkerPos);
}
