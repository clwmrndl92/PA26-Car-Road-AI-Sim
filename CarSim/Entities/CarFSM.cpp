#include "Car.h"
#include "VehicleSegment.h"
#include "Core/DebugConsole.h"
#include "Nav/ReedsShepp.h"
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
    DriveMode next = DecideNextMode();
    if (next != m_mode)
    {
        OnModeExit(m_mode);
        OnModeEnter(next);
        m_mode = next;
    }
}

Car::DriveMode Car::DecideNextMode() const
{
    if (m_mode == DriveMode::Park)
    {
        if (!m_vehicleController.IsFinished())
            return DriveMode::Park;
    }

    constexpr float ARRIVE_DISTANCE = 5.0f;
    bool arrived = m_destLane != nullptr && (m_destLane->GetEndPoint() - GetPosition()).Length() < ARRIVE_DISTANCE;

    if (arrived && GetParkTargetNode() != nullptr)
        return DriveMode::Park;

    if (m_destLane == nullptr || arrived)
        return DriveMode::Stop;

    if (m_parkSpot != nullptr)
    {
        if (m_currentLane == nullptr)
            return DriveMode::Park;

        // 입차: 목적지 레인(주차장 진입 레인)의 끝에 다다르면 RS로 주차칸까지 마무리한다.
        constexpr float PARK_TRIGGER_DISTANCE = 8.0f;
        bool nearFinalLane = m_currentLane == m_destLane &&
                             (m_currentLane->GetEndPoint() - GetPosition()).Length() < PARK_TRIGGER_DISTANCE;
        if (nearFinalLane)
            return DriveMode::Park;
    }

    return DriveMode::Drive;
}

void Car::OnModeEnter(DriveMode mode)
{
    if (mode == DriveMode::Drive)
    {
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        segments.push_back(std::make_unique<SplineFollowSegment>());
        m_vehicleController.BeginPlan(std::move(segments));
    }
    else if (mode == DriveMode::Park)
    {
        // 도착 즉시 RS를 계산하지 않고, 완전히 멈출 때까지 기다린다 (UpdatePark에서 처리).
        m_parkPlanPending = true;
    }
}

void Car::BeginParkPlan()
{
    // 아직 예약 전이고(입차 대기 중) 목표 Park 노드가 있으면, 도착한 지금 실제로 예약을 시도한다.
    if (m_parkSpot == nullptr && m_pendingParkNode != nullptr)
    {
        m_parkSpot = m_RoadDataManager->TryReserveParkSpot(m_pendingParkNode->id);
        m_pendingParkNode = nullptr;
        if (m_parkSpot == nullptr)
        {
            // 빈 자리 없음 — 목적지를 포기한다. m_isExitingPark는 건드리지 않는다(스테일 값일 수
            // 있음) — DecideNextMode/UpdatePark 양쪽 다 m_parkSpot==nullptr이면 이 플래그를
            // 안 보게 돼 있어서, destLane==nullptr만으로 다음 프레임에 Stop에 안착한다.
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
    Vec3 targetPos;
    float targetAngleDeg;

    if (m_isExitingPark)
    {
        // 출차: 주차칸(현재 위치)에서 가장 가까운 레인으로 나간다.
        // GetClosestLane이 고른 레인의 "가장 가까운 지점"이 레인 경계 근처라 차 뒤쪽에 나오면,
        // 그 레인에 그대로 고정해서 후진부터 시키지 말고 후속(successor) 레인으로 넘어가면서
        // m_currentLane도 그 레인으로 같이 갱신한다.
        shared_ptr<Lane> candidateLane = m_RoadDataManager->GetClosestLane(startPos);
        float splinePos = candidateLane->GetSpline().GetSplinePosition(startPos);
        Vec3 candidatePos = candidateLane->GetSpline().GetPositionAt(splinePos);
        Vec3 candidateDir = candidateLane->GetSpline().GetDirectionAt(splinePos);

        constexpr int MAX_LANE_HOPS = 4;
        for (int hop = 0; hop < MAX_LANE_HOPS; ++hop)
        {
            bool isBehind = (candidatePos - startPos).Dot(GetForwardAxis()) < 0.0f;
            if (!isBehind)
                break;

            shared_ptr<Lane> nextLane;
            for (const weak_ptr<Lane> &weak : candidateLane->GetSuccessors())
            {
                if (shared_ptr<Lane> lane = weak.lock())
                {
                    nextLane = lane;
                    break;
                }
            }
            if (!nextLane)
                break;

            candidateLane = nextLane;
            candidatePos = candidateLane->GetSpline().GetPositionAt(0.0f);
            candidateDir = candidateLane->GetSpline().GetDirectionAt(0.0f);
        }

        m_currentLane = candidateLane;
        targetPos = candidatePos;
        targetAngleDeg = DirectionToAngleDeg(candidateDir);
    }
    else
    {
        // 입차: 현재 레인에서 예약해둔 주차칸으로 들어간다.
        targetAngleDeg = DirectionToAngleDeg(m_parkSpot->direction);
        targetPos = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
    }

    ReedsShepp::Path path = ReedsShepp::GetOptimalPath(startPos, startAngleDeg, targetPos, targetAngleDeg, turningRadius);
    m_vehicleController.BeginPlan(BuildParkSegments(path, m_maxSteerAngle));
    RebuildParkDebugRender(path, startPos, startAngleDeg, turningRadius, targetPos, targetAngleDeg);

    DebugConsole::Get().Log("Park path segments: " + to_string(path.size()));
    for (const ReedsShepp::PathElement &element : path)
    {
        const char *steerName = element.steering == ReedsShepp::Steering::Left    ? "Left"
                                : element.steering == ReedsShepp::Steering::Right ? "Right"
                                                                                  : "Straight";
        const char *gearName = element.gear == ReedsShepp::Gear::Forward ? "Forward" : "Backward";
        DebugConsole::Get().Log(std::string("  ") + steerName + " " + gearName + " param=" + to_string(element.param));
    }
}

void Car::OnModeExit(DriveMode mode)
{
    if (!m_vehicleController.IsFinished())
        m_vehicleController.Abort();
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

    MergeOntoLane(m_currentLane, position);
    CalculateSpeedProfile();
}

void Car::UpdateStop()
{
    // 정지 조건(목적지 없음/도착)은 DecideNextMode가 이미 판단했으므로 여기선 감속/정지 동작만 한다.
    m_destLane = nullptr;
    m_currentLane = nullptr;
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
    // TODO: 센서 기반 장애물 회피 로직을 여기에 연결. 지금은 항상 false를 반환해 VehicleController로 넘긴다.
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
        CalculateSpeedProfile();
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
    {
        MergeOntoLane(m_currentLane, position);
        CalculateSpeedProfile();
    }
    return true;
}

void Car::DriveControl()
{
    Vec3 position = m_rigidbody.GetPosition();
    // steering
    constexpr float MIN_LOOKAHEAD_DISTANCE = 5.0f; // 저속/정지 시 최소 lookahead (m)
    constexpr float LOOKAHEAD_TIME = 0.5f;         // 몇 초 앞을 볼지
    float lookaheadDistance = std::max(MIN_LOOKAHEAD_DISTANCE, m_speed * LOOKAHEAD_TIME);
    auto targetPosition = m_currentSpline.GetLookaheadPoint(position, lookaheadDistance);
    m_targetMarker.GetTransform().SetPosition(ToXMFLOAT3(targetPosition));
    float targetSteer = PurePursuit(targetPosition);
    Steer(targetSteer);

    // speed control
    float currentTime = m_currentTime;
    if (currentTime - m_lastProfileTime >= LOOK_PROFILE_TIME / SPEED_PROFILE_COUNT)
    {
        float profileSpeed = m_speedProfile[m_profileIndex].second;
        if (IsOffCourse() || abs(profileSpeed - m_speed) > (5.0f / 3.6f))
        {
            // Calculate/Move 둘 다 내부에서 m_profileIndex를 한 칸 전진시켜 두므로 호출부는 대칭이다.
            CalculateSpeedProfile();
        }
        else
        {
            MoveSpeedProfile();
        }
        m_lastProfileTime = m_currentTime;
    }
    float maxSpeed = CalcMaxSpeed(targetSteer) * 0.8f;
    float targetSpeed = min(m_speedProfile[m_profileIndex].second, maxSpeed);

    Accelerate(targetSpeed);
}
