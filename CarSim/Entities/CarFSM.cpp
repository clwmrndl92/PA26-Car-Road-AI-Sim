#include "Car.h"
#include "Core/DebugConsole.h"
#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

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
    constexpr float ARRIVE_DISTANCE = 5.0f;
    bool arrived = m_destLane != nullptr && (m_destLane->GetEndPoint() - GetPosition()).Length() < ARRIVE_DISTANCE;
    if (m_destLane == nullptr || arrived)
        return DriveMode::Stop;

    // TODO: 진입 각도가 커서 RS 경로가 필요한 상황을 감지하면 DriveMode::Park 반환.
    return DriveMode::Drive;
}

void Car::OnModeEnter(DriveMode mode)
{
    // TODO: Park 모드 진입 시 Reeds-Shepp 경로 계산 등 1회성 셋업을 여기에 추가.
}

void Car::OnModeExit(DriveMode mode)
{
    // TODO: Park 모드가 남긴 상태(RS 경로 등) 정리를 여기에 추가.
}

const char *Car::DriveModeToString(DriveMode mode) const
{
    switch (mode)
    {
    case DriveMode::Stop:
        return "Stop";
    case DriveMode::Park:
        return "Park";
    case DriveMode::Drive:
        return "Drive";
    }
    return "?";
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
    m_currentLane = m_RoadDataManager->GetClosestLane(position);
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
    if (m_speed > 0.0f)
    {
        Accelerate(0.0f);
        return;
    }
    m_destLane = nullptr;
    m_currentLane = nullptr;
}

void Car::UpdatePark()
{
    // TODO: Reeds-Shepp 기반 진입/주차 로직을 여기에 연결. DecideNextMode가 아직 Park을 반환하지
    // 않으므로 지금은 호출되지 않는다.
}

void Car::UpdateDrive()
{
    if (!CheckPath())
        return;
    if (TryAvoidObstacle())
        return;
    DriveControl();
}

bool Car::TryAvoidObstacle()
{
    // TODO: 센서 기반 장애물 회피 로직을 여기에 연결. 지금은 항상 false를 반환해 DriveControl로 넘긴다.
    return false;
}

bool Car::CheckPath()
{
    // path find
    Vec3 position = GetPosition();

    // 다음 레인이 차선변경이면, 레인 끝까지 기다리지 않고 바로 차선변경을 진행한다.
    float laneStartDistance = (m_currentLane->GetEndPoint() - position).Length();
    if (m_pathIndex + 1 < m_path.size() && m_path[m_pathIndex + 1].isLaneChange && laneStartDistance > 3.0f)
    {
        ++m_pathIndex;
        MergeOntoLane(m_path[m_pathIndex].lane, position);
        CalculateSpeedProfile();
        return true;
    }

    // 현재 레인의 끝에 다가가면 경로상 다음 레인으로 넘어간다.
    float laneEndDistance = (m_currentLane->GetEndPoint() - position).Length();
    bool enteredByLaneChange = false;
    while (laneEndDistance < 3.0f)
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
