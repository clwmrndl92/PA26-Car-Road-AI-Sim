#include "Entities/Car.h"
#include "Core/DebugConsole.h"
#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

std::unique_ptr<BTNode> Car::BuildBehaviourTree()
{
    return MakeSequence(
        FindPathNode(),
        MakeSelector(
            StopNode(),
            DriveNode()));
}

std::unique_ptr<BTNode> Car::FindPathNode()
{
    // 경로 (재)탐색 조건
    auto notSearch = std::make_unique<BTCondition>(
        [this]()
        {
            if (m_destLane == nullptr)
                return true;
            if (m_currentLane == nullptr)
                return false;
            if (IsOffCourse())
                return false;
            return true;
        });
    notSearch->name = "NotSearch?";

    // 경로를 탐색
    auto searchPath = std::make_unique<BTAction>(
        [this]()
        {
            if (m_currentLane != nullptr)
                return BTStatus::Success;

            Vec3 position = GetPosition();
            m_currentLane = m_RoadDataManager->GetClosestLane(position);
            m_path = m_RoadDataManager->FindPath(m_currentLane, m_destLane);
            m_pathIndex = 0; // path[0] == 시작(=현재) 레인
            if (m_path.empty())
            {
                m_destLane = nullptr;
                m_currentLane = nullptr;
                return BTStatus::Success;
            }

            MergeOntoLane(m_currentLane, position);
            CalculateSpeedProfile();
            return BTStatus::Success;
        });
    searchPath->name = "SearchPath";

    return MakeSelector(std::move(notSearch), std::move(searchPath));
}

std::unique_ptr<BTNode> Car::StopNode()
{
    // 정지 조건 : 목적지가 없거나 도착
    auto shouldStop = std::make_unique<BTCondition>(
        [this]()
        {
            constexpr float ARRIVE_DISTANCE = 5.0f;
            return m_destLane == nullptr || (m_destLane->GetEndPoint() - GetPosition()).Length() < ARRIVE_DISTANCE;
        });
    shouldStop->name = "ShouldStop?";

    auto brake = std::make_unique<BTAction>(
        [this]()
        {
            if (m_speed > 0.0f)
            {
                Accelerate(0.0f);
                return BTStatus::Running;
            }
            m_destLane = nullptr;
            m_currentLane = nullptr;
            return BTStatus::Success;
        });
    brake->name = "Stop";

    return MakeSequence(std::move(shouldStop), std::move(brake));
}

std::unique_ptr<BTNode> Car::DriveNode()
{
    auto checkPath = std::make_unique<BTAction>(
        [this]()
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
                return BTStatus::Success;
            }

            // 현재 레인의 끝에 다가가면 경로상 다음 레인으로 넘어간다.
            float laneEndDistance = (m_currentLane->GetEndPoint() - position).Length();
            // DebugConsole::Get().Log("laneEndDistance: " + std::to_string(laneEndDistance));
            bool enteredByLaneChange = false;
            while (laneEndDistance < 3.0f)
            {
                if (m_pathIndex + 1 >= m_path.size())
                {
                    m_destLane = nullptr;
                    m_currentLane = nullptr;
                    return BTStatus::Failure;
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
            return BTStatus::Success;
        });
    checkPath->name = "CheckNextNode";
    auto drive = std::make_unique<BTAction>(
        [this]()
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

            return BTStatus::Success;
        });
    drive->name = "DriveControl";
    return MakeSequence(std::move(checkPath), std::move(drive));
}