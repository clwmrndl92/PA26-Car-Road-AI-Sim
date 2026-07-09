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
            if (m_destNode == nullptr)
                return true;
            if (m_currentNode == nullptr)
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
            if (m_currentNode != nullptr)
                return BTStatus::Success;

            Vec3 position = m_rigidbody.GetPosition();
            m_currentNode = m_RoadDataManager->GetClosestNode(position);
            m_path = m_RoadDataManager->FindPath(m_currentNode, m_destNode);
            m_pathIndex = -1;
            if (m_path.empty())
            {
                m_destNode = nullptr;
                m_currentNode = nullptr;
                return BTStatus::Success;
            }

            m_currentSpline = Spline({position - GetForwardAxis(),
                                      position,
                                      m_currentNode->position,
                                      m_currentNode->position + m_currentNode->GetDirection()});
            RebuildSplineRender();
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
            return m_destNode == nullptr || (m_destNode->position - m_rigidbody.GetPosition()).Length() < ARRIVE_DISTANCE;
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
            m_destNode = nullptr;
            m_currentNode = nullptr;
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
            Vec3 position = m_rigidbody.GetPosition();

            float currentNodeDistance = (m_currentNode->position - position).Length();
            // DebugConsole::Get().Log("currentNodeDistance: " + std::to_string(currentNodeDistance));
            auto prevNode = m_currentNode;
            while (currentNodeDistance < 3.0f)
            {
                if (m_pathIndex + 1 >= m_path.size())
                {
                    m_destNode = nullptr;
                    m_currentNode = nullptr;
                    return BTStatus::Failure;
                }
                m_currentNode = m_path[++m_pathIndex];
                m_currentSpline = m_currentNode->lane->GetSpline();
                currentNodeDistance = (m_currentNode->position - position).Length();
                RebuildSplineRender();
            }
            if (prevNode != m_currentNode && prevNode->nodeType == RoadNodeType::ChangeLane)
            {
                float minRadius = powf(m_speed / CURVE_SPEED_COEFF, 2);
                float width = m_RoadDataManager->ROAD_WIDTH;
                float insideRoot = (4 * minRadius * width) - (width * width);
                float L = insideRoot > 0 ? sqrt(insideRoot) : 5.0f;

                Vec3 changePosition = m_currentSpline.GetLookaheadPoint(position, L);
                float changeSplinePosition = m_currentSpline.GetSplinePosition(changePosition);

                auto changeLineSpline = Spline({position - GetForwardAxis(),
                                                position,
                                                changePosition,
                                                changePosition + m_currentSpline.GetDirectionAt(changeSplinePosition)});
                m_currentSpline.AddSplinePointsFront(changeLineSpline.GetSplinePoints(), changeSplinePosition);
                RebuildSplineRender();
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
            DebugConsole::Get().Log("targetSpeed: " + std::to_string(targetSpeed * 3.6f));

            Accelerate(targetSpeed);

            return BTStatus::Success;
        });
    drive->name = "DriveControl";
    return MakeSequence(std::move(checkPath), std::move(drive));
}