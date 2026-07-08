#include "Entities/Car.h"
#include "Core/DebugConsole.h"
#include <algorithm>
#include <limits>

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

            float remainDist = (m_rigidbody.GetPosition() - m_currentNode->position).Length();
            constexpr float REFIND_DISTANCE = 10.0f;
            if (remainDist > m_startDistToNode + REFIND_DISTANCE)
            {
                m_currentNode = nullptr;
                m_startDistToNode = 0.0f;
                return false;
            }
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
            auto closestNode = m_RoadDataManager->GetClosestNode(position);
            m_path = m_RoadDataManager->FindPath(closestNode, m_destNode);
            m_pathIndex = -1;

            m_currentNode = make_shared<RoadNode>();
            m_currentNode->position = closestNode->position;
            m_currentNode->nodeType = RoadNodeType::End;
            m_currentNode->lanePosition = 1;
            m_startDistToNode = (m_currentNode->position - position).Length();
            Spline spline = Spline({position - GetForwardAxis(),
                                    position,
                                    closestNode->position,
                                    closestNode->position + closestNode->GetDirection()});
            m_currentNode->lane = make_shared<Lane>(-1, spline, closestNode->lane->GetRoad());
            if (m_path.empty())
            {
                m_destNode = nullptr;
                m_currentNode = nullptr;
                return BTStatus::Success;
            }
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
    auto checkNext = std::make_unique<BTAction>(
        [this]()
        {
            // path find
            Vec3 position = m_rigidbody.GetPosition();

            float currentNodeDistance = (m_currentNode->position - position).Length();
            DebugConsole::Get().Log("currentNodeDistance: " + std::to_string(currentNodeDistance));
            while (currentNodeDistance < 3.0f)
            {
                if (m_pathIndex + 1 >= m_path.size())
                {
                    m_destNode = nullptr;
                    m_currentNode = nullptr;
                    return BTStatus::Failure;
                }
                m_currentNode = m_path[++m_pathIndex];
                currentNodeDistance = (m_currentNode->position - position).Length();
                m_startDistToNode = currentNodeDistance;
            }
            return BTStatus::Success;
        });
    checkNext->name = "CheckNextNode";
    auto drive = std::make_unique<BTAction>(
        [this]()
        {
            // speed control
            Vec3 position = m_rigidbody.GetPosition();
            float maxSpeed = m_currentNode->lane->GetRoad()->GetSpeedLimit();
            float targetSpeed = maxSpeed;

            float fullBreakTime = m_speed / m_maxBrakeDeceleration + m_maxBrakeDeceleration / BRAKE_RAMP_RATE; // 대충 계산
            float lookDistance = (m_speed / 2 * fullBreakTime) + 5.0f;

            // 현재노드~다음노드까지 확인해서 lookDistance거리안의 가장 높은 곡률 리턴
            float minRadius = std::numeric_limits<float>::max();
            float remainingDistance = lookDistance;
            Vec3 segmentStart = position;
            shared_ptr<RoadNode> segmentNode = m_currentNode;
            size_t pathIndex = m_pathIndex;
            bool isStraight = true;
            while (remainingDistance > 0.0f && segmentNode)
            {
                if (!segmentNode->lane->IsStraight())
                {
                    isStraight = false;
                    minRadius = std::min(minRadius, segmentNode->lane->GetSpline().GetMinRadiusAhead(segmentStart, remainingDistance));
                }

                shared_ptr<RoadNode> nextNode = (pathIndex + 1 < m_path.size()) ? m_path[pathIndex + 1] : nullptr;
                if (!nextNode)
                    break;

                float distanceToNextNode = (nextNode->position - segmentStart).Length();
                if (distanceToNextNode >= remainingDistance)
                    break;

                remainingDistance -= distanceToNextNode;
                segmentStart = nextNode->position;
                segmentNode = nextNode;
                ++pathIndex;
            }
            if (!isStraight)
            {
                constexpr float CURVE_SPEED_COEFF = 1.22f; // 최대 속도 (4.4 * 루트 R)
            }

            Accelerate(1.0f);
            // Accelerate(targetSpeed);

            // steering
            float lookaheadCoeff = 5.0f / m_speed;
            auto targetPosition = m_currentNode->lane->GetSpline().GetLookaheadPoint(position, m_speed * lookaheadCoeff);
            m_targetMarker.GetTransform().SetPosition(ToXMFLOAT3(targetPosition));
            float targetSteer = PurePursuit(targetPosition);
            Steer(targetSteer);
            return BTStatus::Success;
        });
    drive->name = "DriveControl";
    return MakeSequence(std::move(checkNext), std::move(drive));
}