#include "Entities/Car.h"
#include "Core/DebugConsole.h"
#include <algorithm>
#include <limits>

std::unique_ptr<BTNode> Car::BuildBehaviourTree()
{
    return MakeSelector(
        StopNode(),
        ChangeLineNode(),
        DriveNode());
}

std::unique_ptr<BTNode> Car::StopNode()
{
    return MakeSequence(
        std::make_unique<BTCondition>([this]()
                                      { return IsArrived(); }),
        std::make_unique<BTAction>(
            [this]()
            {
            // 차량이 멈췄을 때 수행할 동작
            if(m_speed > 0.0f) {
                Accelerate(0.0f);
                return BTStatus::Running;
            }
            m_destNode = nullptr;
            m_currentNode = nullptr;
            return BTStatus::Success; }));
}

std::unique_ptr<BTNode> Car::ChangeLineNode()
{

    return std::make_unique<BTAction>([this]()
                                      {
        // 차량이 차선을 변경할 때 수행할 동작
        return BTStatus::Failure; });
}

std::unique_ptr<BTNode> Car::DriveNode()
{
    return std::make_unique<BTAction>(
        [this]()
        {
            // path find
            Vec3 position = m_rigidbody.GetPosition();
            if (!m_currentNode)
            {
                auto closestNode = m_RoadDataManager->GetClosestNode(position);
                m_path = m_RoadDataManager->FindPath(closestNode, m_destNode);
                m_pathIndex = -1;

                m_currentNode = make_shared<RoadNode>();
                m_currentNode->position = closestNode->position;
                m_currentNode->nodeType = RoadNodeType::End;
                m_currentNode->lanePosition = 1;
                Spline spline = Spline({position - GetForwardAxis(),
                                        position,
                                        closestNode->position,
                                        closestNode->position + closestNode->GetDirection()});
                m_currentNode->lane = make_shared<Lane>(-1, spline, closestNode->lane->GetRoad());
                if (m_path.empty())
                {
                    m_destNode = nullptr;
                    m_currentNode = nullptr;
                    return BTStatus::Failure;
                }
            };
            float currentNodeDistance = (m_currentNode->position - position).Length();
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
            }

            // speed control
            float maxSpeed = m_currentNode->lane->GetRoad()->GetSpeedLimit();
            float targetSpeed = m_maxSpeed;

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
                targetSpeed = std::min(m_maxSpeed, CURVE_SPEED_COEFF * std::sqrt(minRadius));
                DebugConsole::Get().Log("minRadius: " + std::to_string(minRadius));
                DebugConsole::Get().Log("TargetSpeed (m/s): " + std::to_string(targetSpeed));
                DebugConsole::Get().Log("TargetSpeed (km/h): " + std::to_string(targetSpeed * 3.6f));
            }

            Accelerate(1.0f);
            // Accelerate(targetSpeed);

            // steering
            float lookaheadCoeff = 5.0f / m_speed;
            auto targetPosition = m_currentNode->lane->GetSpline().GetLookaheadPoint(position, m_speed * lookaheadCoeff);
            m_targetMarker.GetTransform().SetPosition(ToXMFLOAT3(targetPosition));
            float targetSteer = PurePursuit(targetPosition);
            Steer(targetSteer);
            return BTStatus::Running;
        });
}