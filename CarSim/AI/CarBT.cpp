#include "Entities/Car.h"
#include "Core/DebugConsole.h"

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
            Vec3 position = m_rigidbody.GetPosition();
            if (!m_currentNode)
            {
                m_currentNode = m_RoadDataManager->GetClosestNode(position);
                m_path = m_RoadDataManager->FindPath(m_currentNode, m_destNode);
                m_pathIndex = 0;
                if (m_path.empty())
                {
                    m_destNode = nullptr;
                    m_currentNode = nullptr;
                    return BTStatus::Failure;
                }
            };
            float currentNodeDistance = (m_currentNode->position - position).Length();
            DebugConsole::Get().Log("position: " + ToString(m_currentNode->position));
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
            }

            const float speed = 5.0f;
            const float lookaheadCoeff = 2.0f;
            Accelerate(speed);
            auto targetPosition = m_currentNode->lane->GetLookaheadPoint(position, speed * lookaheadCoeff, m_currentNode->position);
            m_targetMarker.GetTransform().SetPosition(ToXMFLOAT3(targetPosition));
            auto targetSteer = PurePursuit(targetPosition);
            Steer(targetSteer);
            return BTStatus::Running;
        });
}