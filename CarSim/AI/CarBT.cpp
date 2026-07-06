#include "Entities/Car.h"

std::unique_ptr<BTNode> Car::CreateAccelerateNode()
{
    return std::make_unique<BTAction>([this]()
                                      {
        if (m_speed < 10.0f) {
            Accelerate(1);
            return BTStatus::Running;
        }
        Accelerate(0);
        return BTStatus::Success; });
}

std::unique_ptr<BTNode> Car::BuildBehaviourTree()
{
    auto root = std::make_unique<BTSequence>();
    root->children.push_back(CreateAccelerateNode());
    // 여기에 다른 노드들을 추가
    return root;
}
