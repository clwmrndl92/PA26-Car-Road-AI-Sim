#include "Entities/Car.h"

std::unique_ptr<BTNode> Car::BuildBehaviourTree()
{
    auto root = MakeSelector({
        StopNode(),
        ChangeLineNode(),
        DriveNode(),
    });
    return root;
}

std::unique_ptr<BTNode> Car::StopNode()
{
    return MakeSequence({std::make_unique<BTCondition>([this]()
                                                       { return false; }),
                         std::make_unique<BTAction>([this]()
                                                    {
            // 차량이 멈췄을 때 수행할 동작
            if(m_speed > 0.0f) {
                Accelerate(-1);
                return BTStatus::Running;
            }
            return BTStatus::Success; })});
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
    return std::make_unique<BTAction>([this]()
                                      {
        // 차량이 주행 중일 때 수행할 동작
        return BTStatus::Running; });
}