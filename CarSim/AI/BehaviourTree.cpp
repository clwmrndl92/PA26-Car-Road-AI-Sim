#include "BehaviourTree.h"

BTStatus BTSequence::Tick()
{
    for (auto &child : children)
    {
        BTStatus status = child->Tick();
        if (status != BTStatus::Success)
            return status;
    }
    return BTStatus::Success;
}

BTStatus BTSelector::Tick()
{
    for (auto &child : children)
    {
        BTStatus status = child->Tick();
        if (status != BTStatus::Failure)
            return status;
    }
    return BTStatus::Failure;
}

BTStatus BTCondition::Tick()
{
    return check() ? BTStatus::Success : BTStatus::Failure;
}

BTStatus BTAction::Tick()
{
    return action();
}

std::unique_ptr<BTSequence> MakeSequence(std::vector<std::unique_ptr<BTNode>> children)
{
    auto seq = std::make_unique<BTSequence>();
    seq->children = std::move(children);
    return seq;
}

std::unique_ptr<BTSelector> MakeSelector(std::vector<std::unique_ptr<BTNode>> children)
{
    auto seq = std::make_unique<BTSelector>();
    seq->children = std::move(children);
    return seq;
}
