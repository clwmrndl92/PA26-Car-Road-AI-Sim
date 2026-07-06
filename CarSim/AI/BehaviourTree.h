#pragma once

#include <vector>
#include <memory>
#include <functional>

enum class BTStatus
{
    Success,
    Failure,
    Running
};

struct BTNode
{
    virtual ~BTNode() = default;
    virtual BTStatus Tick() = 0;
};

// AND
struct BTSequence : BTNode
{
    std::vector<std::unique_ptr<BTNode>> children;
    BTStatus Tick() override;
};

// OR
struct BTSelector : BTNode
{
    std::vector<std::unique_ptr<BTNode>> children;
    BTStatus Tick() override;
};

struct BTCondition : BTNode
{
    std::function<bool()> check;
    BTCondition(std::function<bool()> fn) : check(fn) {}
    BTStatus Tick() override;
};

struct BTAction : BTNode
{
    std::function<BTStatus()> action;
    BTAction(std::function<BTStatus()> fn) : action(fn) {}
    BTStatus Tick() override;
};

class BehaviourTree
{
public:
    std::unique_ptr<BTNode> root;
    void Tick()
    {
        if (root)
            root->Tick();
    }
};

// 노드를 쉽게 만들어주는 헬퍼 함수들
std::unique_ptr<BTSequence> MakeSequence(std::vector<std::unique_ptr<BTNode>> children);
