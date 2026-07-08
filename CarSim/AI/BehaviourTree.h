#pragma once

#include <vector>
#include <memory>
#include <functional>
#include <string>

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

    // 디버그 UI용 표시 이름/타입 및 마지막 틱 결과 (BehaviourTree::Draw 참고)
    virtual const char *GetTypeName() const = 0;
    virtual const std::vector<std::unique_ptr<BTNode>> *GetChildren() const { return nullptr; }

    std::string name;
    BTStatus lastStatus = BTStatus::Failure;
    bool tickedThisFrame = false;
};

// AND
struct BTSequence : BTNode
{
    std::vector<std::unique_ptr<BTNode>> children;
    BTStatus Tick() override;
    const char *GetTypeName() const override { return "Sequence"; }
    const std::vector<std::unique_ptr<BTNode>> *GetChildren() const override { return &children; }
};

// OR
struct BTSelector : BTNode
{
    std::vector<std::unique_ptr<BTNode>> children;
    BTStatus Tick() override;
    const char *GetTypeName() const override { return "Selector"; }
    const std::vector<std::unique_ptr<BTNode>> *GetChildren() const override { return &children; }
};

struct BTCondition : BTNode
{
    std::function<bool()> check;
    BTCondition(std::function<bool()> fn) : check(fn) {}
    BTStatus Tick() override;
    const char *GetTypeName() const override { return "Condition"; }
};

struct BTAction : BTNode
{
    std::function<BTStatus()> action;
    BTAction(std::function<BTStatus()> fn) : action(fn) {}
    BTStatus Tick() override;
    const char *GetTypeName() const override { return "Action"; }
};

// 매 틱마다 재귀적으로 tickedThisFrame을 초기화 (이번 프레임에 실행되지 않은 노드와 구분하기 위함)
inline void ResetBTTickFlags(BTNode *node)
{
    if (!node)
        return;
    node->tickedThisFrame = false;
    if (const auto *children = node->GetChildren())
        for (auto &child : *children)
            ResetBTTickFlags(child.get());
}

class BehaviourTree
{
public:
    std::unique_ptr<BTNode> root;
    void Tick()
    {
        if (root)
        {
            ResetBTTickFlags(root.get());
            root->Tick();
        }
    }

    // ImGui 창으로 트리를 텍스트로 그리고, 이번 프레임에 실행된(체크된) 노드를 표시
    void Draw(const char *title) const;
};

// 노드를 쉽게 만들어주는 헬퍼 함수들
template <typename... Children>
std::unique_ptr<BTSequence> MakeSequence(Children &&...children)
{
    auto seq = std::make_unique<BTSequence>();
    (seq->children.push_back(std::forward<Children>(children)), ...);
    return seq;
}

template <typename... Children>
std::unique_ptr<BTSelector> MakeSelector(Children &&...children)
{
    auto sel = std::make_unique<BTSelector>();
    (sel->children.push_back(std::forward<Children>(children)), ...);
    return sel;
}
