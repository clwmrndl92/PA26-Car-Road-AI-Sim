#include "BehaviourTree.h"
#include <imgui.h>

BTStatus BTSequence::Tick()
{
    tickedThisFrame = true;
    BTStatus status = BTStatus::Success;
    for (auto &child : children)
    {
        status = child->Tick();
        if (status != BTStatus::Success)
            break;
    }
    lastStatus = status;
    return status;
}

BTStatus BTSelector::Tick()
{
    tickedThisFrame = true;
    BTStatus status = BTStatus::Failure;
    for (auto &child : children)
    {
        status = child->Tick();
        if (status != BTStatus::Failure)
            break;
    }
    lastStatus = status;
    return status;
}

BTStatus BTCondition::Tick()
{
    tickedThisFrame = true;
    lastStatus = check() ? BTStatus::Success : BTStatus::Failure;
    return lastStatus;
}

BTStatus BTAction::Tick()
{
    tickedThisFrame = true;
    lastStatus = action();
    return lastStatus;
}

static const char *BTStatusToString(BTStatus status)
{
    switch (status)
    {
    case BTStatus::Success:
        return "Success";
    case BTStatus::Failure:
        return "Failure";
    case BTStatus::Running:
        return "Running";
    }
    return "?";
}

static std::string BTNodeLabel(const BTNode *node)
{
    return node->name.empty() ? node->GetTypeName() : node->name + " (" + node->GetTypeName() + ")";
}

static void DrawBTLine(const BTNode *node, const std::string &prefix, bool isLast, bool isRoot)
{
    std::string connector = isRoot ? "" : (isLast ? "`-- " : "|-- ");
    std::string mark = node->tickedThisFrame ? "[v] " : "[ ] ";
    const char *statusStr = node->tickedThisFrame ? BTStatusToString(node->lastStatus) : "-";

    ImGui::Text("%s%s%s%s (%s)", prefix.c_str(), connector.c_str(), mark.c_str(), BTNodeLabel(node).c_str(), statusStr);

    if (const auto *children = node->GetChildren())
    {
        std::string childPrefix = prefix + (isRoot ? "" : (isLast ? "    " : "|   "));
        for (size_t i = 0; i < children->size(); ++i)
            DrawBTLine((*children)[i].get(), childPrefix, i + 1 == children->size(), false);
    }
}

void BehaviourTree::Draw(const char *title) const
{
    if (ImGui::Begin(title))
    {
        if (root)
            DrawBTLine(root.get(), "", true, true);
        else
            ImGui::TextUnformatted("(empty tree)");
    }
    ImGui::End();
}
