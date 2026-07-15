#include "DebugConsole.h"
#include <imgui.h>

DebugConsole &DebugConsole::Get()
{
    static DebugConsole instance;
    return instance;
}

void DebugConsole::LogImpl(const std::string &line)
{
    m_lines.push_back(line);
    if (m_lines.size() > MAX_LINES)
        m_lines.pop_front();
}

void DebugConsole::Draw()
{
    const float padding = 10.0f;
    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    ImVec2 windowPos(displaySize.x - padding, displaySize.y - padding);
    ImGui::SetNextWindowPos(windowPos, ImGuiCond_FirstUseEver, ImVec2(1.0f, 1.0f));
    ImGui::SetNextWindowSize(ImVec2(400.0f, 300.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Console"))
    {
        if (ImGui::Button("Copy All"))
            ImGui::LogToClipboard();

        for (const auto &line : m_lines)
            ImGui::TextUnformatted(line.c_str());

        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);

        ImGui::LogFinish(); // LogToClipboard가 안 켜져 있으면 아무것도 안 함
    }
    ImGui::End();
}
