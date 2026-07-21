#include "DebugConsole.h"
#include <imgui.h>
#include <cstdio>

DebugConsole &DebugConsole::Get()
{
    static DebugConsole instance;
    return instance;
}

DebugConsole::~DebugConsole()
{
    if (m_logFile != nullptr)
        std::fclose(m_logFile);
}

void DebugConsole::LogImpl(const std::string &line)
{
    // 크래시해도 남게 디스크 파일에도 적는다. 매번 flush해서 강제 종료돼도 이미 쓴 줄은 남는다.
    if (m_logFile == nullptr)
        m_logFile = std::fopen("debug_log.txt", "w");

    if (m_logFile != nullptr)
    {
        std::fputs(line.c_str(), m_logFile);
        std::fputc('\n', m_logFile);
        std::fflush(m_logFile);
    }

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
        
        for (const auto &line : m_lines)
        ImGui::TextUnformatted(line.c_str());
        
        if (ImGui::Button("Copy All"))
            ImGui::LogToClipboard();

        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);

        ImGui::LogFinish();
    }
    ImGui::End();
}
