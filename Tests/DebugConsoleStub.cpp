#include "Utill/DebugConsole.h"

// HybridAStar.cpp가 DebugConsole::Log를 호출하기 때문에 링크상 정의가 필요하다. 실제
// ImGui 창(Draw())은 이 테스트 실행 파일에서 절대 호출되지 않으므로, ImGui/렌더링 전체를
// 끌어오지 않도록 로그를 그냥 버퍼에만 쌓는 최소 구현만 둔다.
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
DebugConsole::~DebugConsole()
{
}