#pragma once

#include <cstdio>
#include <deque>
#include <string>

// Global, in-game log window. Call DebugConsole::Log(...) from anywhere
// to print a line, and DebugConsole::Get().Draw() once per frame to show it.
class DebugConsole
{
public:
    static DebugConsole &Get();

    static void Log(const std::string &line) { Get().LogImpl(line); }
    void Draw();

private:
    DebugConsole() = default;
    ~DebugConsole();

    void LogImpl(const std::string &line);

    static constexpr size_t MAX_LINES = 500;
    std::deque<std::string> m_lines;
    // 크래시 진단용: 콘솔 창은 프로세스가 죽으면 같이 닫혀버리므로, 디스크에 남겨 크래시 후에도
    // 확인할 수 있게 한다. 매 줄마다 flush해서 강제 종료돼도 이미 쓴 줄은 남는다.
    FILE *m_logFile = nullptr;
};
