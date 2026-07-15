#pragma once

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

    void LogImpl(const std::string &line);

    static constexpr size_t MAX_LINES = 500;
    std::deque<std::string> m_lines;
};
