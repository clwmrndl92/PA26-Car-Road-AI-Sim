#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <Psapi.h>

#include "DebugConsole.h"
#include <chrono>
#include <cstdio>
#include <string>

// 필요한 곳에 수동으로 찍어보는 성능/메모리 로그.
// 메모리 : PERF_LOG_MEMORY("FindPath 진입 전");
// 성능 : void Foo() { PERF_LOG_SCOPE("Foo"); ... }
namespace PerfLog
{
    // DebugConsole(게임 내 ImGui 창)과 표준출력에 동시에 찍는다. Tests처럼 DebugConsole이
    // ImGui 없이 그냥 버퍼에만 쌓이는 곳(콘솔 앱)에서도 이 printf로 눈에 보이게 하기 위함.
    inline void Emit(const std::string &line)
    {
        DebugConsole::Log(line);
        std::printf("%s\n", line.c_str());
    }

    // 생성부터 소멸까지(return/예외로 스코프를 벗어나는 경우 포함) 걸린 시간을 ms 단위로 로그.
    // 직접 쓰기보다 PERF_LOG_SCOPE(label) / PERF_LOG_MEMORY(label) 매크로로 선언해서 쓰는 걸 권장.
    class ScopedTimer
    {
    public:
        explicit ScopedTimer(std::string label)
            : m_label(std::move(label)), m_begin(std::chrono::steady_clock::now())
        {
        }

        ~ScopedTimer()
        {
            double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m_begin).count();
            Emit(m_label + ": " + std::to_string(ms) + " ms");
        }

        ScopedTimer(const ScopedTimer &) = delete;
        ScopedTimer &operator=(const ScopedTimer &) = delete;

    private:
        std::string m_label;
        std::chrono::steady_clock::time_point m_begin;
    };

    inline void LogMemory(const std::string &label)
    {
        PROCESS_MEMORY_COUNTERS counters{};
        if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)))
        {
            double currentMB = static_cast<double>(counters.WorkingSetSize) / (1024.0 * 1024.0);
            double peakMB = static_cast<double>(counters.PeakWorkingSetSize) / (1024.0 * 1024.0);

            Emit(label + ": Current " + std::to_string(currentMB) + " MB | Peak " + std::to_string(peakMB) + " MB");
        }
    }
}

#define PERF_LOG_CONCAT_INNER(a, b) a##b
#define PERF_LOG_CONCAT(a, b) PERF_LOG_CONCAT_INNER(a, b)

#define PERF_LOG_SCOPE(label) ::PerfLog::ScopedTimer PERF_LOG_CONCAT(perfLogScope_, __LINE__)(label)
#define PERF_LOG_MEMORY(label) ::PerfLog::LogMemory(label)
