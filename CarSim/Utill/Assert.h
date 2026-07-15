#pragma once
#include "DebugConsole.h"
#include <string>
#include <intrin.h>

namespace AssertDetail
{
    inline void ReportFailure(const char *expr, const char *file, int line)
    {
        DebugConsole::Log(std::string("[Assert 실패] ") + expr + " (" + file + ":" + std::to_string(line) + ")");
    }
} // namespace AssertDetail

#if defined(DEBUG) || defined(_DEBUG)
#define Assert(expr)                                                \
    do                                                              \
    {                                                               \
        if (!(expr))                                                \
        {                                                           \
            AssertDetail::ReportFailure(#expr, __FILE__, __LINE__); \
            __debugbreak();                                         \
        }                                                           \
    } while (0)
#else
#define Assert(expr)                                                \
    do                                                              \
    {                                                               \
        if (!(expr))                                                \
            AssertDetail::ReportFailure(#expr, __FILE__, __LINE__); \
    } while (0)
#endif
