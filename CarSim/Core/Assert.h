#pragma once
#include "DebugConsole.h"
#include <string>
#include <intrin.h>

// 표준 assert()와 달리 릴리즈 빌드에서도 사라지지 않고 DebugConsole에 로그를 남긴다.
// 디버그 빌드에서는 추가로 디버거를 중단시킨다. 두 빌드 모두 실행은 계속 이어간다
// (invariant가 깨져도 곧바로 크래시하지 않도록, 호출부는 여전히 fallback을 갖추고 있어야 한다).
namespace AssertDetail
{
inline void ReportFailure(const char *expr, const char *file, int line)
{
    DebugConsole::Get().Log(std::string("[Assert 실패] ") + expr + " (" + file + ":" + std::to_string(line) + ")");
}
} // namespace AssertDetail

#if defined(DEBUG) || defined(_DEBUG)
#define Assert(expr)                                                     \
    do                                                                   \
    {                                                                    \
        if (!(expr))                                                    \
        {                                                                \
            AssertDetail::ReportFailure(#expr, __FILE__, __LINE__);      \
            __debugbreak();                                              \
        }                                                                \
    } while (0)
#else
#define Assert(expr)                                                \
    do                                                              \
    {                                                                \
        if (!(expr))                                                \
            AssertDetail::ReportFailure(#expr, __FILE__, __LINE__); \
    } while (0)
#endif
