#pragma once
#include <functional>
#include <string>
#include <vector>

// 의존성 없는 최소 테스트 하네스. TEST_CASE로 등록한 함수들을 main()이 전부 돌리고
// pass/fail 개수와 실패 지점(파일:라인)을 출력한다. CHECK 실패는 예외로 던져서 해당
// 테스트 케이스만 중단시키고 나머지 케이스는 계속 진행한다.
namespace Test
{
    struct Failure
    {
        std::string message;
    };

    struct Case
    {
        std::string name;
        std::function<void()> fn;
    };

    std::vector<Case> &Registry();

    struct Registrar
    {
        Registrar(const std::string &name, std::function<void()> fn);
    };

    [[noreturn]] void Fail(const std::string &expr, const char *file, int line);
}

#define TEST_CASE(name)                                                          \
    static void name();                                                         \
    static ::Test::Registrar name##_registrar(#name, name);                      \
    static void name()

#define CHECK(expr)                                                              \
    do                                                                          \
    {                                                                          \
        if (!(expr))                                                            \
            ::Test::Fail(#expr, __FILE__, __LINE__);                           \
    } while (0)
