#include "TestHarness.h"
#include <cstdio>

namespace Test
{
    std::vector<Case> &Registry()
    {
        static std::vector<Case> registry;
        return registry;
    }

    Registrar::Registrar(const std::string &name, std::function<void()> fn)
    {
        Registry().push_back({name, std::move(fn)});
    }

    void Fail(const std::string &expr, const char *file, int line)
    {
        throw Failure{std::string(file) + ":" + std::to_string(line) + ": CHECK(" + expr + ") failed"};
    }
}

int main()
{
    int passed = 0;
    int failed = 0;

    for (const Test::Case &test : Test::Registry())
    {
        try
        {
            test.fn();
            std::printf("[ PASS ] %s\n", test.name.c_str());
            ++passed;
        }
        catch (const Test::Failure &failure)
        {
            std::printf("[ FAIL ] %s\n         %s\n", test.name.c_str(), failure.message.c_str());
            ++failed;
        }
        catch (const std::exception &e)
        {
            std::printf("[ FAIL ] %s\n         unexpected exception: %s\n", test.name.c_str(), e.what());
            ++failed;
        }
    }

    std::printf("\n%d passed, %d failed (of %d)\n", passed, failed, passed + failed);
    return failed == 0 ? 0 : 1;
}
