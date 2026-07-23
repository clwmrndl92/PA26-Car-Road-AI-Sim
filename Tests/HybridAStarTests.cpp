#include "TestHarness.h"
#include "Nav/HybridAStar.h"
#include "Nav/ReedsShepp.h"
#include "Utill/MathUtil.h"
#include <chrono>
#include <cstdio>
#include <Utill/PerfLog.h>

// 장애물 없이 정면으로 뚫린 목표 -> 성공하고, 경로 길이가 직선 거리와 크게 차이나지 않아야 함
// (불필요하게 돌아가지 않는지 확인).
TEST_CASE(FindPath_NoObstacles_Succeeds)
{
    HybridAStar::VehicleShape shape;
    Vec3 start(0.0f, 0.0f, 0.0f);
    Vec3 goal(10.0f, 0.0f, 0.0f);

    bool foundPath = false;
    ReedsShepp::Path path = HybridAStar::FindPath(start, 0.0f, goal, 0.0f, {}, shape, foundPath);

    CHECK(foundPath);
    float length = ReedsShepp::GetPathLength(path);
    CHECK(length > 0.0f);
    CHECK(length < 15.0f); // 직선 거리 10 대비 여유 있게
}

// 출발-목표 사이를 가로막는 벽 하나 -> 우회해서라도 성공해야 하고, 우회했으니 직선 거리보다는 길어야 함.
TEST_CASE(FindPath_DetoursAroundWall_Succeeds)
{
    HybridAStar::VehicleShape shape;
    Vec3 start(0.0f, 0.0f, 0.0f);
    Vec3 goal(10.0f, 0.0f, 0.0f);

    HybridAStar::Obstacle wall;
    wall.center = Vec3(5.0f, 0.0f, 0.0f);
    wall.headingRad = 90.0f; // 길이(halfLength) 축이 Z를 향하게 돌려서 진행 방향을 가로막는 벽으로 사용
    wall.halfLength = 3.0f;  // z in [-3, 3]
    wall.halfWidth = 0.5f;   // x in [4.5, 5.5]
    std::vector<HybridAStar::Obstacle> obstacles{wall};

    bool foundPath = false;
    ReedsShepp::Path path = HybridAStar::FindPath(start, 0.0f, goal, 0.0f, obstacles, shape, foundPath);

    CHECK(foundPath);
    CHECK(ReedsShepp::GetPathLength(path) > 10.0f);
}

// 목표를 사방으로 완전히 둘러싼 장애물 -> 도달 불가능하니 실패로 보고해야 하고(크래시/무한루프 없이),
// maxExpansions 예산 안에서 끝나야 함.
TEST_CASE(FindPath_GoalBoxedIn_FailsGracefully)
{
    HybridAStar::VehicleShape shape;
    Vec3 start(0.0f, 0.0f, 0.0f);
    Vec3 goal(20.0f, 0.0f, 0.0f);

    std::vector<HybridAStar::Obstacle> obstacles;
    // 목표를 감싸는 4면의 두꺼운 벽 (틈 없이). heading 0은 halfLength가 X축, heading 90은
    // halfLength가 Z축을 향하므로 남/북 벽과 동/서 벽은 halfLength·halfWidth가 서로 뒤바뀐다.
    obstacles.push_back({Vec3(20.0f, 0.0f, 4.0f), 5.0f, 0.5f, 0.0f});  // 북쪽: x 15~25, z 3.5~4.5
    obstacles.push_back({Vec3(20.0f, 0.0f, -4.0f), 5.0f, 0.5f, 0.0f}); // 남쪽: x 15~25, z -4.5~-3.5
    obstacles.push_back({Vec3(24.0f, 0.0f, 0.0f), 4.5f, 0.5f, 90.0f}); // 동쪽: z -4.5~4.5, x 23.5~24.5
    obstacles.push_back({Vec3(16.0f, 0.0f, 0.0f), 4.5f, 0.5f, 90.0f}); // 서쪽: z -4.5~4.5, x 15.5~16.5

    bool foundPath = true; // 일부러 반대값으로 초기화해서 FindPath가 실제로 false를 채우는지 확인
    HybridAStar::FindPath(start, 0.0f, goal, 0.0f, obstacles, shape, foundPath);

    CHECK(!foundPath);
}

// 위 케이스는 maxExpansions 소진과 open set 소진(HybridAStar.cpp의 두 실패 분기) 중 어느 쪽을
// 타는지 확실치 않다. 이 케이스는 시작 pose를 거대한 장애물 하나로 통째로 뒤덮어서, 첫 확장에서
// 6방향(전/후진 x 좌/직진/우) 전부가 그 장애물 안에 그대로 남아 즉시 충돌 처리된다 -> openSet에
// 아무것도 안 쌓이고 첫 pop 직후 바로 비어서, maxExpansions 값과 무관하게 "open set exhausted"
// 분기(HybridAStar.cpp:307)를 항상, 그것도 즉시(expansions==1) 탄다. PERF_LOG_MEMORY로 그
// 분기를 확인하고 싶을 때 이 케이스로 재현하면 된다.
TEST_CASE(FindPath_StartEngulfed_AlwaysExhaustsOpenSet)
{
    HybridAStar::VehicleShape shape;
    Vec3 start(0.0f, 0.0f, 0.0f);
    Vec3 goal(100.0f, 0.0f, 0.0f); // 어차피 못 가니 값 자체는 중요하지 않음, 멀리 떨어뜨려만 둠

    HybridAStar::Obstacle giant;
    giant.center = Vec3(0.0f, 0.0f, 0.0f);
    giant.halfLength = 10.0f; // stepSize(0.5)로는 절대 못 벗어나는 크기
    giant.halfWidth = 10.0f;
    giant.headingRad = 0.0f;
    std::vector<HybridAStar::Obstacle> obstacles{giant};

    bool foundPath = true; // 일부러 반대값으로 초기화해서 FindPath가 실제로 false를 채우는지 확인
    HybridAStar::FindPath(start, 0.0f, goal, 0.0f, obstacles, shape, foundPath);

    CHECK(!foundPath);
}

// 위 케이스는 반대쪽 실패 분기(open set exhausted)를 확정적으로 태우는 것이었다. 이 케이스는
// HybridAStar.cpp:249의 "maxExpansions 초과" 분기를 확정적으로 태운다. FindPath_DetoursAroundWall_
// Succeeds와 같은 벽으로 직선 숏컷을 막아서(그래야 시작 노드에서 바로 성공해버리지 않음), params.
// maxExpansions를 1로 극단적으로 낮춰서 두 번째 루프 반복에서 무조건 예산 초과로 실패하게 만든다.
TEST_CASE(FindPath_TinyBudget_ExceedsMaxExpansions)
{
    HybridAStar::VehicleShape shape;
    Vec3 start(0.0f, 0.0f, 0.0f);
    Vec3 goal(10.0f, 0.0f, 0.0f);

    HybridAStar::Obstacle wall;
    wall.center = Vec3(5.0f, 0.0f, 0.0f);
    wall.headingRad = 90.0f;
    wall.halfLength = 3.0f;
    wall.halfWidth = 0.5f;
    std::vector<HybridAStar::Obstacle> obstacles{wall};

    HybridAStar::Params params;
    params.maxExpansions = 1; // 시작 노드 한 번만 확장하고 그다음 루프에서 바로 예산 초과되게

    bool foundPath = true; // 일부러 반대값으로 초기화해서 FindPath가 실제로 false를 채우는지 확인
    HybridAStar::FindPath(start, 0.0f, goal, 0.0f, obstacles, shape, foundPath, params);

    CHECK(!foundPath);
}

// Checks whether a complex (chicane-style) road is passable: two obstacles are staggered on
// opposite sides of the z-axis, so avoiding one forces the car into the other unless it weaves
// in an S-curve. The straight-line Reeds-Shepp shortcut from the start pose always hits one of
// the two obstacles, so an actual grid search is required. Unlike the first version of this
// obstacle layout, this one only needs a ~2m lateral shift across a 4m longitudinal gap
// (turning radius is 3m), which is kinematically achievable, so it tests whether the search
// budget is enough rather than testing a geometrically impossible course. maxExpansions=50 is
// meant purely to observe whether the search still succeeds at that budget, so the result is
// printed rather than enforced with CHECK.
TEST_CASE(FindPath_ComplexChicane_Budget50)
{
    HybridAStar::VehicleShape shape;
    Vec3 start(0.0f, 0.0f, 0.0f);
    Vec3 goal(28.0f, 0.0f, 0.0f);

    std::vector<HybridAStar::Obstacle> obstacles{
        {Vec3(8.0f, 0.0f, 1.5f), 3.0f, 1.5f, 0.0f},   // x:[5,11]  z:[0,3]   -> must detour below (z<0)
        {Vec3(18.0f, 0.0f, -1.5f), 3.0f, 1.5f, 0.0f}, // x:[15,21] z:[-3,0]  -> must detour above (z>0)
    };

    HybridAStar::Params params;
    params.maxExpansions = 5000;

    int expansionsUsed = -1;
    params.onSearchFailed = [&expansionsUsed](int expansions)
    { expansionsUsed = expansions; };

    bool foundPath = false;
    auto begin = std::chrono::steady_clock::now();
    ReedsShepp::Path path = HybridAStar::FindPath(start, 0.0f, goal, 0.0f, obstacles, shape, foundPath, params);
    auto end = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - begin).count();

    if (foundPath)
        std::printf("         FindPath_ComplexChicane_Budget50: succeeded, path length %.2f, %.3f ms\n",
                    ReedsShepp::GetPathLength(path), ms);
    else if (expansionsUsed > params.maxExpansions)
        std::printf("         FindPath_ComplexChicane_Budget50: failed, exceeded maxExpansions=%d, %.3f ms\n",
                    params.maxExpansions, ms);
    else
        std::printf("         FindPath_ComplexChicane_Budget50: failed, open set exhausted after only %d expansions (budget was %d), %.3f ms\n",
                    expansionsUsed, params.maxExpansions, ms);
}

// Simpler comparison case for the S-curve above: a single obstacle sits directly on the straight
// path, so the car only has to bulge to one side and back (a "C" detour) instead of reversing
// direction like the chicane. Same maxExpansions=50 budget and diagnostic hook, so the two
// printed results can be compared directly in the presentation.
TEST_CASE(FindPath_SingleObstacle_CDetour_Budget50)
{
    HybridAStar::VehicleShape shape;
    Vec3 start(0.0f, 0.0f, 0.0f);
    Vec3 goal(20.0f, 0.0f, 0.0f);

    std::vector<HybridAStar::Obstacle> obstacles{
        {Vec3(10.0f, 0.0f, 0.0f), 2.0f, 1.5f, 0.0f}, // x:[8,12] z:[-1.5,1.5], centered right on the straight path
    };

    HybridAStar::Params params;
    params.maxExpansions = 5000;

    int expansionsUsed = -1;
    params.onSearchFailed = [&expansionsUsed](int expansions)
    { expansionsUsed = expansions; };

    bool foundPath = false;
    auto begin = std::chrono::steady_clock::now();
    ReedsShepp::Path path = HybridAStar::FindPath(start, 0.0f, goal, 0.0f, obstacles, shape, foundPath, params);
    auto end = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - begin).count();

    if (foundPath)
        std::printf("         FindPath_SingleObstacle_CDetour_Budget50: succeeded, path length %.2f, %.3f ms\n",
                    ReedsShepp::GetPathLength(path), ms);
    else if (expansionsUsed > params.maxExpansions)
        std::printf("         FindPath_SingleObstacle_CDetour_Budget50: failed, exceeded maxExpansions=%d, %.3f ms\n",
                    params.maxExpansions, ms);
    else
        std::printf("         FindPath_SingleObstacle_CDetour_Budget50: failed, open set exhausted after only %d expansions (budget was %d), %.3f ms\n",
                    expansionsUsed, params.maxExpansions, ms);
}

// 발표용 벤치마크: 목표를 완전히 둘러싸 항상 실패하게 만들고(FindPath_GoalBoxedIn_FailsGracefully와
// 동일 배치), maxExpansions를 1/2000/5000으로 바꿔가며 그 확장 횟수까지 도달했을 때의 메모리 증가분
// (진입 전/후 WorkingSet 델타)과 소요 시간을 여러 번 반복해 평균으로 낸다.
TEST_CASE(FindPath_Benchmark_ExpansionsMemoryDelta)
{
    HybridAStar::VehicleShape shape;
    Vec3 start(0.0f, 0.0f, 0.0f);
    Vec3 goal(20.0f, 0.0f, 0.0f);

    std::vector<HybridAStar::Obstacle> obstacles;
    obstacles.push_back({Vec3(20.0f, 0.0f, 4.0f), 5.0f, 0.5f, 0.0f});
    obstacles.push_back({Vec3(20.0f, 0.0f, -4.0f), 5.0f, 0.5f, 0.0f});
    obstacles.push_back({Vec3(24.0f, 0.0f, 0.0f), 4.5f, 0.5f, 90.0f});
    obstacles.push_back({Vec3(16.0f, 0.0f, 0.0f), 4.5f, 0.5f, 90.0f});

    const int expansionTargets[] = {1, 50, 100, 500, 2000, 5000};
    const int trialsPerTarget = 5;

    // 워밍업: 첫 호출의 페이지 폴트/할당자 초기화 비용이 첫 측정치를 왜곡하지 않도록 미리 한 번 태운다.
    {
        HybridAStar::Params warmupParams;
        warmupParams.maxExpansions = 50;
        bool warmupFound = true;
        HybridAStar::FindPath(start, 0.0f, goal, 0.0f, obstacles, shape, warmupFound, warmupParams);
    }

    for (int target : expansionTargets)
    {
        HybridAStar::Params params;
        params.maxExpansions = target;

        double totalDeltaMB = 0.0;
        double totalMs = 0.0;

        for (int trial = 0; trial < trialsPerTarget; ++trial)
        {
            double memBefore = PerfLog::GetWorkingSetMB();
            double memAtFailure = memBefore;
            // nodes/openSet/closedSet가 return과 함께 소멸돼버리므로, 실패가 확정되는 그 순간
            // (컨테이너가 아직 살아있을 때) 내부에서 값을 뽑아온다. 호출이 끝난 뒤 밖에서 재면
            // 이미 정리된 뒤라 델타가 의미 없어진다(때로 음수까지 나옴).
            params.onSearchFailed = [&memAtFailure](int)
            { memAtFailure = PerfLog::GetWorkingSetMB(); };

            auto begin = std::chrono::steady_clock::now();

            bool foundPath = true; // GoalBoxedIn 배치는 항상 실패해야 하니 반대값으로 초기화해서 확인
            HybridAStar::FindPath(start, 0.0f, goal, 0.0f, obstacles, shape, foundPath, params);

            auto end = std::chrono::steady_clock::now();

            CHECK(!foundPath);
            totalDeltaMB += (memAtFailure - memBefore);
            totalMs += std::chrono::duration<double, std::milli>(end - begin).count();
        }

        std::printf("         FindPath_Benchmark_ExpansionsMemoryDelta: maxExpansions=%d -> avg delta %.3f MB / avg %.3f ms (%d trials)\n",
                    target, totalDeltaMB / trialsPerTarget, totalMs / trialsPerTarget, trialsPerTarget);
    }
}

TEST_CASE(IsColliding_Basic)
{
    HybridAStar::VehicleShape shape; // halfLength 2.2, halfWidth 1.0, pivotToCenter 0

    HybridAStar::Obstacle obstacle;
    obstacle.center = Vec3(0.0f, 0.0f, 0.0f);
    obstacle.halfLength = 2.0f;
    obstacle.halfWidth = 2.0f;
    obstacle.headingRad = 0.0f;
    std::vector<HybridAStar::Obstacle> obstacles{obstacle};

    CHECK(HybridAStar::IsColliding(Vec3(0.0f, 0.0f, 0.0f), 0.0f, obstacles, shape));
    CHECK(!HybridAStar::IsColliding(Vec3(10.0f, 0.0f, 0.0f), 0.0f, obstacles, shape));
    CHECK(!HybridAStar::IsColliding(Vec3(50.0f, 0.0f, 50.0f), 0.0f, obstacles, shape));
}

// 프로파일러로 잡아볼 무거운 케이스: 장애물이 몇 개 섞인 시나리오를 여러 번 반복 실행해서
// 표본을 충분히 만든다. 정확성 체크는 최소로 두고, 실행 시간 출력이 핵심.
TEST_CASE(FindPath_Benchmark)
{
    HybridAStar::VehicleShape shape;
    Vec3 start(0.0f, 0.0f, 0.0f);
    Vec3 goal(20.0f, 0.0f, 6.0f);

    std::vector<HybridAStar::Obstacle> obstacles{
        {Vec3(6.0f, 0.0f, 0.0f), 3.0f, 0.5f, 90.0f},
        {Vec3(12.0f, 0.0f, 6.0f), 0.5f, 3.0f, 0.0f},
        {Vec3(16.0f, 0.0f, -2.0f), 2.0f, 1.0f, 30.0f},
    };

    const int iterations = 20;
    auto begin = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; ++i)
    {
        bool foundPath = false;
        HybridAStar::FindPath(start, 0.0f, goal, 0.0f, obstacles, shape, foundPath);
        CHECK(foundPath);
    }
    auto end = std::chrono::steady_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - begin).count();
    std::printf("         FindPath_Benchmark: %.3f ms/call (%d calls)\n", totalMs / iterations, iterations);
}
