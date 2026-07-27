#include "TestHarness.h"
#include "Nav/VehicleCollision.h"
#include <chrono>
#include <cstdio>
#include <random>
#include <vector>

namespace
{
    std::vector<VehicleCollision::Obstacle> MakeObstacles(int count, unsigned seed)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> pos(-100.0f, 100.0f);
        std::uniform_real_distribution<float> heading(0.0f, 6.2831853f);

        std::vector<VehicleCollision::Obstacle> obstacles;
        obstacles.reserve(count);
        for (int i = 0; i < count; ++i)
        {
            VehicleCollision::Obstacle obstacle;
            obstacle.center = Vec3(pos(rng), 0.0f, pos(rng));
            obstacle.halfLength = 2.0f;
            obstacle.halfWidth = 1.0f;
            obstacle.headingRad = heading(rng);
            obstacles.push_back(obstacle);
        }
        return obstacles;
    }

    // obstacleCount가 커질수록 iterations를 반비례로 줄여서, 티어마다 "장애물 검사 총 횟수"가
    // 얼추 같아지게(=걸리는 시간도 비슷해지게) 만든다 -- 그래야 avg us/call로 티어끼리 비교가 쉽다.
    void BenchIsColliding(int obstacleCount)
    {
        constexpr long long TARGET_CHECKS = 2'000'000;
        int iterations = static_cast<int>(TARGET_CHECKS / obstacleCount);

        std::vector<VehicleCollision::Obstacle> obstacles = MakeObstacles(obstacleCount, 1234u + obstacleCount);
        VehicleCollision::VehicleShape shape;

        std::mt19937 rng(5678u + obstacleCount);
        std::uniform_real_distribution<float> pos(-100.0f, 100.0f);
        std::uniform_real_distribution<float> heading(0.0f, 6.2831853f);

        // 결과를 실제로 소비해서, 컴파일러가 IsColliding 호출 자체를 죽은 코드로 지워버리지 못하게 한다.
        volatile int hitCount = 0;

        auto startTime = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iterations; ++i)
        {
            Vec3 position(pos(rng), 0.0f, pos(rng));
            float headingRad = heading(rng);
            if (VehicleCollision::IsColliding(position, headingRad, obstacles, shape))
                ++hitCount;
        }
        auto endTime = std::chrono::high_resolution_clock::now();

        double totalMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        double perCallUs = (totalMs * 1000.0) / iterations;
        std::printf("[bench] IsColliding      obstacles=%3d  iterations=%7d  total=%7.2fms  avg=%6.3fus/call  hits=%d\n",
                    obstacleCount, iterations, totalMs, perCallUs, hitCount);
    }

    // BenchIsColliding과 같은 티어/총량 방식. maxDistance는 obstacle들이 흩어진 범위(-100~100)를
    // 여유있게 덮도록 잡아서, 광선이 실제로 장애물을 맞힐 기회가 IsColliding 쪽 "차량 위치가 장애물과
    // 겹칠 확률"과 비슷한 정도가 되게 했다.
    void BenchRaycastObstacles(int obstacleCount)
    {
        constexpr long long TARGET_CHECKS = 2'000'000;
        constexpr float MAX_DISTANCE = 300.0f;
        int iterations = static_cast<int>(TARGET_CHECKS / obstacleCount);

        std::vector<VehicleCollision::Obstacle> obstacles = MakeObstacles(obstacleCount, 1234u + obstacleCount);

        std::mt19937 rng(5678u + obstacleCount);
        std::uniform_real_distribution<float> pos(-100.0f, 100.0f);
        std::uniform_real_distribution<float> directionDist(0.0f, 6.2831853f);

        // 결과를 실제로 소비해서, 컴파일러가 RaycastObstacles 호출 자체를 죽은 코드로 지워버리지 못하게 한다.
        volatile int hitCount = 0;

        auto startTime = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iterations; ++i)
        {
            Vec3 origin(pos(rng), 0.0f, pos(rng));
            float directionRad = directionDist(rng);
            if (VehicleCollision::RaycastObstacles(origin, directionRad, MAX_DISTANCE, obstacles) >= 0.0f)
                ++hitCount;
        }
        auto endTime = std::chrono::high_resolution_clock::now();

        double totalMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        double perCallUs = (totalMs * 1000.0) / iterations;
        std::printf("[bench] RaycastObstacles obstacles=%3d  iterations=%7d  total=%7.2fms  avg=%6.3fus/call  hits=%d\n",
                    obstacleCount, iterations, totalMs, perCallUs, hitCount);
    }
}

TEST_CASE(Benchmark_VehicleCollision_IsColliding)
{
    std::printf("\n-- VehicleCollision::IsColliding benchmark --\n");
    for (int obstacleCount : {1, 6, 20, 50, 200, 500})
        BenchIsColliding(obstacleCount);
    std::printf("\n");
}

TEST_CASE(Benchmark_VehicleCollision_RaycastObstacles)
{
    std::printf("\n-- VehicleCollision::RaycastObstacles benchmark --\n");
    for (int obstacleCount : {1, 6, 20, 50, 200, 500})
        BenchRaycastObstacles(obstacleCount);
    std::printf("\n");
}
