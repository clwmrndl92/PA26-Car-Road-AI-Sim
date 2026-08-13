#include "TestHarness.h"
#include "Nav/VehicleCollision.h"
#include "Utill/MathUtil.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <utility>
#include <vector>

// 커밋 1c464c8 "코리도어 박스 최적화" 전/후 비교 벤치.
//
// 그 커밋은 Car::AppendSensorConstraintSample의 스윕 루프에서 두 가지를 바꿨다.
//   (1) 조준점 계산: 매 substep마다 참조선 전체를 훑던 OffsetLookaheadPoint를,
//       오프셋 점을 한 번만 만들어두고(aimPoints) 커서 윈도우로 찾는 LookaheadOnPolyline으로 교체.
//   (2) 박스 간격: stepDistance를 절반으로 줄여 OBB 검사 횟수를 2배로 늘림(정밀도 향상).
// 비용 부호가 반대인 두 변경이라, 조준점만 따로(Bench A) + 스윕 전체(Bench B)로 나눠 잰다.
//
// 조준점 함수 본체는 CarFSM.cpp에서 그대로 옮겨왔다(전/후 양쪽). Spline 대신 std::vector<Vec3>를
// 직접 받는데, 원본도 referenceLine.GetSplinePoints()만 건드리므로 동작은 동일하다.
namespace
{
    constexpr float STEER_LOOKAHEAD = 5.0f;
    constexpr float BIKE_SUBSTEP = 0.25f;
    constexpr float SWEEP_MIN = 30.0f;
    constexpr float SWEEP_MAX = 100.0f;
    constexpr float SAFE_GAP = 2.0f;
    constexpr float MIN_SAFE_GAP = 0.3f;

    // Car_0 스펙 기준
    constexpr float HALF_LENGTH = 2.1204f;
    constexpr float HALF_WIDTH = 0.9919f;
    constexpr float PIVOT_TO_CENTER = 1.5f;
    constexpr float WHEELBASE = 3.0f;
    constexpr float MAX_BRAKE = (100.0f / 3.6f) / 3.0f;
    constexpr float MAX_STEER_ANGLE = 0.7853982f; // 45도(저속 기준 상한)

    float DirectionToAngleRad(const Vec3 &direction)
    {
        return atan2f(direction.GetZ(), direction.GetX());
    }

    float PurePursuitSteerAt(const Vec3 &rearAxle, float headingRad, const Vec3 &target, float wheelbase)
    {
        Vec3 forward(cosf(headingRad), 0.0f, sinf(headingRad));
        Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());
        float dx = target.GetX() - rearAxle.GetX();
        float dz = target.GetZ() - rearAxle.GetZ();
        float distance = std::sqrt(dx * dx + dz * dz);
        if (distance < 0.001f)
            return 0.0f;

        float cosAngle = std::clamp((forward.GetX() * dx + forward.GetZ() * dz) / distance, -1.0f, 1.0f);
        float headingError = acosf(cosAngle);
        float directionSign = (right.GetX() * dx + right.GetZ() * dz > 0.0f) ? 1.0f : -1.0f;
        return atanf((2.0f * wheelbase * sinf(headingError)) / distance) * directionSign;
    }

    Vec3 OffsetSampleAt(const std::vector<Vec3> &samples, size_t i, float d)
    {
        const size_t n = samples.size();
        const Vec3 &next = samples[i + 1 < n ? i + 1 : i];
        const Vec3 &prev = samples[i > 0 ? i - 1 : i];
        float tx = next.GetX() - prev.GetX();
        float tz = next.GetZ() - prev.GetZ();
        float len = std::sqrt(tx * tx + tz * tz);
        float rx = len > 1e-5f ? tz / len : 0.0f;
        float rz = len > 1e-5f ? -tx / len : 0.0f;
        return Vec3(samples[i].GetX() + rx * d, samples[i].GetY(), samples[i].GetZ() + rz * d);
    }

    // 최적화 전: 호출마다 참조선 전체를 훑고, 샘플마다 오프셋 점을 새로 만든다(sqrt 2회/샘플).
    Vec3 OffsetLookaheadPoint_Old(const std::vector<Vec3> &samples, const Vec3 &from, float lookahead,
                                  float d, bool reversed)
    {
        const size_t n = samples.size();
        if (n == 0)
            return from;
        if (n == 1)
            return OffsetSampleAt(samples, 0, d);

        size_t closest = 0;
        float bestDistance = std::numeric_limits<float>::max();
        Vec3 current;
        for (size_t i = 0; i < n; ++i)
        {
            Vec3 p = OffsetSampleAt(samples, i, d);
            float distance = (p - from).Length();
            if (distance < bestDistance)
            {
                bestDistance = distance;
                closest = i;
                current = p;
            }
        }

        float accumulated = 0.0f;
        size_t index = closest;
        while (accumulated < lookahead)
        {
            if (reversed ? index == 0 : index + 1 >= n)
                break;
            size_t nextIndex = reversed ? index - 1 : index + 1;
            Vec3 nextPoint = OffsetSampleAt(samples, nextIndex, d);
            accumulated += (nextPoint - current).Length();
            current = nextPoint;
            index = nextIndex;
        }
        return current;
    }

    // 최적화 후: 오프셋 점은 미리 만들어 두고, 직전 위치 주변 ±8개만 본다(sqrt 없음).
    Vec3 LookaheadOnPolyline_New(const std::vector<Vec3> &points, const Vec3 &from, float lookahead,
                                 bool reversed, size_t &cursor)
    {
        const size_t n = points.size();
        if (n == 0)
            return from;

        constexpr size_t SEARCH_WINDOW = 8;
        size_t begin = 0;
        size_t end = n;
        if (cursor < n)
        {
            begin = cursor > SEARCH_WINDOW ? cursor - SEARCH_WINDOW : 0;
            end = std::min(n, cursor + SEARCH_WINDOW + 1);
        }

        float bestDistance = std::numeric_limits<float>::max();
        size_t closest = begin;
        for (size_t i = begin; i < end; ++i)
        {
            float distance = (points[i] - from).LengthSq();
            if (distance < bestDistance)
            {
                bestDistance = distance;
                closest = i;
            }
        }
        cursor = closest;

        float accumulated = 0.0f;
        size_t index = closest;
        while (accumulated < lookahead)
        {
            if (reversed ? index == 0 : index + 1 >= n)
                break;
            size_t nextIndex = reversed ? index - 1 : index + 1;
            accumulated += (points[nextIndex] - points[index]).Length();
            index = nextIndex;
        }
        return points[index];
    }

    // 완만한 호. spacing은 Spline(CURVE_RESOLUTION=50)이 실제로 뱉는 간격(약 0.5~0.7m)에 맞췄다.
    std::vector<Vec3> MakeRoadPolyline(int pointCount, float spacing, float curvatureRadius)
    {
        std::vector<Vec3> points;
        points.reserve(pointCount);
        float theta = 0.0f;
        for (int i = 0; i < pointCount; ++i)
        {
            points.push_back(Vec3(curvatureRadius * sinf(theta), 0.0f, curvatureRadius * (1.0f - cosf(theta))));
            theta += spacing / curvatureRadius;
        }
        return points;
    }

    std::vector<VehicleCollision::Obstacle> MakeSweepObstacles(int count, unsigned seed)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> along(0.0f, 120.0f);
        std::uniform_real_distribution<float> lateral(-8.0f, 8.0f);
        std::uniform_real_distribution<float> heading(0.0f, 6.2831853f);

        std::vector<VehicleCollision::Obstacle> obstacles;
        obstacles.reserve(count);
        for (int i = 0; i < count; ++i)
        {
            VehicleCollision::Obstacle obstacle;
            obstacle.center = Vec3(along(rng), 0.0f, lateral(rng));
            obstacle.halfLength = 2.0f;
            obstacle.halfWidth = 1.0f;
            obstacle.headingRad = heading(rng);
            obstacles.push_back(obstacle);
        }
        return obstacles;
    }

    struct SweepConfig
    {
        bool useNewLookahead = true;
        float stepScale = 0.5f;   // 최적화 후 = 0.5, 전 = 1.0
        bool checkCollision = true;
    };

    struct SweepResult
    {
        Vec3 endPosition;
        int boxChecks = 0;
        int lookaheadCalls = 0;
    };

    // AppendSensorConstraintSample의 스윕 루프를 그대로 옮긴 것. 조준점 계산 방식과 stepDistance만
    // config로 갈아끼운다.
    SweepResult RunSweep(const std::vector<Vec3> &refSamples, const std::vector<Vec3> &aimPoints,
                         const std::vector<VehicleCollision::Obstacle> &obstacles, float targetOffset,
                         float speed, const SweepConfig &config)
    {
        VehicleCollision::VehicleShape shape;
        shape.pivotToCenter = PIVOT_TO_CENTER;
        shape.halfLength = HALF_LENGTH;
        shape.halfWidth = HALF_WIDTH;

        float carLength = HALF_LENGTH * 2.0f;
        float maxDistance = std::clamp(speed * speed / (2.0f * MAX_BRAKE) + SAFE_GAP, SWEEP_MIN, SWEEP_MAX);
        float stepDistance = (carLength - MIN_SAFE_GAP) * config.stepScale;
        int sweepSteps = std::max(static_cast<int>(maxDistance / stepDistance), 8);
        int substeps = std::max(1, static_cast<int>(std::ceil(stepDistance / BIKE_SUBSTEP)));
        float substepDistance = stepDistance / static_cast<float>(substeps);

        Vec3 bikePos = Vec3(0.0f, 0.0f, 0.0f);
        float bikeHeadingRad = 0.0f;
        size_t aimCursor = std::numeric_limits<size_t>::max();

        SweepResult result;
        auto advance = [&](float distance)
        {
            Vec3 aim = config.useNewLookahead
                           ? LookaheadOnPolyline_New(aimPoints, bikePos, STEER_LOOKAHEAD, false, aimCursor)
                           : OffsetLookaheadPoint_Old(refSamples, bikePos, STEER_LOOKAHEAD, targetOffset, false);
            ++result.lookaheadCalls;
            float steerAngle = std::clamp(PurePursuitSteerAt(bikePos, bikeHeadingRad, aim, WHEELBASE),
                                          -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
            bikeHeadingRad -= distance * tanf(steerAngle) / WHEELBASE;
            bikePos += Vec3(cosf(bikeHeadingRad), 0.0f, sinf(bikeHeadingRad)) * distance;
        };

        int leadSubsteps = std::max(1, static_cast<int>(std::ceil(shape.halfLength / BIKE_SUBSTEP)));
        float leadDistance = shape.halfLength / static_cast<float>(leadSubsteps);
        for (int s = 0; s < leadSubsteps; ++s)
            advance(leadDistance);

        for (int i = 0; i <= sweepSteps; ++i)
        {
            if (config.checkCollision)
            {
                ++result.boxChecks;
                if (VehicleCollision::FindColliding(bikePos, bikeHeadingRad, obstacles, shape) != nullptr)
                    break;
            }
            for (int s = 0; s < substeps; ++s)
                advance(substepDistance);
        }

        result.endPosition = bikePos;
        return result;
    }

    // aimPoints 사전계산까지 포함해야 공정하다(최적화 후엔 호출마다 한 번씩 만든다).
    double TimeSweep(const std::vector<Vec3> &refSamples, const std::vector<VehicleCollision::Obstacle> &obstacles,
                     float targetOffset, float speed, const SweepConfig &config, int iterations,
                     SweepResult *outSample)
    {
        volatile float sink = 0.0f;
        auto startTime = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iterations; ++i)
        {
            std::vector<Vec3> aimPoints;
            if (config.useNewLookahead)
            {
                aimPoints.reserve(refSamples.size());
                for (size_t k = 0; k < refSamples.size(); ++k)
                    aimPoints.push_back(OffsetSampleAt(refSamples, k, targetOffset));
            }
            SweepResult result = RunSweep(refSamples, aimPoints, obstacles, targetOffset, speed, config);
            sink = sink + result.endPosition.GetX();
            if (outSample != nullptr)
                *outSample = result;
        }
        auto endTime = std::chrono::high_resolution_clock::now();
        (void)sink;
        return std::chrono::duration<double, std::milli>(endTime - startTime).count();
    }

    void BenchTier(int pointCount, float speedKmh, bool checkCollision, int obstacleCount)
    {
        constexpr float SPACING = 0.6f;
        constexpr float RADIUS = 220.0f;
        constexpr float TARGET_OFFSET = 2.0f;
        constexpr int ITERATIONS = 3000;

        std::vector<Vec3> refSamples = MakeRoadPolyline(pointCount, SPACING, RADIUS);
        std::vector<VehicleCollision::Obstacle> obstacles =
            checkCollision ? MakeSweepObstacles(obstacleCount, 99u + pointCount) : std::vector<VehicleCollision::Obstacle>{};
        float speed = speedKmh / 3.6f;

        SweepConfig before;
        before.useNewLookahead = false;
        before.stepScale = 1.0f;
        before.checkCollision = checkCollision;

        SweepConfig after;
        after.useNewLookahead = true;
        after.stepScale = 0.5f;
        after.checkCollision = checkCollision;

        // 박스 간격 변경을 빼고 조준점 교체만의 효과를 보기 위한 중간 구성
        SweepConfig lookaheadOnly = before;
        lookaheadOnly.useNewLookahead = true;

        SweepResult beforeSample;
        SweepResult afterSample;
        SweepResult lookaheadOnlySample;

        TimeSweep(refSamples, obstacles, TARGET_OFFSET, speed, before, 200, nullptr); // warm-up
        double beforeMs = TimeSweep(refSamples, obstacles, TARGET_OFFSET, speed, before, ITERATIONS, &beforeSample);
        double lookaheadOnlyMs = TimeSweep(refSamples, obstacles, TARGET_OFFSET, speed, lookaheadOnly, ITERATIONS, &lookaheadOnlySample);
        double afterMs = TimeSweep(refSamples, obstacles, TARGET_OFFSET, speed, after, ITERATIONS, &afterSample);

        double beforeUs = (beforeMs * 1000.0) / ITERATIONS;
        double lookaheadOnlyUs = (lookaheadOnlyMs * 1000.0) / ITERATIONS;
        double afterUs = (afterMs * 1000.0) / ITERATIONS;

        std::printf("  n=%4d v=%2.0fkm/h | before %8.2fus (%3d box, %4d aim) | lookahead-only %8.2fus (x%5.2f)"
                    " | after %8.2fus (%3d box, %4d aim, x%5.2f)\n",
                    pointCount, speedKmh,
                    beforeUs, beforeSample.boxChecks, beforeSample.lookaheadCalls,
                    lookaheadOnlyUs, beforeUs / lookaheadOnlyUs,
                    afterUs, afterSample.boxChecks, afterSample.lookaheadCalls, beforeUs / afterUs);
    }

    // 커서 윈도우(±8)가 전체 탐색과 같은 점을 고르는지. substep 0.25m에 샘플 간격 0.6m면
    // 한 스텝에 1칸도 안 움직이니 윈도우를 벗어날 일이 없어야 한다.
    void VerifyEquivalence(int pointCount)
    {
        constexpr float SPACING = 0.6f;
        constexpr float RADIUS = 220.0f;
        constexpr float TARGET_OFFSET = 2.0f;

        std::vector<Vec3> refSamples = MakeRoadPolyline(pointCount, SPACING, RADIUS);
        std::vector<Vec3> aimPoints;
        aimPoints.reserve(refSamples.size());
        for (size_t k = 0; k < refSamples.size(); ++k)
            aimPoints.push_back(OffsetSampleAt(refSamples, k, TARGET_OFFSET));

        SweepConfig oldCfg;
        oldCfg.useNewLookahead = false;
        oldCfg.stepScale = 1.0f;
        oldCfg.checkCollision = false;

        SweepConfig newCfg = oldCfg;
        newCfg.useNewLookahead = true; // stepScale은 같게 둬야 궤적이 비교 가능

        SweepResult oldResult = RunSweep(refSamples, aimPoints, {}, TARGET_OFFSET, 70.0f / 3.6f, oldCfg);
        SweepResult newResult = RunSweep(refSamples, aimPoints, {}, TARGET_OFFSET, 70.0f / 3.6f, newCfg);

        float divergence = (oldResult.endPosition - newResult.endPosition).Length();
        std::printf("  n=%4d  end-point divergence = %.6f m  %s\n", pointCount, divergence,
                    divergence < 0.05f ? "OK" : "*** MISMATCH ***");
        CHECK(divergence < 0.05f);
    }

    // 제안된 Car::ScanSensors(레이팬)와 현재 코드(박스스윕)의 순수 스캔 비용 비교.
    // 레이 배치(원점/각도/사거리)는 ScanSensors 본문을 그대로 옮겼다 -- 정면 17개
    // (직진 3 + 5도 2 + 15/30/45/60/75/90도 좌우 12) + 좌우 블라인드스팟/MOBIL 12개 = 29개.
    constexpr int SENSOR_SCAN_RAY_COUNT = 29;

    struct RayDef
    {
        Vec3 origin;
        float rightAngle;
        float maxDistance;
    };

    std::vector<RayDef> BuildScanSensorRays(const Vec3 &center, const Vec3 &forward, const Vec3 &right,
                                            float halfLength, float halfWidth, float farLength,
                                            float sideRayLength, float mobilRayMax)
    {
        Vec3 frontCenter = center + forward * halfLength;
        Vec3 frontLeft = frontCenter - right * halfWidth;
        Vec3 frontRight = frontCenter + right * halfWidth;
        Vec3 sideLeft = center - right * halfWidth;
        Vec3 sideRight = center + right * halfWidth;
        Vec3 rearCenter = center - forward * halfLength;
        Vec3 rearLeft = rearCenter - right * halfWidth;
        Vec3 rearRight = rearCenter + right * halfWidth;

        std::vector<RayDef> rays = {
            {frontCenter, 0.0f, farLength},
            {frontRight, 0.0f, farLength},
            {frontLeft, 0.0f, farLength},
            {frontRight, ToRadians(5.0f), farLength},
            {frontLeft, ToRadians(-5.0f), farLength},
            {frontRight, ToRadians(15.0f), farLength * 0.5f},
            {frontLeft, ToRadians(-15.0f), farLength * 0.5f},
            {frontRight, ToRadians(30.0f), farLength * 0.5f},
            {frontLeft, ToRadians(-30.0f), farLength * 0.5f},
            {frontRight, ToRadians(45.0f), farLength * 0.5f},
            {frontLeft, ToRadians(-45.0f), farLength * 0.5f},
            {frontRight, ToRadians(60.0f), farLength * 0.5f},
            {frontLeft, ToRadians(-60.0f), farLength * 0.5f},
            {frontRight, ToRadians(75.0f), farLength * 0.5f},
            {frontLeft, ToRadians(-75.0f), farLength * 0.5f},
            {frontRight, ToRadians(90.0f), farLength * 0.5f},
            {frontLeft, ToRadians(-90.0f), farLength * 0.5f},
        };

        for (int side = -1; side <= 1; side += 2)
        {
            float sign = static_cast<float>(side);
            const Vec3 &frontCorner = side < 0 ? frontLeft : frontRight;
            const Vec3 &sideMid = side < 0 ? sideLeft : sideRight;
            const Vec3 &rearCorner = side < 0 ? rearLeft : rearRight;

            rays.push_back({frontCorner, sign * ToRadians(90.0f), sideRayLength});
            rays.push_back({sideMid, sign * ToRadians(90.0f), sideRayLength});
            rays.push_back({rearCorner, sign * ToRadians(90.0f), sideRayLength});
            rays.push_back({rearCorner, sign * ToRadians(165.0f), mobilRayMax});
            rays.push_back({rearCorner, sign * ToRadians(150.0f), mobilRayMax});
            rays.push_back({rearCorner, sign * ToRadians(135.0f), mobilRayMax});
        }
        return rays;
    }

    // ScanSensors 1회 = 레이마다 RaycastObstaclesHitAll(전부 수집, nearest만이 아님 --
    // 평행필터 때문에 hit을 다 훑어야 한다).
    int RunSensorScan(const std::vector<RayDef> &rays, float headingRad,
                      const std::vector<VehicleCollision::Obstacle> &obstacles)
    {
        int hitCount = 0;
        std::vector<std::pair<const VehicleCollision::Obstacle *, float>> hits;
        for (const RayDef &ray : rays)
        {
            hits.clear();
            float directionRad = headingRad - ray.rightAngle;
            VehicleCollision::RaycastObstaclesHitAll(ray.origin, directionRad, ray.maxDistance, obstacles, hits);
            hitCount += static_cast<int>(hits.size());
        }
        return hitCount;
    }

    // 레이팬 / 박스스윕(최적화 전) / 박스스윕(최적화 후) 세 가지를 같은 obstacleCount에서 나란히 잰다.
    // 도로 n은 150(대표값) 고정 -- 레이팬은 도로/스플라인과 무관하고, 박스스윕(후)도 최적화 이후엔
    // n 의존성이 거의 사라졌으므로(Benchmark_CorridorSweep_Full 참고) 대표값 하나면 충분하다.
    //
    // 장애물을 시드 하나로만 뽑으면 "스윕 경로에 우연히 하나도 안 걸림(=끝까지 스윕)"과
    // "바로 걸림(=1박스 조기종료)"이 obstacleCount와 무관하게 시드 운으로 갈려서 재현이 안 된다.
    // TRIALS번 서로 다른 배치로 뽑아 평균내야 obstacleCount에 대해 매끈한 곡선이 나온다.
    void BenchSensorScanVsSweep(int obstacleCount)
    {
        constexpr int REPRESENTATIVE_N = 150;
        constexpr float SPACING = 0.6f;
        constexpr float RADIUS = 220.0f;
        constexpr float TARGET_OFFSET = 2.0f;
        constexpr int SWEEP_ITERATIONS_PER_TRIAL = 300;
        constexpr int TRIALS = 30;

        std::vector<Vec3> refSamples = MakeRoadPolyline(REPRESENTATIVE_N, SPACING, RADIUS);
        float speed = 70.0f / 3.6f;

        SweepConfig before;
        before.useNewLookahead = false;
        before.stepScale = 1.0f;
        before.checkCollision = true;

        SweepConfig after;
        after.useNewLookahead = true;
        after.stepScale = 0.5f;
        after.checkCollision = true;

        Vec3 center(0.0f, 0.0f, 0.0f);
        Vec3 forward(1.0f, 0.0f, 0.0f);
        Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());
        std::vector<RayDef> rays = BuildScanSensorRays(center, forward, right, HALF_LENGTH, HALF_WIDTH, 60.0f, 2.5f, 60.0f);
        int rayIterationsPerTrial = std::max(20, 60000 / (std::max(1, obstacleCount) * SENSOR_SCAN_RAY_COUNT));

        // 워밍업(첫 트라이얼의 배치로 한 번 몸풀기 -- 결과엔 안 넣는다)
        std::vector<VehicleCollision::Obstacle> warmupObstacles = MakeSweepObstacles(obstacleCount, 1000u);
        TimeSweep(refSamples, warmupObstacles, TARGET_OFFSET, speed, before, 100, nullptr);
        TimeSweep(refSamples, warmupObstacles, TARGET_OFFSET, speed, after, 100, nullptr);
        volatile int warmupSink = 0;
        for (int i = 0; i < 100; ++i)
            warmupSink += RunSensorScan(rays, 0.0f, warmupObstacles);
        (void)warmupSink;

        double beforeUsSum = 0.0, afterUsSum = 0.0, rayUsSum = 0.0;
        double beforeBoxSum = 0.0, afterBoxSum = 0.0;

        for (int trial = 0; trial < TRIALS; ++trial)
        {
            std::vector<VehicleCollision::Obstacle> obstacles =
                MakeSweepObstacles(obstacleCount, 1000u + static_cast<unsigned>(trial) * 97u + obstacleCount);

            SweepResult beforeSample;
            SweepResult afterSample;
            double beforeMs = TimeSweep(refSamples, obstacles, TARGET_OFFSET, speed, before, SWEEP_ITERATIONS_PER_TRIAL, &beforeSample);
            double afterMs = TimeSweep(refSamples, obstacles, TARGET_OFFSET, speed, after, SWEEP_ITERATIONS_PER_TRIAL, &afterSample);
            beforeUsSum += (beforeMs * 1000.0) / SWEEP_ITERATIONS_PER_TRIAL;
            afterUsSum += (afterMs * 1000.0) / SWEEP_ITERATIONS_PER_TRIAL;
            beforeBoxSum += beforeSample.boxChecks;
            afterBoxSum += afterSample.boxChecks;

            volatile int sink = 0;
            auto rayStart = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < rayIterationsPerTrial; ++i)
                sink += RunSensorScan(rays, 0.0f, obstacles);
            auto rayEnd = std::chrono::high_resolution_clock::now();
            (void)sink;
            double rayMs = std::chrono::duration<double, std::milli>(rayEnd - rayStart).count();
            rayUsSum += (rayMs * 1000.0) / rayIterationsPerTrial;
        }

        double beforeUs = beforeUsSum / TRIALS;
        double afterUs = afterUsSum / TRIALS;
        double rayUs = rayUsSum / TRIALS;
        double beforeBox = beforeBoxSum / TRIALS;
        double afterBox = afterBoxSum / TRIALS;

        std::printf("  obstacles=%3d (%d placements avg) | ray(%2dray) %8.2fus/call | box sweep before %8.2fus/call (avg %4.1f box)"
                    " | box sweep after %7.2fus/call (avg %4.1f box)\n",
                    obstacleCount, TRIALS, SENSOR_SCAN_RAY_COUNT, rayUs, beforeUs, beforeBox, afterUs, afterBox);
    }

    // 위 벤치의 "box" 수는 근처 장애물에 바로 걸려 early-exit한 흔한 케이스(보통 1개)다.
    // 도로가 비어 있어 끝까지 스윕하는 최악의 경우엔 몇 개나 도는지 별도로 계산해서 보여준다 --
    // Car::m_maxSpeed(200km/h) 기준. 레이팬(29개)과 견줄 대상은 이 숫자다.
    void PrintWorstCaseBoxCounts()
    {
        constexpr float GAME_MAX_SPEED = 200.0f / 3.6f;
        constexpr float carLength = HALF_LENGTH * 2.0f;

        float maxDistance = std::clamp(GAME_MAX_SPEED * GAME_MAX_SPEED / (2.0f * MAX_BRAKE) + SAFE_GAP, SWEEP_MIN, SWEEP_MAX);
        int stepsBefore = std::max(static_cast<int>(maxDistance / (carLength - MIN_SAFE_GAP)), 8);
        int stepsAfter = std::max(static_cast<int>(maxDistance / ((carLength - MIN_SAFE_GAP) * 0.5f)), 8);

        std::printf("  worst case (empty road, %.0fkm/h top speed, maxDistance=%.0fm clamped): "
                    "before sweeps up to %d boxes, after up to %d boxes -- ray fan always fires %d rays\n",
                    GAME_MAX_SPEED * 3.6f, maxDistance, stepsBefore, stepsAfter, SENSOR_SCAN_RAY_COUNT);
    }
}

TEST_CASE(Benchmark_CorridorSweep_Lookahead)
{
    std::printf("\n-- corridor sweep: lookahead only (no OBB checks) --\n");
    for (int pointCount : {80, 150, 300, 600})
        BenchTier(pointCount, 70.0f, false, 0);
    std::printf("\n");
}

TEST_CASE(Benchmark_CorridorSweep_Full)
{
    std::printf("\n-- corridor sweep: full (with 20 OBB checks) --\n");
    for (int pointCount : {80, 150, 300, 600})
        BenchTier(pointCount, 70.0f, true, 20);
    std::printf("  (low speed: maxDistance clamped to SWEEP_MIN=30m)\n");
    for (int pointCount : {150, 300})
        BenchTier(pointCount, 30.0f, true, 20);
    std::printf("\n");
}

TEST_CASE(Benchmark_CorridorSweep_Equivalence)
{
    std::printf("\n-- corridor sweep: cursor window equivalence check --\n");
    for (int pointCount : {80, 150, 300, 600})
        VerifyEquivalence(pointCount);
    std::printf("\n");
}

TEST_CASE(Benchmark_SensorScan_vs_CorridorSweep)
{
    std::printf("\n-- ScanSensors (raycast fan, %d rays) vs corridor sweep (box, current code) --\n", SENSOR_SCAN_RAY_COUNT);
    std::printf("  (road n=%d fixed for the sweep side; the ray fan doesn't depend on road length)\n", 150);
    PrintWorstCaseBoxCounts();
    for (int obstacleCount : {1, 6, 20, 50, 200})
        BenchSensorScanVsSweep(obstacleCount);
    std::printf("\n");
}
