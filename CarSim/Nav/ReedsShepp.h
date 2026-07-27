#pragma once
#include <functional>
#include <vector>
#include "Utill/MathUtil.h"

// Reeds-Shepp curves: 전진/후진이 모두 가능한 차량(바이시클 모델)의 두 자세(위치+방향) 사이
// 최단 경로. Reeds, J.A.; Shepp, L.A. "Optimal paths for a car that goes both forwards and
// backwards." Pacific J. Math. 145 (1990) 공식을 그대로 포팅함.
namespace ReedsShepp
{
    enum class Steering
    {
        Left = -1,
        Right = 1,
        Straight = 0
    };

    enum class Gear
    {
        Forward = 1,
        Backward = -1
    };

    // 경로의 한 세그먼트. param은 실제 물리 거리(직선은 이동 거리, 곡선은 호의 길이 = 각도 * turningRadius).
    struct PathElement
    {
        float param;
        Steering steering;
        Gear gear;
    };

    using Path = std::vector<PathElement>;

    struct Leg
    {
        std::vector<Vec3> points;
        Gear gear;
        size_t endIndex = 0; // leg가 실제로 끝나는 지점 (pure pursuit용 연장점 x)
    };

    float GetPathLength(const Path &path);

    // 모든 후보가 막히면 빈 경로를 반환한다.
    Path GetOptimalPath(const Vec3 &start, float startAngleRad,
                        const Vec3 &end, float endAngleRad,
                        float turningRadius,
                        const std::function<bool(const Path &)> &isCollisionFree = nullptr);

    // path를 start/startAngleRad에서 시작해 실제 월드 좌표 폴리라인으로 샘플링 (디버그 렌더링 전용)
    std::vector<Vec3> GetDebugPath(const Path &path, const Vec3 &start, float startAngleRad,
                                   float turningRadius, float sampleSpacing = 0.5f);

    // GetDebugPath와 같은 점들을, 각 점의 heading과 함께 반환
    struct PoseSample
    {
        Vec3 position;
        float headingRad;
    };
    std::vector<PoseSample> GetPoses(const Path &path, const Vec3 &start, float startAngleRad,
                                     float turningRadius, float sampleSpacing = 0.5f);

    // path를 기어가 바뀌는 지점마다 나눠, 각 leg의 월드좌표 폴리라인을 반환
    std::vector<Leg> GetLegs(const Path &path, const Vec3 &start, float startAngleRad,
                             float turningRadius, float sampleSpacing = 0.5f);
}
