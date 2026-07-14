#pragma once
#include <vector>
#include "Utill/MathUtil.h"

// Reeds-Shepp curves: 전진/후진이 모두 가능한 차량(바이시클 모델)의 두 자세(위치+방향) 사이
// 최단 경로. Reeds, J.A.; Shepp, L.A. "Optimal paths for a car that goes both forwards and
// backwards." Pacific J. Math. 145 (1990) 공식을 그대로 포팅함 (참고: reeds-shepp-curves-master).
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

    float GetPathLength(const Path &path);

    // start/end는 XZ 평면(y는 무시) 위의 위치, 각도는 도(degree) 단위이며 atan2(z, x) 규약
    // (각도 0 = +X 방향, 양수 방향 = +X에서 +Z로 회전). turningRadius는 차량의 최소 회전 반경.
    // 존재하는 경로가 없으면 빈 벡터를 반환한다.
    Path GetOptimalPath(const Vec3 &start, float startAngleDeg,
                        const Vec3 &end, float endAngleDeg,
                        float turningRadius);

    // path를 start/startAngleDeg에서 시작해 실제 월드 좌표 폴리라인으로 샘플링한다 (디버그 렌더링용).
    // ArcMoveSegment/Car::ApplyMotion과 같은 부호 규약(기어와 무관하게 Left/Right가 같은 곡률 방향)을
    // 그대로 따른다.
    std::vector<Vec3> SamplePath(const Path &path, const Vec3 &start, float startAngleDeg,
                                 float turningRadius, float sampleSpacing = 0.5f);
}
