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

    // 기어가 같은 구간(leg) 하나의 월드좌표 폴리라인. points[0]는 이 leg의 시작 pose(이전 leg의
    // 끝, 또는 경로 전체의 시작)와 일치한다.
    struct Leg
    {
        std::vector<Vec3> points;
        Gear gear;
        // points[endIndex]가 이 leg가 실제로 끝나는 지점(목표 pose). 경로 전체의 마지막 leg는
        // endIndex 이후에 최종 헤딩 방향으로 연장된 점들이 추가로 붙어있을 수 있다 (Pure Pursuit이
        // 도착 직전까지 최종 정렬 방향을 계속 조준하게 해 마지막 정렬 오차를 줄이기 위함 -- 실제
        // 정지/완료 판정은 이 endIndex 기준으로 한다).
        size_t endIndex = 0;
    };

    float GetPathLength(const Path &path);

    // start/end는 XZ 평면(y는 무시) 위의 위치, 각도는 라디안 단위이며 atan2(z, x) 규약
    // (각도 0 = +X 방향, 양수 방향 = +X에서 +Z로 회전). turningRadius는 차량의 최소 회전 반경.
    // 존재하는 경로가 없으면 빈 벡터를 반환한다.
    Path GetOptimalPath(const Vec3 &start, float startAngleRad,
                        const Vec3 &end, float endAngleRad,
                        float turningRadius);

    // path를 start/startAngleRad에서 시작해 실제 월드 좌표 폴리라인으로 샘플링한다 (디버그 렌더링용).
    // Car::ApplyMotion과 같은 부호 규약(기어와 무관하게 Left/Right가 같은 곡률 방향)을 그대로 따른다.
    std::vector<Vec3> SamplePath(const Path &path, const Vec3 &start, float startAngleRad,
                                 float turningRadius, float sampleSpacing = 0.5f);

    // path를 기어가 바뀌는 지점마다 나눠, 각 leg의 월드좌표 폴리라인을 반환한다 (Pure Pursuit
    // 기반 추종용 -- RSFollowSegment가 leg 하나씩 순서대로 실행한다). 부호 규약은 SamplePath와 동일.
    std::vector<Leg> SampleLegs(const Path &path, const Vec3 &start, float startAngleRad,
                                float turningRadius, float sampleSpacing = 0.5f);
}
