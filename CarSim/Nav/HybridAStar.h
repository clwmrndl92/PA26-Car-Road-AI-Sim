#pragma once
#include <vector>
#include "Utill/MathUtil.h"
#include "ReedsShepp.h"

// Hybrid A*: 격자(3D: x, z, heading) 기반 A* 확장 + 매 확장마다 목표까지 Reeds-Shepp
// 숏컷(장애물 없으면 즉시 채택)을 시도하는 탐색. 참고: Dolgov et al., "Practical Search
// Techniques in Path Planning for Autonomous Driving" (2008).
// 반환값은 ReedsShepp::Path와 동일한 형식이라 BuildParkSegments(CarFSM.cpp)에 그대로 넣을 수 있다.
namespace HybridAStar
{
    // 회전된 사각형(OBB) 장애물. center는 XZ 평면 위 중심, headingDeg는 ReedsShepp와 같은
    // 각도 규약(atan2(z, x), degree). halfLength는 heading 방향, halfWidth는 그 수직 방향 반길이.
    struct Obstacle
    {
        Vec3 center;
        float halfLength;
        float halfWidth;
        float headingDeg = 0.0f;
    };

    // 계획 대상 차량의 기구학/형상 파라미터. start/goal 위치는 이 차량의 pivot(후륜축, RS와 동일
    // 기준점) 기준이고, 충돌판정용 사각형 중심은 pivot에서 heading 방향으로 pivotToCenter만큼
    // 떨어진 지점이다 (CarSpec::colliderOffset.z에 대응).
    struct VehicleShape
    {
        float wheelbase = 3.0f;
        float maxSteerAngleDeg = 45.0f;
        float pivotToCenter = 0.0f;
        float halfLength = 2.2f;
        float halfWidth = 1.0f;
    };

    struct Params
    {
        float stepSize = 2.0f;           // 한 스텝(전진/후진 1회)의 이동 거리
        float gridResolution = 1.0f;     // closed-set을 나누는 x/z 격자 한 칸 크기
        float headingResolution = 15.0f; // closed-set을 나누는 heading 격자 한 칸 크기(도)
        float reverseCostMul = 1.5f;     // 후진 스텝 비용 배율
        float gearChangeCost = 3.0f;     // 이전 스텝과 기어(전/후진)가 바뀔 때 추가 비용
        float steerChangeCost = 0.5f;    // 이전 스텝과 조향이 바뀔 때 추가 비용
        int maxExpansions = 20000;       // 이 횟수만큼 노드를 확장해도 못 찾으면 실패 처리
    };

    // start/goal은 XZ 평면 위 pivot 위치 + heading(도, atan2(z, x) 규약, ReedsShepp와 동일).
    // obstacles는 이미 만들어진 장애물 사각형 목록(예: 반경 R 안의 다른 차량들을 사각형으로 치환한 것).
    // 경로를 못 찾으면 빈 벡터를 반환한다.
    ReedsShepp::Path FindPath(const Vec3 &start, float startHeadingDeg,
                              const Vec3 &goal, float goalHeadingDeg,
                              const std::vector<Obstacle> &obstacles,
                              const VehicleShape &shape,
                              const Params &params = {});

    // position/headingDeg에 있는 차량(shape)이 obstacles 중 하나와 겹치면 true. 검색 트리 확장 없이
    // 단발 충돌판정만 필요한 곳(예: 주행 중 실시간 회피의 코리도어 스윕)에서 재사용하려고 공개해둔다.
    bool IsColliding(const Vec3 &position, float headingDeg,
                     const std::vector<Obstacle> &obstacles, const VehicleShape &shape);
}
