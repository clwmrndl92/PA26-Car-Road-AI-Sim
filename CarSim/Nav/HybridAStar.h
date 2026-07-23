#pragma once
#include <functional>
#include <vector>
#include "Utill/MathUtil.h"
#include "ReedsShepp.h"

// Hybrid A*: 격자(3D: x, z, heading) 기반 A* 확장 + Reeds-Shepp

namespace HybridAStar
{
    // 회전된 사각형(OBB) 장애물.
    struct Obstacle
    {
        Vec3 center;
        float halfLength;
        float halfWidth;
        float headingRad = 0.0f;
    };

    struct VehicleShape
    {
        float wheelbase = 3.0f;
        float maxSteerAngleRad = ToRadians(45.0f);
        float pivotToCenter = 0.0f;
        float halfLength = 2.2f;
        float halfWidth = 1.0f;
    };

    struct Params
    {
        float stepSize = 0.5f;                         // 한 스텝(전진/후진 1회)의 이동 거리
        float gridResolution = 0.5f;                   // closed-set을 나누는 x/z 격자 한 칸 크기
        float headingResolutionRad = ToRadians(10.0f); // closed-set을 나누는 heading 격자 한 칸 크기
        float reverseCostMul = 1.2f;                   // 후진 스텝 비용 배율
        float gearChangeCost = 2.0f;                   // 이전 스텝과 기어(전/후진)가 바뀔 때 추가 비용
        float steerChangeCost = 0.5f;                  // 이전 스텝과 조향이 바뀔 때 추가 비용
        int maxExpansions = 50;                        // 격자가 촘촘해진 만큼 예산도 늘림

        // 벤치마크 전용 훅: 실패로 확정되어 내부 컨테이너(nodes/openSet/closedSet)가 아직 살아있는
        // 시점에, 그때까지 소비한 확장 횟수와 함께 호출된다. 비어있으면 아무 영향 없음.
        // expansions == maxExpansions+1이면 예산 초과, 그보다 작으면 open set 소진.
        std::function<void(int expansions)> onSearchFailed;
    };

    ReedsShepp::Path FindPath(const Vec3 &start, float startHeadingRad,
                              const Vec3 &goal, float goalHeadingRad,
                              const std::vector<Obstacle> &obstacles,
                              const VehicleShape &shape,
                              bool &foundPath,
                              const Params &params = {});

    // position/headingRad에 있는 차량(shape)이 obstacles 중 하나와 겹치면 true. 검색 트리 확장 없이
    // 단발 충돌판정만 필요한 곳(예: 주행 중 실시간 회피의 바운딩박스 스윕)에서 재사용하려고 공개해둔다.
    bool IsColliding(const Vec3 &position, float headingRad,
                     const std::vector<Obstacle> &obstacles, const VehicleShape &shape);
}
