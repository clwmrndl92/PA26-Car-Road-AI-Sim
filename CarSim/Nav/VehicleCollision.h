#pragma once
#include <vector>
#include "Utill/MathUtil.h"

// 차량/장애물 OBB 충돌판정. 원래 HybridAStar.h에 있었으나, 경로탐색(Hybrid A*)과 무관하게
// RoadDataManager/EditApp의 장애물 정의, Car의 실시간 바운딩박스 스윕 판정에서 재사용되어 분리했다.
namespace VehicleCollision
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
        float pivotToCenter = 0.0f;
        float halfLength = 2.2f;
        float halfWidth = 1.0f;
    };

    // position/headingRad에 있는 차량(shape)이 obstacles 중 하나와 겹치면 true.
    bool IsColliding(const Vec3 &position, float headingRad,
                     const std::vector<Obstacle> &obstacles, const VehicleShape &shape);
}
