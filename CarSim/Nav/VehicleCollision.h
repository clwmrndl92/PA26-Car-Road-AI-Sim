#pragma once
#include <vector>
#include "Utill/MathUtil.h"

// 차량/장애물 OBB 충돌판정. 원래 HybridAStar.h에 있었으나, 경로탐색(Hybrid A*)과 무관하게
// RoadDataManager/EditApp의 장애물 정의, Car의 실시간 바운딩박스 스윕 판정에서 재사용되어 분리했다.
namespace VehicleCollision
{
    // 정적(맵 고정) / 동적(레이로 매프레임 잡히는 대상) 구분 태그. 테스트용 -- ScanRoadSpeedConstraints(정적)
    // vs 레이 감지(동적) 분리 실험에 씀.
    enum class ObstacleType
    {
        Static,
        Dynamic
    };

    // 회전된 사각형(OBB) 장애물.
    struct Obstacle
    {
        Vec3 center;
        float halfLength;
        float halfWidth;
        float headingRad = 0.0f;
        float speed = 0.0f; // headingRad 방향 기준 스칼라 속도. 정적 장애물은 0(기본값)로 둔다.
        ObstacleType type = ObstacleType::Static;
        bool isVehicle = false; // Car를 변환해 넣은 항목인가 -- 다른 경로(nearby car 목록)로 이미 처리되는 걸 걸러내는 용도.
        // 이 대상이 나에게 양보할 차인가. ego별로 만드는 센서 목록(BuildSensorObstacles)에서만 의미 있다.
        bool yieldsToEgo = false;
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

    // IsColliding과 같은 판정이지만, 겹친 obstacle을 직접 가리키는 포인터를 반환한다(없으면 nullptr).
    // 어떤 장애물과 부딪혔는지(그리고 그 obstacle.speed)를 알아야 하는 호출부용.
    const Obstacle *FindColliding(const Vec3 &position, float headingRad,
                                  const std::vector<Obstacle> &obstacles, const VehicleShape &shape);

    // origin에서 directionRad 방향(XZ 평면, y 무시)으로 뻗은 반직선이 obstacles 중 하나와 만나는
    // 가장 가까운 거리(0 이상)를 반환한다. maxDistance 안에 아무것도 안 맞으면 -1.
    float RaycastObstacles(const Vec3 &origin, float directionRad, float maxDistance,
                           const std::vector<Obstacle> &obstacles);

    // RaycastObstacles와 같은 판정이지만, 가장 가까이 맞은 obstacle을 직접 가리키는 포인터를 반환한다
    // (아무것도 못 맞히면 nullptr). outDistance가 nullptr이 아니면 그 지점까지의 거리를 써준다.
    // 무엇에 맞았는지(그리고 그 obstacle.speed)를 알아야 하는 호출부용.
    const Obstacle *RaycastObstaclesHit(const Vec3 &origin, float directionRad, float maxDistance,
                                        const std::vector<Obstacle> &obstacles, float *outDistance);
}
