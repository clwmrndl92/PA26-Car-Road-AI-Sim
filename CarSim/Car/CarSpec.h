#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>

// 행동 계획 비용 가중치
struct BehaviorWeights
{
    float speed_under = 1.0f;      // 목표속도 대비 도달속도(3초 뒤 예상속도) 부족분에 대한 가중치
    float speed_over = 4.0f;       // 과속 페널티
    float laneKeep = 5.0f;         // 목표 오프셋이 가장 가까운 밴드 중심에서 벗어난 거리(m)에 물리는 차선유지 끌림
    float lateralAccel = 0.5f;     // 궤적 중 최대 횡가속(m/s^2)에 물리는 승차감 가중치 (커브/차선변경)
    float inertia = 1.0f;          // 직전에 고른 목표(오프셋/속도)와 달라졌을 때 물리는 가중치
    float signalViolation = 20.0f; // 신호를 지켜야 하는데 못 멈추고 통과하는 후보에 물리는 가중치
    float following = 3.0f;        // 앞차와의 시간헤드웨이가 목표(1.5s)보다 짧을 때 부족분에 물리는 가중치
    float bigVehicle = 2.0f;       // 대형차(자기보다 훨씬 넓은 차) 옆을 지날 때 원하는 여유 부족분에 물리는 가중치
    float edgeMargin = 1.0f;       // 도로 가장자리에 바짝 붙는(원하는 여유 부족) 정도에 물리는 가중치
};

struct CarSpec
{
    CarSpec(const char *name, const char *modelPath, JPH::Vec3 halfExtents, JPH::Vec3 renderOffset,
            JPH::Vec3 colliderOffset, float wheelbase, float mass, BehaviorWeights behaviorWeights = {})
        : name(name), modelPath(modelPath), halfExtents(halfExtents), renderOffset(renderOffset),
          colliderOffset(colliderOffset), wheelbase(wheelbase), mass(mass), behaviorWeights(behaviorWeights)
    {
    }

    const char *name;
    const char *modelPath;
    JPH::Vec3 halfExtents;
    JPH::Vec3 renderOffset;
    JPH::Vec3 colliderOffset;
    float wheelbase;
    float mass;

    BehaviorWeights behaviorWeights;
};

enum class CarType
{
    Car0,
    Car1,
    Count,
};

const CarSpec &GetCarSpec(CarType type);
