#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>

// 행동 계획 비용 가중치
struct BehaviorWeights
{
    float speed = 1.0f;            // w1: 목표속도 대비 도달속도(3초 뒤 예상속도) 부족분에 대한 가중치
    float laneChange = 5.0f;       // w2: 차선변경 자체에 물리는 고정 비용 가중치
    float inertia = 1.0f;          // w4: 직전에 고른 후보(차선/속도 결정)와 달라졌을 때 물리는 가중치
    float signalViolation = 20.0f; // w5: 신호를 지켜야 하는데 못 멈추고 통과하는 후보에 물리는 가중치
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
