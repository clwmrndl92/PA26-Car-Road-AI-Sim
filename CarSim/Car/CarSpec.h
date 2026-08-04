#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>

// 운전자 성격 파라미터. IDM/MOBIL 기본값에 곱/치환되며, Car 디버그 UI 창에서 실시간 조절 가능.
struct CarPersonality
{
    float speedFactor = 1.0f;         // 목표속도 = 도로 제한속도 * speedFactor (작을수록 신중, 커질수록 과감)
    float headwayFactor = 1.0f;       // IDM 안전거리(s0)·시간간격(T)에 곱하는 계수 (작을수록 바짝 붙음)
    float jerkUp = 4.0f;              // 가속 방향 저크 상한 (m/s^3)
    float jerkDown = 15.0f;           // 제동 방향 저크 상한 (m/s^3)
    float brakeFactor = 1.0f;         // IDM 쾌적감속(b)에 곱하는 계수 (클수록 더 세게 감속)
    float politeness = 0.2f;          // MOBIL 이타성 계수 (0=완전 이기주의 ~ 0.5=현실적 양보)
    float laneChangeLerpAlpha = 0.2f; // 차선변경 횡오프셋 Lerp 비율 (리플랜 주기마다 목표로 이만큼 이동, 클수록 급하게 붙음)
};

struct CarSpec
{
    CarSpec(const char *name, const char *modelPath, JPH::Vec3 halfExtents, JPH::Vec3 renderOffset,
            JPH::Vec3 colliderOffset, float wheelbase, float mass, CarPersonality personality = {})
        : name(name), modelPath(modelPath), halfExtents(halfExtents), renderOffset(renderOffset),
          colliderOffset(colliderOffset), wheelbase(wheelbase), mass(mass), personality(personality)
    {
    }

    const char *name;
    const char *modelPath;
    JPH::Vec3 halfExtents;
    JPH::Vec3 renderOffset;
    JPH::Vec3 colliderOffset;
    float wheelbase;
    float mass;
    CarPersonality personality;
};

enum class CarType
{
    Car0,
    Car1,
    Count,
};
enum class CarPersonalityType
{
    Normal,
    Aggressive,
    Cautious,
};

const CarSpec &GetCarSpec(CarType type);
const CarPersonality &GetCarPersonality(CarPersonalityType type);
