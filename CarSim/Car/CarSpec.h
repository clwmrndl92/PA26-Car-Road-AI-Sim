#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>

// 레이싱 드라이버 + 차량 편차. 전부 Car의 기준값(BASE_*)에 곱해진다.
// 개체마다 다른 값을 받아 랩타임 차이 -- 곧 추월 기회 -- 를 만드는 게 목적이라,
// 모든 항목이 "어떤 추월 상황을 만드는가"에 대응하도록 골랐다.
// Car 디버그 UI 창에서 결과값을 확인할 수 있다.
struct CarPersonality
{
    float topSpeedFactor = 1.0f; // 최고속도 배수. 직선 속도차 -> 스트레이트 추월
    float gripFactor = 1.0f;     // 타이어 마찰원 반경 배수. 코너 통과속도 -> 랩타임 차의 대부분
    float accelFactor = 1.0f;    // 가속 배수. 코너 탈출 가속 -> 다음 직선 진입 속도차
    float brakeFactor = 1.0f;    // 제동 배수. 브레이킹 포인트 -> 코너 진입 다이브
    float headwayFactor = 1.0f;  // 차간시간·최소갭 배수. 작을수록 바짝 붙어 압박한다
    float jerkUp = 30.0f;        // 가속 저크 상한(m/s^3). 스로틀을 얼마나 거칠게 여는가
    float jerkDown = 60.0f;      // 제동 저크 상한(m/s^3). 브레이크를 얼마나 급하게 밟는가
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
    Jeep,
    LittleTruck,
    Van,
    Count,
};

// 드라이버 등급 프리셋. 수동 스폰 UI에서 고르는 용도이고,
// 실제 그리드는 MakeRacerPersonality로 연속 분포를 쓴다.
enum class CarPersonalityType
{
    Ace,      // 그립·제동을 한계까지, 바짝 붙어 압박
    Balanced, //
    Rookie,   // 코너가 느리고 일찍 제동, 차간이 넓다
    Count,
};

const CarSpec &GetCarSpec(CarType type);

// skill 0=루키 ~ 1=에이스. 같은 스킬이라도 jitterSeed로 축마다 조금씩 흔들어
// 완전히 동일한 차가 두 대 나오지 않게 한다(jitterSeed 0이면 지터 없음).
CarPersonality MakeRacerPersonality(float skill, unsigned int jitterSeed);
CarPersonality GetCarPersonality(CarPersonalityType type);
