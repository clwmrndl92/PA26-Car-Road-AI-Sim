#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>

// 운전자 성격 파라미터. IDM/MOBIL 기본값에 곱/치환되며, Car 디버그 UI 창에서 실시간 조절 가능.
struct CarPersonality
{
    float speedFactor = 1.0f;         // 목표속도 = 도로 제한속도 * speedFactor (작을수록 신중, 커질수록 과감)
    float headwayFactor = 1.0f;       // IDM 안전거리(s0)·시간간격(T)에 곱하는 계수 (작을수록 바짝 붙음)
    float maxAccel = (100.0f / 3.6f) / 14.0f; // 최대가속(m/s^2), 0-100km/h 기준 초
    float jerkUp = 4.0f;              // 가속 방향 저크 상한 (m/s^3)
    float jerkDown = 15.0f;           // 제동 방향 저크 상한 (m/s^3)
    float brakeFactor = 1.0f;         // IDM 쾌적감속(b)에 곱하는 계수 (클수록 더 세게 감속)
    float politeness = 0.2f;          // MOBIL 이타성 계수 (0=완전 이기주의 ~ 0.5=현실적 양보)
    float laneChangeLerpAlpha = 0.2f; // 횡오프셋 Lerp 비율 (리플랜 주기마다 목표로 이만큼 이동, 클수록 급하게 붙음)

    // 아래는 레이스 모드 전용 편차. 전부 Car의 레이싱 기준값(RACE_BASE_*)에 곱해진다.
    // 일반 주행에는 쓰이지 않으므로 기본값 1.0이면 기존 동작 그대로다. 개체마다 다른 값을
    // 받아 랩타임 차이 -- 곧 추월 기회 -- 를 만드는 게 목적이라, 모든 항목이 "어떤 추월
    // 상황을 만드는가"에 대응하도록 골랐다.
    float topSpeedFactor = 1.0f; // 최고속도 배수. 직선 속도차 -> 스트레이트 추월
    float gripFactor = 1.0f;     // 타이어 마찰원 반경 배수. 코너 통과속도 -> 랩타임 차의 대부분
    float accelFactor = 1.0f;    // 가속 배수. 코너 탈출 가속 -> 다음 직선 진입 속도차
};

// 레이스 그리드용 연속 분포. skill 0=루키 ~ 1=에이스이고, 같은 스킬이라도 jitterSeed로
// 축마다 조금씩 흔들어 완전히 동일한 차가 두 대 나오지 않게 한다(jitterSeed 0이면 지터 없음).
CarPersonality MakeRacerPersonality(float skill, unsigned int jitterSeed);

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
enum class CarPersonalityType
{
    Normal,
    Aggressive,
    Cautious,
    Siren,
};

const CarSpec &GetCarSpec(CarType type);
const CarPersonality &GetCarPersonality(CarPersonalityType type);
