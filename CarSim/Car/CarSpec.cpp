#include "CarSpec.h"

const CarSpec &GetCarSpec(CarType type)
{
    static const CarSpec specs[] = {
        CarSpec("Car_0", "Model\\car_1.obj", JPH::Vec3(0.9919f, 0.9674f, 2.1204f), JPH::Vec3(0.0f, 0.0f, 1.5f),
                JPH::Vec3(0.0f, 0.96f, 1.5f), 3.0f, 1300.0f),
        CarSpec("Car_1", "Model\\car_2.obj", JPH::Vec3(1.3421f, 0.9073f, 2.8342f), JPH::Vec3(0.0f, 0.0f, 1.5f),
                JPH::Vec3(0.0f, 0.9f, 1.5f), 3.40f, 1600.0f),
        CarSpec("Car_Jeep", "Model\\car_jeep.obj", JPH::Vec3(1.3952f, 0.9922f, 2.5856f), JPH::Vec3(0.0f, 0.0f, 1.65f),
                JPH::Vec3(0.0f, 0.99f, 1.65f), 3.26f, 1900.0f),
        CarSpec("Car_LittleTruck", "Model\\car_littletruck.obj", JPH::Vec3(1.1088f, 1.1468f, 2.6419f),
                JPH::Vec3(0.0f, 0.0f, 1.72f), JPH::Vec3(0.0f, 1.15f, 1.72f), 3.42f, 1850.0f),
        // car_truck.obj는 원본 메시가 전방(-Z)/후방(+Z)이 다른 모델과 반대라 정점을 180도 회전시켜 정리함
        CarSpec("Car_Truck", "Model\\car_truck.obj", JPH::Vec3(1.5596f, 1.6568f, 5.3268f), JPH::Vec3(0.0f, 0.0f, 3.79f),
                JPH::Vec3(0.0f, 1.66f, 3.79f), 7.39f, 8000.0f),
        CarSpec("Car_Van", "Model\\car_van.obj", JPH::Vec3(1.1859f, 0.9729f, 2.8078f), JPH::Vec3(0.0f, 0.0f, 1.8f),
                JPH::Vec3(0.0f, 0.97f, 1.8f), 3.58f, 2000.0f),
    };
    return specs[static_cast<size_t>(type)];
}

const CarPersonality &GetCarPersonality(CarPersonalityType type)
{
    // float speedFactor = 1.0f;         // 목표속도 = 도로 제한속도 * speedFactor (작을수록 신중, 커질수록 과감)
    // float headwayFactor = 1.0f;       // IDM 안전거리(s0)·시간간격(T)에 곱하는 계수 (작을수록 바짝 붙음)
    // float jerkUp = 4.0f;              // 가속 방향 저크 상한 (m/s^3)
    // float jerkDown = 15.0f;           // 제동 방향 저크 상한 (m/s^3)
    // float brakeFactor = 1.0f;         // IDM 쾌적감속(b)에 곱하는 계수 (클수록 더 세게 감속)
    // float politeness = 0.2f;          // MOBIL 이타성 계수 (0=완전 이기주의 ~ 0.5=현실적 양보)
    // float laneChangeLerpAlpha = 0.2f; // 차선변경 횡오프셋 Lerp 비율 (리플랜 주기마다 목표로 이만큼 이동, 클수록 급하게 붙음)
    static const CarPersonality personalities[] = {
        CarPersonality{1.0f, 1.0f, 4.0f, 15.0f, 1.0f, 0.2f, 0.2f}, // Normal
        CarPersonality{1.2f, 0.8f, 6.0f, 20.0f, 1.2f, 0.1f, 0.3f}, // Aggressive
        CarPersonality{0.8f, 1.2f, 3.0f, 10.0f, 0.8f, 0.3f, 0.1f}, // Cautious
    };
    return personalities[static_cast<size_t>(type)];
}
