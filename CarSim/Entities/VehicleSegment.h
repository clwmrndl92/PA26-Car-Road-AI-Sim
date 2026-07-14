#pragma once
#include "Nav/ReedsShepp.h"

class Car;

// VehicleController가 실행하는 저수준 주행 지시 한 조각. Tick()이 매 틱 불려 그 틱에
// 필요한 조향/가속 명령을 Car에 직접 내리고, IsDone()으로 세그먼트가 끝났는지 판단한다.
// VehicleController는 세그먼트의 종류(고정 곡률 이동 vs 스플라인 추종)를 몰라도 된다.
class VehicleSegment
{
public:
    virtual ~VehicleSegment() = default;
    virtual void Tick(Car &car) = 0;
    virtual bool IsDone() const = 0;
    // 이 세그먼트를 실행하려면 어느 기어가 필요한지. VehicleController가 세그먼트 전환 시
    // 현재 기어와 비교해서 다르면 정지 후 기어를 바꾼다.
    virtual ReedsShepp::Gear GetRequiredGear() const { return ReedsShepp::Gear::Forward; }
};

// Reeds-Shepp 세그먼트 하나(고정 조향각 + 기어 + 거리)를 그대로 실행한다. Park 등
// 저속 매뉴버에 쓰인다.
class ArcMoveSegment : public VehicleSegment
{
public:
    // steerAngle: Steering::Left/Right일 때 적용할 조향각 크기(라디안, 부호 없음).
    ArcMoveSegment(ReedsShepp::Steering steering, ReedsShepp::Gear gear, float distance, float steerAngle);

    void Tick(Car &car) override;
    bool IsDone() const override { return m_traveled >= m_distance; }
    ReedsShepp::Gear GetRequiredGear() const override { return m_gear; }

private:
    static constexpr float MANEUVER_SPEED = 1.5f; // 저속 주차 기동 속도 (m/s)
    // RS는 "순간적으로 조향한 뒤 일정 곡률로 이동"을 가정하므로, Steer()의 램프가 끝나기 전에
    // 움직이면 계획한 경로에서 벗어난다. 이 오차 이내로 들어와야 조향이 끝났다고 보고 움직인다.
    static constexpr float STEER_ALIGN_TOLERANCE = 0.035f; // 약 2도
    // 남은 거리 기준 감속 프로파일에 쓰는 가정 감속도 (m/s^2). 이게 없으면 세그먼트 끝에서도
    // MANEUVER_SPEED로 계속 밀다가 IsDone() 이후에야 감속을 시작해 목표 거리를 오버슈트한다.
    static constexpr float DECEL_ESTIMATE = 0.4f;

    ReedsShepp::Steering m_steering;
    ReedsShepp::Gear m_gear;
    float m_distance;
    float m_steerAngle;
    float m_traveled = 0.0f;
};

// 기존 정속주행(스플라인을 Pure Pursuit + 속도 프로파일로 추종)을 그대로 실행한다.
// 스플라인 자체는 Car::m_currentSpline(CheckPath가 갱신)을 그대로 쓰므로 별도 상태가 없고,
// 끝나는 시점은 DriveMode가 밖에서 결정하므로 IsDone은 항상 false다.
class SplineFollowSegment : public VehicleSegment
{
public:
    void Tick(Car &car) override;
    bool IsDone() const override { return false; }
};
