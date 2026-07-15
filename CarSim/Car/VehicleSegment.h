#pragma once
#include "Nav/ReedsShepp.h"
#include <optional>

class Car;

constexpr float STEER_ALIGN_TOLERANCE = 0.035f; // 약 2도

// VehicleController가 실행하는 저수준 주행 지시 한 조각. Tick()이 매 틱 불려 그 틱에
// 필요한 조향/가속 명령을 Car에 직접 내리고, IsDone()으로 세그먼트가 끝났는지 판단한다.
// VehicleController는 세그먼트의 종류(고정 곡률 이동 vs 스플라인 추종)를 몰라도 된다.
class VehicleSegment
{
public:
    virtual ~VehicleSegment() = default;
    virtual void Tick(Car &car) = 0;
    virtual bool IsDone() const = 0;
    virtual ReedsShepp::Gear GetRequiredGear() const { return ReedsShepp::Gear::Forward; }
    virtual std::optional<float> GetRequiredSteerAngle() const { return std::nullopt; }
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
    std::optional<float> GetRequiredSteerAngle() const override;

private:
    static constexpr float MANEUVER_SPEED = 3.0f; // 저속 주차 기동 속도 (m/s)
    // 남은 거리 기준 감속 프로파일에 쓰는 가정 감속도 (m/s^2).
    static constexpr float DECEL_ESTIMATE = 0.4f;

    ReedsShepp::Steering m_steering;
    ReedsShepp::Gear m_gear;
    float m_distance;
    float m_steerAngle;
    float m_traveled = 0.0f;
};

// 기존 정속주행(스플라인을 Pure Pursuit + 속도 프로파일로 추종)을 그대로 실행한다.
class SplineFollowSegment : public VehicleSegment
{
public:
    void Tick(Car &car) override;
    bool IsDone() const override { return false; }
};

// 출차(Park) 매뉴버가 꺾어놓은 조향각을 중앙(0)으로 되돌린다. ArcMoveSegment와 같은 방식으로
// 정렬될 때까지는 제자리에서 대기하고(가속 0), 정렬되면 끝난다.
class CenterSteerSegment : public VehicleSegment
{
public:
    void Tick(Car &car) override;
    bool IsDone() const override { return m_aligned; }

private:
    static constexpr float CENTER_STEER_RAMP_RATE = 1.0f;
    bool m_aligned = false;
};
