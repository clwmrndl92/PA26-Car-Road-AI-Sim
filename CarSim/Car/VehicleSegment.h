#pragma once
#include "Nav/ReedsShepp.h"
#include <limits>
#include <optional>
#include <vector>

class Car;

constexpr float STEER_ALIGN_TOLERANCE = 0.035f; // 약 2도

class VehicleSegment
{
public:
    virtual ~VehicleSegment() = default;
    virtual void Tick(Car &car) = 0;
    virtual bool IsDone() const = 0;
    virtual ReedsShepp::Gear GetRequiredGear() const { return ReedsShepp::Gear::Forward; }
    virtual std::optional<float> GetRequiredSteerAngle() const { return std::nullopt; }
};

class RSFollowSegment : public VehicleSegment
{
public:
    RSFollowSegment(std::vector<Vec3> points, ReedsShepp::Gear gear, size_t endIndex, bool isFinalLeg);

    void Tick(Car &car) override;
    bool IsDone() const override { return m_done; }
    ReedsShepp::Gear GetRequiredGear() const override { return m_gear; }

private:
    static constexpr float MANEUVER_SPEED = 3.0f;
    static constexpr float DECEL_ESTIMATE = 0.4f;
    static constexpr float FINISH_DISTANCE = 0.3f;
    static constexpr float OVERSHOOT_CHECK_DISTANCE = 1.0f;
    static constexpr float MIN_LOOKAHEAD = 1.5f;
    static constexpr size_t SEARCH_WINDOW = 20;
    size_t ClosestIndex(const Vec3 &position);

    std::vector<Vec3> m_points;
    ReedsShepp::Gear m_gear;
    size_t m_endIndex;
    bool m_isFinalLeg;
    bool m_done = false;
    size_t m_lastIndex = 0;
    float m_prevRemaining = std::numeric_limits<float>::max();
};

// Reeds-Shepp 경로의 PathElement 하나를 오차 없이 실행 (pure pursuit X)
class RSExactSegment : public VehicleSegment
{
public:
    RSExactSegment(ReedsShepp::PathElement element, float steerAngle);

    void Tick(Car &car) override;
    bool IsDone() const override { return m_done; }
    ReedsShepp::Gear GetRequiredGear() const override { return m_element.gear; }
    std::optional<float> GetRequiredSteerAngle() const override { return m_steerAngle; }

private:
    static constexpr float STEER_RAMP_RATE = 1.0f;  // 조향 단계 램프 속도 (rad/s)
    static constexpr float MANEUVER_SPEED = 1.5f;   // 정밀 보정용이라 일반 RS 매뉴버보다 더 저속
    static constexpr float DECEL_ESTIMATE = 0.4f;   // 남은 거리 기준 감속 프로파일에 쓰는 가정 감속도 (m/s^2)
    static constexpr float FINISH_DISTANCE = 0.05f; // 목표 거리 도달 판정 (m) -- 정밀 보정용이라 타이트하게
    static constexpr float STOP_SPEED = 0.05f;      // 이 이하면 완전히 멈췄다고 보고 완료 처리 (m/s)

    ReedsShepp::PathElement m_element;
    float m_steerAngle;
    bool m_isSteering = true;
    float m_traveled = 0.0f;
    bool m_done = false;
};

class SplineFollowSegment : public VehicleSegment
{
public:
    void Tick(Car &car) override;
    bool IsDone() const override { return false; }
};

class ReverseSegment : public VehicleSegment
{
public:
    explicit ReverseSegment(float distance) : m_distance(distance) {}

    void Tick(Car &car) override;
    bool IsDone() const override { return m_done; }
    ReedsShepp::Gear GetRequiredGear() const override { return ReedsShepp::Gear::Backward; }
    std::optional<float> GetRequiredSteerAngle() const override { return 0.0f; }

private:
    static constexpr float REVERSE_STEER_RAMP_RATE = 1.0f;
    static constexpr float REVERSE_SPEED = 1.5f;   // 저속 후진 속도 (m/s)
    static constexpr float DECEL_ESTIMATE = 0.4f;  // 남은 거리 기준 감속 프로파일에 쓰는 가정 감속도 (m/s^2)
    static constexpr float FINISH_DISTANCE = 0.1f; // 목표 거리 도달 판정 (m)
    static constexpr float STOP_SPEED = 0.05f;     // 이 이하면 완전히 멈췄다고 보고 완료 처리 (m/s)

    float m_distance;
    float m_traveled = 0.0f;
    bool m_done = false;
};

class CenterSteerSegment : public VehicleSegment
{
public:
    void Tick(Car &car) override;
    bool IsDone() const override { return m_aligned; }

private:
    static constexpr float CENTER_STEER_RAMP_RATE = 1.0f;
    bool m_aligned = false;
};
