#include "VehicleSegment.h"
#include "Car.h"
#include <algorithm>
#include <cmath>

ArcMoveSegment::ArcMoveSegment(ReedsShepp::Steering steering, ReedsShepp::Gear gear, float distance, float steerAngle)
    : m_steering(steering), m_gear(gear), m_distance(distance), m_steerAngle(steerAngle)
{
}

std::optional<float> ArcMoveSegment::GetRequiredSteerAngle() const
{
    if (m_steering == ReedsShepp::Steering::Left)
        return -m_steerAngle;
    if (m_steering == ReedsShepp::Steering::Right)
        return m_steerAngle;
    return 0.0f;
}

void ArcMoveSegment::Tick(Car &car)
{
    static constexpr float PARKING_STEER_RAMP_RATE = 1.0f;
    static constexpr float STEER_EXACT_TOLERANCE = 1e-4f;
    float steer = *GetRequiredSteerAngle();
    car.Steer(steer, PARKING_STEER_RAMP_RATE);

    // 목표 조향각까지 다 돌아가기 전에는 정지 상태를 유지하고, 정렬된 뒤에만 움직인다.
    if (std::fabs(car.GetSteerAngle() - steer) > STEER_EXACT_TOLERANCE)
    {
        car.Accelerate(0.0f);
        return;
    }

    float remaining = std::max(0.0f, m_distance - m_traveled);
    float targetSpeed = std::min(MANEUVER_SPEED, std::sqrt(2.0f * DECEL_ESTIMATE * remaining));
    car.Accelerate(targetSpeed);
    m_traveled += car.GetSpeed() * car.GetDeltaTime();
}

void SplineFollowSegment::Tick(Car &car)
{
    car.DriveControl();
}

void CenterSteerSegment::Tick(Car &car)
{
    car.Accelerate(0.0f);
    car.Steer(0.0f, CENTER_STEER_RAMP_RATE);
    if (std::fabs(car.GetSteerAngle()) <= STEER_ALIGN_TOLERANCE)
        m_aligned = true;
}
