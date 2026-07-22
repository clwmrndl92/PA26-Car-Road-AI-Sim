#include "VehicleSegment.h"
#include "Car.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <Utill/DebugConsole.h>

RSFollowSegment::RSFollowSegment(std::vector<Vec3> points, ReedsShepp::Gear gear, size_t endIndex, bool isFinalLeg)
    : m_points(std::move(points)), m_gear(gear), m_endIndex(endIndex), m_isFinalLeg(isFinalLeg)
{
}

size_t RSFollowSegment::ClosestIndex(const Vec3 &position)
{
    size_t begin = m_lastIndex;
    size_t end = std::min(m_points.size() - 1, m_lastIndex + SEARCH_WINDOW);

    size_t closest = begin;
    float closestDist = std::numeric_limits<float>::max();
    for (size_t i = begin; i <= end; ++i)
    {
        float dist = (m_points[i] - position).Length();
        if (dist < closestDist)
        {
            closestDist = dist;
            closest = i;
        }
    }
    m_lastIndex = closest;
    return closest;
}

void RSFollowSegment::Tick(Car &car)
{
    static constexpr float PARKING_STEER_RAMP_RATE = 1.0f;

    if (m_points.size() < 2)
    {
        m_done = true;
        car.Accelerate(0.0f);
        return;
    }

    Vec3 rigidPosition = car.GetRigidbodyPosition();
    size_t closest = ClosestIndex(rigidPosition);

    // remaining은 실제 목표(m_endIndex)까지의 거리 -- 마지막 leg는 그 뒤로 정렬용 연장 점들이
    // 더 있을 수 있지만(ReedsShepp::SampleLegs), 완료/제동 판정은 그 연장분을 무시하고 실제
    // 목표 지점 기준으로 한다.
    float remaining = 0.0f;
    if (closest < m_endIndex)
    {
        for (size_t i = closest; i < m_endIndex; ++i)
            remaining += (m_points[i + 1] - m_points[i]).Length();
    }
    else
    {
        remaining = (m_points[m_endIndex] - rigidPosition).Length();
    }

    if (remaining < FINISH_DISTANCE)
    {
        m_done = true;
        car.Accelerate(0.0f);
        return;
    }

    if (remaining < OVERSHOOT_CHECK_DISTANCE && remaining > m_prevRemaining)
    {
        m_done = true;
        car.Accelerate(0.0f);
        return;
    }
    m_prevRemaining = remaining;

    float lookahead = std::max(MIN_LOOKAHEAD, car.GetSpeed() * 1.0f);
    size_t targetIndex = closest;
    float accumulated = 0.0f;
    while (accumulated < lookahead && targetIndex + 1 < m_points.size())
    {
        accumulated += (m_points[targetIndex + 1] - m_points[targetIndex]).Length();
        ++targetIndex;
    }

    float targetSteer = car.PurePursuit(m_points[targetIndex]);
    car.Steer(targetSteer, PARKING_STEER_RAMP_RATE);

    float targetSpeed = std::min(MANEUVER_SPEED, std::sqrt(2.0f * DECEL_ESTIMATE * remaining));
    car.Accelerate(targetSpeed);
}

RSExactSegment::RSExactSegment(ReedsShepp::PathElement element, float steerAngle)
    : m_element(element), m_steerAngle(steerAngle)
{
}

void RSExactSegment::Tick(Car &car)
{
    if (m_isSteering)
    {
        car.Accelerate(0.0f);
        car.Steer(m_steerAngle, STEER_RAMP_RATE);
        if (std::fabs(car.GetSteerAngle() - m_steerAngle) <= STEER_ALIGN_TOLERANCE)
            m_isSteering = false;
        return;
    }

    float remaining = m_element.param - m_traveled;
    if (remaining <= FINISH_DISTANCE)
    {
        car.Accelerate(0.0f);
        if (car.GetSpeed() <= STOP_SPEED)
            m_done = true;
        return;
    }

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
