#include "VehicleSegment.h"
#include "Car.h"
#include <algorithm>
#include <cmath>
#include <limits>

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

// [스냅 비활성화 - 나중에 다시 쓸 수도 있어 남겨둠]
// Vec3 RSFollowSegment::TangentAt(size_t index) const
// {
//     size_t next = std::min(index + 1, m_points.size() - 1);
//     size_t prev = (index == 0) ? 0 : index - 1;
//     if (next == prev)
//         return Vec3(0.0f, 0.0f, 0.0f);
//     return (m_points[next] - m_points[prev]).Normalized();
// }

void RSFollowSegment::Tick(Car &car)
{
    static constexpr float PARKING_STEER_RAMP_RATE = 1.0f;
    // 목표각에 가까울수록 느려지는 지수 감쇠 (초당 감쇠 비율, 1/s) -- 더 이상 기어가 안 바뀌는
    // 마지막 leg에서만 쓴다. 중간 leg(매뉴버 도중 여러 번 방향을 바꾸는 구간)는 경로 추종
    // 정확도가 더 중요해 기존 선형 램프(Steer)를 그대로 쓴다.
    static constexpr float PARKING_STEER_EASE_RATE = 3.0f;

    if (m_points.size() < 2)
    {
        m_done = true;
        car.Accelerate(0.0f);
        return;
    }

    Vec3 position = car.GetRigidbodyPosition();
    size_t closest = ClosestIndex(position);

    // [스냅 비활성화 - 나중에 다시 쓸 수도 있어 남겨둠] 경로와 이미 충분히 가깝고(위치) 방향도
    // 비슷하면, 그 지점에 정확히 붙여 Pure Pursuit의 정상상태 추종 오차가 누적되지 않게 한다.
    // 대신 지금은 아래 MIN_LOOKAHEAD를 줄여서 추종 정확도를 확보한다.
    // Vec3 tangent = TangentAt(closest);
    // if (tangent.Length() > 1e-4f)
    // {
    //     Vec3 closestPoint = m_points[closest];
    //     float distanceToPath = (closestPoint - position).Length();
    //     float headingDot = std::clamp(car.GetForwardAxis().Dot(tangent), -1.0f, 1.0f);
    //     if (distanceToPath < SNAP_DISTANCE && std::acos(headingDot) < SNAP_ANGLE)
    //     {
    //         car.SetRotation(tangent);
    //         car.SetPosition(closestPoint + tangent * car.GetWheelbase());
    //         position = closestPoint;
    //         closest = ClosestIndex(position);
    //     }
    // }

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
        remaining = (m_points[m_endIndex] - position).Length();
    }

    if (remaining < FINISH_DISTANCE)
    {
        m_done = true;
        car.Accelerate(0.0f);
        return;
    }

    float lookahead = std::max(MIN_LOOKAHEAD, car.GetSpeed() * 1.0f);
    size_t targetIndex = closest;
    float accumulated = 0.0f;
    while (accumulated < lookahead && targetIndex + 1 < m_points.size())
    {
        accumulated += (m_points[targetIndex + 1] - m_points[targetIndex]).Length();
        ++targetIndex;
    }

    float targetSteer = car.PurePursuit(m_points[targetIndex]);
    if (m_isFinalLeg)
        car.SteerEase(targetSteer, PARKING_STEER_EASE_RATE);
    else
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
