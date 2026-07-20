#include "VehicleSegment.h"
#include "Car.h"
#include <algorithm>
#include <cmath>
#include <limits>

RSFollowSegment::RSFollowSegment(std::vector<Vec3> points, ReedsShepp::Gear gear)
    : m_points(std::move(points)), m_gear(gear)
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

    float remaining = 0.0f;
    for (size_t i = closest; i + 1 < m_points.size(); ++i)
        remaining += (m_points[i + 1] - m_points[i]).Length();

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
    car.Steer(targetSteer, PARKING_STEER_RAMP_RATE);

    float targetSpeed = std::min(MANEUVER_SPEED, std::sqrt(2.0f * DECEL_ESTIMATE * remaining));
    car.Accelerate(targetSpeed);
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
