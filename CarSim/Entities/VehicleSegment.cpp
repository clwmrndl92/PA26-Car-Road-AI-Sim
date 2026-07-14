#include "VehicleSegment.h"
#include "Car.h"
#include <algorithm>
#include <cmath>

ArcMoveSegment::ArcMoveSegment(ReedsShepp::Steering steering, ReedsShepp::Gear gear, float distance, float steerAngle)
    : m_steering(steering), m_gear(gear), m_distance(distance), m_steerAngle(steerAngle)
{
}

void ArcMoveSegment::Tick(Car &car)
{
    // Steer()의 부호 규약(양수 = 오른쪽 조향)은 Car::PurePursuit과 동일하게 맞춘다.
    // 조향각 자체는 기어와 무관하게 Left/Right 레이블로만 고정된다 — 같은 바퀴 각도로 전진/후진하면
    // 같은 원(중심)을 반대 방향으로 도는 것뿐이라, 기어에 따라 조향각 부호를 바꾸면 안 된다
    // (바꾸면 원이 바뀌어버려서, 예를 들어 3점 회전(Left-전진/Right-후진/Left-전진)의 세 원이
    // 전부 같은 원으로 붕괴한다).
    float steer = 0.0f;
    if (m_steering == ReedsShepp::Steering::Left)
        steer = -m_steerAngle;
    else if (m_steering == ReedsShepp::Steering::Right)
        steer = m_steerAngle;

    car.Steer(steer);

    // 목표 조향각까지 다 돌아가기 전에는 정지 상태를 유지하고, 정렬된 뒤에만 움직인다.
    if (std::fabs(car.GetSteerAngle() - steer) > STEER_ALIGN_TOLERANCE)
    {
        car.Accelerate(0.0f);
        return;
    }

    // 남은 거리 기준으로 감속해서 세그먼트 끝(m_distance)에서 속도가 0에 가까워지도록 한다
    // (그냥 MANEUVER_SPEED로 계속 밀면 IsDone()이 될 때까지 감속을 안 해서 목표를 오버슈트한다).
    float remaining = std::max(0.0f, m_distance - m_traveled);
    float targetSpeed = std::min(MANEUVER_SPEED, std::sqrt(2.0f * DECEL_ESTIMATE * remaining));
    car.Accelerate(targetSpeed);
    m_traveled += car.GetSpeed() * car.GetDeltaTime();
}

void SplineFollowSegment::Tick(Car &car)
{
    car.DriveControl();
}
