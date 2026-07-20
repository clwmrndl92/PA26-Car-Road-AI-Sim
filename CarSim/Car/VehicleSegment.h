#pragma once
#include "Nav/ReedsShepp.h"
#include <optional>
#include <vector>

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

// Reeds-Shepp 경로 중 기어가 같은 한 구간(leg)을 Pure Pursuit로 추종한다. 매 틱 실제 위치
// 기준으로 조향을 다시 계산해 부드럽게 이어진다. 정확도는 스냅(위치/방향을 경로에 강제로
// 붙이는 방식, 현재 비활성화 -- Tick()의 주석 참고) 대신 lookahead 거리를 줄여서 확보한다.
// Park/Avoid 저속 매뉴버에 쓰인다.
class RSFollowSegment : public VehicleSegment
{
public:
    RSFollowSegment(std::vector<Vec3> points, ReedsShepp::Gear gear);

    void Tick(Car &car) override;
    bool IsDone() const override { return m_done; }
    ReedsShepp::Gear GetRequiredGear() const override { return m_gear; }

private:
    static constexpr float MANEUVER_SPEED = 3.0f; // 저속 주차 기동 속도 (m/s)
    // 남은 거리 기준 감속 프로파일에 쓰는 가정 감속도 (m/s^2).
    static constexpr float DECEL_ESTIMATE = 0.4f;
    static constexpr float FINISH_DISTANCE = 0.3f; // leg 끝까지 이 거리 이내면 완료로 본다 (m).
    // Pure Pursuit lookahead 최소값. 주차 매뉴버의 회전반경(대략 휠베이스 크기)과 비슷하거나 더
    // 크면 곡선 구간에서 안쪽으로 크게 잘라 도는 정상상태 오차가 커진다 -- 스냅 대신 이 값을
    // 작게 잡아 추종 정확도를 확보한다(단, 너무 작으면 조향이 떨리므로 과도하게 줄이지 말 것).
    static constexpr float MIN_LOOKAHEAD = 1.5f;
    // 최근접점 탐색을 m_lastIndex부터 이만큼(점 개수, 0.5m 간격 기준 ~10m)만 앞으로 훑는다.
    // RS 주차 경로는 급커브가 많아 인덱스는 멀어도 유클리드 거리는 가까운 자기교차 구간이
    // 흔한데, 전체를 훑으면 그런 엉뚱한(너무 앞선/뒤처진) 점으로 잘못 타겟팅해 경로를
    // 건너뛰거나(remaining이 갑자기 0 근처로 계산돼 그 자리에서 멈춰버림) 한다.
    static constexpr size_t SEARCH_WINDOW = 20;

    // [스냅 비활성화 - 나중에 다시 쓸 수도 있어 남겨둠] 스냅을 허용할 최대 위치/방향 오차.
    // static constexpr float SNAP_DISTANCE = 0.1f;
    // static constexpr float SNAP_ANGLE = ToRadians(3.0f);

    // points 상에서, m_lastIndex 이후 전방 윈도우 안에서 position에 가장 가까운 인덱스를 찾고
    // m_lastIndex를 갱신한다(뒤로는 가지 않음 -- 자기교차 구간에서 엉뚱한 점으로 튀는 걸 방지).
    size_t ClosestIndex(const Vec3 &position);
    // [스냅 비활성화 - 나중에 다시 쓸 수도 있어 남겨둠] index 지점의 진행 방향 접선(다음-이전
    // 점 차). 폴리라인 자체의 순방향이며 기어와 무관하다.
    // Vec3 TangentAt(size_t index) const;

    std::vector<Vec3> m_points;
    ReedsShepp::Gear m_gear;
    bool m_done = false;
    size_t m_lastIndex = 0;
};

// 기존 정속주행(스플라인을 Pure Pursuit + 속도 프로파일로 추종)을 그대로 실행한다.
class SplineFollowSegment : public VehicleSegment
{
public:
    void Tick(Car &car) override;
    bool IsDone() const override { return false; }
};

// 출차(Park) 매뉴버가 꺾어놓은 조향각을 중앙(0)으로 되돌린다. 정렬될 때까지는 제자리에서
// 대기하고(가속 0), 정렬되면 끝난다.
class CenterSteerSegment : public VehicleSegment
{
public:
    void Tick(Car &car) override;
    bool IsDone() const override { return m_aligned; }

private:
    static constexpr float CENTER_STEER_RAMP_RATE = 1.0f;
    bool m_aligned = false;
};
