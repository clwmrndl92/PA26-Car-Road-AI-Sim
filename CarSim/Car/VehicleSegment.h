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
    // endIndex: points[endIndex]가 실제 목표 지점. 마지막 leg는 endIndex 이후에 최종 정렬용으로
    // 연장된 점들이 더 있을 수 있는데(ReedsShepp::SampleLegs 참고), 그 연장분은 Pure Pursuit의
    // 조준점 후보로만 쓰고 완료/제동 판정에는 쓰지 않는다.
    // isFinalLeg: 전체 RS 경로의 마지막 leg(이후로 기어가 다시 안 바뀜)인지 -- 중간 leg는 경로
    // 추종 정확도가 중요해 기존 선형 램프(Steer)를 그대로 쓰고, 마지막 leg만 Pure Pursuit 목표
    // 조향각에 지수 감쇠(SteerEase)로 부드럽게 붙는다.
    RSFollowSegment(std::vector<Vec3> points, ReedsShepp::Gear gear, size_t endIndex, bool isFinalLeg);

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
    size_t m_endIndex;
    bool m_isFinalLeg;
    bool m_done = false;
    size_t m_lastIndex = 0;
};

// Reeds-Shepp 경로의 PathElement 하나를 오차 누적 없이 그대로 실행한다: 완전히 멈춘 채로 정확한
// 조향각까지 돌리고(그 사이 GetRequiredSteerAngle을 통해 VehicleController가 차가 움직이지 못하게
// 막아준다), 정렬되면 그 조향각을 고정한 채 정확한 거리(param)만큼만 가속-감속해서 이동한 뒤 다시
// 완전히 멈춘다. RSFollowSegment(Pure Pursuit)와 달리 매 틱 목표점을 다시 겨냥하지 않으므로 추종
// 오차가 누적되지 않는다 -- 대신 element마다 정지를 거쳐야 해서 느리다. 짧고 정확해야 하는 최종
// 정렬 보정에만 쓴다.
class RSExactSegment : public VehicleSegment
{
public:
    RSExactSegment(ReedsShepp::PathElement element, float steerAngle);

    void Tick(Car &car) override;
    bool IsDone() const override { return m_done; }
    ReedsShepp::Gear GetRequiredGear() const override { return m_element.gear; }
    std::optional<float> GetRequiredSteerAngle() const override { return m_steerAngle; }

private:
    static constexpr float STEER_RAMP_RATE = 1.0f; // 조향 단계 램프 속도 (rad/s)
    static constexpr float MANEUVER_SPEED = 1.5f;  // 정밀 보정용이라 일반 RS 매뉴버보다 더 저속
    static constexpr float DECEL_ESTIMATE = 0.4f;  // 남은 거리 기준 감속 프로파일에 쓰는 가정 감속도 (m/s^2)
    static constexpr float FINISH_DISTANCE = 0.05f; // 목표 거리 도달 판정 (m) -- 정밀 보정용이라 타이트하게
    static constexpr float STOP_SPEED = 0.05f;       // 이 이하면 완전히 멈췄다고 보고 완료 처리 (m/s)

    ReedsShepp::PathElement m_element;
    float m_steerAngle;
    bool m_isSteering = true; // true면 아직 조향 정렬 단계, false면 그 각을 고정한 채 이동 단계
    float m_traveled = 0.0f;
    bool m_done = false;
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
