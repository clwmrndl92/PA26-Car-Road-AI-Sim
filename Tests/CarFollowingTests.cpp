#include "TestHarness.h"
#include "Nav/CarFollowing.h"
#include <cmath>

namespace
{
    CarFollowing::Params DefaultParams()
    {
        CarFollowing::Params p;
        p.v0 = 30.0f;
        p.T = 1.5f;
        p.s0 = 2.0f;
        p.a = 1.5f;
        p.b = 2.0f;
        p.delta = 4.0f;
        p.coolness = 1.0f;
        return p;
    }

    bool IsFinite(float v) { return std::isfinite(v); }
}

// [예외 처리 1] 겹침/역전(gap<=0.001) -> 강제 비상 제동(-b*4)을 즉시 반환해야 함.
TEST_CASE(CarFollowing_Overlap_ReturnsEmergencyBrake)
{
    CarFollowing::Params p = DefaultParams();
    float accel = CarFollowing::CalculateAcceleration(10.0f, 0.0f, 5.0f, 0.0f, 0.0f, p);
    CHECK(std::abs(accel - (-p.b * 4.0f)) < 0.001f);

    accel = CarFollowing::CalculateAcceleration(10.0f, 0.0f, 5.0f, 0.0f, -3.0f, p); // 역전(겹침)도 동일 처리
    CHECK(std::abs(accel - (-p.b * 4.0f)) < 0.001f);
}

// 앞이 완전히 뚫려있고(gap 매우 큼) 목표 속도보다 느릴 때: a_free에 근접한 양의 가속을 내야 함.
TEST_CASE(CarFollowing_OpenRoad_AcceleratesTowardsFreeFlow)
{
    CarFollowing::Params p = DefaultParams();
    float accel = CarFollowing::CalculateAcceleration(10.0f, 0.0f, p.v0, 0.0f, 100000.0f, p);

    float speedRatio = 10.0f / p.v0;
    float expectedFree = p.a * (1.0f - std::pow(speedRatio, p.delta));
    CHECK(accel > 0.0f);
    CHECK(std::abs(accel - expectedFree) < 0.05f);
}

// [예외 처리 2] 내 속도가 정확히 v0와 같아 a_free가 0에 수렴할 때: NaN 없이 0에 가까운 값을 내야 함.
TEST_CASE(CarFollowing_AtDesiredSpeed_NoNaN)
{
    CarFollowing::Params p = DefaultParams();
    float accel = CarFollowing::CalculateAcceleration(p.v0, 0.0f, p.v0, 0.0f, 100000.0f, p);
    CHECK(IsFinite(accel));
    CHECK(std::abs(accel) < 0.5f);
}

// z >= 1 (원하는 안전거리보다 실제 간격이 좁음) -> IDM 제동 영역이라 감속(음수)이어야 함.
TEST_CASE(CarFollowing_TooClose_Decelerates)
{
    CarFollowing::Params p = DefaultParams();
    // s* 는 s0 + v*T 근처(같은 속도라 delta_v=0) = 2 + 10*1.5 = 17. gap을 그보다 훨씬 좁게.
    float accel = CarFollowing::CalculateAcceleration(10.0f, 0.0f, 10.0f, 0.0f, 5.0f, p);
    CHECK(accel < 0.0f);
    CHECK(IsFinite(accel));
}

// [예외 처리 3] CAH 분모(leaderSpeed^2 - 2*gap*a_leader_eff)가 0이 되는 특이점: NaN/Inf 없이
// 유효 가속도(a_cah = effective_leader_accel)를 내야 함. effective_leader_accel은 p.a로 clamp되므로
// leaderAccel을 크게 줘서 clamp를 확정시키고, egoSpeed=0으로 두어 조건절(closing-risk 분기)에 들어가게 한다.
TEST_CASE(CarFollowing_CAHSingularity_NoNaN)
{
    CarFollowing::Params p = DefaultParams();
    float gap = 5.0f;
    float leaderSpeed = std::sqrt(2.0f * gap * p.a); // leaderSpeed^2 == 2*gap*p.a -> 분모 0
    float leaderAccel = p.a + 10.0f;                 // clamp돼서 effective_leader_accel == p.a
    float accel = CarFollowing::CalculateAcceleration(0.0f, 0.0f, leaderSpeed, leaderAccel, gap, p);
    CHECK(IsFinite(accel));
}

// 급격한 끼어들기(가까운 거리에 느린 리더) 상황에서도 결과가 항상 유한해야 함 (전역 강건성).
TEST_CASE(CarFollowing_CutIn_StaysFinite)
{
    CarFollowing::Params p = DefaultParams();
    float accel = CarFollowing::CalculateAcceleration(25.0f, 0.0f, 2.0f, -3.0f, 1.0f, p);
    CHECK(IsFinite(accel));
    CHECK(accel < 0.0f); // 빠르게 다가가는 상황이므로 감속해야 함
}
