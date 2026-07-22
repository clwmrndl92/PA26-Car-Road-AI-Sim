#include "TestHarness.h"
#include "Nav/TrafficSignal.h"

TEST_CASE(TrafficSignal_WithinRedWindow_ReturnsRed)
{
    CHECK(TrafficSignal::GetColor(20.0f, 12.0f, 0.0f, 5.0f) == TrafficSignal::Color::Red);
}

TEST_CASE(TrafficSignal_WithinGreenWindow_ReturnsGreen)
{
    CHECK(TrafficSignal::GetColor(20.0f, 12.0f, 0.0f, 15.0f) == TrafficSignal::Color::Green);
}

TEST_CASE(TrafficSignal_AtRedGreenBoundary_ReturnsGreen)
{
    // t == redDuration은 이미 초록 구간 (경계는 [0, redDuration)만 빨강).
    CHECK(TrafficSignal::GetColor(20.0f, 12.0f, 0.0f, 12.0f) == TrafficSignal::Color::Green);
}

TEST_CASE(TrafficSignal_WrapsAcrossMultipleCycles)
{
    // 45초 = 2주기(40) + 5초 -> 주기 안 5초 지점과 같은 결과여야 함.
    CHECK(TrafficSignal::GetColor(20.0f, 12.0f, 0.0f, 45.0f) == TrafficSignal::Color::Red);
    CHECK(TrafficSignal::GetColor(20.0f, 12.0f, 0.0f, 55.0f) == TrafficSignal::Color::Green);
}

TEST_CASE(TrafficSignal_PhaseOffsetShiftsBoundary)
{
    // phaseOffset=10인 신호는 simTime=5일 때 (5+10)=15초 지점 -> 초록.
    CHECK(TrafficSignal::GetColor(20.0f, 12.0f, 10.0f, 5.0f) == TrafficSignal::Color::Green);
}

TEST_CASE(TrafficSignal_NegativeSimTime_StillWrapsCorrectly)
{
    // -5초 -> fmod가 음수를 줄 수 있는 경계 케이스. 주기 보정 후 15초 지점과 같아야 함(초록).
    CHECK(TrafficSignal::GetColor(20.0f, 12.0f, 0.0f, -5.0f) == TrafficSignal::Color::Green);
}
