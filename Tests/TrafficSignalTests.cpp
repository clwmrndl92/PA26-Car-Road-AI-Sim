#include "TestHarness.h"
#include "Nav/TrafficSignal.h"

namespace
{
    // green=8, yellow=3, red=12 -> cycle=23. Phase order: [0,8)=Green, [8,11)=Yellow, [11,23)=Red.
    TrafficSignal::Color ColorAt(float simTime, float phaseOffset = 0.0f)
    {
        return TrafficSignal::GetColor(8.0f, 3.0f, 12.0f, phaseOffset, simTime);
    }
}

TEST_CASE(TrafficSignal_WithinGreenWindow_ReturnsGreen)
{
    CHECK(ColorAt(0.0f) == TrafficSignal::Color::Green);
    CHECK(ColorAt(5.0f) == TrafficSignal::Color::Green);
}

TEST_CASE(TrafficSignal_WithinYellowWindow_ReturnsYellow)
{
    CHECK(ColorAt(8.0f) == TrafficSignal::Color::Yellow);
    CHECK(ColorAt(10.0f) == TrafficSignal::Color::Yellow);
}

TEST_CASE(TrafficSignal_WithinRedWindow_ReturnsRed)
{
    CHECK(ColorAt(11.0f) == TrafficSignal::Color::Red);
    CHECK(ColorAt(20.0f) == TrafficSignal::Color::Red);
}

TEST_CASE(TrafficSignal_AtGreenYellowBoundary_ReturnsYellow)
{
    // t == greenDuration은 이미 노란불 구간 (초록은 [0, green)만).
    CHECK(ColorAt(8.0f) == TrafficSignal::Color::Yellow);
}

TEST_CASE(TrafficSignal_AtYellowRedBoundary_ReturnsRed)
{
    CHECK(ColorAt(11.0f) == TrafficSignal::Color::Red);
}

TEST_CASE(TrafficSignal_WrapsAcrossMultipleCycles)
{
    // 23(cycle) + 5초 -> 주기 안 5초 지점(초록)과 같은 결과여야 함.
    CHECK(ColorAt(28.0f) == TrafficSignal::Color::Green);
    // 23 + 9초 -> 주기 안 9초 지점(노랑)과 같아야 함.
    CHECK(ColorAt(32.0f) == TrafficSignal::Color::Yellow);
}

TEST_CASE(TrafficSignal_PhaseOffsetShiftsBoundary)
{
    // phaseOffset=10인 신호는 simTime=0일 때 (0+10)=10초 지점 -> 노랑.
    CHECK(ColorAt(0.0f, 10.0f) == TrafficSignal::Color::Yellow);
}

TEST_CASE(TrafficSignal_NegativeSimTime_StillWrapsCorrectly)
{
    // -3초 -> fmod가 음수를 줄 수 있는 경계 케이스. 주기 보정 후 20초 지점(빨강)과 같아야 함.
    CHECK(ColorAt(-3.0f) == TrafficSignal::Color::Red);
}
