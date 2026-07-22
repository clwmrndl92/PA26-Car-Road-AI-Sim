#pragma once

// 상태 없이 simTime만으로 매번 계산하는 순수 함수 -- 그래서 틱/동기화가 따로 필요 없다.
namespace TrafficSignal
{
    enum class Color
    {
        Green,
        Yellow,
        Red
    };

    // Green -> Yellow -> Red -> 순환. phaseOffset은 교차로별 위상차용.
    Color GetColor(float greenDuration, float yellowDuration, float redDuration, float phaseOffset, float simTime);
}
