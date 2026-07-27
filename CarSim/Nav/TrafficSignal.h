#pragma once

namespace TrafficSignal
{
    enum class Color
    {
        Green,
        Yellow,
        Red
    };

    Color GetColor(float greenDuration, float yellowDuration, float redDuration, float phaseOffset, float simTime);
}
