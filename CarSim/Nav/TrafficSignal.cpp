#include "TrafficSignal.h"
#include <cmath>

namespace TrafficSignal
{
    Color GetColor(float greenDuration, float yellowDuration, float redDuration, float phaseOffset, float simTime)
    {
        float cycleLength = greenDuration + yellowDuration + redDuration;

        float t = std::fmod(simTime + phaseOffset, cycleLength);
        if (t < 0.0f)
            t += cycleLength;

        if (t < greenDuration)
            return Color::Green;
        if (t < greenDuration + yellowDuration)
            return Color::Yellow;
        return Color::Red;
    }
}
