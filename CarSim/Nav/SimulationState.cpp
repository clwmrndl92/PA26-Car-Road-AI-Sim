#include "SimulationState.h"

TrafficSignal::Color SimulationState::GetSignalColor(float phaseOffset, float greenDuration, float yellowDuration,
                                                     float redDuration) const
{
    return TrafficSignal::GetColor(greenDuration, yellowDuration, redDuration, phaseOffset, m_simTime);
}

void SimulationState::UnregisterCar(Car *car)
{
    m_cars.erase(std::remove(m_cars.begin(), m_cars.end(), car), m_cars.end());
}
