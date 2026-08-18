#include "VehicleController.h"
#include "Car.h"

void VehicleController::BeginPlan(std::vector<std::unique_ptr<VehicleSegment>> segments)
{
    m_segments = std::move(segments);
    m_index = 0;
}

bool VehicleController::IsFinished() const
{
    return m_index >= m_segments.size();
}

void VehicleController::Abort()
{
    m_segments.clear();
    m_index = 0;
}

void VehicleController::Tick(Car &car)
{
    if (IsFinished())
        return;

    m_segments[m_index]->Tick(car);
    if (m_segments[m_index]->IsDone())
        ++m_index;
}
