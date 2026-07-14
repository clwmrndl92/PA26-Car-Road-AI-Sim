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

    // 다음 세그먼트가 지금과 다른 기어를 요구하면, 완전히 멈춘 뒤 기어 변경
    bool wantsReverse = m_segments[m_index]->GetRequiredGear() == ReedsShepp::Gear::Backward;
    if (wantsReverse != car.IsReverse())
    {
        if (car.GetSpeed() > 0.0f)
        {
            car.EmergBrake();
            return;
        }
        car.ChangeGear();
        return;
    }

    m_segments[m_index]->Tick(car);
    if (m_segments[m_index]->IsDone())
        ++m_index;
}
