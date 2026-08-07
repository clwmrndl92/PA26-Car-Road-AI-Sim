#pragma once
#include <memory>
#include <vector>
#include "VehicleSegment.h"

class Car;

class VehicleController
{
public:
    void BeginPlan(std::vector<std::unique_ptr<VehicleSegment>> segments);
    void Tick(Car &car);
    bool IsFinished() const;

    // Abort 자체는 조향/가속에 어떤 명령도 내리지 않음 -> 외부 처리
    void Abort();

private:
    std::vector<std::unique_ptr<VehicleSegment>> m_segments;
    size_t m_index = 0;
};
