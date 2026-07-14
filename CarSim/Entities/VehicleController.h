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

    // 실행 중이던 계획을 그냥 버린다. 감속/제동 등 그 다음에 뭘 할지는 호출자(AI FSM) 책임 —
    // Abort 자체는 조향/가속에 어떤 명령도 내리지 않는다.
    void Abort();

private:
    std::vector<std::unique_ptr<VehicleSegment>> m_segments;
    size_t m_index = 0;
};
