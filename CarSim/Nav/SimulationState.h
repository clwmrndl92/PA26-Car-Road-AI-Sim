#pragma once
#include <algorithm>
#include <memory>
#include <unordered_set>
#include <vector>
#include "TrafficSignal.h"

class Car;
struct RoadNode;

class SimulationState
{
public:
    void Tick(float dt) { m_simTime += dt; } // 전역 시뮬레이션 시계 누적
    TrafficSignal::Color GetSignalColor(float phaseOffset) const;

    // 시뮬레이션에 존재하는 모든 차
    void RegisterCar(Car *car) { m_cars.push_back(car); }
    void UnregisterCar(Car *car) { m_cars.erase(std::remove(m_cars.begin(), m_cars.end(), car), m_cars.end()); }
    const std::vector<Car *> &GetCars() const { return m_cars; }

    std::shared_ptr<RoadNode> TryReserveParkSpot(int parkNodeId, const std::unordered_set<int> &excludeIds = {});
    void ReleaseParkSpot(int spotNodeId);

private:
    static constexpr float SIGNAL_GREEN_DURATION = 8.0f;  // 초록불 지속 시간
    static constexpr float SIGNAL_YELLOW_DURATION = 3.0f; // 노란불 지속 시간
    static constexpr float SIGNAL_RED_DURATION = 12.0f;   // 빨간불 지속 시간

    std::unordered_set<int> m_reservedParkSpotIds; // 예약된(다른 차가 목표로 잡은) ParkSpot 노드 id
    std::vector<Car *> m_cars;
    float m_simTime = 0.0f; // Tick()으로만 누적되는 전역 시뮬레이션 시계.
};
