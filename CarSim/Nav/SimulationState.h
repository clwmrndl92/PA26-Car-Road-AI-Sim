#pragma once
#include <algorithm>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "TrafficSignal.h"
#include "Car/RaceLine.h"

class Car;
struct RoadNode;

class SimulationState
{
public:
    void Tick(float dt)
    {
        m_simTime += dt; // 전역 시뮬레이션 시계 누적
        UpdateStandings();
    }
    float GetSimTime() const { return m_simTime; }
    // durations는 신호(RoadNode)마다 다르게 줄 수 있다 -- 이 클래스는 전역 시계(m_simTime)만 들고 있다.
    TrafficSignal::Color GetSignalColor(float phaseOffset, float greenDuration, float yellowDuration,
                                        float redDuration) const;

    // 시뮬레이션에 존재하는 모든 차
    void RegisterCar(Car *car) { m_cars.push_back(car); }
    void UnregisterCar(Car *car);
    const std::vector<Car *> &GetCars() const { return m_cars; }
    // Car::m_nearbyCars는 0.2초 주기 캐시라 그 사이 삭제된 차의 포인터를 들고 있을 수 있다.
    // 역참조 없이 포인터 값만 등록부와 비교하므로, 이미 delete된 대상에도 안전하게 쓸 수 있다.
    bool IsCarAlive(const Car *car) const { return std::find(m_cars.begin(), m_cars.end(), car) != m_cars.end(); }

    // 트랙 전체 레이싱 라인. QP가 비싸서 전략별로 시작할 때 한 번만 풀고 모든 차가 공유한다.
    // 인덱스는 Car::ApexStrategy 값. 여기서 enum을 쓰지 않는 건 Car.h를 끌어오지 않기 위해서다.
    void SetRaceLines(std::vector<RaceLine> lines) { m_raceLines = std::move(lines); }
    const RaceLine *GetRaceLine(int strategy) const
    {
        if (strategy < 0 || static_cast<size_t>(strategy) >= m_raceLines.size())
            return nullptr;
        return m_raceLines[strategy].IsValid() ? &m_raceLines[strategy] : nullptr;
    }
    bool HasRaceLines() const { return !m_raceLines.empty(); }

    // 진행도 순으로 순위를 매기고, 순위 변동을 추월/피추월로 센다(레이스 모드 전용).
    void UpdateStandings();

    std::shared_ptr<RoadNode> TryReserveParkSpot(int parkNodeId, const std::unordered_set<int> &excludeIds = {});
    void ReleaseParkSpot(int spotNodeId);

    // Vehicles entering from the same road share a reservation; other approaches wait.
    bool TryReserveJunction(int junctionId, int incomingRoadId, const Car *owner);
    bool IsJunctionAvailable(int junctionId, int incomingRoadId, const Car *owner) const;
    void ReleaseJunction(int junctionId, const Car *owner);

private:
    struct JunctionReservation
    {
        int incomingRoadId = -1;
        std::unordered_set<const Car *> owners;
    };

    std::unordered_set<int> m_reservedParkSpotIds; // 예약된(다른 차가 목표로 잡은) ParkSpot 노드 id
    std::vector<RaceLine> m_raceLines;
    float m_lastStandingsTime = -1000.0f;
    std::vector<Car *> m_cars;
    std::unordered_map<int, JunctionReservation> m_reservedJunctionOwners;
    float m_simTime = 0.0f; // Tick()으로만 누적되는 전역 시뮬레이션 시계.
};
