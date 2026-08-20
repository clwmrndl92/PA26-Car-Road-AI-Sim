// Car.h를 먼저 넣는다. RoadDataManager.h가 using namespace std를 하기 때문에,
// 그 뒤에 Windows/DirectX 헤더가 들어오면 std::byte와 rpcndr.h의 byte가 충돌한다.
#include "Car/Car.h"
#include "SimulationState.h"
#include "RoadDataManager.h"

TrafficSignal::Color SimulationState::GetSignalColor(float phaseOffset, float greenDuration, float yellowDuration,
                                                     float redDuration) const
{
    return TrafficSignal::GetColor(greenDuration, yellowDuration, redDuration, phaseOffset, m_simTime);
}

// 레이스 순위. 매 프레임 정렬할 이유가 없어서 주기를 둔다.
void SimulationState::UpdateStandings()
{
    constexpr float STANDINGS_INTERVAL = 0.25f;

    if (m_raceLines.empty() || m_cars.size() < 2 || m_simTime - m_lastStandingsTime < STANDINGS_INTERVAL)
        return;
    m_lastStandingsTime = m_simTime;

    // 레이스 중인 차만 센다. 라인이 만들어져 있어도 일반 주행 차는 진행도가 없어서,
    // 같이 넣으면 순위가 무의미하게 흔들리고 추월 카운트가 부풀려진다.
    std::vector<Car *> ordered;
    for (Car *car : m_cars)
        if (car->IsRaceMode())
            ordered.push_back(car);
    if (ordered.size() < 2)
        return;

    std::sort(ordered.begin(), ordered.end(),
              [](const Car *a, const Car *b) { return a->GetRaceProgress() > b->GetRaceProgress(); });

    for (size_t i = 0; i < ordered.size(); ++i)
        ordered[i]->UpdateRacePosition(static_cast<int>(i) + 1, m_simTime);
}

void SimulationState::UnregisterCar(Car *car)
{
    m_cars.erase(std::remove(m_cars.begin(), m_cars.end(), car), m_cars.end());
    for (auto it = m_reservedJunctionOwners.begin(); it != m_reservedJunctionOwners.end();)
    {
        it->second.owners.erase(car);
        if (it->second.owners.empty())
            it = m_reservedJunctionOwners.erase(it);
        else
            ++it;
    }
}

std::shared_ptr<RoadNode> SimulationState::TryReserveParkSpot(int parkNodeId, const std::unordered_set<int> &excludeIds)
{
    std::shared_ptr<RoadNode> parkNode = RoadDataManager::Get().GetNode(parkNodeId);
    if (!parkNode)
        return nullptr;

    for (const std::weak_ptr<RoadNode> &weakChild : parkNode->children)
    {
        std::shared_ptr<RoadNode> spot = weakChild.lock();
        if (!spot || spot->nodeType != RoadNodeType::ParkSpot)
            continue;
        if (m_reservedParkSpotIds.count(spot->id) || excludeIds.count(spot->id))
            continue;

        m_reservedParkSpotIds.insert(spot->id);
        return spot;
    }
    return nullptr;
}

void SimulationState::ReleaseParkSpot(int spotNodeId)
{
    m_reservedParkSpotIds.erase(spotNodeId);
}

bool SimulationState::TryReserveJunction(int junctionId, int incomingRoadId, const Car *owner)
{
    if (junctionId < 0 || incomingRoadId < 0 || owner == nullptr)
        return true;

    auto [it, inserted] = m_reservedJunctionOwners.try_emplace(junctionId);
    JunctionReservation &reservation = it->second;
    if (inserted)
        reservation.incomingRoadId = incomingRoadId;
    if (reservation.incomingRoadId != incomingRoadId)
        return false;

    reservation.owners.insert(owner);
    return true;
}

bool SimulationState::IsJunctionAvailable(int junctionId, int incomingRoadId, const Car *owner) const
{
    if (junctionId < 0 || incomingRoadId < 0 || owner == nullptr)
        return true;

    auto it = m_reservedJunctionOwners.find(junctionId);
    return it == m_reservedJunctionOwners.end() || it->second.incomingRoadId == incomingRoadId ||
           it->second.owners.find(owner) != it->second.owners.end();
}

void SimulationState::ReleaseJunction(int junctionId, const Car *owner)
{
    auto it = m_reservedJunctionOwners.find(junctionId);
    if (it == m_reservedJunctionOwners.end())
        return;

    it->second.owners.erase(owner);
    if (it->second.owners.empty())
        m_reservedJunctionOwners.erase(it);
}
