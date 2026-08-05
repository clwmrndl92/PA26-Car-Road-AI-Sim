#include "SimulationState.h"
#include "RoadDataManager.h"

TrafficSignal::Color SimulationState::GetSignalColor(float phaseOffset, float greenDuration, float yellowDuration,
                                                     float redDuration) const
{
    return TrafficSignal::GetColor(greenDuration, yellowDuration, redDuration, phaseOffset, m_simTime);
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
