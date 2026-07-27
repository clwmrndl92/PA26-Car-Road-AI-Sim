#include "SimulationState.h"
#include "RoadDataManager.h"

TrafficSignal::Color SimulationState::GetSignalColor(float phaseOffset) const
{
    return TrafficSignal::GetColor(SIGNAL_GREEN_DURATION, SIGNAL_YELLOW_DURATION, SIGNAL_RED_DURATION, phaseOffset, m_simTime);
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
