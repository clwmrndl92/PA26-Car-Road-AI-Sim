#include "RoadDataManager.h"
#include <fstream>
#include <nlohmann/json.hpp>
#include <queue>
#include <cmath>
#include <unordered_set>
#include <map>
#include <limits>
#include <algorithm>
#include <Core/DebugConsole.h>

void RoadDataManager::Init(const string &filePath)
{
    BuildRoadData(filePath);
}

void RoadDataManager::BuildRoadData(const string &filePath)
{
    ifstream file(filePath);
    nlohmann::json root = nlohmann::json::parse(file, nullptr, false);
    if (root.is_discarded())
        return;

    m_roads.clear();
    m_lanes.clear();
    m_nodes.clear();
    m_edges.clear();

    map<int, shared_ptr<Road>> roadById;
    for (const nlohmann::json &roadJson : root.value("roads", nlohmann::json::array()))
    {
        int id = roadJson.value("id", 0);
        float speedLimit = roadJson.value("speed_limit", 0.0f) / 3.6f;

        auto road = make_shared<Road>(id, speedLimit);
        m_roads.push_back(road);
        roadById[id] = road;
    }

    map<int, shared_ptr<Lane>> laneById;
    for (const nlohmann::json &laneJson : root.value("lanes", nlohmann::json::array()))
    {
        int id = laneJson.value("id", 0);
        int roadId = laneJson.value("road", 0);

        shared_ptr<Road> road;
        auto roadIt = roadById.find(roadId);
        if (roadIt != roadById.end())
            road = roadIt->second;

        vector<Vec3> controlPoints;
        for (const nlohmann::json &point : laneJson.value("control_points", nlohmann::json::array()))
        {
            if (point.size() < 3)
                continue;
            float x = point[0].get<float>();
            float y = point[1].get<float>();
            float z = point[2].get<float>();
            controlPoints.push_back(Vec3(x, y, z));
        }

        auto lane = make_shared<Lane>(id, Spline(controlPoints), road);
        m_lanes.push_back(lane);
        laneById[id] = lane;
    }

    map<int, shared_ptr<RoadNode>> nodeById;
    for (const nlohmann::json &nodeJson : root.value("nodes", nlohmann::json::array()))
    {
        int id = nodeJson.value("id", 0);

        const nlohmann::json &posJson = nodeJson.value("position", nlohmann::json::array());
        if (posJson.size() < 3)
            continue;
        Vec3 position(posJson[0].get<float>(), posJson[1].get<float>(), posJson[2].get<float>());

        int laneId = nodeJson.value("lane", 0);
        shared_ptr<Lane> lane;
        auto laneIt = laneById.find(laneId);
        if (laneIt != laneById.end())
            lane = laneIt->second;

        string typeStr = nodeJson.value("type", "normal");
        const auto &nodeTypeByName = GetRoadNodeTypeByName();
        auto nodeTypeIt = nodeTypeByName.find(typeStr);
        RoadNodeType nodeType = nodeTypeIt != nodeTypeByName.end() ? nodeTypeIt->second : RoadNodeType::Unkown;

        float lanePosition = nodeJson.value("lane_pos", 0);

        float limitSpeed = nodeJson.value("limit_speed", 999) / 3.6f;

        auto node = make_shared<RoadNode>();
        node->id = id;
        node->position = position;
        node->nodeType = nodeType;
        node->lanePosition = lanePosition;
        node->limitSpeed = limitSpeed;
        node->lane = lane;

        m_nodes.push_back(node);
        nodeById[id] = node;
    }

    for (const nlohmann::json &linkJson : root.value("links", nlohmann::json::array()))
    {
        int id = linkJson.value("id", 0);
        int sourceId = linkJson.value("source", 0);
        int targetId = linkJson.value("target", 0);
        float length = linkJson.value("length", 0.0f);

        auto sourceIt = nodeById.find(sourceId);
        auto targetIt = nodeById.find(targetId);
        if (sourceIt == nodeById.end() || targetIt == nodeById.end())
            continue;

        auto edge = make_shared<RoadEdge>();
        edge->id = id;
        edge->endNode = targetIt->second;
        edge->length = length;

        sourceIt->second->edges.push_back(edge);
        m_edges.push_back(edge);
    }
}

shared_ptr<RoadNode> RoadDataManager::GetClosestNode(const Vec3 &position) const
{
    shared_ptr<RoadNode> closestNode;
    float closestDistance = numeric_limits<float>::max();
    for (const shared_ptr<RoadNode> &node : m_nodes)
    {
        float distance = (node->position - position).Length();
        if (distance < closestDistance)
        {
            closestDistance = distance;
            closestNode = node;
        }
    }
    return closestNode;
}
vector<shared_ptr<RoadNode>> RoadDataManager::FindPath(const shared_ptr<RoadNode> &startNode, const shared_ptr<RoadNode> &destNode) const
{
    if (startNode == nullptr || destNode == nullptr)
        return {};

    priority_queue<pair<float, shared_ptr<RoadNode>>, vector<pair<float, shared_ptr<RoadNode>>>, greater<pair<float, shared_ptr<RoadNode>>>> openList;
    openList.emplace(0, startNode);

    map<int, float> gScore = {{startNode->id, 0}};
    map<int, shared_ptr<RoadNode>> parent = {{startNode->id, nullptr}};
    unordered_set<int> visited;

    while (!openList.empty())
    {
        auto current = openList.top().second;
        openList.pop();

        if (visited.count(current->id))
            continue;
        visited.insert(current->id);

        if (current->id == destNode->id)
        {
            vector<shared_ptr<RoadNode>> path;
            for (shared_ptr<RoadNode> node = current; node; node = parent[node->id])
            {
                path.push_back(node);
                DebugConsole::Get().Log("node: " + std::to_string(node->id));
            }
            reverse(path.begin(), path.end());
            return path;
        }
        for (const shared_ptr<RoadEdge> &edge : current->edges)
        {
            shared_ptr<RoadNode> neighborNode = edge->endNode.lock();
            if (!neighborNode)
                continue;

            float tentativeScore = gScore[current->id] + edge->length;
            float currentgScore = gScore.find(neighborNode->id) != gScore.end() ? gScore[neighborNode->id] : INFINITY;
            if (tentativeScore < currentgScore)
            {
                {
                    parent[neighborNode->id] = current;
                    gScore[neighborNode->id] = tentativeScore;

                    float fScore = tentativeScore + (neighborNode->position - destNode->position).Length();
                    openList.emplace(fScore, neighborNode);
                }
            }
        }
    }
    return {};
}

const shared_ptr<RoadNode> RoadDataManager::GetNode(int nodeId) const
{
    for (const shared_ptr<RoadNode> &node : m_nodes)
    {
        if (node->id == nodeId)
            return node;
    }
    return nullptr;
}
