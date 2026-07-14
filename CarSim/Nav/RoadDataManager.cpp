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
#include <Core/Assert.h>

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
    m_obstacles.clear();

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

        // Catmull-Rom 스플라인은 컨트롤 포인트가 4개 미만이면 빈 스플라인이 되어 이 레인이 통째로 무효화된다.
        Assert(controlPoints.size() >= 4);
        auto lane = make_shared<Lane>(id, Spline(controlPoints), road);
        m_lanes.push_back(lane);
        laneById[id] = lane;
    }

    // left/right 인접 레인(optional): 전방 참조가 있을 수 있어 모든 레인 생성 후 해석한다.
    for (const nlohmann::json &laneJson : root.value("lanes", nlohmann::json::array()))
    {
        auto laneIt = laneById.find(laneJson.value("id", 0));
        if (laneIt == laneById.end())
            continue;

        if (laneJson.contains("left"))
        {
            auto leftIt = laneById.find(laneJson.value("left", 0));
            if (leftIt != laneById.end())
                laneIt->second->SetLeft(leftIt->second);
        }
        if (laneJson.contains("right"))
        {
            auto rightIt = laneById.find(laneJson.value("right", 0));
            if (rightIt != laneById.end())
                laneIt->second->SetRight(rightIt->second);
        }
    }

    map<int, shared_ptr<RoadNode>> nodeById;
    for (const nlohmann::json &nodeJson : root.value("nodes", nlohmann::json::array()))
    {
        int id = nodeJson.value("id", 0);

        const nlohmann::json &posJson = nodeJson.value("position", nlohmann::json::array());
        if (posJson.size() < 3)
            continue;
        Vec3 position(posJson[0].get<float>(), posJson[1].get<float>(), posJson[2].get<float>());

        // direction(optional): ParkSpot처럼 자기만의 목표 heading이 필요한 노드용. 없으면 +X.
        Vec3 direction(1.0f, 0.0f, 0.0f);
        const nlohmann::json &dirJson = nodeJson.value("direction", nlohmann::json::array());
        if (dirJson.size() >= 3)
            direction = Vec3(dirJson[0].get<float>(), dirJson[1].get<float>(), dirJson[2].get<float>());

        string typeStr = nodeJson.value("type", "unknown");
        const auto &nodeTypeByName = GetRoadNodeTypeByName();
        auto nodeTypeIt = nodeTypeByName.find(typeStr);
        RoadNodeType nodeType = nodeTypeIt != nodeTypeByName.end() ? nodeTypeIt->second : RoadNodeType::Unkown;

        auto node = make_shared<RoadNode>();
        node->id = id;
        node->position = position;
        node->direction = direction;
        node->nodeType = nodeType;

        m_nodes.push_back(node);
        nodeById[id] = node;
    }

    // children(optional): 전방 참조가 있을 수 있어(예: Park보다 ParkSpot이 뒤에 나옴) 모든 노드
    // 생성 후 해석한다 (레인의 left/right와 같은 이유).
    for (const nlohmann::json &nodeJson : root.value("nodes", nlohmann::json::array()))
    {
        auto nodeIt = nodeById.find(nodeJson.value("id", 0));
        if (nodeIt == nodeById.end())
            continue;

        for (const nlohmann::json &childIdJson : nodeJson.value("child", nlohmann::json::array()))
        {
            auto childIt = nodeById.find(childIdJson.get<int>());
            if (childIt != nodeById.end())
                nodeIt->second->children.push_back(childIt->second);
        }
    }

    // obstacles(임시): 실제 인식 파이프라인이 들어오기 전까지, 손으로 채운 사각형 장애물 목록.
    // "position":[x,y,z], "size":[length,width](전체 길이/폭, heading 방향 기준), "rotation"(도, ReedsShepp와
    // 같은 atan2(z,x) 규약).
    for (const nlohmann::json &obstacleJson : root.value("obstacles", nlohmann::json::array()))
    {
        const nlohmann::json &posJson = obstacleJson.value("position", nlohmann::json::array());
        const nlohmann::json &sizeJson = obstacleJson.value("size", nlohmann::json::array());
        if (posJson.size() < 3 || sizeJson.size() < 2)
            continue;

        HybridAStar::Obstacle obstacle;
        obstacle.center = Vec3(posJson[0].get<float>(), posJson[1].get<float>(), posJson[2].get<float>());
        obstacle.halfLength = sizeJson[0].get<float>() * 0.5f;
        obstacle.halfWidth = sizeJson[1].get<float>() * 0.5f;
        obstacle.headingDeg = obstacleJson.value("rotation", 0.0f);
        m_obstacles.push_back(obstacle);
    }

    BuildLaneAdjacency();
}

void RoadDataManager::BuildLaneAdjacency()
{
    // 매 빌드마다 m_lanes는 새로 생성되므로 successors는 비어 있는 상태에서 시작한다.
    for (const shared_ptr<Lane> &from : m_lanes)
    {
        const Vec3 &fromEnd = from->GetEndPoint();
        Vec3 fromDir = from->GetSpline().GetDirectionAt(1.0f);

        for (const shared_ptr<Lane> &to : m_lanes)
        {
            if (from == to)
                continue;

            // from의 끝점이 to의 시작점과 일치할 때만 진행 방향으로 이어붙인다.
            if ((to->GetStartPoint() - fromEnd).Length() > CONNECT_EPSILON)
                continue;

            constexpr float MAX_CONNECT_ANGLE_COS = -0.5f; // cos(120°)
            Vec3 toDir = to->GetSpline().GetDirectionAt(0.0f);
            if (fromDir.Dot(toDir) <= MAX_CONNECT_ANGLE_COS)
                continue;

            from->AddSuccessor(to);
        }
    }
}

shared_ptr<Lane> RoadDataManager::GetClosestLane(const Vec3 &position) const
{
    shared_ptr<Lane> closestLane;
    float closestDistance = numeric_limits<float>::max();
    for (const shared_ptr<Lane> &lane : m_lanes)
    {
        for (const Vec3 &point : lane->GetSpline().GetSplinePoints())
        {
            float distance = (point - position).Length();
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestLane = lane;
            }
        }
    }
    return closestLane;
}

shared_ptr<Lane> RoadDataManager::GetClosestLaneEnd(const Vec3 &position) const
{
    shared_ptr<Lane> closestLane;
    float closestDistance = numeric_limits<float>::max();
    for (const shared_ptr<Lane> &lane : m_lanes)
    {
        float distance = (lane->GetEndPoint() - position).Length();
        if (distance < closestDistance)
        {
            closestDistance = distance;
            closestLane = lane;
        }
    }
    return closestLane;
}

vector<LaneStep> RoadDataManager::FindPath(const shared_ptr<Lane> &startLane, const shared_ptr<Lane> &destLane) const
{
    if (startLane == nullptr || destLane == nullptr)
        return {};

    // 어디서 왔는지 + 그 진입이 차선변경이었는지 기록해 경로를 되짚는다.
    struct Came
    {
        shared_ptr<Lane> from;
        bool viaLaneChange = false;
    };

    priority_queue<pair<float, shared_ptr<Lane>>, vector<pair<float, shared_ptr<Lane>>>, greater<pair<float, shared_ptr<Lane>>>> openList;
    openList.emplace(0.0f, startLane);

    map<int, float> gScore = {{startLane->GetId(), 0.0f}};
    map<int, Came> cameFrom = {{startLane->GetId(), {nullptr, false}}};
    unordered_set<int> visited;

    const Vec3 goal = destLane->GetEndPoint();

    while (!openList.empty())
    {
        shared_ptr<Lane> current = openList.top().second;
        openList.pop();
        DebugConsole::Get().Log("Find path " + ToString(current->GetId()));

        if (visited.count(current->GetId()))
            continue;
        visited.insert(current->GetId());

        if (current->GetId() == destLane->GetId())
        {
            vector<LaneStep> path;
            for (shared_ptr<Lane> lane = current; lane;)
            {
                const Came &came = cameFrom[lane->GetId()];
                path.push_back({lane, came.viaLaneChange});
                lane = came.from;
            }
            reverse(path.begin(), path.end());
            return path;
        }

        auto relax = [&](const shared_ptr<Lane> &neighbor, float cost, bool laneChange)
        {
            if (!neighbor)
                return;
            float tentative = gScore[current->GetId()] + cost;
            float known = gScore.count(neighbor->GetId()) ? gScore[neighbor->GetId()] : INFINITY;
            if (tentative < known)
            {
                gScore[neighbor->GetId()] = tentative;
                cameFrom[neighbor->GetId()] = {current, laneChange};
                float fScore = tentative + (neighbor->GetEndPoint() - goal).Length();
                openList.emplace(fScore, neighbor);
            }
        };

        // 진행: 현재 레인을 끝까지 달려(길이만큼 비용) 다음 레인으로.
        for (const weak_ptr<Lane> &weak : current->GetSuccessors())
            relax(weak.lock(), current->GetLength(), false);
        // 차선변경: 좌/우 인접 레인으로 페널티 비용으로 이동.
        relax(current->GetLeft().lock(), LANE_CHANGE_COST, true);
        relax(current->GetRight().lock(), LANE_CHANGE_COST, true);
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

shared_ptr<RoadNode> RoadDataManager::TryReserveParkSpot(int parkNodeId)
{
    shared_ptr<RoadNode> parkNode = GetNode(parkNodeId);
    if (!parkNode)
        return nullptr;

    for (const weak_ptr<RoadNode> &weakChild : parkNode->children)
    {
        shared_ptr<RoadNode> spot = weakChild.lock();
        if (!spot || spot->nodeType != RoadNodeType::ParkSpot)
            continue;
        if (m_reservedParkSpotIds.count(spot->id))
            continue;

        m_reservedParkSpotIds.insert(spot->id);
        return spot;
    }
    return nullptr;
}

void RoadDataManager::ReleaseParkSpot(int spotNodeId)
{
    m_reservedParkSpotIds.erase(spotNodeId);
}
