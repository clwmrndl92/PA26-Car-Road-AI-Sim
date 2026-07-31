#include "RoadDataManager.h"
#include <fstream>
#include <nlohmann/json.hpp>
#include <queue>
#include <cmath>
#include <cstdlib>
#include <unordered_set>
#include <map>
#include <limits>
#include <algorithm>
#include "Utill/DebugConsole.h"
#include "Utill/Assert.h"

namespace
{
    RoadDataManager *s_pInstance = nullptr;

    // control_points 배열([[x,y,z], ...])을 Vec3 목록으로. 레인/주차레인 파싱 공용.
    vector<Vec3> ParseControlPoints(const nlohmann::json &pointsJson)
    {
        vector<Vec3> controlPoints;
        for (const nlohmann::json &point : pointsJson)
        {
            if (point.size() < 3)
                continue;
            controlPoints.push_back(Vec3(point[0].get<float>(), point[1].get<float>(), point[2].get<float>()));
        }
        return controlPoints;
    }

    // {type,color,width} 오브젝트를 BoundaryMark로. 키 없으면 기본값(None/White/0).
    BoundaryMark ParseBoundaryMark(const nlohmann::json &markJson)
    {
        BoundaryMark mark;
        const auto &typeByName = GetBoundaryMarkTypeByName();
        auto typeIt = typeByName.find(markJson.value("type", "none"));
        if (typeIt != typeByName.end())
            mark.type = typeIt->second;

        const auto &colorByName = GetBoundaryMarkColorByName();
        auto colorIt = colorByName.find(markJson.value("color", "white"));
        if (colorIt != colorByName.end())
            mark.color = colorIt->second;

        mark.width = markJson.value("width", 0.0f);
        return mark;
    }

    ContactPoint ParseContactPoint(const string &s)
    {
        return s == "end" ? ContactPoint::End : ContactPoint::Start;
    }

    // {type,id,contact} 오브젝트를 RoadLink로. valid=true로 표시.
    RoadLink ParseRoadLink(const nlohmann::json &linkJson)
    {
        RoadLink link;
        link.type = linkJson.value("type", string("road")) == "junction" ? ElementType::Junction : ElementType::Road;
        link.elementId = linkJson.value("id", -1);
        link.contact = ParseContactPoint(linkJson.value("contact", string("start")));
        link.valid = true;
        return link;
    }
}

RoadDataManager::RoadDataManager()
{
    if (s_pInstance)
        throw std::exception("RoadDataManager is a singleton!");
    s_pInstance = this;
}

RoadDataManager::~RoadDataManager()
{
    s_pInstance = nullptr;
}

RoadDataManager &RoadDataManager::Get()
{
    if (!s_pInstance)
        throw std::exception("RoadDataManager needs an instance!");
    return *s_pInstance;
}

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
    m_roadById.clear();
    m_nodes.clear();
    m_obstacles.clear();
    m_laneSections.clear();
    m_centerMarks.clear();
    m_junctions.clear();
    m_roadSignals.clear();

    for (const nlohmann::json &roadJson : root.value("roads", nlohmann::json::array()))
    {
        int id = roadJson.value("id", 0);
        float speedLimit = roadJson.value("speed_limit", 0.0f) / 3.6f;

        auto road = make_shared<Road>(id, speedLimit);
        m_roads.push_back(road);
        m_roadById[id] = road;

        // junction(optional): 이 도로가 소속된 교차로 id(내부 연결도로). 없으면 -1.
        road->SetJunctionId(roadJson.value("junction", -1));

        // link(optional): predecessor/successor 명시 링크(track A).
        if (roadJson.contains("link"))
        {
            const nlohmann::json &linkJson = roadJson["link"];
            if (linkJson.contains("predecessor"))
                road->SetPredecessor(ParseRoadLink(linkJson["predecessor"]));
            if (linkJson.contains("successor"))
                road->SetSuccessor(ParseRoadLink(linkJson["successor"]));
        }

        // 참조선(optional): 도로 중앙선 control_points. Catmull-Rom은 4점 이상 필요.
        vector<Vec3> refPoints = ParseControlPoints(roadJson.value("reference_line", nlohmann::json::array()));
        if (refPoints.size() >= 4)
            road->SetReferenceLine(Spline(refPoints));

        // 중앙선 마킹(optional): 참조선 위 마킹.
        if (roadJson.contains("center_mark"))
            m_centerMarks[id] = ParseBoundaryMark(roadJson["center_mark"]);

        // 횡단면(optional): laneSection별 밴드들. sStart 오름차순 정렬해 GetLateralProfile이 이진탐색 없이 뒤에서 스캔.
        vector<LaneSection> sections;
        for (const nlohmann::json &sectionJson : roadJson.value("lane_sections", nlohmann::json::array()))
        {
            LaneSection section;
            section.sStart = sectionJson.value("s_start", 0.0f);
            for (const nlohmann::json &bandJson : sectionJson.value("bands", nlohmann::json::array()))
            {
                LaneBand band;
                band.centerOffset = bandJson.value("center_offset", 0.0f);
                band.width = bandJson.value("width", 0.0f);
                band.speedLimit = bandJson.value("speed_limit", 0.0f) / 3.6f;

                const auto &laneTypeByName = GetLaneTypeByName();
                auto typeIt = laneTypeByName.find(bandJson.value("type", "driving"));
                band.type = typeIt != laneTypeByName.end() ? typeIt->second : LaneType::Driving;

                if (bandJson.contains("boundary_mark"))
                    band.boundaryMark = ParseBoundaryMark(bandJson["boundary_mark"]);

                section.bands.push_back(band);
            }
            sections.push_back(section);
        }
        if (!sections.empty())
        {
            sort(sections.begin(), sections.end(),
                 [](const LaneSection &a, const LaneSection &b)
                 { return a.sStart < b.sStart; });
            m_laneSections[id] = std::move(sections);
        }
    }

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
        node->signalPhaseOffset = nodeJson.value("phase_offset", 0.0f); // traffic_light 전용, 없으면 0

        m_nodes[id] = node;
    }

    // children(optional): 전방 참조가 있을 수 있어(예: Park보다 ParkSpot이 뒤에 나옴) 모든 노드
    // 생성 후 해석한다 (레인의 left/right와 같은 이유).
    for (const nlohmann::json &nodeJson : root.value("nodes", nlohmann::json::array()))
    {
        auto nodeIt = m_nodes.find(nodeJson.value("id", 0));
        if (nodeIt == m_nodes.end())
            continue;

        for (const nlohmann::json &childIdJson : nodeJson.value("child", nlohmann::json::array()))
        {
            auto childIt = m_nodes.find(childIdJson.get<int>());
            if (childIt != m_nodes.end())
                nodeIt->second->children.push_back(childIt->second);
        }
    }

    // roads(optional, traffic_light 전용): 신호가 걸린 road id들 -> 노드 역참조 맵.
    for (const nlohmann::json &nodeJson : root.value("nodes", nlohmann::json::array()))
    {
        auto nodeIt = m_nodes.find(nodeJson.value("id", 0));
        if (nodeIt == m_nodes.end() || nodeIt->second->nodeType != RoadNodeType::TrafficLight)
            continue;

        for (const nlohmann::json &roadIdJson : nodeJson.value("roads", nlohmann::json::array()))
            m_roadSignals[roadIdJson.get<int>()] = nodeIt->second;
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

        VehicleCollision::Obstacle obstacle;
        obstacle.center = Vec3(posJson[0].get<float>(), posJson[1].get<float>(), posJson[2].get<float>());
        obstacle.halfLength = sizeJson[0].get<float>() * 0.5f;
        obstacle.halfWidth = sizeJson[1].get<float>() * 0.5f;
        obstacle.headingRad = ToRadians(obstacleJson.value("rotation", 0.0f));
        m_obstacles.push_back(obstacle);
    }

    // dynamic_obstacles(테스트용): start~end 구간을 등속으로 왕복하는 장애물.
    // "start"/"end":[x,y,z], "size":[length,width], "speed"(m/s, 기본 1).
    m_dynamicObstacleDefs.clear();
    for (const nlohmann::json &dynJson : root.value("dynamic_obstacles", nlohmann::json::array()))
    {
        const nlohmann::json &startJson = dynJson.value("start", nlohmann::json::array());
        const nlohmann::json &endJson = dynJson.value("end", nlohmann::json::array());
        const nlohmann::json &sizeJson = dynJson.value("size", nlohmann::json::array());
        if (startJson.size() < 3 || endJson.size() < 3 || sizeJson.size() < 2)
            continue;

        DynamicObstacleState state;
        state.start = Vec3(startJson[0].get<float>(), startJson[1].get<float>(), startJson[2].get<float>());
        state.end = Vec3(endJson[0].get<float>(), endJson[1].get<float>(), endJson[2].get<float>());
        state.halfLength = sizeJson[0].get<float>() * 0.5f;
        state.halfWidth = sizeJson[1].get<float>() * 0.5f;
        state.speed = dynJson.value("speed", 1.0f);
        m_dynamicObstacleDefs.push_back(state);
    }
    UpdateDynamicObstacles(0.0f); // dt=0으로 한 번 돌려 start 위치의 초기 스냅샷을 채워둔다.

    // junctions(optional, track A): 다갈래 교차로. 스키마만 읽어 저장(라우팅 확장은 이후 단계).
    for (const nlohmann::json &junctionJson : root.value("junctions", nlohmann::json::array()))
    {
        Junction junction;
        junction.id = junctionJson.value("id", -1);
        for (const nlohmann::json &connJson : junctionJson.value("connections", nlohmann::json::array()))
        {
            Connection conn;
            conn.incomingRoad = connJson.value("incoming_road", -1);
            conn.connectingRoad = connJson.value("connecting_road", -1);
            conn.contact = ParseContactPoint(connJson.value("contact", string("start")));
            for (const nlohmann::json &linkJson : connJson.value("lane_links", nlohmann::json::array()))
                conn.laneLinks.push_back({linkJson.value("from", 0), linkJson.value("to", 0)});
            junction.connections.push_back(conn);
        }
        m_junctions[junction.id] = std::move(junction);
    }

    BuildRoadSuccessors();
}

void RoadDataManager::UpdateDynamicObstacles(float dt)
{
    m_dynamicObstacles.clear();
    m_dynamicObstacles.reserve(m_dynamicObstacleDefs.size());
    for (DynamicObstacleState &state : m_dynamicObstacleDefs)
    {
        Vec3 delta = state.end - state.start;
        float segLength = delta.Length();

        VehicleCollision::Obstacle obstacle;
        obstacle.halfLength = state.halfLength;
        obstacle.halfWidth = state.halfWidth;
        obstacle.type = VehicleCollision::ObstacleType::Dynamic;

        if (segLength < 0.001f) // start==end: 왕복 구간이 없으니 그 자리에 정지.
        {
            obstacle.center = state.start;
            m_dynamicObstacles.push_back(obstacle);
            continue;
        }

        state.traveled += state.speed * dt;
        float period = segLength * 2.0f; // start->end->start 한 바퀴 길이
        float phase = std::fmod(state.traveled, period);
        if (phase < 0.0f)
            phase += period;

        Vec3 dir = delta * (1.0f / segLength);
        bool forward = phase <= segLength;
        float alongSeg = forward ? phase : period - phase; // 왕복 후반부(phase>segLength)는 end->start로 되짚음

        obstacle.center = state.start + dir * alongSeg;
        Vec3 travelDir = forward ? dir : dir * -1.0f; // atan2(z,x) 규약(ReedsShepp/Car와 동일)
        obstacle.headingRad = atan2f(travelDir.GetZ(), travelDir.GetX());
        obstacle.speed = state.speed;
        m_dynamicObstacles.push_back(obstacle);
    }
}

void RoadDataManager::BuildRoadSuccessors()
{
    // 명시 링크(track A)로 successor 그래프를 구성한다. successor가 road면 그 road,
    // junction이면 그 junction의 connection 중 incomingRoad==현재인 connectingRoad들로 팬아웃.
    m_roadSuccessors.clear();
    for (const shared_ptr<Road> &road : m_roads)
    {
        const RoadLink &succ = road->GetSuccessor();
        if (!succ.valid)
            continue;

        if (succ.type == ElementType::Road)
        {
            if (shared_ptr<Road> next = GetRoad(succ.elementId))
                m_roadSuccessors[road->GetId()].push_back(next);
        }
        else if (const Junction *junction = GetJunction(succ.elementId))
        {
            for (const Connection &conn : junction->connections)
                if (conn.incomingRoad == road->GetId())
                    if (shared_ptr<Road> connecting = GetRoad(conn.connectingRoad))
                        m_roadSuccessors[road->GetId()].push_back(connecting);
        }
    }
}

const vector<shared_ptr<Road>> &RoadDataManager::GetRoadSuccessors(int roadId) const
{
    static const vector<shared_ptr<Road>> empty;
    auto it = m_roadSuccessors.find(roadId);
    return it != m_roadSuccessors.end() ? it->second : empty;
}

shared_ptr<Road> RoadDataManager::GetRoad(int roadId) const
{
    auto it = m_roadById.find(roadId);
    return it != m_roadById.end() ? it->second : nullptr;
}

shared_ptr<RoadNode> RoadDataManager::GetSignalNodeForRoad(int roadId) const
{
    auto it = m_roadSignals.find(roadId);
    return it != m_roadSignals.end() ? it->second : nullptr;
}

RoadPose RoadDataManager::GetClosestRoad(const Vec3 &position) const
{
    RoadPose best;
    best.dist = numeric_limits<float>::max();
    for (const shared_ptr<Road> &road : m_roads)
    {
        const Spline &ref = road->GetReferenceLine();
        if (ref.GetSplinePoints().size() < 2)
            continue;
        float t = ref.GetSplinePosition(position);
        Vec3 onRef = ref.GetPositionAt(t);
        float dist = (onRef - position).Length();
        if (dist < best.dist)
        {
            best.dist = dist;
            best.road = road;
            best.t = t;
            // d 부호: 진행방향 오른쪽(+). right=(dir.z,0,-dir.x).
            Vec3 dir = ref.GetDirectionAt(t);
            Vec3 rightN(dir.GetZ(), 0.0f, -dir.GetX());
            best.d = (position - onRef).Dot(rightN);
        }
    }
    if (best.dist == numeric_limits<float>::max())
        best = RoadPose{};
    return best;
}

Spline RoadDataManager::BuildOffsetSpline(const shared_ptr<Road> &road, float d) const
{
    if (road == nullptr)
        return Spline();

    const Spline &ref = road->GetReferenceLine();
    if (std::abs(d) < 1e-3f)
        return ref; // 오프셋 0이면 참조선 그대로(data2가 이 경우 = 정확)

    const vector<Vec3> &samples = ref.GetSplinePoints();
    if (samples.size() < 2)
        return ref;

    // 참조선의 이미 계산된 샘플점을 진행방향 오른쪽 법선으로 d만큼 밀기만 한다(재적합 없음).
    // Catmull-Rom을 다시 돌리지 않고 FromPoints로 그대로 감싸 O(n) — EditApp OffsetReferencePolyline과 동일 규약.
    vector<Vec3> offset;
    offset.reserve(samples.size());
    const size_t n = samples.size();
    for (size_t i = 0; i < n; ++i)
    {
        const Vec3 &next = samples[i + 1 < n ? i + 1 : i];
        const Vec3 &prev = samples[i > 0 ? i - 1 : i];
        float tx = next.GetX() - prev.GetX();
        float tz = next.GetZ() - prev.GetZ();
        float len = std::sqrt(tx * tx + tz * tz);
        float rx = len > 1e-5f ? tz / len : 0.0f;
        float rz = len > 1e-5f ? -tx / len : 0.0f;
        offset.push_back(Vec3(samples[i].GetX() + rx * d, samples[i].GetY(), samples[i].GetZ() + rz * d));
    }
    return Spline::FromPoints(std::move(offset));
}

const LaneSection *RoadDataManager::GetLateralProfile(const shared_ptr<Road> &road, float s) const
{
    if (road == nullptr)
        return nullptr;

    auto it = m_laneSections.find(road->GetId());
    if (it == m_laneSections.end() || it->second.empty())
        return nullptr;

    // sStart 오름차순이므로 s 이하인 마지막 구간이 담당 구간. s가 첫 구간보다 앞이면 첫 구간으로 클램프.
    const vector<LaneSection> &sections = it->second;
    const LaneSection *result = &sections.front();
    for (const LaneSection &section : sections)
    {
        if (section.sStart > s)
            break;
        result = &section;
    }
    return result;
}

const BoundaryMark *RoadDataManager::GetCenterMark(const shared_ptr<Road> &road) const
{
    if (road == nullptr)
        return nullptr;

    auto it = m_centerMarks.find(road->GetId());
    return it != m_centerMarks.end() ? &it->second : nullptr;
}

const Junction *RoadDataManager::GetJunction(int junctionId) const
{
    auto it = m_junctions.find(junctionId);
    return it != m_junctions.end() ? &it->second : nullptr;
}

vector<shared_ptr<Road>> RoadDataManager::FindPath(const shared_ptr<Road> &startRoad, const shared_ptr<Road> &destRoad) const
{
    if (startRoad == nullptr || destRoad == nullptr)
        return {};
    if (startRoad->GetId() == destRoad->GetId())
        return {startRoad};

    priority_queue<pair<float, shared_ptr<Road>>, vector<pair<float, shared_ptr<Road>>>, greater<pair<float, shared_ptr<Road>>>> openList;
    openList.emplace(0.0f, startRoad);

    map<int, float> gScore = {{startRoad->GetId(), 0.0f}};
    map<int, shared_ptr<Road>> cameFrom = {{startRoad->GetId(), nullptr}};
    unordered_set<int> visited;

    const vector<Vec3> &destPts = destRoad->GetReferenceLine().GetSplinePoints();
    const Vec3 goal = destPts.empty() ? Vec3::sZero() : destPts.back();

    while (!openList.empty())
    {
        shared_ptr<Road> current = openList.top().second;
        openList.pop();

        if (visited.count(current->GetId()))
            continue;
        visited.insert(current->GetId());

        if (current->GetId() == destRoad->GetId())
        {
            vector<shared_ptr<Road>> path;
            for (shared_ptr<Road> road = current; road;)
            {
                path.push_back(road);
                road = cameFrom[road->GetId()];
            }
            reverse(path.begin(), path.end());
            return path;
        }

        // 진행: 현재 road를 끝까지 달려(참조선 길이만큼 비용) 후속 road로.
        for (const shared_ptr<Road> &neighbor : GetRoadSuccessors(current->GetId()))
        {
            if (!neighbor)
                continue;
            float tentative = gScore[current->GetId()] + current->GetLength();
            float known = gScore.count(neighbor->GetId()) ? gScore[neighbor->GetId()] : INFINITY;
            if (tentative < known)
            {
                gScore[neighbor->GetId()] = tentative;
                cameFrom[neighbor->GetId()] = current;
                const vector<Vec3> &nbPts = neighbor->GetReferenceLine().GetSplinePoints();
                float h = nbPts.empty() ? 0.0f : (nbPts.back() - goal).Length();
                openList.emplace(tentative + h, neighbor);
            }
        }
    }
    return {};
}

const shared_ptr<RoadNode> RoadDataManager::GetNode(int nodeId) const
{
    auto it = m_nodes.find(nodeId);
    return it != m_nodes.end() ? it->second : nullptr;
}

shared_ptr<RoadNode> RoadDataManager::GetRandomDestNode() const
{
    vector<shared_ptr<RoadNode>> candidates;
    for (const auto &[id, node] : m_nodes)
    {
        if (node->nodeType != RoadNodeType::ParkSpot && node->nodeType != RoadNodeType::TrafficLight)
            candidates.push_back(node);
    }
    if (candidates.empty())
        return nullptr;

    return candidates[rand() % candidates.size()];
}
