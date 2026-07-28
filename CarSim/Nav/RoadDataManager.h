#pragma once
#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include "Spline.h"
#include "Lane.h"
#include "Road.h"
#include "VehicleCollision.h"

using namespace std;

struct RoadNode;

struct LaneStep
{
    shared_ptr<Lane> lane;
    bool isLaneChange = false;
};

class RoadDataManager
{
public:
    RoadDataManager();
    ~RoadDataManager();

    static RoadDataManager &Get();

    void Init(const string &filePath);
    void BuildRoadData(const string &filePath);

    shared_ptr<Lane> GetClosestLane(const Vec3 &position) const;
    shared_ptr<Lane> GetClosestLaneEnd(const Vec3 &position) const; // 끝점이 목표 지점에 가장 가까운 레인
    shared_ptr<Lane> GetClosestParkLane(const Vec3 &position, int parkId) const;
    vector<LaneStep> FindPath(const shared_ptr<Lane> &startLane, const shared_ptr<Lane> &destLane) const;

public:
    const vector<shared_ptr<Lane>> &GetLanes() const { return m_lanes; };
    const unordered_map<int, shared_ptr<RoadNode>> &GetNodes() const { return m_nodes; };
    const shared_ptr<RoadNode> GetNode(int nodeId) const;
    shared_ptr<RoadNode> GetRandomDestNode() const;
    const vector<VehicleCollision::Obstacle> &GetObstacles() const { return m_obstacles; }

    const vector<shared_ptr<Lane>> *GetParkingLanes(int parkNodeId) const;

public:
    static constexpr float ROAD_WIDTH = 3.2f;       // 차선 폭
    static constexpr float CONNECT_EPSILON = 0.1f;  // 두 레인의 끝점/시작점이 이 거리 안이면 이어진 것으로 본다.
    static constexpr float LANE_CHANGE_COST = 5.0f; // 차선변경(좌/우 인접 레인으로 이동) 간선의 비용

private:
    // Park의 주차레인 집합에 대해 따로 호출(집합 간에는 연결하지 않음 = Park 노드가 handoff 지점).
    void BuildSuccessors(const vector<shared_ptr<Lane>> &lanes);

private:
    vector<shared_ptr<Lane>> m_lanes;
    vector<shared_ptr<Road>> m_roads;
    unordered_map<int, shared_ptr<RoadNode>> m_nodes; // node id -> RoadNode
    vector<VehicleCollision::Obstacle> m_obstacles;
    unordered_map<int, vector<shared_ptr<Lane>>> m_parkingLanes;
};

enum class RoadNodeType
{
    Park,
    ParkSpot,
    TrafficLight,
    Unkown
};

inline const unordered_map<string, RoadNodeType> &GetRoadNodeTypeByName()
{
    static const unordered_map<string, RoadNodeType> map = {
        {"unknown", RoadNodeType::Unkown},
        {"park", RoadNodeType::Park},
        {"park_spot", RoadNodeType::ParkSpot},
        {"traffic_light", RoadNodeType::TrafficLight}};
    return map;
}

// 도로 위 '지점 이벤트' 마커(정지선/신호/양보 등)
struct RoadNode
{
    int id;
    Vec3 position;
    Vec3 direction;
    RoadNodeType nodeType = RoadNodeType::Unkown;
    vector<weak_ptr<RoadNode>> children; // 예: Park 노드가 자기 소유의 ParkSpot 노드들을 참조
    float signalPhaseOffset = 0.0f;      // traffic_light 노드 전용
};
