#pragma once
#include <string>
#include <unordered_map>
#include <unordered_set>
#include "Spline.h"
#include "Lane.h"
#include "Road.h"
#include "HybridAStar.h"
#include "TrafficSignal.h"

using namespace std;

struct RoadNode;

// 라우팅 결과 한 스텝: 따라갈 레인 + 이 레인으로 들어올 때 차선변경이 필요한지 여부.
struct LaneStep
{
    shared_ptr<Lane> lane;
    bool isLaneChange = false;
};

class RoadDataManager
{
public:
    RoadDataManager() = default;
    ~RoadDataManager() = default;
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
    const vector<HybridAStar::Obstacle> &GetObstacles() const { return m_obstacles; }

    const vector<shared_ptr<Lane>> *GetParkingLanes(int parkNodeId) const;

    shared_ptr<RoadNode> TryReserveParkSpot(int parkNodeId, const unordered_set<int> &excludeIds = {});
    void ReleaseParkSpot(int spotNodeId);

    // 전역 시뮬레이션 시계 누적 (CarSim::UpdateScene에서 매 프레임 한 번만 호출)
    void Tick(float dt) { m_simTime += dt; }
    TrafficSignal::Color GetSignalColor(float phaseOffset) const;

public:
    static constexpr float ROAD_WIDTH = 3.2f;             // 차선 폭
    static constexpr float CONNECT_EPSILON = 0.1f;        // 두 레인의 끝점/시작점이 이 거리 안이면 이어진 것으로 본다.
    static constexpr float LANE_CHANGE_COST = 5.0f;       // 차선변경(좌/우 인접 레인으로 이동) 간선의 비용
    static constexpr float CURVE_SPEED_COEFF = 1.22f;     // 최대 코너링 속도 = CURVE_SPEED_COEFF * sqrt(R)
    static constexpr float SIGNAL_GREEN_DURATION = 8.0f;  // 초록불 지속 시간
    static constexpr float SIGNAL_YELLOW_DURATION = 3.0f; // 노란불 지속 시간
    static constexpr float SIGNAL_RED_DURATION = 12.0f;   // 빨간불 지속 시간

private:
    // 주어진 레인 집합 안에서 끝점<->시작점을 공간 매칭해 successors를 구성한다. 메인 레인과 각
    // Park의 주차레인 집합에 대해 따로 호출한다(집합 간에는 연결하지 않음 = Park 노드가 handoff 지점).
    void BuildSuccessors(const vector<shared_ptr<Lane>> &lanes);

private:
    vector<shared_ptr<Lane>> m_lanes;
    vector<shared_ptr<Road>> m_roads;
    unordered_map<int, shared_ptr<RoadNode>> m_nodes; // node id -> RoadNode
    vector<HybridAStar::Obstacle> m_obstacles;
    unordered_set<int> m_reservedParkSpotIds; // 예약된(다른 차가 목표로 잡은) ParkSpot 노드 id
    unordered_map<int, vector<shared_ptr<Lane>>> m_parkingLanes;
    float m_simTime = 0.0f; // Tick()으로만 누적되는 전역 시뮬레이션 시계. 신호 색 계산 전용.
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
