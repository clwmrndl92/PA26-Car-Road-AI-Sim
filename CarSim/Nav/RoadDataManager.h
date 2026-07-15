#pragma once
#include <string>
#include <unordered_map>
#include <unordered_set>
#include "Spline.h"
#include "Lane.h"
#include "Road.h"
#include "HybridAStar.h"

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

    // 현재 위치가 올라가 있는(가장 가까운) 레인
    shared_ptr<Lane> GetClosestLane(const Vec3 &position) const;
    // 끝점이 목표 지점에 가장 가까운 레인 (목적지 = 그 레인 끝에 도착)
    shared_ptr<Lane> GetClosestLaneEnd(const Vec3 &position) const;
    vector<LaneStep> FindPath(const shared_ptr<Lane> &startLane, const shared_ptr<Lane> &destLane) const;

public:
    const vector<shared_ptr<Lane>> &GetLanes() const { return m_lanes; };
    const vector<shared_ptr<RoadNode>> &GetNodes() const { return m_nodes; };
    const shared_ptr<RoadNode> GetNode(int nodeId) const;
    // data.json의 "obstacles"(임시 데이터: 실제 장애물 인식 파이프라인이 들어오기 전까지 손으로 채운
    // 사각형 목록)를 그대로 반환한다. 실시간 회피의 코리도어 검사 등에서 쓴다.
    const vector<HybridAStar::Obstacle> &GetObstacles() const { return m_obstacles; }

    // parkNodeId(Park 타입 노드)의 children 중 아직 예약되지 않은 ParkSpot을 하나 찾아 예약하고
    // 반환한다. 빈 자리가 없으면 nullptr. 예약 상태는 정적 도로 데이터(RoadNode)와 분리해서
    // 여기서만 관리한다.
    shared_ptr<RoadNode> TryReserveParkSpot(int parkNodeId);
    // spotNodeId의 예약을 해제한다 (출차 완료 시 호출).
    void ReleaseParkSpot(int spotNodeId);

public:
    static constexpr float ROAD_WIDTH = 3.2f;         // 차선 폭
    static constexpr float CONNECT_EPSILON = 0.1f;    // 두 레인의 끝점/시작점이 이 거리 안이면 이어진 것으로 본다.
    static constexpr float LANE_CHANGE_COST = 5.0f;   // 차선변경(좌/우 인접 레인으로 이동) 간선의 비용
    static constexpr float CURVE_SPEED_COEFF = 1.22f; // 최대 코너링 속도 = CURVE_SPEED_COEFF * sqrt(R)

private:
    // 레인 끝점 <-> 다른 레인 시작점을 공간 매칭해 successors를 자동 구성한다.
    void BuildLaneAdjacency();

private:
    vector<shared_ptr<Lane>> m_lanes;
    vector<shared_ptr<Road>> m_roads;
    vector<shared_ptr<RoadNode>> m_nodes;
    vector<HybridAStar::Obstacle> m_obstacles;
    unordered_set<int> m_reservedParkSpotIds; // 예약된(다른 차가 목표로 잡은) ParkSpot 노드 id
};

enum class RoadNodeType
{
    Unkown,
    Park,
    ParkSpot
};

inline const unordered_map<string, RoadNodeType> &GetRoadNodeTypeByName()
{
    static const unordered_map<string, RoadNodeType> map = {
        {"unknown", RoadNodeType::Unkown},
        {"park", RoadNodeType::Park},
        {"park_spot", RoadNodeType::ParkSpot}};
    return map;
}

// 도로 위 '지점 이벤트' 마커(정지선/신호/양보 등). 라우팅 그래프의 정점이 아니라 레인에 붙는 부가 정보.
struct RoadNode
{
    int id;
    Vec3 position;
    Vec3 direction;
    RoadNodeType nodeType = RoadNodeType::Unkown;
    // 예: Park 노드가 자기 소유의 ParkSpot 노드들을 참조 (소유는 RoadDataManager가 하므로
    // Lane의 left/right처럼 weak_ptr로만 참조해 순환참조 누수를 막는다).
    vector<weak_ptr<RoadNode>> children;
};
