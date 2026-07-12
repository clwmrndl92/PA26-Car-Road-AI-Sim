#pragma once
#include <string>
#include <unordered_map>
#include "Spline.h"
#include "Lane.h"
#include "Road.h"

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

public:
    static constexpr float ROAD_WIDTH = 3.0f;
    // 두 레인의 끝점/시작점이 이 거리 안이면 이어진 것으로 본다 (좌표가 0.5 격자라 여유 충분).
    static constexpr float CONNECT_EPSILON = 0.1f;
    // 차선변경(좌/우 인접 레인으로 이동) 간선의 비용. 잦은 차선변경을 억제한다.
    static constexpr float LANE_CHANGE_COST = 15.0f;

private:
    // 레인 끝점 <-> 다른 레인 시작점을 공간 매칭해 successors를 자동 구성한다.
    void BuildLaneAdjacency();

private:
    vector<shared_ptr<Lane>> m_lanes;
    vector<shared_ptr<Road>> m_roads;
    vector<shared_ptr<RoadNode>> m_nodes;
};

enum class RoadNodeType
{
    Unkown,
    Start,
    End,
    Stop,
    ChangeLane
};

inline const unordered_map<string, RoadNodeType> &GetRoadNodeTypeByName()
{
    static const unordered_map<string, RoadNodeType> map = {
        {"unknown", RoadNodeType::Unkown},
        {"start", RoadNodeType::Start},
        {"end", RoadNodeType::End},
        {"stop", RoadNodeType::Stop},
        {"changeLane", RoadNodeType::ChangeLane},
    };
    return map;
}

// 도로 위 '지점 이벤트' 마커(정지선/신호/양보 등). 라우팅 그래프의 정점이 아니라 레인에 붙는 부가 정보.
struct RoadNode
{
    int id;
    Vec3 position;
    RoadNodeType nodeType = RoadNodeType::End;
    float lanePosition = 1.0f;
    shared_ptr<Lane> lane;
    float limitSpeed = 999.0f;

    Vec3 GetDirection() const { return lane->GetSpline().GetDirectionAt(lanePosition); }
    float GetLimitSpeed() const { return min(limitSpeed, lane->GetRoad()->GetSpeedLimit()); }
};
