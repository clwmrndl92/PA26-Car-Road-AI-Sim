#pragma once
#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include "Spline.h"
#include "Road.h"
#include "VehicleCollision.h"

using namespace std;

struct RoadNode;

enum class LaneType
{
    Driving,
    None,
    Unknown
};

inline const unordered_map<string, LaneType> &GetLaneTypeByName()
{
    static const unordered_map<string, LaneType> map = {
        {"driving", LaneType::Driving},
        {"none", LaneType::None}};
    return map;
}

// 중앙 참조선 기준 바깥쪽 경계에 그려지는 차선 마킹
struct BoundaryMark
{
    enum class Type
    {
        None,
        Solid,
        Broken,
        DoubleSolid
    } type = Type::None;
    enum class Color
    {
        White,
        Yellow
    } color = Color::White;
    float width = 0.0f;
};

inline const unordered_map<string, BoundaryMark::Type> &GetBoundaryMarkTypeByName()
{
    static const unordered_map<string, BoundaryMark::Type> map = {
        {"none", BoundaryMark::Type::None},
        {"solid", BoundaryMark::Type::Solid},
        {"broken", BoundaryMark::Type::Broken},
        {"double_solid", BoundaryMark::Type::DoubleSolid}};
    return map;
}

inline const unordered_map<string, BoundaryMark::Color> &GetBoundaryMarkColorByName()
{
    static const unordered_map<string, BoundaryMark::Color> map = {
        {"white", BoundaryMark::Color::White},
        {"yellow", BoundaryMark::Color::Yellow}};
    return map;
}

// s의 함수로 본 도로 횡단면의 한 띠(차로 하나)
struct LaneBand
{
    float centerOffset = 0.0f;         // 참조선 기준 d (+오른쪽, -왼쪽)
    float width = 0.0f;                // 차로 폭
    Vec3 direction;                    // 진행방향(1단계에선 전부 동일 방향)
    LaneType type = LaneType::Driving; // driving / none 등
    BoundaryMark boundaryMark;         // 이 띠의 바깥쪽 경계 마킹
    float speedLimit = 0.0f;           // 차로별 제한속도
};

// OpenDRIVE laneSection 대응 — 특정 s 구간의 횡단면
struct LaneSection
{
    float sStart = 0.0f;
    vector<LaneBand> bands;
};

// 교차로 연결 하나의 진입밴드 -> 연결밴드 매핑(offset 인계).
struct LaneLink
{
    int from = 0;
    int to = 0;
};

// junction의 연결 하나: incomingRoad에서 connectingRoad로 들어가는 경로.
struct Connection
{
    int incomingRoad = -1;
    int connectingRoad = -1; // junction=<id>로 표시된 내부 연결도로
    ContactPoint contact = ContactPoint::Start;
    vector<LaneLink> laneLinks;
};

// OpenDRIVE junction — road가 succ/pred로 가리키는 다갈래 지점.
struct Junction
{
    int id = -1;
    vector<Connection> connections;
};

// 월드 위치를 road 참조선에 투영한 결과.
struct RoadPose
{
    shared_ptr<Road> road;
    float t = 0.0f;    // 참조선 파라미터 [0,1]
    float d = 0.0f;    // 참조선 기준 횡오프셋(+오른쪽)
    float dist = 0.0f; // 참조선까지 거리(투영 잔차)
};

class RoadDataManager
{
public:
    RoadDataManager();
    ~RoadDataManager();

    static RoadDataManager &Get();

    void Init(const string &filePath);
    void BuildRoadData(const string &filePath);

    // 위치를 참조선에 투영해 가장 가까운 road를 찾는다. 없으면 road==nullptr.
    RoadPose GetClosestRoad(const Vec3 &position) const;
    // road 링크(successor) 그래프 A*. 반환 = start~dest road 시퀀스(포함). 실패 시 빈 벡터.
    vector<shared_ptr<Road>> FindPath(const shared_ptr<Road> &startRoad, const shared_ptr<Road> &destRoad) const;
    // road 참조선을 d만큼 옆으로 민 주행 스플라인. d≈0이면 참조선 그대로.
    Spline BuildOffsetSpline(const shared_ptr<Road> &road, float d) const;

public:
    const vector<shared_ptr<Road>> &GetRoads() const { return m_roads; }
    shared_ptr<Road> GetRoad(int roadId) const;
    const unordered_map<int, shared_ptr<RoadNode>> &GetNodes() const { return m_nodes; };
    const shared_ptr<RoadNode> GetNode(int nodeId) const;
    shared_ptr<RoadNode> GetRandomDestNode() const;
    const vector<VehicleCollision::Obstacle> &GetObstacles() const { return m_obstacles; }

    // road의 참조선 s 위치 횡단면. 없으면 nullptr.
    const LaneSection *GetLateralProfile(const shared_ptr<Road> &road, float s) const;
    // road 중앙선(참조선 위) 마킹. 없으면 nullptr.
    const BoundaryMark *GetCenterMark(const shared_ptr<Road> &road) const;
    // id로 junction 조회. 없으면 nullptr.
    const Junction *GetJunction(int junctionId) const;
    // road에 걸린 신호 노드. 없으면 nullptr.
    shared_ptr<RoadNode> GetSignalNodeForRoad(int roadId) const;
    // road의 후속 road들(명시 successor 링크 기반). 없으면 빈 벡터.
    const vector<shared_ptr<Road>> &GetRoadSuccessors(int roadId) const;

public:
    static constexpr float ROAD_WIDTH = 3.5f; // 차선 폭

private:
    // 명시 successor 링크(Road::GetSuccessor, junction이면 connection)로 successor 그래프를 짠다.
    void BuildRoadSuccessors();

private:
    vector<shared_ptr<Road>> m_roads;
    unordered_map<int, shared_ptr<Road>> m_roadById;
    unordered_map<int, shared_ptr<RoadNode>> m_nodes; // node id -> RoadNode
    vector<VehicleCollision::Obstacle> m_obstacles;
    unordered_map<int, vector<LaneSection>> m_laneSections;             // road id -> sStart 오름차순 횡단면들
    unordered_map<int, BoundaryMark> m_centerMarks;                     // road id -> 중앙선 마킹
    unordered_map<int, Junction> m_junctions;                          // junction id -> Junction
    unordered_map<int, shared_ptr<RoadNode>> m_roadSignals;            // road id -> 신호 노드
    unordered_map<int, vector<shared_ptr<Road>>> m_roadSuccessors;     // road id -> 후속 road들
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
