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

// 차로의 주행 방향. 참조선 s가 늘어나는 쪽이 Forward, 거스르는 쪽이 Backward(왕복 도로의 마주 오는 차로).
enum class LaneDirection
{
    Forward,
    Backward
};

inline const unordered_map<string, LaneDirection> &GetLaneDirectionByName()
{
    static const unordered_map<string, LaneDirection> map = {
        {"forward", LaneDirection::Forward},
        {"backward", LaneDirection::Backward}};
    return map;
}

// 참조선 s를 '진행방향으로 증가하는' 값으로 바꿀 때 곱하는 부호.
inline float GetTravelSign(LaneDirection direction) { return direction == LaneDirection::Backward ? -1.0f : 1.0f; }

inline LaneDirection GetOppositeDirection(LaneDirection direction)
{
    return direction == LaneDirection::Forward ? LaneDirection::Backward : LaneDirection::Forward;
}

// s의 함수로 본 도로 횡단면의 한 띠(차로 하나)
struct LaneBand
{
    float centerOffset = 0.0f;                        // 참조선 기준 d (+오른쪽, -왼쪽). 방향과 무관하게 항상 참조선 프레임.
    float width = 0.0f;                               // 차로 폭
    LaneDirection direction = LaneDirection::Forward; // 이 차로의 주행 방향
    LaneType type = LaneType::Driving;                // driving / none 등
    BoundaryMark boundaryMark;                        // 이 띠의 바깥쪽 경계 마킹
    float speedLimit = 0.0f;                          // 차로별 제한속도
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

// 경로 한 칸: 어느 road를 어느 방향으로 달리는가. 왕복 도로에선 road id만으로 진행방향이 안 정해진다.
struct RoadRef
{
    shared_ptr<Road> road;
    LaneDirection direction = LaneDirection::Forward;
};

// 월드 위치를 road 참조선에 투영한 결과.
struct RoadPose
{
    shared_ptr<Road> road;
    float t = 0.0f;                                   // 참조선 파라미터 [0,1]
    float d = 0.0f;                                   // 참조선 기준 횡오프셋(+오른쪽)
    float dist = 0.0f;                                // 참조선까지 거리(투영 잔차)
    LaneDirection direction = LaneDirection::Forward; // heading으로 고른 주행 방향
};

class RoadDataManager
{
public:
    RoadDataManager();
    ~RoadDataManager();

    static RoadDataManager &Get();

    void Init(const string &filePath);
    void BuildRoadData(const string &filePath);

    // 테스트용 왕복(patrol) 동적 장애물: start~end 구간을 등속으로 오간다. 매프레임 위치를 갱신.
    void UpdateDynamicObstacles(float dt);

    // 위치를 참조선에 투영해 가장 가까운 road를 찾는다. 없으면 road==nullptr. 방향은 밴드가 있는 쪽.
    RoadPose GetClosestRoad(const Vec3 &position) const;
    // 위와 같되 heading과 같은 쪽 차로 방향을 고른다. 그 방향 driving 밴드가 없는 단방향 도로면 있는 쪽으로 맞춘다.
    // 주차장 통로(Road::IsParkingRoad)는 일반 주행용 탐색이라 대상에서 제외 -- roaming/routing이 주차장으로 새지 않게.
    RoadPose GetClosestRoad(const Vec3 &position, const Vec3 &heading) const;
    // GetClosestRoad와 같되 주차장 통로만 대상으로 찾는다. 주차 시퀀스(FindBestParkingSpline/출차)가 쓴다.
    RoadPose GetClosestParkingRoad(const Vec3 &position, const Vec3 &heading) const;
    // road 링크(successor) 그래프 A*. 노드는 (road, 진행방향) -- 왕복 도로는 방향마다 다음 road가 다르다.
    // 반환 = start~dest road 시퀀스(포함). 실패 시 빈 벡터.
    vector<RoadRef> FindPath(const RoadRef &start, const shared_ptr<Road> &destRoad) const;
    // road 참조선을 d만큼 옆으로 민 주행 스플라인. Backward면 같은 곡선의 점 순서를 뒤집어 역주행선으로 만든다.
    // d는 방향과 무관하게 항상 참조선 프레임이므로 부호를 뒤집지 않는다.
    Spline BuildOffsetSpline(const shared_ptr<Road> &road, float d,
                             LaneDirection direction = LaneDirection::Forward) const;
    // from->to 전환 시 오프셋 인계. to가 from을 incomingRoad로 둔 junction connecting road고 laneLinks가 있으면
    // fromOffset이 속한 진입 밴드에 매핑된 연결 밴드의 centerOffset을 돌려준다(다른 오프셋 연결 허용).
    // 매핑이 없으면 fromOffset 그대로 -- 두 road의 참조선이 이 지점에서 기하학적으로 안 맞을 수 있으니
    // outMapped=false로 알려서 호출측이 실제 위치 투영 등 다른 수단으로 보정하게 한다.
    float ResolveConnectingOffset(const RoadRef &from, const RoadRef &to, float fromOffset, bool *outMapped = nullptr) const;
    // from에서 to로 넘어갈 수 있는 from.road 쪽 진입 밴드들(진행방향 driving만). 좌/우회전 전용 차로 판정용.
    // 빈 벡터 = '갈 수 없음'이 아니라 '차로 제약이 없는 전이'(junction 밖 직결 등) -- 호출측이 구분해야 한다.
    vector<const LaneBand *> GetEntryBands(const RoadRef &from, const RoadRef &to) const;

public:
    const vector<shared_ptr<Road>> &GetRoads() const { return m_roads; }
    shared_ptr<Road> GetRoad(int roadId) const;
    const unordered_map<int, shared_ptr<RoadNode>> &GetNodes() const { return m_nodes; };
    const shared_ptr<RoadNode> GetNode(int nodeId) const;
    shared_ptr<RoadNode> GetRandomDestNode() const;
    shared_ptr<RoadNode> GetRandomParkNode() const;
    const vector<VehicleCollision::Obstacle> &GetObstacles() const { return m_obstacles; }
    // UpdateDynamicObstacles가 매프레임 갱신하는 왕복 장애물의 현재 위치/헤딩/속도.
    const vector<VehicleCollision::Obstacle> &GetDynamicObstacles() const { return m_dynamicObstacles; }

    // road의 참조선 s 위치 횡단면. 없으면 nullptr.
    const LaneSection *GetLateralProfile(const shared_ptr<Road> &road, float s) const;
    // road 중앙선(참조선 위) 마킹. 없으면 nullptr.
    const BoundaryMark *GetCenterMark(const shared_ptr<Road> &road) const;
    // id로 junction 조회. 없으면 nullptr.
    const Junction *GetJunction(int junctionId) const;
    // road에 걸린 신호 노드. nextRoadId(경로상 다음 road = 실제로 타려는 이동)가 어느 phase의
    // gatedMovements에 속하면 그 노드를, 없으면 gatedMovements가 비어있는(전체 적용) 노드를 돌려준다.
    // nextRoadId<0(다음 road를 모름)이면 전체 적용 노드만 후보로 본다. 둘 다 없으면 nullptr.
    shared_ptr<RoadNode> GetSignalNodeForRoad(int roadId, int nextRoadId = -1) const;
    // 그 방향으로 달릴 때의 후속 road들. Forward는 successor 링크, Backward는 predecessor 링크로 나간다.
    const vector<RoadRef> &GetRoadSuccessors(int roadId, LaneDirection direction) const;

    // 진행방향이 맞는 driving 밴드 중 d에 가장 가까운 것. 방향/타입이 맞는 밴드가 없으면 조건을 풀어 다시 찾는다
    // -- 방향을 안 적은 기존 데이터도 그대로 굴러가야 한다. 밴드 자체가 없으면 nullptr.
    const LaneBand *FindNearestBand(const shared_ptr<Road> &road, float d, LaneDirection direction) const;
    // 진행방향이 맞는 driving 밴드들을 centerOffset 오름차순으로. 없으면 빈 벡터(폴백 판단은 호출측).
    vector<const LaneBand *> GetDrivingBands(const shared_ptr<Road> &road, LaneDirection direction) const;
    // 그 방향으로 달렸을 때 road를 빠져나가는 끝점(Forward면 참조선 끝, Backward면 시작).
    Vec3 GetTravelEnd(const shared_ptr<Road> &road, LaneDirection direction) const;

public:
    static constexpr float ROAD_WIDTH = 4.0f; // 차선 폭

private:
    // 명시 링크(Forward=successor / Backward=predecessor, junction이면 connection)로 방향별 successor 그래프를 짠다.
    void BuildRoadSuccessors();
    // m_roadSuccessors 키: 같은 road도 방향마다 다음이 다르므로 방향 비트를 섞는다.
    static int SuccessorKey(int roadId, LaneDirection direction)
    {
        return roadId * 2 + (direction == LaneDirection::Backward ? 1 : 0);
    }

    // start~end 구간을 왕복하는 테스트용 동적 장애물의 정의 + 진행 상태.
    struct DynamicObstacleState
    {
        Vec3 start;
        Vec3 end;
        float halfLength = 0.0f;
        float halfWidth = 0.0f;
        float speed = 0.0f;    // 구간을 오가는 등속 스칼라 속도(m/s)
        float traveled = 0.0f; // start 기준 누적 이동거리. [0, 2*구간길이) 범위로 랩되며 왕복을 표현.
    };

private:
    vector<shared_ptr<Road>> m_roads;
    unordered_map<int, shared_ptr<Road>> m_roadById;
    unordered_map<int, shared_ptr<RoadNode>> m_nodes; // node id -> RoadNode
    vector<VehicleCollision::Obstacle> m_obstacles;
    vector<DynamicObstacleState> m_dynamicObstacleDefs;             // dynamic_obstacles 로드 결과 + 런타임 진행상태
    vector<VehicleCollision::Obstacle> m_dynamicObstacles;          // UpdateDynamicObstacles가 매프레임 다시 채우는 현재 위치 스냅샷
    unordered_map<int, vector<LaneSection>> m_laneSections;         // road id -> sStart 오름차순 횡단면들
    unordered_map<int, BoundaryMark> m_centerMarks;                 // road id -> 중앙선 마킹
    unordered_map<int, Junction> m_junctions;                       // junction id -> Junction
    unordered_map<int, vector<shared_ptr<RoadNode>>> m_roadSignals; // road id -> 그 road에 걸린 신호 노드들(이동별 phase 여러 개 가능)
    unordered_map<int, vector<RoadRef>> m_roadSuccessors;           // SuccessorKey(road id, 방향) -> 후속 (road, 방향)들
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
    // traffic_light 노드 전용: 이 phase가 실제로 막는 "다음 road"(교차로 connecting road, 예: 직진/좌회전) id들.
    // 비어있으면 접근도로(roads)에서 나가는 모든 이동에 적용(하위호환 기본값). 목록에 없는 이동(예: 우회전)은
    // 이 신호로 안 막힌다 -- 걸고 싶으면 별도 traffic_light 노드를 그 이동 id로 추가하면 된다.
    vector<int> gatedMovements;
    // traffic_light 노드 전용: 신호 주기(초). 없으면 기본값(8/3/12)을 그대로 쓴다.
    float signalGreenDuration = 8.0f;
    float signalYellowDuration = 3.0f;
    float signalRedDuration = 12.0f;
};
