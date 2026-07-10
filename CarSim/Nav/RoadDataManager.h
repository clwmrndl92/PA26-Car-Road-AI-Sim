#pragma once
#include <string>
#include <unordered_map>
#include "Spline.h"
#include "Road.h"

using namespace std;

struct RoadNode;
struct RoadEdge;

class RoadDataManager
{
public:
    RoadDataManager() = default;
    ~RoadDataManager() = default;
    void Init(const string &filePath);
    void BuildRoadData(const string &filePath);
    shared_ptr<RoadNode> GetClosestNode(const Vec3 &position) const;

    vector<shared_ptr<RoadNode>> FindPath(const shared_ptr<RoadNode> &startNode, const shared_ptr<RoadNode> &destNode) const;

public:
    const vector<shared_ptr<RoadNode>> &GetNodes() const { return m_nodes; };
    const shared_ptr<RoadNode> GetNode(int nodeId) const;
    const vector<shared_ptr<RoadEdge>> &GetEdges() const { return m_edges; };

public:
    static constexpr float ROAD_WIDTH = 3.0f;

private:
    vector<shared_ptr<Road>> m_roads;
    vector<shared_ptr<RoadNode>> m_nodes;
    vector<shared_ptr<RoadEdge>> m_edges;
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

struct RoadEdge
{
    int id;
    weak_ptr<RoadNode> endNode;
    float length;
    Spline spline; // 비어있으면(GetControlPointCount()==0) 미리 만들어진 곡선이 없다는 뜻 -> 주행 중 동적으로 생성
};

struct RoadNode
{
    int id;
    Vec3 position;
    vector<shared_ptr<RoadEdge>> edges;
    RoadNodeType nodeType = RoadNodeType::End;
    float limitSpeed = 999.0f;

    // TODO(제한속도 재설계): Road 참조가 없어져서 지금은 node별 limitSpeed만 반영됨
    float GetLimitSpeed() const { return limitSpeed; }

    shared_ptr<RoadEdge> GetEdgeTo(int nodeId) const
    {
        for (const shared_ptr<RoadEdge> &edge : edges)
        {
            shared_ptr<RoadNode> end = edge->endNode.lock();
            if (end && end->id == nodeId)
                return edge;
        }
        return nullptr;
    }
};
