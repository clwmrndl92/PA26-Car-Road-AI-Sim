#pragma once
#include <string>
#include "Spline.h"
#include "Lane.h"
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
    const vector<shared_ptr<Lane>> &GetLanes() const { return m_lanes; };
    const vector<shared_ptr<RoadNode>> &GetNodes() const { return m_nodes; };
    const shared_ptr<RoadNode> GetNode(int nodeId) const;

private:
    vector<shared_ptr<Lane>> m_lanes;
    vector<shared_ptr<Road>> m_roads;
    vector<shared_ptr<RoadNode>> m_nodes;
    vector<shared_ptr<RoadEdge>> m_edges;
};

enum class RoadNodeType
{
    Normal,
    Stop,
    ChangeLane
};

struct RoadNode
{
    int id;
    Vec3 position;
    vector<shared_ptr<RoadEdge>> edges;
    RoadNodeType nodeType;
    shared_ptr<Lane> lane;
};

struct RoadEdge
{
    int id;
    weak_ptr<RoadNode> endNode;
    float length;
};
