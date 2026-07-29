#ifndef EDITAPP_H
#define EDITAPP_H

#include "Core/GameApp.h"
#include <vector>
#include <string>
#include <filesystem>

class EditApp : public GameApp
{
public:
    EditApp(HINSTANCE hInstance, const std::wstring &windowName, int initWidth, int initHeight);
    ~EditApp();

    void InitCamera() override;
    void UpdateScene(float dt) override;
    void UpdateCamera(float dt) override;
    void UpdateUI(float dt) override;
    void DrawScene();

private:
    enum class Selection
    {
        None,
        Lane,
        Road,
        Node,
        Marking,
        Obstacle
    };

    enum class MarkingLineType
    {
        Solid,
        Dashed
    };

    enum class MarkingColor
    {
        White,
        Yellow,
        Gray
    };

    struct EditLane
    {
        int id = -1;
        int road = -1;
        int left = -1;
        int right = -1;
        // 주차레인이면 true: road/left/right 대신 park(소속 Park 노드 id)를 쓰고, 저장 시 "lanes"가
        // 아니라 "parking_lanes" 배열로 나간다 (런타임 RoadDataManager의 메인/주차 분리와 일치).
        bool isParking = false;
        int park = -1;
        std::vector<DirectX::XMFLOAT3> points;
    };

    enum class BoundaryMarkType
    {
        None,
        Solid,
        Broken,
        DoubleSolid
    };

    // 밴드 바깥쪽 경계 / 중앙선 마킹. RoadDataManager::BoundaryMark와 필드를 맞췄다.
    struct EditBoundaryMark
    {
        BoundaryMarkType type = BoundaryMarkType::None;
        MarkingColor color = MarkingColor::White;
        float width = 0.15f;
    };

    // s의 함수로 본 도로 횡단면의 한 띠(차로 하나). RoadDataManager::LaneBand와 매칭.
    struct EditBand
    {
        float centerOffset = 1.75f; // 참조선 기준 d (+오른쪽, -왼쪽)
        float width = 3.5f;
        char type[16] = "driving"; // "driving" | "none"
        int speedLimit = 40;
        EditBoundaryMark boundaryMark; // 바깥쪽 경계(|d| 큰 쪽)에 그려지는 마킹
    };

    // OpenDRIVE laneSection 대응. RoadDataManager::LaneSection과 매칭.
    struct EditLaneSection
    {
        float sStart = 0.0f;
        std::vector<EditBand> bands;
    };

    enum class EditElementType
    {
        Road,
        Junction
    };

    enum class EditContactPoint
    {
        Start,
        End
    };

    // OpenDRIVE <link>의 predecessor/successor 하나. RoadDataManager::RoadLink와 매칭.
    struct EditRoadLink
    {
        bool valid = false;
        EditElementType type = EditElementType::Road;
        int elementId = -1;
        EditContactPoint contact = EditContactPoint::Start;
    };

    struct EditRoad
    {
        int id = -1;
        char name[64] = "road";
        int speedLimit = 40;
        std::vector<DirectX::XMFLOAT3> referenceLine; // 도로 중앙 참조선 control points
        bool hasCenterMark = false;
        EditBoundaryMark centerMark; // 참조선 위 중앙선 마킹
        std::vector<EditLaneSection> laneSections;
        int junction = -1; // -1=일반 도로, 아니면 소속 junction(내부 연결도로)
        EditRoadLink predecessor;
        EditRoadLink successor;
    };

    // 교차로 연결의 진입밴드 -> 연결밴드 매핑.
    struct EditLaneLink
    {
        int from = 0;
        int to = 0;
    };

    struct EditConnection
    {
        int incomingRoad = -1;
        int connectingRoad = -1;
        EditContactPoint contact = EditContactPoint::Start;
        std::vector<EditLaneLink> laneLinks;
    };

    struct EditJunction
    {
        int id = -1;
        std::vector<EditConnection> connections;
    };

    struct EditNode
    {
        int id = -1;
        DirectX::XMFLOAT3 position{0.0f, 0.0f, 0.0f};
        // Park/ParkSpot처럼 자기만의 목표 heading이 필요한 노드용. RoadDataManager의 RoadNode와
        // 필드를 맞춘 것 (기본값도 로더의 fallback인 +X와 동일).
        DirectX::XMFLOAT3 direction{1.0f, 0.0f, 0.0f};
        char type[32] = "unknown"; // "unknown" | "park" | "park_spot" | "traffic_light" (RoadNodeType과 매칭)
        // 예: Park 노드가 자기 소유의 ParkSpot 노드 id들을 참조 (RoadNode::children과 같은 개념).
        std::vector<int> children;
        // traffic_light 노드 전용: 이 신호가 걸린 road id들 (RoadDataManager가 m_roadSignals로 역참조).
        std::vector<int> roads;
        // traffic_light 노드 전용: TrafficSignal::GetColor의 phaseOffset. 신호마다 다르게 둬서
        // 교차로 안 서로 다른 방향끼리 엇갈리게(또는 여러 교차로를 동기화) 할 때 쓴다.
        float phaseOffset = 0.0f;
    };

    // Freehand road-marking line (lane paint, median, shoulder), independent of EditLane's
    // topology data. Purely visual, saved to its own JSON (see SaveMarkingsToJson).
    struct EditMarking
    {
        int id = -1;
        MarkingLineType type = MarkingLineType::Solid;
        float width = 0.15f;
        MarkingColor color = MarkingColor::White;
        float dashLength = 3.0f; // only used when type == Dashed
        float dashGap = 5.0f;    // only used when type == Dashed
        std::vector<DirectX::XMFLOAT3> points;
    };

    // 회전된 사각형 장애물. RoadDataManager/VehicleCollision::Obstacle과 필드를 맞췄다(length=heading
    // 방향 전체 길이, width=수직 방향 전체 폭, rotation=도, atan2(z,x) 규약, ReedsShepp와 동일).
    struct EditObstacle
    {
        int id = -1;
        DirectX::XMFLOAT3 position{0.0f, 0.0f, 0.0f};
        float length = 4.0f;
        float width = 2.0f;
        float rotation = 0.0f;
    };

    // UI windows
    void DrawToolbarWindow();
    void DrawLaneListWindow();
    void DrawLaneEditWindow();
    void DrawRoadListWindow();
    void DrawRoadEditWindow();
    void DrawJunctionListWindow();
    void DrawNodeListWindow();
    void DrawNodeEditWindow();
    void DrawMarkingListWindow();
    void DrawMarkingEditWindow();
    void DrawObstacleListWindow();
    void DrawObstacleEditWindow();

    // Interaction / rendering
    void UpdateDrag();
    // 클릭 지점이 어떤 레인의 디버그 스플라인(빨간 선)에 충분히 가까우면 그 레인을 선택한다.
    // 선택되면 true (UpdateDrag가 그 결과로 드래그를 새로 시작하지 않도록).
    bool PickLaneUnderMouse(const Ray &ray);
    void RebuildRenderObjects();
    void SaveToJson();
    void LoadFromJson(const std::filesystem::path &path);
    void SaveMarkingsToJson();
    void LoadMarkingsFromJson(const std::filesystem::path &path);

    std::vector<EditLane> m_Lanes;
    std::vector<EditRoad> m_Roads;
    std::vector<EditJunction> m_Junctions;
    std::vector<EditNode> m_Nodes;
    std::vector<EditMarking> m_Markings;
    std::vector<EditObstacle> m_Obstacles;
    int m_NextLaneId = 1;
    int m_NextRoadId = 1;
    int m_NextJunctionId = 1;
    int m_NextNodeId = 1;
    int m_NextMarkingId = 1;
    int m_NextObstacleId = 1;

    Selection m_Selection = Selection::None;
    int m_SelectedLane = -1;     // index into m_Lanes when m_Selection == Lane
    int m_SelectedRoad = -1;     // index into m_Roads when m_Selection == Road
    int m_SelectedBand = -1;     // index into selected road's section 0 bands (Road edit)
    int m_SelectedNode = -1;     // index into m_Nodes when m_Selection == Node
    int m_SelectedMarking = -1;  // index into m_Markings when m_Selection == Marking
    int m_SelectedObstacle = -1; // index into m_Obstacles when m_Selection == Obstacle
    int m_DraggingPoint = -1;

    std::string m_LastSavePath;
    std::string m_LastMarkingsSavePath;

    std::vector<RenderObject> m_PointRenders;    // control-point & node spheres
    std::vector<RenderObject> m_SplineRenders;   // red spline polylines (one per lane, always shown)
    std::vector<RenderObject> m_RoadRenders;     // reference lines (green) + band marks (ribbons), always shown
    std::vector<RenderObject> m_MarkingRenders;  // marking-line ribbons (solid/dashed), always shown
    std::vector<RenderObject> m_ObstacleRenders; // blue obstacle rectangle outlines, always shown

    static constexpr float CP_RADIUS = 0.4f;
    static constexpr float NODE_RADIUS = 0.5f;
    static constexpr float OBSTACLE_MARKER_RADIUS = 0.5f;
    static constexpr float GRID_SNAP = 1.0f;
    static constexpr float LANE_GRID_SNAP = 0.5f;
    static constexpr float MARKING_GRID_SNAP = 0.05f;
};

#endif
