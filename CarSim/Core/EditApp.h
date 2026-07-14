#ifndef EDITAPP_H
#define EDITAPP_H

#include "GameApp.h"
#include <vector>
#include <string>
#include <filesystem>

// Road data editor.
// Reuses GameApp's window / free camera and adds:
//  - lane / road / node list windows (Add -> auto id)
//  - a per-lane edit window (road / left / right ids + control points)
//  - a per-node edit window (type / direction / children + draggable position)
//  - control points & node positions rendered as spheres, drag-and-drop on the
//    grid (1-unit snap); a red Catmull-Rom spline once a lane has >= 4 points
//  - a fixed top-right toolbar with a Save button (-> CarSim/Nav/<timestamp>.json)
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
        Node,
        Marking
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
        std::vector<DirectX::XMFLOAT3> points;
    };

    struct EditRoad
    {
        int id = -1;
        char name[64] = "road";
        int speedLimit = 40;
    };

    struct EditNode
    {
        int id = -1;
        DirectX::XMFLOAT3 position{0.0f, 0.0f, 0.0f};
        // Park/ParkSpot처럼 자기만의 목표 heading이 필요한 노드용. RoadDataManager의 RoadNode와
        // 필드를 맞춘 것 (기본값도 로더의 fallback인 +X와 동일).
        DirectX::XMFLOAT3 direction{1.0f, 0.0f, 0.0f};
        char type[32] = "unknown"; // "unknown" | "park" | "park_spot" (RoadNodeType과 매칭)
        // 예: Park 노드가 자기 소유의 ParkSpot 노드 id들을 참조 (RoadNode::children과 같은 개념).
        std::vector<int> children;
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

    // UI windows
    void DrawToolbarWindow();
    void DrawLaneListWindow();
    void DrawLaneEditWindow();
    void DrawRoadListWindow();
    void DrawNodeListWindow();
    void DrawNodeEditWindow();
    void DrawMarkingListWindow();
    void DrawMarkingEditWindow();

    // Interaction / rendering
    void UpdateDrag();
    void RebuildRenderObjects();
    void SaveToJson();
    void LoadFromJson(const std::filesystem::path &path);
    void SaveMarkingsToJson();
    void LoadMarkingsFromJson(const std::filesystem::path &path);

    std::vector<EditLane> m_Lanes;
    std::vector<EditRoad> m_Roads;
    std::vector<EditNode> m_Nodes;
    std::vector<EditMarking> m_Markings;
    int m_NextLaneId = 1;
    int m_NextRoadId = 1;
    int m_NextNodeId = 1;
    int m_NextMarkingId = 1;

    Selection m_Selection = Selection::None;
    int m_SelectedLane = -1;    // index into m_Lanes when m_Selection == Lane
    int m_SelectedNode = -1;    // index into m_Nodes when m_Selection == Node
    int m_SelectedMarking = -1; // index into m_Markings when m_Selection == Marking
    int m_DraggingPoint = -1;

    std::string m_LastSavePath;
    std::string m_LastMarkingsSavePath;

    std::vector<RenderObject> m_PointRenders;   // control-point & node spheres
    std::vector<RenderObject> m_SplineRenders;  // red spline polylines (one per lane, always shown)
    std::vector<RenderObject> m_MarkingRenders; // marking-line ribbons (solid/dashed), always shown

    static constexpr float CP_RADIUS = 0.4f;
    static constexpr float NODE_RADIUS = 0.5f;
    static constexpr float GRID_SNAP = 1.0f;
    static constexpr float LANE_GRID_SNAP = 0.5f;
    static constexpr float MARKING_GRID_SNAP = 0.05f;
};

#endif
