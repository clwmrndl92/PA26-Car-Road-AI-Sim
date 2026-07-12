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
//  - a per-node edit window (type / description + draggable position)
//  - control points & node positions rendered as spheres, drag-and-drop on the
//    grid (1-unit snap); a red Catmull-Rom spline once a lane has >= 4 points
//  - a fixed top-right toolbar with a Save button (-> Data/<timestamp>.json)
class EditApp : public GameApp
{
public:
    EditApp(HINSTANCE hInstance, const std::wstring &windowName, int initWidth, int initHeight);
    ~EditApp();

    void UpdateScene(float dt) override;
    void UpdateCamera(float dt) override;
    void UpdateUI(float dt) override;
    void DrawScene();

private:
    enum class Selection
    {
        None,
        Lane,
        Node
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
        char type[32] = "end";
        char description[128] = "";
    };

    // UI windows
    void DrawToolbarWindow();
    void DrawLaneListWindow();
    void DrawLaneEditWindow();
    void DrawRoadListWindow();
    void DrawNodeListWindow();
    void DrawNodeEditWindow();

    // Interaction / rendering
    void UpdateDrag();
    void RebuildRenderObjects();
    void SaveToJson();
    void LoadFromJson(const std::filesystem::path &path);

    std::vector<EditLane> m_Lanes;
    std::vector<EditRoad> m_Roads;
    std::vector<EditNode> m_Nodes;
    int m_NextLaneId = 1;
    int m_NextRoadId = 1;
    int m_NextNodeId = 1;

    Selection m_Selection = Selection::None;
    int m_SelectedLane = -1; // index into m_Lanes when m_Selection == Lane
    int m_SelectedNode = -1; // index into m_Nodes when m_Selection == Node
    int m_DraggingPoint = -1;

    std::string m_LastSavePath;

    std::vector<RenderObject> m_PointRenders;   // control-point & node spheres
    std::vector<RenderObject> m_SplineRenders;  // red spline polylines (one per lane, always shown)

    static constexpr float CP_RADIUS = 0.4f;
    static constexpr float NODE_RADIUS = 0.5f;
    static constexpr float GRID_SNAP = 1.0f;
};

#endif
