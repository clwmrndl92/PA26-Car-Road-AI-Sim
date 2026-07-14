#include "EditApp.h"
#include <XUtil.h>
#include <DXTrace.h>
#include <Collision.h>
#include <Geometry.h>
#include "Nav/Spline.h"
#include "Utill/MathUtil.h"
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstdio>
#include <cfloat>
#include <ctime>
#include <fstream>
#include <filesystem>
#include <algorithm>

using namespace DirectX;

EditApp::EditApp(HINSTANCE hInstance, const std::wstring &windowName, int initWidth, int initHeight)
    : GameApp(hInstance, windowName, initWidth, initHeight)
{
}

EditApp::~EditApp()
{
}

void EditApp::InitCamera()
{
    GameApp::InitCamera();

    // Road/marking data is authored flat on y = 0, so start with a bird's-eye view straight
    // down onto the XZ plane instead of GameApp's default forward-facing view.
    auto cam = std::dynamic_pointer_cast<FreeCamera>(m_pCamera);
    if (!cam)
        return;

    XMFLOAT3 eye(0.0f, 60.0f, 0.0f);
    cam->LookAt(eye, XMFLOAT3(0.0f, 0.0f, 0.0f), XMFLOAT3(0.0f, 0.0f, 1.0f));

    m_BasicEffect.SetViewMatrix(cam->GetViewMatrixXM());
    m_BasicEffect.SetEyePos(cam->GetPosition());
}

void EditApp::UpdateScene(float dt)
{
    // Drag first (uses the camera/mouse from the previous frame) so the edit
    // window rebuilt below already shows the updated positions.
    UpdateDrag();

    // GameApp handles camera (via our UpdateCamera), builds the ImGui windows
    // via UpdateUI(), and calls ImGui::Render().
    GameApp::UpdateScene(dt);

    // Rebuild the spheres + spline from the current data for this frame's draw.
    RebuildRenderObjects();
}

void EditApp::UpdateCamera(float dt)
{
    auto cam = std::dynamic_pointer_cast<FreeCamera>(m_pCamera);
    if (!cam)
        return;

    ImGuiIO &io = ImGui::GetIO();
    constexpr float MOVE_SPEED = 10.0f;

    // WASD moves parallel to the XZ plane regardless of view pitch:
    //  - Walk() uses cross(right, up), i.e. the look direction flattened onto XZ.
    //  - Strafe() uses the right axis, which stays horizontal (no roll).
    if (ImGui::IsKeyDown(ImGuiKey_W))
        cam->Walk(dt * MOVE_SPEED);
    if (ImGui::IsKeyDown(ImGuiKey_S))
        cam->Walk(-dt * MOVE_SPEED);
    if (ImGui::IsKeyDown(ImGuiKey_A))
        cam->Strafe(-dt * MOVE_SPEED);
    if (ImGui::IsKeyDown(ImGuiKey_D))
        cam->Strafe(dt * MOVE_SPEED);
    if (ImGui::IsKeyDown(ImGuiKey_Q))
        cam->Translate(XMFLOAT3(0, -1, 0), dt * MOVE_SPEED);
    if (ImGui::IsKeyDown(ImGuiKey_E))
        cam->Translate(XMFLOAT3(0, 1, 0), dt * MOVE_SPEED);

    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right))
    {
        cam->Pitch(io.MouseDelta.y * 0.01f);
        cam->RotateY(io.MouseDelta.x * 0.01f);
    }

    // Scroll wheel = zoom along the view direction (distance control).
    if (!io.WantCaptureMouse && io.MouseWheel != 0.0f)
        cam->MoveForward(io.MouseWheel * 2.0f);
}

void EditApp::UpdateDrag()
{
    if (!m_pCamera)
    {
        m_DraggingPoint = -1;
        return;
    }

    // Collect the currently draggable points (and their pick radius) based on
    // the active selection: a lane's control points, or a single node.
    std::vector<XMFLOAT3 *> pts;
    float radius = CP_RADIUS;
    float snap = GRID_SNAP;

    if (m_Selection == Selection::Lane && m_SelectedLane >= 0 && m_SelectedLane < (int)m_Lanes.size())
    {
        for (auto &p : m_Lanes[m_SelectedLane].points)
            pts.push_back(&p);
        radius = CP_RADIUS;
        snap = LANE_GRID_SNAP;
    }
    else if (m_Selection == Selection::Node && m_SelectedNode >= 0 && m_SelectedNode < (int)m_Nodes.size())
    {
        pts.push_back(&m_Nodes[m_SelectedNode].position);
        radius = NODE_RADIUS;
    }
    else if (m_Selection == Selection::Marking && m_SelectedMarking >= 0 && m_SelectedMarking < (int)m_Markings.size())
    {
        for (auto &p : m_Markings[m_SelectedMarking].points)
            pts.push_back(&p);
        radius = CP_RADIUS;
        snap = MARKING_GRID_SNAP; // marking lines need finer placement (line width is 0.15)
    }
    else
    {
        m_DraggingPoint = -1;
        return;
    }

    ImGuiIO &io = ImGui::GetIO();

    // Release the drag when the left button comes up.
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        m_DraggingPoint = -1;

    // Begin a drag: pick the nearest point under the cursor. Guard with
    // WantCaptureMouse so clicks on ImGui windows don't start a drag.
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse)
    {
        Ray ray = Ray::ScreenToRay(*m_pCamera, io.MousePos.x, io.MousePos.y);

        float bestDist = FLT_MAX;
        int best = -1;
        for (int i = 0; i < (int)pts.size(); ++i)
        {
            BoundingSphere sphere(*pts[i], radius);
            float dist = 0.0f;
            if (ray.Hit(sphere, &dist) && dist < bestDist)
            {
                bestDist = dist;
                best = i;
            }
        }
        m_DraggingPoint = best;
    }

    // Continue the drag: intersect the ray with the point's horizontal plane,
    // then snap x/z to the grid. y is kept (points live on y = 0).
    if (m_DraggingPoint >= 0 && m_DraggingPoint < (int)pts.size() &&
        ImGui::IsMouseDown(ImGuiMouseButton_Left))
    {
        Ray ray = Ray::ScreenToRay(*m_pCamera, io.MousePos.x, io.MousePos.y);
        XMFLOAT3 &p = *pts[m_DraggingPoint];

        if (fabsf(ray.direction.y) > 1e-5f)
        {
            float t = (p.y - ray.origin.y) / ray.direction.y;
            if (t > 0.0f)
            {
                float x = ray.origin.x + t * ray.direction.x;
                float z = ray.origin.z + t * ray.direction.z;
                p.x = roundf(x / snap) * snap;
                p.z = roundf(z / snap) * snap;
            }
        }
    }
}

void EditApp::RebuildRenderObjects()
{
    m_PointRenders.clear();
    m_SplineRenders.clear();
    m_MarkingRenders.clear();

    // Every lane's spline (red) is always shown, even while editing something else.
    for (const auto &lane : m_Lanes)
    {
        if (lane.points.size() < 4)
            continue;

        std::vector<Vec3> cps;
        cps.reserve(lane.points.size());
        for (const auto &p : lane.points)
            cps.push_back(ToVec3(p));

        Spline spline(cps);
        const std::vector<Vec3> &samples = spline.GetSplinePoints();

        std::vector<XMFLOAT3> line;
        line.reserve(samples.size());
        for (const Vec3 &s : samples)
        {
            XMFLOAT3 f = ToXMFLOAT3(s);
            f.y += 0.1f; // lift slightly above the ground so it isn't z-fighting
            line.push_back(f);
        }
        if (line.size() < 2)
            continue;

        Model *pLine = m_ModelManager.CreateFromGeometry("edit_spline_" + std::to_string(lane.id),
                                                         Geometry::CreatePolyline(line));
        pLine->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f));
        pLine->materials[0].Set<float>("$Opacity", 1.0f);

        RenderObject &ro = m_SplineRenders.emplace_back();
        ro.SetModel(pLine);
    }

    // Every marking line's ribbon (solid or dashed) is always shown. Freehand, independent of
    // lane data: >=4 points get a Catmull-Rom spline centerline (same as lanes), 2-3 points are
    // used as-is (straight segments).
    for (const auto &marking : m_Markings)
    {
        if (marking.points.size() < 2)
            continue;

        std::vector<XMFLOAT3> samples;
        if (marking.points.size() >= 4)
        {
            std::vector<Vec3> cps;
            cps.reserve(marking.points.size());
            for (const auto &p : marking.points)
                cps.push_back(ToVec3(p));

            Spline spline(cps);
            for (const Vec3 &s : spline.GetSplinePoints())
                samples.push_back(ToXMFLOAT3(s));
        }
        else
        {
            samples = marking.points;
        }
        if (samples.size() < 2)
            continue;

        for (XMFLOAT3 &s : samples)
            s.y += 0.05f; // lift slightly above the ground (below the lane spline's +0.1f lift)

        GeometryData geo = marking.type == MarkingLineType::Dashed
                               ? Geometry::CreateDashedRibbon(samples, marking.width, marking.dashLength, marking.dashGap)
                               : Geometry::CreateRibbon(samples, marking.width);
        if (geo.vertices.empty())
            continue;

        Model *pMarking = m_ModelManager.CreateFromGeometry("edit_marking_" + std::to_string(marking.id), geo);
        XMFLOAT4 color;
        switch (marking.color)
        {
        case MarkingColor::Yellow:
            color = XMFLOAT4(1.0f, 0.85f, 0.0f, 1.0f);
            break;
        case MarkingColor::Gray:
            color = XMFLOAT4(0.22f, 0.22f, 0.22f, 1.0f);
            break;
        default:
            color = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
            break;
        }
        pMarking->materials[0].Set<XMFLOAT4>("$DiffuseColor", color);
        pMarking->materials[0].Set<float>("$Opacity", 1.0f);

        RenderObject &ro = m_MarkingRenders.emplace_back();
        ro.SetModel(pMarking);
    }

    // Node spheres (always visible for context): green, selected one yellow.
    Model *pNode = m_ModelManager.CreateFromGeometry("edit_node", Geometry::CreateSphere(NODE_RADIUS));
    pNode->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f));
    pNode->materials[0].Set<float>("$Opacity", 1.0f);

    Model *pNodeSel = m_ModelManager.CreateFromGeometry("edit_node_sel", Geometry::CreateSphere(NODE_RADIUS));
    pNodeSel->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 1.0f, 0.0f, 1.0f));
    pNodeSel->materials[0].Set<float>("$Opacity", 1.0f);

    for (int i = 0; i < (int)m_Nodes.size(); ++i)
    {
        bool selected = (m_Selection == Selection::Node && i == m_SelectedNode);
        RenderObject &ro = m_PointRenders.emplace_back();
        ro.SetModel(selected ? pNodeSel : pNode);
        ro.GetTransform().SetPosition(m_Nodes[i].position);
    }

    // Selected lane or marking: control-point spheres (orange), on top of its always-shown
    // spline/ribbon.
    const std::vector<XMFLOAT3> *selectedPoints = nullptr;
    if (m_Selection == Selection::Lane && m_SelectedLane >= 0 && m_SelectedLane < (int)m_Lanes.size())
        selectedPoints = &m_Lanes[m_SelectedLane].points;
    else if (m_Selection == Selection::Marking && m_SelectedMarking >= 0 && m_SelectedMarking < (int)m_Markings.size())
        selectedPoints = &m_Markings[m_SelectedMarking].points;

    if (selectedPoints)
    {
        Model *pSphere = m_ModelManager.CreateFromGeometry("edit_cp", Geometry::CreateSphere(CP_RADIUS));
        pSphere->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 0.6f, 0.0f, 1.0f));
        pSphere->materials[0].Set<float>("$Opacity", 1.0f);

        for (const auto &p : *selectedPoints)
        {
            RenderObject &ro = m_PointRenders.emplace_back();
            ro.SetModel(pSphere);
            ro.GetTransform().SetPosition(p);
        }
    }
}

namespace
{
    // Serializes a JSON object whose top-level values are all arrays so each array element is
    // one compact line, while the surrounding structure (keys, brackets) stays pretty-printed.
    // nlohmann's dump(indent) only offers "fully pretty" or "fully compact", not a mix, so this
    // builds the text by hand.
    std::string DumpPrettyArraysCompactElements(const nlohmann::json &root)
    {
        std::string out = "{\n";
        size_t keyIndex = 0;
        for (auto it = root.begin(); it != root.end(); ++it, ++keyIndex)
        {
            out += "  \"" + it.key() + "\": ";

            const nlohmann::json &arr = it.value();
            if (arr.empty())
            {
                out += "[]";
            }
            else
            {
                out += "[\n";
                for (size_t i = 0; i < arr.size(); ++i)
                {
                    out += "    " + arr[i].dump();
                    if (i + 1 < arr.size())
                        out += ",";
                    out += "\n";
                }
                out += "  ]";
            }

            if (keyIndex + 1 < root.size())
                out += ",";
            out += "\n";
        }
        out += "}\n";
        return out;
    }
}

void EditApp::SaveToJson()
{
    using nlohmann::json;
    json root;

    root["roads"] = json::array();
    for (const auto &r : m_Roads)
    {
        root["roads"].push_back({{"id", r.id},
                                 {"name", r.name},
                                 {"speed_limit", r.speedLimit}});
    }

    root["lanes"] = json::array();
    for (const auto &l : m_Lanes)
    {
        json jl;
        jl["id"] = l.id;
        jl["road"] = l.road;
        if (l.left >= 0)
            jl["left"] = l.left;
        if (l.right >= 0)
            jl["right"] = l.right;

        json cps = json::array();
        for (const auto &p : l.points)
            cps.push_back({p.x, p.y, p.z});
        jl["control_points"] = cps;

        root["lanes"].push_back(jl);
    }

    root["nodes"] = json::array();
    for (const auto &n : m_Nodes)
    {
        json jn;
        jn["id"] = n.id;
        jn["position"] = {n.position.x, n.position.y, n.position.z};
        jn["direction"] = {n.direction.x, n.direction.y, n.direction.z};
        jn["type"] = n.type;
        if (!n.children.empty())
            jn["child"] = n.children;
        root["nodes"].push_back(jn);
    }

    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);

    std::string path = std::string(NAV_DATA_DIR "/") + stamp + ".json";
    std::ofstream ofs(path);
    if (ofs)
    {
        ofs << DumpPrettyArraysCompactElements(root);
        m_LastSavePath = path;
    }
    else
    {
        m_LastSavePath = "FAILED: " + path;
    }
}

void EditApp::LoadFromJson(const std::filesystem::path &path)
{
    std::ifstream ifs(path);
    if (!ifs)
    {
        m_LastSavePath = "LOAD FAILED (open)";
        return;
    }

    nlohmann::json root = nlohmann::json::parse(ifs, nullptr, false);
    if (root.is_discarded())
    {
        m_LastSavePath = "LOAD FAILED (parse)";
        return;
    }

    m_Roads.clear();
    m_Lanes.clear();
    m_Nodes.clear();

    auto copyStr = [](char *dst, size_t n, const std::string &s)
    {
        snprintf(dst, n, "%s", s.c_str());
    };

    for (const auto &jr : root.value("roads", nlohmann::json::array()))
    {
        EditRoad r;
        r.id = jr.value("id", -1);
        copyStr(r.name, sizeof(r.name), jr.value("name", std::string("road")));
        r.speedLimit = jr.value("speed_limit", 40);
        m_Roads.push_back(r);
    }

    for (const auto &jl : root.value("lanes", nlohmann::json::array()))
    {
        EditLane l;
        l.id = jl.value("id", -1);
        l.road = jl.value("road", -1);
        l.left = jl.value("left", -1);
        l.right = jl.value("right", -1);
        for (const auto &pt : jl.value("control_points", nlohmann::json::array()))
        {
            if (pt.is_array() && pt.size() >= 3)
                l.points.push_back(XMFLOAT3(pt[0].get<float>(), pt[1].get<float>(), pt[2].get<float>()));
        }
        m_Lanes.push_back(std::move(l));
    }

    for (const auto &jn : root.value("nodes", nlohmann::json::array()))
    {
        EditNode n;
        n.id = jn.value("id", -1);
        const auto &pos = jn.value("position", nlohmann::json::array());
        if (pos.is_array() && pos.size() >= 3)
            n.position = XMFLOAT3(pos[0].get<float>(), pos[1].get<float>(), pos[2].get<float>());
        const auto &dir = jn.value("direction", nlohmann::json::array());
        if (dir.is_array() && dir.size() >= 3)
            n.direction = XMFLOAT3(dir[0].get<float>(), dir[1].get<float>(), dir[2].get<float>());
        copyStr(n.type, sizeof(n.type), jn.value("type", std::string("unknown")));
        for (const auto &childIdJson : jn.value("child", nlohmann::json::array()))
            n.children.push_back(childIdJson.get<int>());
        m_Nodes.push_back(n);
    }

    // Reset id counters so newly-added items don't collide with loaded ones.
    m_NextLaneId = 1;
    for (const auto &l : m_Lanes)
        m_NextLaneId = std::max(m_NextLaneId, l.id + 1);
    m_NextRoadId = 1;
    for (const auto &r : m_Roads)
        m_NextRoadId = std::max(m_NextRoadId, r.id + 1);
    m_NextNodeId = 1;
    for (const auto &n : m_Nodes)
        m_NextNodeId = std::max(m_NextNodeId, n.id + 1);

    m_Selection = Selection::None;
    m_SelectedLane = -1;
    m_SelectedNode = -1;
    m_DraggingPoint = -1;

    m_LastSavePath = "Loaded: " + path.filename().string();
}

void EditApp::SaveMarkingsToJson()
{
    using nlohmann::json;
    json root;

    root["markings"] = json::array();
    for (const auto &m : m_Markings)
    {
        json jm;
        jm["id"] = m.id;
        jm["type"] = (m.type == MarkingLineType::Dashed) ? "dashed" : "solid";
        jm["width"] = m.width;
        jm["color"] = m.color == MarkingColor::Yellow ? "yellow" : m.color == MarkingColor::Gray ? "gray"
                                                                                                 : "white";
        jm["dash_length"] = m.dashLength;
        jm["dash_gap"] = m.dashGap;

        // Non-gray (white/yellow) lane paint needs to sit above gray asphalt-colored lines to
        // render on top of them, so bump its saved y instead of leaving it at the drawn y.
        json pts = json::array();
        for (const auto &p : m.points)
        {
            float y = (m.color == MarkingColor::Gray) ? p.y : 0.01f;
            pts.push_back({p.x, y, p.z});
        }
        jm["points"] = pts;

        root["markings"].push_back(jm);
    }

    std::filesystem::create_directories(NAV_DATA_DIR "/");

    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);

    std::string path = std::string(NAV_DATA_DIR "/") + stamp + "marking.json";
    std::ofstream ofs(path);
    if (ofs)
    {
        ofs << DumpPrettyArraysCompactElements(root);
        m_LastMarkingsSavePath = path;
    }
    else
    {
        m_LastMarkingsSavePath = "FAILED: " + path;
    }
}

void EditApp::LoadMarkingsFromJson(const std::filesystem::path &path)
{
    std::ifstream ifs(path);
    if (!ifs)
    {
        m_LastMarkingsSavePath = "LOAD FAILED (open)";
        return;
    }

    nlohmann::json root = nlohmann::json::parse(ifs, nullptr, false);
    if (root.is_discarded())
    {
        m_LastMarkingsSavePath = "LOAD FAILED (parse)";
        return;
    }

    m_Markings.clear();

    for (const auto &jm : root.value("markings", nlohmann::json::array()))
    {
        EditMarking m;
        m.id = jm.value("id", -1);
        m.type = (jm.value("type", std::string("solid")) == "dashed") ? MarkingLineType::Dashed : MarkingLineType::Solid;
        m.width = jm.value("width", 0.15f);
        std::string colorStr = jm.value("color", std::string("white"));
        m.color = (colorStr == "yellow") ? MarkingColor::Yellow : (colorStr == "gray") ? MarkingColor::Gray
                                                                                       : MarkingColor::White;
        m.dashLength = jm.value("dash_length", 3.0f);
        m.dashGap = jm.value("dash_gap", 5.0f);
        for (const auto &pt : jm.value("points", nlohmann::json::array()))
        {
            if (pt.is_array() && pt.size() >= 3)
                m.points.push_back(XMFLOAT3(pt[0].get<float>(), pt[1].get<float>(), pt[2].get<float>()));
        }
        m_Markings.push_back(std::move(m));
    }

    m_NextMarkingId = 1;
    for (const auto &m : m_Markings)
        m_NextMarkingId = std::max(m_NextMarkingId, m.id + 1);

    if (m_Selection == Selection::Marking)
    {
        m_Selection = Selection::None;
        m_SelectedMarking = -1;
        m_DraggingPoint = -1;
    }

    m_LastMarkingsSavePath = "Loaded: " + path.filename().string();
}

void EditApp::UpdateUI(float dt)
{
    // Keep GameApp's Debug Grid window (top-left).
    GameApp::UpdateUI(dt);

    DrawToolbarWindow();
    DrawLaneListWindow();
    DrawRoadListWindow();
    DrawNodeListWindow();
    DrawMarkingListWindow();

    if (m_Selection == Selection::Lane)
        DrawLaneEditWindow();
    else if (m_Selection == Selection::Node)
        DrawNodeEditWindow();
    else if (m_Selection == Selection::Marking)
        DrawMarkingEditWindow();
}

void EditApp::DrawToolbarWindow()
{
    // Pin to the top-right corner of the main viewport.
    ImGuiViewport *vp = ImGui::GetMainViewport();
    ImVec2 pos(vp->WorkPos.x + vp->WorkSize.x - 10.0f, vp->WorkPos.y + 10.0f);
    ImGui::SetNextWindowPos(pos, ImGuiCond_Always, ImVec2(1.0f, 0.0f));

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse;
    if (ImGui::Begin("Editor", nullptr, flags))
    {
        if (ImGui::Button("Save"))
            SaveToJson();
        ImGui::SameLine();
        if (ImGui::Button("Load"))
            ImGui::OpenPopup("LoadPopup");

        if (ImGui::BeginPopup("LoadPopup"))
        {
            namespace fs = std::filesystem;
            std::error_code ec;

            std::vector<fs::path> files;
            if (fs::exists(NAV_DATA_DIR, ec))
            {
                for (const auto &entry : fs::directory_iterator(NAV_DATA_DIR, ec))
                {
                    if (entry.is_regular_file() && entry.path().extension() == ".json")
                        files.push_back(entry.path());
                }
            }
            // Timestamp filenames sort chronologically, so descending = newest first.
            std::sort(files.begin(), files.end(), [](const fs::path &a, const fs::path &b)
                      { return a.filename().string() > b.filename().string(); });

            if (files.empty())
            {
                ImGui::TextDisabled("No .json in " NAV_DATA_DIR "/");
            }
            else
            {
                for (const auto &f : files)
                {
                    if (ImGui::Selectable(f.filename().string().c_str()))
                    {
                        LoadFromJson(f);
                        ImGui::CloseCurrentPopup();
                    }
                }
            }
            ImGui::EndPopup();
        }

        if (!m_LastSavePath.empty())
            ImGui::Text("%s", m_LastSavePath.c_str());

        ImGui::Separator();

        if (ImGui::Button("Save Lines"))
            SaveMarkingsToJson();
        ImGui::SameLine();
        if (ImGui::Button("Load Lines"))
            ImGui::OpenPopup("LoadMarkingsPopup");

        if (ImGui::BeginPopup("LoadMarkingsPopup"))
        {
            namespace fs = std::filesystem;
            std::error_code ec;

            std::vector<fs::path> files;
            if (fs::exists(NAV_DATA_DIR "/", ec))
            {
                for (const auto &entry : fs::directory_iterator(NAV_DATA_DIR "/", ec))
                {
                    if (entry.is_regular_file() && entry.path().extension() == ".json")
                        files.push_back(entry.path());
                }
            }
            std::sort(files.begin(), files.end(), [](const fs::path &a, const fs::path &b)
                      { return a.filename().string() > b.filename().string(); });

            if (files.empty())
            {
                ImGui::TextDisabled("No .json in " NAV_DATA_DIR "/");
            }
            else
            {
                for (const auto &f : files)
                {
                    if (ImGui::Selectable(f.filename().string().c_str()))
                    {
                        LoadMarkingsFromJson(f);
                        ImGui::CloseCurrentPopup();
                    }
                }
            }
            ImGui::EndPopup();
        }

        if (!m_LastMarkingsSavePath.empty())
            ImGui::Text("%s", m_LastMarkingsSavePath.c_str());
    }
    ImGui::End();
}

void EditApp::DrawLaneListWindow()
{
    if (ImGui::Begin("Lanes"))
    {
        if (ImGui::Button("Add Lane"))
        {
            EditLane lane;
            lane.id = m_NextLaneId++;
            m_Lanes.push_back(lane);
            m_Selection = Selection::Lane;
            m_SelectedLane = (int)m_Lanes.size() - 1;
        }

        ImGui::Separator();

        for (int i = 0; i < (int)m_Lanes.size(); ++i)
        {
            char label[64];
            snprintf(label, sizeof(label), "Lane %d (road %d)", m_Lanes[i].id, m_Lanes[i].road);
            bool selected = (m_Selection == Selection::Lane && i == m_SelectedLane);

            ImGui::PushID(i);
            float avail = ImGui::GetContentRegionAvail().x;
            if (ImGui::Selectable(label, selected, 0, ImVec2(avail - 28.0f, 0.0f)))
            {
                m_Selection = Selection::Lane;
                m_SelectedLane = i;
            }
            ImGui::SameLine();
            bool erased = ImGui::SmallButton("X");
            ImGui::PopID();

            if (erased)
            {
                m_Lanes.erase(m_Lanes.begin() + i);
                if (m_Selection == Selection::Lane)
                {
                    if (m_SelectedLane == i)
                    {
                        m_Selection = Selection::None;
                        m_SelectedLane = -1;
                    }
                    else if (m_SelectedLane > i)
                    {
                        --m_SelectedLane;
                    }
                }
                break;
            }
        }
    }
    ImGui::End();
}

void EditApp::DrawLaneEditWindow()
{
    if (m_SelectedLane < 0 || m_SelectedLane >= (int)m_Lanes.size())
    {
        m_Selection = Selection::None;
        return;
    }

    EditLane &lane = m_Lanes[m_SelectedLane];
    bool open = true;
    if (ImGui::Begin("Lane Edit", &open))
    {
        ImGui::Text("Lane ID: %d", lane.id);
        ImGui::InputInt("Road ID", &lane.road);
        ImGui::InputInt("Left", &lane.left);
        ImGui::InputInt("Right", &lane.right);

        ImGui::Separator();

        if (ImGui::Button("Add Control Point"))
        {
            XMFLOAT3 p(0.0f, 0.0f, 0.0f);
            if (!lane.points.empty())
            {
                p = lane.points.back();
                p.x += 2.0f; // offset so it doesn't overlap the previous one
            }
            lane.points.push_back(p);
        }

        ImGui::Text("Control Points: %d", (int)lane.points.size());
        for (int i = 0; i < (int)lane.points.size(); ++i)
        {
            XMFLOAT3 &p = lane.points[i];
            ImGui::PushID(i);

            ImGui::Text("[%d]", i);
            ImGui::SameLine();
            float pos[3] = {p.x, p.y, p.z};
            ImGui::SetNextItemWidth(180.0f);
            if (ImGui::InputFloat3("##pos", pos))
                p = XMFLOAT3(pos[0], pos[1], pos[2]);
            ImGui::SameLine();
            bool erased = ImGui::SmallButton("X");

            ImGui::PopID();
            if (erased)
            {
                lane.points.erase(lane.points.begin() + i);
                break;
            }
        }
    }
    ImGui::End();

    if (!open)
        m_Selection = Selection::None;
}

void EditApp::DrawRoadListWindow()
{
    if (ImGui::Begin("Roads"))
    {
        if (ImGui::Button("Add Road"))
        {
            EditRoad road;
            road.id = m_NextRoadId++;
            m_Roads.push_back(road);
        }

        ImGui::Separator();

        for (int i = 0; i < (int)m_Roads.size(); ++i)
        {
            ImGui::PushID(i);
            ImGui::Text("Road %d", m_Roads[i].id);
            ImGui::SameLine();
            bool erased = ImGui::SmallButton("Delete");
            if (!erased)
            {
                ImGui::InputText("name", m_Roads[i].name, sizeof(m_Roads[i].name));
                ImGui::InputInt("speed", &m_Roads[i].speedLimit);
            }
            ImGui::Separator();
            ImGui::PopID();

            if (erased)
            {
                m_Roads.erase(m_Roads.begin() + i);
                break;
            }
        }
    }
    ImGui::End();
}

void EditApp::DrawNodeListWindow()
{
    if (ImGui::Begin("Nodes"))
    {
        if (ImGui::Button("Add Node"))
        {
            EditNode node;
            node.id = m_NextNodeId++;
            m_Nodes.push_back(node);
            m_Selection = Selection::Node;
            m_SelectedNode = (int)m_Nodes.size() - 1;
        }

        ImGui::Separator();

        for (int i = 0; i < (int)m_Nodes.size(); ++i)
        {
            char label[64];
            snprintf(label, sizeof(label), "Node %d", m_Nodes[i].id);
            bool selected = (m_Selection == Selection::Node && i == m_SelectedNode);

            ImGui::PushID(i);
            float avail = ImGui::GetContentRegionAvail().x;
            if (ImGui::Selectable(label, selected, 0, ImVec2(avail - 28.0f, 0.0f)))
            {
                m_Selection = Selection::Node;
                m_SelectedNode = i;
            }
            ImGui::SameLine();
            bool erased = ImGui::SmallButton("X");
            ImGui::PopID();

            if (erased)
            {
                m_Nodes.erase(m_Nodes.begin() + i);
                if (m_Selection == Selection::Node)
                {
                    if (m_SelectedNode == i)
                    {
                        m_Selection = Selection::None;
                        m_SelectedNode = -1;
                    }
                    else if (m_SelectedNode > i)
                    {
                        --m_SelectedNode;
                    }
                }
                break;
            }
        }
    }
    ImGui::End();
}

void EditApp::DrawNodeEditWindow()
{
    if (m_SelectedNode < 0 || m_SelectedNode >= (int)m_Nodes.size())
    {
        m_Selection = Selection::None;
        return;
    }

    EditNode &node = m_Nodes[m_SelectedNode];
    bool open = true;
    if (ImGui::Begin("Node Edit", &open))
    {
        ImGui::Text("Node ID: %d", node.id);
        float pos[3] = {node.position.x, node.position.y, node.position.z};
        if (ImGui::InputFloat3("Position", pos))
            node.position = XMFLOAT3(pos[0], pos[1], pos[2]);
        ImGui::TextDisabled("(or drag the yellow sphere)");

        float dir[3] = {node.direction.x, node.direction.y, node.direction.z};
        if (ImGui::InputFloat3("Direction", dir))
            node.direction = XMFLOAT3(dir[0], dir[1], dir[2]);
        ImGui::TextDisabled("(ParkSpot's target heading; unused by other types)");

        // RoadDataManager::GetRoadNodeTypeByName()이 인식하는 값만 골라 오타를 방지한다.
        static const char *typeNames[] = {"unknown", "park", "park_spot"};
        int typeIdx = 0;
        for (int i = 0; i < IM_ARRAYSIZE(typeNames); ++i)
        {
            if (std::string(node.type) == typeNames[i])
            {
                typeIdx = i;
                break;
            }
        }
        if (ImGui::Combo("type", &typeIdx, typeNames, IM_ARRAYSIZE(typeNames)))
            snprintf(node.type, sizeof(node.type), "%s", typeNames[typeIdx]);

        ImGui::Separator();
        ImGui::Text("Children (e.g. Park -> ParkSpot ids)");
        int eraseIdx = -1;
        for (int i = 0; i < (int)node.children.size(); ++i)
        {
            ImGui::PushID(i);
            ImGui::SetNextItemWidth(100.0f);
            ImGui::InputInt("##child", &node.children[i]);
            ImGui::SameLine();
            if (ImGui::SmallButton("X"))
                eraseIdx = i;
            ImGui::PopID();
        }
        if (eraseIdx >= 0)
            node.children.erase(node.children.begin() + eraseIdx);
        if (ImGui::Button("Add Child"))
            node.children.push_back(0);
    }
    ImGui::End();

    if (!open)
        m_Selection = Selection::None;
}

void EditApp::DrawMarkingListWindow()
{
    if (ImGui::Begin("Lines"))
    {
        if (ImGui::Button("Add Line"))
        {
            EditMarking marking;
            marking.id = m_NextMarkingId++;
            m_Markings.push_back(marking);
            m_Selection = Selection::Marking;
            m_SelectedMarking = (int)m_Markings.size() - 1;
        }

        ImGui::Separator();

        for (int i = 0; i < (int)m_Markings.size(); ++i)
        {
            char label[64];
            snprintf(label, sizeof(label), "Line %d (%s)", m_Markings[i].id,
                     m_Markings[i].type == MarkingLineType::Dashed ? "dashed" : "solid");
            bool selected = (m_Selection == Selection::Marking && i == m_SelectedMarking);

            ImGui::PushID(i);
            float avail = ImGui::GetContentRegionAvail().x;
            if (ImGui::Selectable(label, selected, 0, ImVec2(avail - 28.0f, 0.0f)))
            {
                m_Selection = Selection::Marking;
                m_SelectedMarking = i;
            }
            ImGui::SameLine();
            bool erased = ImGui::SmallButton("X");
            ImGui::PopID();

            if (erased)
            {
                m_Markings.erase(m_Markings.begin() + i);
                if (m_Selection == Selection::Marking)
                {
                    if (m_SelectedMarking == i)
                    {
                        m_Selection = Selection::None;
                        m_SelectedMarking = -1;
                    }
                    else if (m_SelectedMarking > i)
                    {
                        --m_SelectedMarking;
                    }
                }
                break;
            }
        }
    }
    ImGui::End();
}

void EditApp::DrawMarkingEditWindow()
{
    if (m_SelectedMarking < 0 || m_SelectedMarking >= (int)m_Markings.size())
    {
        m_Selection = Selection::None;
        return;
    }

    EditMarking &marking = m_Markings[m_SelectedMarking];
    bool open = true;
    if (ImGui::Begin("Line Edit", &open))
    {
        ImGui::Text("Line ID: %d", marking.id);

        int typeIdx = (marking.type == MarkingLineType::Dashed) ? 1 : 0;
        const char *typeNames[] = {"Solid", "Dashed"};
        if (ImGui::Combo("Type", &typeIdx, typeNames, 2))
            marking.type = (typeIdx == 1) ? MarkingLineType::Dashed : MarkingLineType::Solid;

        int colorIdx = (int)marking.color;
        const char *colorNames[] = {"White", "Yellow", "Gray"};
        if (ImGui::Combo("Color", &colorIdx, colorNames, 3))
            marking.color = (MarkingColor)colorIdx;

        ImGui::DragFloat("Width", &marking.width, 0.01f, 0.01f, 5.0f, "%.2f");

        if (marking.type == MarkingLineType::Dashed)
        {
            ImGui::DragFloat("Dash Length", &marking.dashLength, 0.1f, 0.01f, 50.0f, "%.2f");
            ImGui::DragFloat("Dash Gap", &marking.dashGap, 0.1f, 0.0f, 50.0f, "%.2f");
        }

        ImGui::Separator();

        if (ImGui::Button("Add Point"))
        {
            XMFLOAT3 p(0.0f, 0.0f, 0.0f);
            if (!marking.points.empty())
            {
                p = marking.points.back();
                p.x += 2.0f; // offset so it doesn't overlap the previous one
            }
            marking.points.push_back(p);
        }

        ImGui::Text("Points: %d", (int)marking.points.size());
        for (int i = 0; i < (int)marking.points.size(); ++i)
        {
            XMFLOAT3 &p = marking.points[i];
            ImGui::PushID(i);

            ImGui::Text("[%d]", i);
            ImGui::SameLine();
            float pos[3] = {p.x, p.y, p.z};
            ImGui::SetNextItemWidth(180.0f);
            if (ImGui::InputFloat3("##pos", pos, "%.2f"))
                p = XMFLOAT3(pos[0], pos[1], pos[2]);
            ImGui::SameLine();
            bool erased = ImGui::SmallButton("X");

            ImGui::PopID();
            if (erased)
            {
                marking.points.erase(marking.points.begin() + i);
                break;
            }
        }
    }
    ImGui::End();

    if (!open)
        m_Selection = Selection::None;
}

void EditApp::DrawScene()
{
    // Create render target view for the back buffer (mirrors GameApp::DrawScene).
    if (m_FrameCount < m_BackBufferCount)
    {
        ComPtr<ID3D11Texture2D> pBackBuffer;
        m_pSwapChain->GetBuffer(0, IID_PPV_ARGS(pBackBuffer.GetAddressOf()));
        CD3D11_RENDER_TARGET_VIEW_DESC rtvDesc(D3D11_RTV_DIMENSION_TEXTURE2D, DXGI_FORMAT_R8G8B8A8_UNORM_SRGB);
        m_pd3dDevice->CreateRenderTargetView(pBackBuffer.Get(), &rtvDesc, m_pRenderTargetViews[m_FrameCount].ReleaseAndGetAddressOf());
    }

    float black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    m_pd3dImmediateContext->ClearRenderTargetView(GetBackBufferRTV(), black);
    m_pd3dImmediateContext->ClearDepthStencilView(m_pDepthTexture->GetDepthStencil(), D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);
    ID3D11RenderTargetView *pRTVs[1] = {GetBackBufferRTV()};
    m_pd3dImmediateContext->OMSetRenderTargets(1, pRTVs, m_pDepthTexture->GetDepthStencil());
    D3D11_VIEWPORT viewport = m_pCamera->GetViewPort();
    m_pd3dImmediateContext->RSSetViewports(1, &viewport);

    m_BasicEffect.SetRenderDefault();
    for (auto &obj : m_GameObjects)
        obj->Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    for (auto &ro : m_PointRenders)
        ro.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    for (auto &ro : m_MarkingRenders)
        ro.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);

    m_BasicEffect.SetRenderLines();
    if (m_ShowGridXZ)
        m_GridXZ.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    if (m_ShowGridXY)
        m_GridXY.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    if (m_ShowGridYZ)
        m_GridYZ.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    for (auto &spline : m_SplineRenders)
        spline.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    m_BasicEffect.SetRenderDefault();

    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

    HR(m_pSwapChain->Present(0, m_IsDxgiFlipModel ? DXGI_PRESENT_ALLOW_TEARING : 0));
}
