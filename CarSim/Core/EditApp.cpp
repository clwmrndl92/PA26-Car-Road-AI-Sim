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

    if (m_Selection == Selection::Lane && m_SelectedLane >= 0 && m_SelectedLane < (int)m_Lanes.size())
    {
        for (auto &p : m_Lanes[m_SelectedLane].points)
            pts.push_back(&p);
        radius = CP_RADIUS;
    }
    else if (m_Selection == Selection::Node && m_SelectedNode >= 0 && m_SelectedNode < (int)m_Nodes.size())
    {
        pts.push_back(&m_Nodes[m_SelectedNode].position);
        radius = NODE_RADIUS;
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
                p.x = roundf(x / GRID_SNAP) * GRID_SNAP;
                p.z = roundf(z / GRID_SNAP) * GRID_SNAP;
            }
        }
    }
}

void EditApp::RebuildRenderObjects()
{
    m_PointRenders.clear();
    m_SplineRenders.clear();

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

    // Selected lane: control-point spheres (orange) + red spline.
    if (m_Selection == Selection::Lane && m_SelectedLane >= 0 && m_SelectedLane < (int)m_Lanes.size())
    {
        EditLane &lane = m_Lanes[m_SelectedLane];

        Model *pSphere = m_ModelManager.CreateFromGeometry("edit_cp", Geometry::CreateSphere(CP_RADIUS));
        pSphere->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 0.6f, 0.0f, 1.0f));
        pSphere->materials[0].Set<float>("$Opacity", 1.0f);

        for (const auto &p : lane.points)
        {
            RenderObject &ro = m_PointRenders.emplace_back();
            ro.SetModel(pSphere);
            ro.GetTransform().SetPosition(p);
        }
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
        root["nodes"].push_back({{"id", n.id},
                                 {"position", {n.position.x, n.position.y, n.position.z}},
                                 {"type", n.type},
                                 {"description", n.description}});
    }

    std::filesystem::create_directories("Data");

    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);

    std::string path = std::string("Data/") + stamp + ".json";
    std::ofstream ofs(path);
    if (ofs)
    {
        ofs << root.dump(2);
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
        copyStr(n.type, sizeof(n.type), jn.value("type", std::string("end")));
        copyStr(n.description, sizeof(n.description), jn.value("description", std::string("")));
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

void EditApp::UpdateUI(float dt)
{
    // Keep GameApp's Debug Grid window (top-left).
    GameApp::UpdateUI(dt);

    DrawToolbarWindow();
    DrawLaneListWindow();
    DrawRoadListWindow();
    DrawNodeListWindow();

    if (m_Selection == Selection::Lane)
        DrawLaneEditWindow();
    else if (m_Selection == Selection::Node)
        DrawNodeEditWindow();
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
            if (fs::exists("Data", ec))
            {
                for (const auto &entry : fs::directory_iterator("Data", ec))
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
                ImGui::TextDisabled("No .json in Data/");
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
            if (ImGui::Selectable(label, selected))
            {
                m_Selection = Selection::Lane;
                m_SelectedLane = i;
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
            ImGui::Text("[%d] (%.1f, %.1f, %.1f)", i, p.x, p.y, p.z);
            ImGui::SameLine();
            ImGui::PushID(i);
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
            ImGui::InputText("name", m_Roads[i].name, sizeof(m_Roads[i].name));
            ImGui::InputInt("speed", &m_Roads[i].speedLimit);
            ImGui::Separator();
            ImGui::PopID();
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
            if (ImGui::Selectable(label, selected))
            {
                m_Selection = Selection::Node;
                m_SelectedNode = i;
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
        ImGui::Text("Position: (%.1f, %.1f, %.1f)", node.position.x, node.position.y, node.position.z);
        ImGui::TextDisabled("(drag the yellow sphere to move)");
        ImGui::InputText("type", node.type, sizeof(node.type));
        ImGui::InputText("description", node.description, sizeof(node.description));
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
