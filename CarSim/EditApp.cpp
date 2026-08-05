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
    else if (m_Selection == Selection::Road && m_SelectedRoad >= 0 && m_SelectedRoad < (int)m_Roads.size())
    {
        for (auto &p : m_Roads[m_SelectedRoad].referenceLine)
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
    else if (m_Selection == Selection::Obstacle && m_SelectedObstacle >= 0 && m_SelectedObstacle < (int)m_Obstacles.size())
    {
        pts.push_back(&m_Obstacles[m_SelectedObstacle].position);
        radius = OBSTACLE_MARKER_RADIUS;
    }
    else if (m_Selection == Selection::DynamicObstacle && m_SelectedDynamicObstacle >= 0 &&
             m_SelectedDynamicObstacle < (int)m_DynamicObstacles.size())
    {
        pts.push_back(&m_DynamicObstacles[m_SelectedDynamicObstacle].start);
        pts.push_back(&m_DynamicObstacles[m_SelectedDynamicObstacle].end);
        radius = OBSTACLE_MARKER_RADIUS;
    }
    else
    {
        // Nothing (draggable) selected -- leave pts empty so the click-handling below falls
        // straight through to lane-spline picking instead of bailing out early.
        radius = CP_RADIUS;
        snap = GRID_SNAP;
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

        // Missed every draggable control point: try picking a lane by its debug spline
        // instead, so clicking the red line selects that lane in the Lane window.
        if (best == -1)
            PickLaneUnderMouse(ray);
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

namespace
{
    // XZ-plane distance from a point to a segment (splines are authored flat on y = 0, so the
    // vertical component only matters for the initial ray/plane intersection, not for picking).
    float DistancePointToSegmentXZ(const XMFLOAT3 &p, const XMFLOAT3 &a, const XMFLOAT3 &b)
    {
        float abx = b.x - a.x, abz = b.z - a.z;
        float apx = p.x - a.x, apz = p.z - a.z;
        float lenSq = abx * abx + abz * abz;
        float t = lenSq > 1e-6f ? std::clamp((apx * abx + apz * abz) / lenSq, 0.0f, 1.0f) : 0.0f;
        float dx = p.x - (a.x + abx * t);
        float dz = p.z - (a.z + abz * t);
        return std::sqrt(dx * dx + dz * dz);
    }

    // 참조선 control points를 스플라인으로 샘플링해 각 점을 진행방향 오른쪽으로 d만큼 민 폴리라인.
    // 오른쪽 = (dir.z, 0, -dir.x) (북쪽 진행 시 +X). d>0=오른쪽, d<0=왼쪽. y는 liftY만큼 띄운다.
    std::vector<XMFLOAT3> OffsetReferencePolyline(const std::vector<XMFLOAT3> &refPoints, float d, float liftY)
    {
        std::vector<XMFLOAT3> out;
        if (refPoints.size() < 4)
            return out;

        std::vector<Vec3> cps;
        cps.reserve(refPoints.size());
        for (const auto &p : refPoints)
            cps.push_back(ToVec3(p));

        Spline spline(cps);
        const std::vector<Vec3> &samples = spline.GetSplinePoints();
        if (samples.size() < 2)
            return out;

        out.reserve(samples.size());
        const size_t n = samples.size();
        for (size_t i = 0; i < n; ++i)
        {
            XMFLOAT3 a = ToXMFLOAT3(samples[i]);
            XMFLOAT3 next = ToXMFLOAT3(samples[i + 1 < n ? i + 1 : i]);
            XMFLOAT3 prev = ToXMFLOAT3(samples[i > 0 ? i - 1 : i]);
            float tx = next.x - prev.x, tz = next.z - prev.z; // 중앙차분 접선
            float len = std::sqrt(tx * tx + tz * tz);
            float rx = len > 1e-5f ? tz / len : 0.0f;
            float rz = len > 1e-5f ? -tx / len : 0.0f;
            out.push_back(XMFLOAT3(a.x + rx * d, a.y + liftY, a.z + rz * d));
        }
        return out;
    }
}

bool EditApp::PickLaneUnderMouse(const Ray &ray)
{
    // Splines are rendered lifted to y = 0.1f (RebuildRenderObjects); intersect the ray with
    // that plane to get a world-space click point, then find the nearest lane polyline to it.
    constexpr float SPLINE_RENDER_Y = 0.1f;
    constexpr float LANE_PICK_RADIUS = 1.0f;

    if (fabsf(ray.direction.y) <= 1e-5f)
        return false;

    float t = (SPLINE_RENDER_Y - ray.origin.y) / ray.direction.y;
    if (t <= 0.0f)
        return false;

    XMFLOAT3 clickPoint(ray.origin.x + t * ray.direction.x, SPLINE_RENDER_Y, ray.origin.z + t * ray.direction.z);

    float bestDist = LANE_PICK_RADIUS;
    int best = -1;
    for (int i = 0; i < (int)m_Lanes.size(); ++i)
    {
        const EditLane &lane = m_Lanes[i];
        if (lane.points.size() < 4)
            continue;

        std::vector<Vec3> cps;
        cps.reserve(lane.points.size());
        for (const auto &p : lane.points)
            cps.push_back(ToVec3(p));

        Spline spline(cps);
        const std::vector<Vec3> &samples = spline.GetSplinePoints();
        for (size_t s = 0; s + 1 < samples.size(); ++s)
        {
            float dist = DistancePointToSegmentXZ(clickPoint, ToXMFLOAT3(samples[s]), ToXMFLOAT3(samples[s + 1]));
            if (dist < bestDist)
            {
                bestDist = dist;
                best = i;
            }
        }
    }

    if (best < 0)
        return false;

    m_Selection = Selection::Lane;
    m_SelectedLane = best;
    return true;
}

void EditApp::RebuildRenderObjects()
{
    m_PointRenders.clear();
    m_SplineRenders.clear();
    m_RoadRenders.clear();
    m_MarkingRenders.clear();
    m_ObstacleRenders.clear();
    m_DynamicObstacleRenders.clear();
    m_DynamicObstaclePathRenders.clear();

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
        // 주행 레인 = 빨강, 주차레인 = 시안 (한눈에 구분).
        XMFLOAT4 splineColor = lane.isParking ? XMFLOAT4(0.0f, 0.85f, 1.0f, 1.0f)
                                              : XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f);
        pLine->materials[0].Set<XMFLOAT4>("$DiffuseColor", splineColor);
        pLine->materials[0].Set<float>("$Opacity", 1.0f);

        RenderObject &ro = m_SplineRenders.emplace_back();
        ro.SetModel(pLine);
    }

    // Every road's reference line (green) + its band/center marks (ribbons) are always shown.
    // Marks are placed by offsetting the reference spline laterally (right = +, left = -).
    auto roadMarkColor = [](MarkingColor c) -> XMFLOAT4
    {
        switch (c)
        {
        case MarkingColor::Yellow:
            return XMFLOAT4(1.0f, 0.85f, 0.0f, 1.0f);
        case MarkingColor::Gray:
            return XMFLOAT4(0.22f, 0.22f, 0.22f, 1.0f);
        default:
            return XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
        }
    };
    for (const auto &road : m_Roads)
    {
        if (road.referenceLine.size() < 4)
            continue;

        // 도로 바닥 아스팔트(회색): 밴드 전체 d범위를 덮는 리본. y를 마킹보다 낮게 깔아 밑에 깔린다.
        float dMin = FLT_MAX, dMax = -FLT_MAX;
        for (const auto &sec : road.laneSections)
            for (const auto &b : sec.bands)
            {
                dMin = std::min(dMin, b.centerOffset - b.width * 0.5f);
                dMax = std::max(dMax, b.centerOffset + b.width * 0.5f);
            }
        if (dMax > dMin)
        {
            std::vector<XMFLOAT3> asphaltLine = OffsetReferencePolyline(road.referenceLine, (dMin + dMax) * 0.5f, 0.02f);
            if (asphaltLine.size() >= 2)
            {
                GeometryData geo = Geometry::CreateRibbon(asphaltLine, dMax - dMin);
                if (!geo.vertices.empty())
                {
                    Model *pAsphalt = m_ModelManager.CreateFromGeometry("edit_asphalt_" + std::to_string(road.id), geo);
                    pAsphalt->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.18f, 0.18f, 0.18f, 1.0f));
                    pAsphalt->materials[0].Set<float>("$Opacity", 1.0f);
                    m_RoadRenders.emplace_back().SetModel(pAsphalt);
                }
            }
        }

        // 참조선: 초록 리본(폭 0.15).
        std::vector<XMFLOAT3> refLine = OffsetReferencePolyline(road.referenceLine, 0.0f, 0.05f);
        if (refLine.size() >= 2)
        {
            GeometryData geo = Geometry::CreateRibbon(refLine, 0.15f);
            if (!geo.vertices.empty())
            {
                Model *pRef = m_ModelManager.CreateFromGeometry("edit_refline_" + std::to_string(road.id), geo);
                pRef->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.1f, 0.9f, 0.3f, 1.0f));
                pRef->materials[0].Set<float>("$Opacity", 1.0f);
                m_RoadRenders.emplace_back().SetModel(pRef);
            }
        }

        // 마킹 리본 한 줄(또는 복선이면 두 줄) 생성. d = 참조선 기준 오프셋.
        auto addMark = [&](float d, const EditBoundaryMark &mk, const std::string &tag)
        {
            if (mk.type == BoundaryMarkType::None)
                return;
            auto ribbon = [&](float dd, const std::string &name)
            {
                std::vector<XMFLOAT3> poly = OffsetReferencePolyline(road.referenceLine, dd, 0.06f);
                if (poly.size() < 2)
                    return;
                GeometryData geo = mk.type == BoundaryMarkType::Broken
                                       ? Geometry::CreateDashedRibbon(poly, mk.width, 3.0f, 5.0f)
                                       : Geometry::CreateRibbon(poly, mk.width);
                if (geo.vertices.empty())
                    return;
                Model *pm = m_ModelManager.CreateFromGeometry(name, geo);
                pm->materials[0].Set<XMFLOAT4>("$DiffuseColor", roadMarkColor(mk.color));
                pm->materials[0].Set<float>("$Opacity", 1.0f);
                m_RoadRenders.emplace_back().SetModel(pm);
            };
            if (mk.type == BoundaryMarkType::DoubleSolid)
            {
                ribbon(d - 0.12f, tag + "_a");
                ribbon(d + 0.12f, tag + "_b");
            }
            else
            {
                ribbon(d, tag);
            }
        };

        // 중앙선: 참조선 위(d=0).
        if (road.hasCenterMark)
            addMark(0.0f, road.centerMark, "edit_centermark_" + std::to_string(road.id));

        // 밴드별 바깥쪽 경계 마킹(|d| 큰 쪽 = centerOffset + sign*width/2).
        for (int si = 0; si < (int)road.laneSections.size(); ++si)
        {
            const EditLaneSection &sec = road.laneSections[si];
            for (int bi = 0; bi < (int)sec.bands.size(); ++bi)
            {
                const EditBand &b = sec.bands[bi];
                float outer = b.centerOffset + (b.centerOffset >= 0.0f ? 1.0f : -1.0f) * b.width * 0.5f;
                addMark(outer, b.boundaryMark,
                        "edit_bandmark_" + std::to_string(road.id) + "_" + std::to_string(si) + "_" + std::to_string(bi));
            }
        }
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

    // Obstacle center spheres (draggable, always visible for context): blue, selected one yellow.
    Model *pObstacleMarker = m_ModelManager.CreateFromGeometry("edit_obstacle_marker", Geometry::CreateSphere(OBSTACLE_MARKER_RADIUS));
    pObstacleMarker->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.0f, 0.4f, 1.0f, 1.0f));
    pObstacleMarker->materials[0].Set<float>("$Opacity", 1.0f);

    Model *pObstacleMarkerSel = m_ModelManager.CreateFromGeometry("edit_obstacle_marker_sel", Geometry::CreateSphere(OBSTACLE_MARKER_RADIUS));
    pObstacleMarkerSel->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 1.0f, 0.0f, 1.0f));
    pObstacleMarkerSel->materials[0].Set<float>("$Opacity", 1.0f);

    for (int i = 0; i < (int)m_Obstacles.size(); ++i)
    {
        bool selected = (m_Selection == Selection::Obstacle && i == m_SelectedObstacle);
        RenderObject &ro = m_PointRenders.emplace_back();
        ro.SetModel(selected ? pObstacleMarkerSel : pObstacleMarker);
        ro.GetTransform().SetPosition(m_Obstacles[i].position);
    }

    // Obstacle footprints (filled blue planes), position/size/rotation exactly as authored.
    for (int i = 0; i < (int)m_Obstacles.size(); ++i)
    {
        const EditObstacle &obstacle = m_Obstacles[i];
        float headingRad = ToRadians(obstacle.rotation);
        XMFLOAT3 forward(cosf(headingRad), 0.0f, sinf(headingRad));
        XMFLOAT3 right(-forward.z, 0.0f, forward.x);
        float halfLength = obstacle.length * 0.5f;
        float halfWidth = obstacle.width * 0.5f;

        auto corner = [&](float alongSign, float acrossSign)
        {
            XMFLOAT3 p(obstacle.position.x + forward.x * halfLength * alongSign + right.x * halfWidth * acrossSign,
                       obstacle.position.y + 0.1f,
                       obstacle.position.z + forward.z * halfLength * alongSign + right.z * halfWidth * acrossSign);
            return p;
        };

        Model *pPlane = m_ModelManager.CreateFromGeometry(
            "edit_obstacle_plane_" + std::to_string(obstacle.id),
            Geometry::CreateQuad(corner(1.0f, 1.0f), corner(1.0f, -1.0f), corner(-1.0f, -1.0f), corner(-1.0f, 1.0f)));
        pPlane->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.0f, 0.4f, 1.0f, 1.0f));
        pPlane->materials[0].Set<float>("$Opacity", 1.0f);

        RenderObject &ro = m_ObstacleRenders.emplace_back();
        ro.SetModel(pPlane);
    }

    // Dynamic-obstacle start/end spheres (draggable, always visible for context): orange,
    // selected one yellow. Both endpoints of the same obstacle share the marker model.
    Model *pDynMarker = m_ModelManager.CreateFromGeometry("edit_dyn_obstacle_marker", Geometry::CreateSphere(OBSTACLE_MARKER_RADIUS));
    pDynMarker->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 0.5f, 0.0f, 1.0f));
    pDynMarker->materials[0].Set<float>("$Opacity", 1.0f);

    Model *pDynMarkerSel = m_ModelManager.CreateFromGeometry("edit_dyn_obstacle_marker_sel", Geometry::CreateSphere(OBSTACLE_MARKER_RADIUS));
    pDynMarkerSel->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 1.0f, 0.0f, 1.0f));
    pDynMarkerSel->materials[0].Set<float>("$Opacity", 1.0f);

    for (int i = 0; i < (int)m_DynamicObstacles.size(); ++i)
    {
        bool selected = (m_Selection == Selection::DynamicObstacle && i == m_SelectedDynamicObstacle);
        RenderObject &roStart = m_PointRenders.emplace_back();
        roStart.SetModel(selected ? pDynMarkerSel : pDynMarker);
        roStart.GetTransform().SetPosition(m_DynamicObstacles[i].start);

        RenderObject &roEnd = m_PointRenders.emplace_back();
        roEnd.SetModel(selected ? pDynMarkerSel : pDynMarker);
        roEnd.GetTransform().SetPosition(m_DynamicObstacles[i].end);
    }

    // Dynamic-obstacle footprint (filled orange plane at start, oriented toward end) + a line
    // to end showing the patrol path -- position/size exactly as authored, heading is derived
    // (RoadDataManager::UpdateDynamicObstacles computes it the same way at runtime).
    for (int i = 0; i < (int)m_DynamicObstacles.size(); ++i)
    {
        const EditDynamicObstacle &obstacle = m_DynamicObstacles[i];
        XMFLOAT3 forward(obstacle.end.x - obstacle.start.x, 0.0f, obstacle.end.z - obstacle.start.z);
        float len = std::sqrt(forward.x * forward.x + forward.z * forward.z);
        if (len > 1e-5f)
        {
            forward.x /= len;
            forward.z /= len;
        }
        else
        {
            forward = XMFLOAT3(1.0f, 0.0f, 0.0f);
        }
        XMFLOAT3 right(-forward.z, 0.0f, forward.x);
        float halfLength = obstacle.length * 0.5f;
        float halfWidth = obstacle.width * 0.5f;

        auto corner = [&](float alongSign, float acrossSign)
        {
            XMFLOAT3 p(obstacle.start.x + forward.x * halfLength * alongSign + right.x * halfWidth * acrossSign,
                       obstacle.start.y + 0.1f,
                       obstacle.start.z + forward.z * halfLength * alongSign + right.z * halfWidth * acrossSign);
            return p;
        };

        Model *pPlane = m_ModelManager.CreateFromGeometry(
            "edit_dyn_obstacle_plane_" + std::to_string(obstacle.id),
            Geometry::CreateQuad(corner(1.0f, 1.0f), corner(1.0f, -1.0f), corner(-1.0f, -1.0f), corner(-1.0f, 1.0f)));
        pPlane->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 0.5f, 0.0f, 1.0f));
        pPlane->materials[0].Set<float>("$Opacity", 1.0f);

        RenderObject &roPlane = m_DynamicObstacleRenders.emplace_back();
        roPlane.SetModel(pPlane);

        XMFLOAT3 pathStart(obstacle.start.x, obstacle.start.y + 0.1f, obstacle.start.z);
        XMFLOAT3 pathEnd(obstacle.end.x, obstacle.end.y + 0.1f, obstacle.end.z);
        Model *pPath = m_ModelManager.CreateFromGeometry(
            "edit_dyn_obstacle_path_" + std::to_string(obstacle.id), Geometry::CreateLine(pathStart, pathEnd));
        pPath->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 0.5f, 0.0f, 1.0f));
        pPath->materials[0].Set<float>("$Opacity", 1.0f);

        RenderObject &roPath = m_DynamicObstaclePathRenders.emplace_back();
        roPath.SetModel(pPath);
    }

    // Selected lane or marking: control-point spheres (orange), on top of its always-shown
    // spline/ribbon.
    const std::vector<XMFLOAT3> *selectedPoints = nullptr;
    if (m_Selection == Selection::Lane && m_SelectedLane >= 0 && m_SelectedLane < (int)m_Lanes.size())
        selectedPoints = &m_Lanes[m_SelectedLane].points;
    else if (m_Selection == Selection::Road && m_SelectedRoad >= 0 && m_SelectedRoad < (int)m_Roads.size())
        selectedPoints = &m_Roads[m_SelectedRoad].referenceLine;
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
    template <typename JsonType>
    std::string DumpPrettyArraysCompactElements(const JsonType &root)
    {
        std::string out = "{\n";
        size_t keyIndex = 0;
        for (auto it = root.begin(); it != root.end(); ++it, ++keyIndex)
        {
            out += "  \"" + it.key() + "\": ";

            const JsonType &arr = it.value();
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
    // ordered_json: 일반 json은 키를 알파벳순으로 저장해 control_points가 항상 맨 앞으로 가버린다.
    // 레인 오브젝트의 필드 순서(id -> road/park -> left/right -> control_points)를 그대로
    // 지키기 위해 삽입 순서를 보존하는 ordered_json을 쓴다.
    using json = nlohmann::ordered_json;
    json root;

    // float 부정확성이 dump에 남지 않도록 double로 반올림(SaveMarkingsToJson과 동일).
    auto round2 = [](double v)
    { return std::round(v * 100.0) / 100.0; };

    root["lanes"] = json::array();
    json parkingLanes = json::array();
    for (const auto &l : m_Lanes)
    {
        json cps = json::array();
        for (const auto &p : l.points)
            cps.push_back({round2(p.x), round2(p.y), round2(p.z)});

        json jl;
        jl["id"] = l.id;
        if (l.isParking)
        {
            // 주차레인: park(소속 Park 노드) + control_points만. 메인 "lanes"와 분리.
            jl["park"] = l.park;
            jl["control_points"] = cps;
            parkingLanes.push_back(jl);
        }
        else
        {
            jl["road"] = l.road;
            if (l.left >= 0)
                jl["left"] = l.left;
            if (l.right >= 0)
                jl["right"] = l.right;
            jl["control_points"] = cps;
            root["lanes"].push_back(jl);
        }
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
        if (!n.roads.empty())
            jn["roads"] = n.roads;
        if (n.phaseOffset != 0.0f)
            jn["phase_offset"] = n.phaseOffset;
        if (!n.movements.empty())
            jn["movements"] = n.movements;
        if (n.greenDuration != 8.0f)
            jn["green_duration"] = n.greenDuration;
        if (n.yellowDuration != 3.0f)
            jn["yellow_duration"] = n.yellowDuration;
        if (n.redDuration != 12.0f)
            jn["red_duration"] = n.redDuration;
        root["nodes"].push_back(jn);
    }

    root["obstacles"] = json::array();
    for (const auto &o : m_Obstacles)
    {
        json jo;
        jo["position"] = {o.position.x, o.position.y, o.position.z};
        jo["size"] = {o.length, o.width};
        jo["rotation"] = o.rotation;
        root["obstacles"].push_back(jo);
    }

    root["dynamic_obstacles"] = json::array();
    for (const auto &d : m_DynamicObstacles)
    {
        json jd;
        jd["start"] = {d.start.x, d.start.y, d.start.z};
        jd["end"] = {d.end.x, d.end.y, d.end.z};
        jd["size"] = {d.length, d.width};
        jd["speed"] = d.speed;
        root["dynamic_obstacles"].push_back(jd);
    }

    root["parking_lanes"] = parkingLanes;

    auto markToJson = [&](const EditBoundaryMark &mk)
    {
        json jm;
        jm["type"] = mk.type == BoundaryMarkType::Solid ? "solid" : mk.type == BoundaryMarkType::Broken ? "broken"
                                                                : mk.type == BoundaryMarkType::DoubleSolid ? "double_solid"
                                                                                                          : "none";
        jm["color"] = mk.color == MarkingColor::Yellow ? "yellow" : mk.color == MarkingColor::Gray ? "gray"
                                                                                                   : "white";
        jm["width"] = round2(mk.width);
        return jm;
    };

    auto linkToJson = [&](const EditRoadLink &lk)
    {
        json jlk;
        jlk["type"] = lk.type == EditElementType::Junction ? "junction" : "road";
        jlk["id"] = lk.elementId;
        jlk["contact"] = lk.contact == EditContactPoint::End ? "end" : "start";
        return jlk;
    };

    root["roads"] = json::array();
    for (const auto &r : m_Roads)
    {
        json jr;
        jr["id"] = r.id;
        jr["name"] = r.name;
        jr["speed_limit"] = r.speedLimit;

        if (!r.referenceLine.empty())
        {
            json rl = json::array();
            for (const auto &p : r.referenceLine)
                rl.push_back({round2(p.x), round2(p.y), round2(p.z)});
            jr["reference_line"] = rl;
        }
        if (r.hasCenterMark)
            jr["center_mark"] = markToJson(r.centerMark);
        if (!r.laneSections.empty())
        {
            json secs = json::array();
            for (const auto &sec : r.laneSections)
            {
                json js;
                js["s_start"] = round2(sec.sStart);
                json bands = json::array();
                for (const auto &b : sec.bands)
                {
                    json jb;
                    jb["center_offset"] = round2(b.centerOffset);
                    jb["width"] = round2(b.width);
                    jb["type"] = b.type;
                    jb["speed_limit"] = b.speedLimit;
                    if (b.backward)
                        jb["direction"] = "backward"; // 기본값(forward)은 안 적어 기존 데이터와 diff가 안 생기게
                    jb["boundary_mark"] = markToJson(b.boundaryMark);
                    bands.push_back(jb);
                }
                js["bands"] = bands;
                secs.push_back(js);
            }
            jr["lane_sections"] = secs;
        }
        if (r.junction != -1)
            jr["junction"] = r.junction;
        if (r.predecessor.valid || r.successor.valid)
        {
            json jlink;
            if (r.predecessor.valid)
                jlink["predecessor"] = linkToJson(r.predecessor);
            if (r.successor.valid)
                jlink["successor"] = linkToJson(r.successor);
            jr["link"] = jlink;
        }
        root["roads"].push_back(jr);
    }

    root["junctions"] = json::array();
    for (const auto &j : m_Junctions)
    {
        json jj;
        jj["id"] = j.id;
        json conns = json::array();
        for (const auto &c : j.connections)
        {
            json jc;
            jc["incoming_road"] = c.incomingRoad;
            jc["connecting_road"] = c.connectingRoad;
            jc["contact"] = c.contact == EditContactPoint::End ? "end" : "start";
            json links = json::array();
            for (const auto &ll : c.laneLinks)
                links.push_back({{"from", ll.from}, {"to", ll.to}});
            jc["lane_links"] = links;
            conns.push_back(jc);
        }
        jj["connections"] = conns;
        root["junctions"].push_back(jj);
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
    m_Junctions.clear();
    m_Lanes.clear();
    m_Nodes.clear();
    m_Obstacles.clear();
    m_DynamicObstacles.clear();

    auto copyStr = [](char *dst, size_t n, const std::string &s)
    {
        snprintf(dst, n, "%s", s.c_str());
    };

    auto parseContact = [](const nlohmann::json &j, const char *key)
    {
        return j.value(std::string(key), std::string("start")) == "end" ? EditContactPoint::End : EditContactPoint::Start;
    };

    auto parseLink = [&](const nlohmann::json &jl)
    {
        EditRoadLink lk;
        lk.valid = true;
        lk.type = jl.value("type", std::string("road")) == "junction" ? EditElementType::Junction : EditElementType::Road;
        lk.elementId = jl.value("id", -1);
        lk.contact = parseContact(jl, "contact");
        return lk;
    };

    auto parseMark = [](const nlohmann::json &jm)
    {
        EditBoundaryMark mk;
        std::string t = jm.value("type", std::string("none"));
        mk.type = t == "solid" ? BoundaryMarkType::Solid : t == "broken" ? BoundaryMarkType::Broken
                              : t == "double_solid"    ? BoundaryMarkType::DoubleSolid
                                                       : BoundaryMarkType::None;
        std::string c = jm.value("color", std::string("white"));
        mk.color = c == "yellow" ? MarkingColor::Yellow : c == "gray" ? MarkingColor::Gray
                                                                      : MarkingColor::White;
        mk.width = jm.value("width", 0.15f);
        return mk;
    };

    for (const auto &jr : root.value("roads", nlohmann::json::array()))
    {
        EditRoad r;
        r.id = jr.value("id", -1);
        copyStr(r.name, sizeof(r.name), jr.value("name", std::string("road")));
        r.speedLimit = jr.value("speed_limit", 40);

        for (const auto &pt : jr.value("reference_line", nlohmann::json::array()))
        {
            if (pt.is_array() && pt.size() >= 3)
                r.referenceLine.push_back(XMFLOAT3(pt[0].get<float>(), pt[1].get<float>(), pt[2].get<float>()));
        }
        if (jr.contains("center_mark"))
        {
            r.hasCenterMark = true;
            r.centerMark = parseMark(jr["center_mark"]);
        }
        for (const auto &js : jr.value("lane_sections", nlohmann::json::array()))
        {
            EditLaneSection sec;
            sec.sStart = js.value("s_start", 0.0f);
            for (const auto &jb : js.value("bands", nlohmann::json::array()))
            {
                EditBand b;
                b.centerOffset = jb.value("center_offset", 0.0f);
                b.width = jb.value("width", 3.5f);
                copyStr(b.type, sizeof(b.type), jb.value("type", std::string("driving")));
                b.speedLimit = jb.value("speed_limit", 40);
                b.backward = jb.value("direction", std::string("forward")) == "backward";
                if (jb.contains("boundary_mark"))
                    b.boundaryMark = parseMark(jb["boundary_mark"]);
                sec.bands.push_back(b);
            }
            r.laneSections.push_back(std::move(sec));
        }
        r.junction = jr.value("junction", -1);
        if (jr.contains("link"))
        {
            const auto &jlink = jr["link"];
            if (jlink.contains("predecessor"))
                r.predecessor = parseLink(jlink["predecessor"]);
            if (jlink.contains("successor"))
                r.successor = parseLink(jlink["successor"]);
        }
        m_Roads.push_back(std::move(r));
    }

    for (const auto &jj : root.value("junctions", nlohmann::json::array()))
    {
        EditJunction j;
        j.id = jj.value("id", -1);
        for (const auto &jc : jj.value("connections", nlohmann::json::array()))
        {
            EditConnection c;
            c.incomingRoad = jc.value("incoming_road", -1);
            c.connectingRoad = jc.value("connecting_road", -1);
            c.contact = parseContact(jc, "contact");
            for (const auto &jll : jc.value("lane_links", nlohmann::json::array()))
                c.laneLinks.push_back({jll.value("from", 0), jll.value("to", 0)});
            j.connections.push_back(std::move(c));
        }
        m_Junctions.push_back(std::move(j));
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

    // 주차레인도 같은 m_Lanes에 담되 isParking=true로 구분(저장 시 다시 parking_lanes로 나간다).
    for (const auto &jl : root.value("parking_lanes", nlohmann::json::array()))
    {
        EditLane l;
        l.isParking = true;
        l.id = jl.value("id", -1);
        l.park = jl.value("park", -1);
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
        for (const auto &roadIdJson : jn.value("roads", nlohmann::json::array()))
            n.roads.push_back(roadIdJson.get<int>());
        n.phaseOffset = jn.value("phase_offset", 0.0f);
        for (const auto &movementIdJson : jn.value("movements", nlohmann::json::array()))
            n.movements.push_back(movementIdJson.get<int>());
        n.greenDuration = jn.value("green_duration", 8.0f);
        n.yellowDuration = jn.value("yellow_duration", 3.0f);
        n.redDuration = jn.value("red_duration", 12.0f);
        m_Nodes.push_back(n);
    }

    int obstacleId = 1;
    for (const auto &jo : root.value("obstacles", nlohmann::json::array()))
    {
        EditObstacle o;
        o.id = obstacleId++; // data.json엔 obstacle id가 없어서 로드 순서로 부여
        const auto &pos = jo.value("position", nlohmann::json::array());
        if (pos.is_array() && pos.size() >= 3)
            o.position = XMFLOAT3(pos[0].get<float>(), pos[1].get<float>(), pos[2].get<float>());
        const auto &size = jo.value("size", nlohmann::json::array());
        if (size.is_array() && size.size() >= 2)
        {
            o.length = size[0].get<float>();
            o.width = size[1].get<float>();
        }
        o.rotation = jo.value("rotation", 0.0f);
        m_Obstacles.push_back(o);
    }

    int dynamicObstacleId = 1;
    for (const auto &jd : root.value("dynamic_obstacles", nlohmann::json::array()))
    {
        EditDynamicObstacle d;
        d.id = dynamicObstacleId++; // data.json엔 id가 없어서 로드 순서로 부여 (obstacles와 동일)
        const auto &startJson = jd.value("start", nlohmann::json::array());
        if (startJson.is_array() && startJson.size() >= 3)
            d.start = XMFLOAT3(startJson[0].get<float>(), startJson[1].get<float>(), startJson[2].get<float>());
        const auto &endJson = jd.value("end", nlohmann::json::array());
        if (endJson.is_array() && endJson.size() >= 3)
            d.end = XMFLOAT3(endJson[0].get<float>(), endJson[1].get<float>(), endJson[2].get<float>());
        const auto &sizeJson = jd.value("size", nlohmann::json::array());
        if (sizeJson.is_array() && sizeJson.size() >= 2)
        {
            d.length = sizeJson[0].get<float>();
            d.width = sizeJson[1].get<float>();
        }
        d.speed = jd.value("speed", 1.0f);
        m_DynamicObstacles.push_back(d);
    }

    // Reset id counters so newly-added items don't collide with loaded ones.
    m_NextLaneId = 1;
    for (const auto &l : m_Lanes)
        m_NextLaneId = std::max(m_NextLaneId, l.id + 1);
    m_NextRoadId = 1;
    for (const auto &r : m_Roads)
        m_NextRoadId = std::max(m_NextRoadId, r.id + 1);
    m_NextJunctionId = 1;
    for (const auto &j : m_Junctions)
        m_NextJunctionId = std::max(m_NextJunctionId, j.id + 1);
    m_NextNodeId = 1;
    for (const auto &n : m_Nodes)
        m_NextNodeId = std::max(m_NextNodeId, n.id + 1);
    m_NextObstacleId = 1;
    for (const auto &o : m_Obstacles)
        m_NextObstacleId = std::max(m_NextObstacleId, o.id + 1);
    m_NextDynamicObstacleId = 1;
    for (const auto &d : m_DynamicObstacles)
        m_NextDynamicObstacleId = std::max(m_NextDynamicObstacleId, d.id + 1);

    m_Selection = Selection::None;
    m_SelectedLane = -1;
    m_SelectedRoad = -1;
    m_SelectedBand = -1;
    m_SelectedNode = -1;
    m_SelectedObstacle = -1;
    m_SelectedDynamicObstacle = -1;
    m_DraggingPoint = -1;

    m_LastSavePath = "Loaded: " + path.filename().string();
}

void EditApp::SaveMarkingsToJson()
{
    using nlohmann::json;
    json root;

    // Round in double (not float) so e.g. 53.2f's inherent float imprecision doesn't survive
    // into the json dump as 53.20000076293945 — the rounded double matches the double literal
    // 53.2 exactly, which the json library's shortest-round-trip printer renders cleanly.
    auto round2 = [](double v)
    { return std::round(v * 100.0) / 100.0; };

    root["markings"] = json::array();
    for (const auto &m : m_Markings)
    {
        json jm;
        jm["id"] = m.id;
        jm["type"] = (m.type == MarkingLineType::Dashed) ? "dashed" : "solid";
        jm["width"] = round2(m.width);
        jm["color"] = m.color == MarkingColor::Yellow ? "yellow" : m.color == MarkingColor::Gray ? "gray"
                                                                                                 : "white";
        jm["dash_length"] = round2(m.dashLength);
        jm["dash_gap"] = round2(m.dashGap);

        // Non-gray (white/yellow) lane paint needs to sit above gray asphalt-colored lines to
        // render on top of them, so bump its saved y instead of leaving it at the drawn y.
        json pts = json::array();
        for (const auto &p : m.points)
        {
            float y = (m.color == MarkingColor::Gray) ? p.y : 0.01f;
            pts.push_back({round2(p.x), round2(y), round2(p.z)});
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
    DrawJunctionListWindow();
    DrawNodeListWindow();
    DrawMarkingListWindow();
    DrawObstacleListWindow();
    DrawDynamicObstacleListWindow();

    if (m_Selection == Selection::Lane)
        DrawLaneEditWindow();
    else if (m_Selection == Selection::Road)
        DrawRoadEditWindow();
    else if (m_Selection == Selection::Node)
        DrawNodeEditWindow();
    else if (m_Selection == Selection::Marking)
        DrawMarkingEditWindow();
    else if (m_Selection == Selection::Obstacle)
        DrawObstacleEditWindow();
    else if (m_Selection == Selection::DynamicObstacle)
        DrawDynamicObstacleEditWindow();
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
        ImGui::SameLine();
        if (ImGui::Button("Add Parking Lane"))
        {
            EditLane lane;
            lane.id = m_NextLaneId++;
            lane.isParking = true;
            m_Lanes.push_back(lane);
            m_Selection = Selection::Lane;
            m_SelectedLane = (int)m_Lanes.size() - 1;
        }

        ImGui::Separator();

        for (int i = 0; i < (int)m_Lanes.size(); ++i)
        {
            char label[64];
            if (m_Lanes[i].isParking)
                snprintf(label, sizeof(label), "ParkLane %d (park %d)", m_Lanes[i].id, m_Lanes[i].park);
            else
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
        if (lane.isParking)
        {
            ImGui::TextDisabled("(parking lane)");
            ImGui::InputInt("Park Node ID", &lane.park);
        }
        else
        {
            ImGui::InputInt("Road ID", &lane.road);
            ImGui::InputInt("Left", &lane.left);
            ImGui::InputInt("Right", &lane.right);
        }

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
            m_Selection = Selection::Road;
            m_SelectedRoad = (int)m_Roads.size() - 1;
            m_SelectedBand = -1;
        }

        ImGui::Separator();

        for (int i = 0; i < (int)m_Roads.size(); ++i)
        {
            char label[64];
            snprintf(label, sizeof(label), "Road %d (%s)", m_Roads[i].id, m_Roads[i].name);
            bool selected = (m_Selection == Selection::Road && i == m_SelectedRoad);

            ImGui::PushID(i);
            float avail = ImGui::GetContentRegionAvail().x;
            if (ImGui::Selectable(label, selected, 0, ImVec2(avail - 28.0f, 0.0f)))
            {
                m_Selection = Selection::Road;
                m_SelectedRoad = i;
                m_SelectedBand = -1;
            }
            ImGui::SameLine();
            bool erased = ImGui::SmallButton("X");
            ImGui::PopID();

            if (erased)
            {
                m_Roads.erase(m_Roads.begin() + i);
                if (m_Selection == Selection::Road)
                {
                    if (m_SelectedRoad == i)
                    {
                        m_Selection = Selection::None;
                        m_SelectedRoad = -1;
                    }
                    else if (m_SelectedRoad > i)
                    {
                        --m_SelectedRoad;
                    }
                }
                break;
            }
        }
    }
    ImGui::End();
}

namespace
{
    // BoundaryMarkType <-> combo 인덱스 (콤보 항목 순서와 enum 순서 일치).
    const char *const kMarkTypeNames[] = {"none", "solid", "broken", "double_solid"};
    const char *const kMarkColorNames[] = {"white", "yellow"};
}

void EditApp::DrawRoadEditWindow()
{
    if (m_SelectedRoad < 0 || m_SelectedRoad >= (int)m_Roads.size())
    {
        m_Selection = Selection::None;
        return;
    }

    EditRoad &road = m_Roads[m_SelectedRoad];
    bool open = true;
    if (ImGui::Begin("Road Edit", &open))
    {
        ImGui::Text("Road ID: %d", road.id);
        ImGui::InputText("name", road.name, sizeof(road.name));
        ImGui::InputInt("speed limit", &road.speedLimit);

        // 마킹 편집 공통 위젯: 타입/색/폭.
        auto editMark = [](const char *label, EditBoundaryMark &mk)
        {
            ImGui::PushID(label);
            int typeIdx = (int)mk.type;
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::Combo("type", &typeIdx, kMarkTypeNames, IM_ARRAYSIZE(kMarkTypeNames)))
                mk.type = (BoundaryMarkType)typeIdx;
            ImGui::SameLine();
            int colorIdx = (mk.color == MarkingColor::Yellow) ? 1 : 0;
            ImGui::SetNextItemWidth(90.0f);
            if (ImGui::Combo("color", &colorIdx, kMarkColorNames, IM_ARRAYSIZE(kMarkColorNames)))
                mk.color = colorIdx == 1 ? MarkingColor::Yellow : MarkingColor::White;
            ImGui::SameLine();
            ImGui::SetNextItemWidth(70.0f);
            ImGui::InputFloat("width", &mk.width, 0.0f, 0.0f, "%.2f");
            ImGui::PopID();
        };

        ImGui::Separator();
        ImGui::Text("Center mark");
        ImGui::Checkbox("has center mark", &road.hasCenterMark);
        if (road.hasCenterMark)
            editMark("center", road.centerMark);

        ImGui::Separator();
        ImGui::Text("Reference line");
        if (ImGui::Button("Add Ref Point"))
        {
            XMFLOAT3 p(0.0f, 0.0f, 0.0f);
            if (!road.referenceLine.empty())
            {
                p = road.referenceLine.back();
                p.z += 2.0f;
            }
            road.referenceLine.push_back(p);
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(need >= 4 for a spline)");
        for (int i = 0; i < (int)road.referenceLine.size(); ++i)
        {
            XMFLOAT3 &p = road.referenceLine[i];
            ImGui::PushID(1000 + i);
            ImGui::Text("[%d]", i);
            ImGui::SameLine();
            float pos[3] = {p.x, p.y, p.z};
            ImGui::SetNextItemWidth(180.0f);
            if (ImGui::InputFloat3("##rp", pos))
                p = XMFLOAT3(pos[0], pos[1], pos[2]);
            ImGui::SameLine();
            bool erased = ImGui::SmallButton("X");
            ImGui::PopID();
            if (erased)
            {
                road.referenceLine.erase(road.referenceLine.begin() + i);
                break;
            }
        }

        ImGui::Separator();
        ImGui::Text("Lane bands");
        // phase1: 섹션 하나로 운용. 없으면 즉석에서 하나 만든다.
        if (road.laneSections.empty())
            road.laneSections.emplace_back();
        EditLaneSection &sec = road.laneSections[0];

        if (ImGui::Button("Add Band"))
        {
            EditBand b;
            if (!sec.bands.empty())
                b.centerOffset = sec.bands.back().centerOffset + sec.bands.back().width;
            sec.bands.push_back(b);
        }
        for (int i = 0; i < (int)sec.bands.size(); ++i)
        {
            EditBand &b = sec.bands[i];
            ImGui::PushID(2000 + i);
            ImGui::Text("Band %d", i);
            ImGui::SameLine();
            bool erased = ImGui::SmallButton("X");
            if (!erased)
            {
                ImGui::SetNextItemWidth(90.0f);
                ImGui::InputFloat("offset", &b.centerOffset, 0.0f, 0.0f, "%.2f");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0f);
                ImGui::InputFloat("width##b", &b.width, 0.0f, 0.0f, "%.2f");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(90.0f);
                ImGui::InputInt("spd", &b.speedLimit);
                ImGui::InputText("type##b", b.type, sizeof(b.type));
                ImGui::Checkbox("backward (reverse lane)", &b.backward);
                editMark("bandmark", b.boundaryMark);
            }
            ImGui::Separator();
            ImGui::PopID();
            if (erased)
            {
                sec.bands.erase(sec.bands.begin() + i);
                break;
            }
        }

        ImGui::Separator();
        ImGui::Text("Links");
        ImGui::InputInt("junction (-1=none)", &road.junction);
        auto editLink = [](const char *label, EditRoadLink &lk)
        {
            ImGui::PushID(label);
            ImGui::Checkbox(label, &lk.valid);
            if (lk.valid)
            {
                const char *types[] = {"road", "junction"};
                int typeIdx = lk.type == EditElementType::Junction ? 1 : 0;
                ImGui::SetNextItemWidth(90.0f);
                if (ImGui::Combo("target", &typeIdx, types, 2))
                    lk.type = typeIdx == 1 ? EditElementType::Junction : EditElementType::Road;
                ImGui::SameLine();
                ImGui::SetNextItemWidth(70.0f);
                ImGui::InputInt("id##lk", &lk.elementId, 0, 0);
                ImGui::SameLine();
                const char *contacts[] = {"start", "end"};
                int contactIdx = lk.contact == EditContactPoint::End ? 1 : 0;
                ImGui::SetNextItemWidth(80.0f);
                if (ImGui::Combo("contact", &contactIdx, contacts, 2))
                    lk.contact = contactIdx == 1 ? EditContactPoint::End : EditContactPoint::Start;
            }
            ImGui::PopID();
        };
        editLink("predecessor", road.predecessor);
        editLink("successor", road.successor);
    }
    ImGui::End();

    if (!open)
        m_Selection = Selection::None;
}

void EditApp::DrawJunctionListWindow()
{
    if (ImGui::Begin("Junctions"))
    {
        if (ImGui::Button("Add Junction"))
        {
            EditJunction j;
            j.id = m_NextJunctionId++;
            m_Junctions.push_back(j);
        }

        ImGui::Separator();

        const char *contacts[] = {"start", "end"};
        for (int i = 0; i < (int)m_Junctions.size(); ++i)
        {
            EditJunction &j = m_Junctions[i];
            ImGui::PushID(i);
            ImGui::Text("Junction %d", j.id);
            ImGui::SameLine();
            bool erased = ImGui::SmallButton("Delete");
            if (!erased)
            {
                if (ImGui::SmallButton("Add Connection"))
                    j.connections.emplace_back();
                for (int ci = 0; ci < (int)j.connections.size(); ++ci)
                {
                    EditConnection &c = j.connections[ci];
                    ImGui::PushID(ci);
                    ImGui::SetNextItemWidth(70.0f);
                    ImGui::InputInt("in", &c.incomingRoad, 0, 0);
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(70.0f);
                    ImGui::InputInt("conn", &c.connectingRoad, 0, 0);
                    ImGui::SameLine();
                    int contactIdx = c.contact == EditContactPoint::End ? 1 : 0;
                    ImGui::SetNextItemWidth(70.0f);
                    if (ImGui::Combo("ct", &contactIdx, contacts, 2))
                        c.contact = contactIdx == 1 ? EditContactPoint::End : EditContactPoint::Start;
                    ImGui::SameLine();
                    bool connErased = ImGui::SmallButton("X");

                    if (ImGui::SmallButton("Add LaneLink"))
                        c.laneLinks.push_back({});
                    for (int li = 0; li < (int)c.laneLinks.size(); ++li)
                    {
                        EditLaneLink &ll = c.laneLinks[li];
                        ImGui::PushID(li);
                        ImGui::SetNextItemWidth(60.0f);
                        ImGui::InputInt("from", &ll.from, 0, 0);
                        ImGui::SameLine();
                        ImGui::SetNextItemWidth(60.0f);
                        ImGui::InputInt("to", &ll.to, 0, 0);
                        ImGui::SameLine();
                        bool llErased = ImGui::SmallButton("x");
                        ImGui::PopID();
                        if (llErased)
                        {
                            c.laneLinks.erase(c.laneLinks.begin() + li);
                            break;
                        }
                    }
                    ImGui::Separator();
                    ImGui::PopID();
                    if (connErased)
                    {
                        j.connections.erase(j.connections.begin() + ci);
                        break;
                    }
                }
            }
            ImGui::Separator();
            ImGui::PopID();
            if (erased)
            {
                m_Junctions.erase(m_Junctions.begin() + i);
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
        static const char *typeNames[] = {"unknown", "park", "park_spot", "traffic_light"};
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

        ImGui::Separator();
        ImGui::Text("Governed Roads (traffic_light only)");
        int eraseRoadIdx = -1;
        for (int i = 0; i < (int)node.roads.size(); ++i)
        {
            ImGui::PushID(i);
            ImGui::SetNextItemWidth(100.0f);
            ImGui::InputInt("##road", &node.roads[i]);
            ImGui::SameLine();
            if (ImGui::SmallButton("X"))
                eraseRoadIdx = i;
            ImGui::PopID();
        }
        if (eraseRoadIdx >= 0)
            node.roads.erase(node.roads.begin() + eraseRoadIdx);
        if (ImGui::Button("Add Road##signal"))
            node.roads.push_back(0);

        ImGui::Separator();
        ImGui::Text("Movements (traffic_light only; empty = blocks every movement out of Governed Roads)");
        {
            // Governed Roads(접근도로)를 incomingRoad로 갖는 junction connection들의 connectingRoad가 후보.
            std::vector<int> candidates;
            for (int approachId : node.roads)
            {
                for (const EditJunction &junction : m_Junctions)
                {
                    for (const EditConnection &conn : junction.connections)
                    {
                        if (conn.incomingRoad == approachId &&
                            std::find(candidates.begin(), candidates.end(), conn.connectingRoad) == candidates.end())
                            candidates.push_back(conn.connectingRoad);
                    }
                }
            }

            if (candidates.empty())
                ImGui::TextDisabled("(Governed Roads를 incoming_road로 갖는 junction connection이 없음)");

            for (int connectingId : candidates)
            {
                bool checked = std::find(node.movements.begin(), node.movements.end(), connectingId) != node.movements.end();
                std::string label = "-> road " + std::to_string(connectingId);
                ImGui::PushID(connectingId);
                if (ImGui::Checkbox(label.c_str(), &checked))
                {
                    if (checked)
                        node.movements.push_back(connectingId);
                    else
                        node.movements.erase(std::remove(node.movements.begin(), node.movements.end(), connectingId),
                                             node.movements.end());
                }
                ImGui::PopID();
            }
        }

        ImGui::Separator();
        ImGui::DragFloat("Phase Offset (traffic_light only)", &node.phaseOffset, 0.5f, 0.0f, 0.0f, "%.1f");
        ImGui::TextDisabled("(seconds; same value on multiple signals = in sync)");
        ImGui::DragFloat("Green (s)", &node.greenDuration, 0.5f, 0.1f, 300.0f, "%.1f");
        ImGui::DragFloat("Yellow (s)", &node.yellowDuration, 0.5f, 0.0f, 300.0f, "%.1f");
        ImGui::DragFloat("Red (s)", &node.redDuration, 0.5f, 0.1f, 300.0f, "%.1f");
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

void EditApp::DrawObstacleListWindow()
{
    if (ImGui::Begin("Obstacles"))
    {
        if (ImGui::Button("Add Obstacle"))
        {
            EditObstacle obstacle;
            obstacle.id = m_NextObstacleId++;
            m_Obstacles.push_back(obstacle);
            m_Selection = Selection::Obstacle;
            m_SelectedObstacle = (int)m_Obstacles.size() - 1;
        }

        ImGui::Separator();

        for (int i = 0; i < (int)m_Obstacles.size(); ++i)
        {
            char label[64];
            snprintf(label, sizeof(label), "Obstacle %d", m_Obstacles[i].id);
            bool selected = (m_Selection == Selection::Obstacle && i == m_SelectedObstacle);

            ImGui::PushID(i);
            float avail = ImGui::GetContentRegionAvail().x;
            if (ImGui::Selectable(label, selected, 0, ImVec2(avail - 28.0f, 0.0f)))
            {
                m_Selection = Selection::Obstacle;
                m_SelectedObstacle = i;
            }
            ImGui::SameLine();
            bool erased = ImGui::SmallButton("X");
            ImGui::PopID();

            if (erased)
            {
                m_Obstacles.erase(m_Obstacles.begin() + i);
                if (m_Selection == Selection::Obstacle)
                {
                    if (m_SelectedObstacle == i)
                    {
                        m_Selection = Selection::None;
                        m_SelectedObstacle = -1;
                    }
                    else if (m_SelectedObstacle > i)
                    {
                        --m_SelectedObstacle;
                    }
                }
                break;
            }
        }
    }
    ImGui::End();
}

void EditApp::DrawObstacleEditWindow()
{
    if (m_SelectedObstacle < 0 || m_SelectedObstacle >= (int)m_Obstacles.size())
    {
        m_Selection = Selection::None;
        return;
    }

    EditObstacle &obstacle = m_Obstacles[m_SelectedObstacle];
    bool open = true;
    if (ImGui::Begin("Obstacle Edit", &open))
    {
        ImGui::Text("Obstacle ID: %d", obstacle.id);
        float pos[3] = {obstacle.position.x, obstacle.position.y, obstacle.position.z};
        if (ImGui::InputFloat3("Position", pos))
            obstacle.position = XMFLOAT3(pos[0], pos[1], pos[2]);
        ImGui::TextDisabled("(or drag the blue sphere)");

        ImGui::DragFloat("Length (heading dir)", &obstacle.length, 0.1f, 0.1f, 100.0f, "%.2f");
        ImGui::DragFloat("Width", &obstacle.width, 0.1f, 0.1f, 100.0f, "%.2f");
        ImGui::DragFloat("Rotation (deg)", &obstacle.rotation, 1.0f, -180.0f, 180.0f, "%.1f");
        ImGui::TextDisabled("(0deg = +X, same atan2(z,x) convention as ReedsShepp)");
    }
    ImGui::End();

    if (!open)
        m_Selection = Selection::None;
}

void EditApp::DrawDynamicObstacleListWindow()
{
    if (ImGui::Begin("Dynamic Obstacles"))
    {
        if (ImGui::Button("Add Dynamic Obstacle"))
        {
            EditDynamicObstacle obstacle;
            obstacle.id = m_NextDynamicObstacleId++;
            m_DynamicObstacles.push_back(obstacle);
            m_Selection = Selection::DynamicObstacle;
            m_SelectedDynamicObstacle = (int)m_DynamicObstacles.size() - 1;
        }

        ImGui::Separator();

        for (int i = 0; i < (int)m_DynamicObstacles.size(); ++i)
        {
            char label[64];
            snprintf(label, sizeof(label), "Dynamic Obstacle %d", m_DynamicObstacles[i].id);
            bool selected = (m_Selection == Selection::DynamicObstacle && i == m_SelectedDynamicObstacle);

            ImGui::PushID(i);
            float avail = ImGui::GetContentRegionAvail().x;
            if (ImGui::Selectable(label, selected, 0, ImVec2(avail - 28.0f, 0.0f)))
            {
                m_Selection = Selection::DynamicObstacle;
                m_SelectedDynamicObstacle = i;
            }
            ImGui::SameLine();
            bool erased = ImGui::SmallButton("X");
            ImGui::PopID();

            if (erased)
            {
                m_DynamicObstacles.erase(m_DynamicObstacles.begin() + i);
                if (m_Selection == Selection::DynamicObstacle)
                {
                    if (m_SelectedDynamicObstacle == i)
                    {
                        m_Selection = Selection::None;
                        m_SelectedDynamicObstacle = -1;
                    }
                    else if (m_SelectedDynamicObstacle > i)
                    {
                        --m_SelectedDynamicObstacle;
                    }
                }
                break;
            }
        }
    }
    ImGui::End();
}

void EditApp::DrawDynamicObstacleEditWindow()
{
    if (m_SelectedDynamicObstacle < 0 || m_SelectedDynamicObstacle >= (int)m_DynamicObstacles.size())
    {
        m_Selection = Selection::None;
        return;
    }

    EditDynamicObstacle &obstacle = m_DynamicObstacles[m_SelectedDynamicObstacle];
    bool open = true;
    if (ImGui::Begin("Dynamic Obstacle Edit", &open))
    {
        ImGui::Text("Dynamic Obstacle ID: %d", obstacle.id);
        float start[3] = {obstacle.start.x, obstacle.start.y, obstacle.start.z};
        if (ImGui::InputFloat3("Start", start))
            obstacle.start = XMFLOAT3(start[0], start[1], start[2]);
        float end[3] = {obstacle.end.x, obstacle.end.y, obstacle.end.z};
        if (ImGui::InputFloat3("End", end))
            obstacle.end = XMFLOAT3(end[0], end[1], end[2]);
        ImGui::TextDisabled("(or drag the orange spheres)");

        ImGui::DragFloat("Length (heading dir)", &obstacle.length, 0.1f, 0.1f, 100.0f, "%.2f");
        ImGui::DragFloat("Width", &obstacle.width, 0.1f, 0.1f, 100.0f, "%.2f");
        ImGui::DragFloat("Speed (m/s)", &obstacle.speed, 0.1f, 0.0f, 50.0f, "%.2f");
        ImGui::TextDisabled("Patrols start<->end at constant speed; heading follows travel direction.");
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
    for (auto &ro : m_RoadRenders)
        ro.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    for (auto &obstacle : m_ObstacleRenders)
        obstacle.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    for (auto &ro : m_DynamicObstacleRenders)
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
    for (auto &ro : m_DynamicObstaclePathRenders)
        ro.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    m_BasicEffect.SetRenderDefault();

    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

    HR(m_pSwapChain->Present(0, m_IsDxgiFlipModel ? DXGI_PRESENT_ALLOW_TEARING : 0));
}
