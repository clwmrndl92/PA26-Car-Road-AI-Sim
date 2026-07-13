#include "CarSim.h"
#include <XUtil.h>
#include <DXTrace.h>
#include <Geometry.h>
#include <Entities/Car.h>
#include "DebugConsole.h"
#include "Nav/Spline.h"

using namespace DirectX;

CarSim::CarSim(HINSTANCE hInstance, const std::wstring &windowName, int initWidth, int initHeight)
    : GameApp(hInstance, windowName, initWidth, initHeight)
{
}

CarSim::~CarSim()
{
}

bool CarSim::Init()
{
    if (!GameApp::Init())
        return false;

    m_RoadDataManager.Init(NAV_DATA_DIR "/data.json");
    m_MarkingDataManager.Init(NAV_DATA_DIR "/marking.json");

    if (!InitResource())
        return false;

    return true;
}

void CarSim::FocusOnObject(const std::shared_ptr<Car> &obj)
{
    m_PickedObjectName = obj->GetName();
    m_pPickedObject = obj;
    XMFLOAT3 target = obj->GetBoundingBox().Center;

    auto newCam = std::make_shared<FocusCamera>();
    newCam->SetViewPort(0.0f, 0.0f, (float)m_ClientWidth, (float)m_ClientHeight);
    newCam->SetFrustum(XM_PI / 3, AspectRatio(), 1.0f, 1000.0f);
    newCam->SetTarget(target);
    newCam->SetDistance(15.0f);
    newCam->SetDistanceMinMax(3.0f, 100.0f);
    newCam->SetRotationX(XM_PIDIV4);
    m_pCamera = newCam;
    m_CameraMode = CameraMode::Focus;

    m_BasicEffect.SetProjMatrix(m_pCamera->GetProjMatrixXM());
}

void CarSim::UpdateCamera(float dt)
{
    auto pickedObj = m_pPickedObject.lock();
    for (auto &car : m_CarObjects)
    {
        car->SetFocused(car == pickedObj);
    }
    if (auto picked = m_pPickedObject.lock())
    {
        if (auto cam3rd = std::dynamic_pointer_cast<FocusCamera>(m_pCamera))
            cam3rd->SetTarget(picked->GetBoundingBox().Center);
    }

    auto cam3rd = std::dynamic_pointer_cast<FocusCamera>(m_pCamera);
    auto cam1st = std::dynamic_pointer_cast<FreeCamera>(m_pCamera);

    ImGuiIO &io = ImGui::GetIO();

    // Left click: pick object or switch to free camera
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse)
    {
        Ray ray = Ray::ScreenToRay(*m_pCamera, io.MousePos.x, io.MousePos.y);

        float distObj = FLT_MAX;
        std::shared_ptr<Car> hitObj;
        for (auto &obj : m_CarObjects)
        {
            float d = FLT_MAX;
            if (ray.Hit(obj->GetBoundingBox(), &d) && d < distObj)
            {
                distObj = d;
                hitObj = obj;
            }
        }

        if (hitObj)
        {
            FocusOnObject(hitObj);
        }
        else
        {
            m_PickedObjectName = "";
            m_pPickedObject.reset();
            // Switch to free camera from current position
            if (m_CameraMode != CameraMode::Free)
            {
                auto newCam = std::make_shared<FreeCamera>();
                newCam->SetViewPort(0.0f, 0.0f, (float)m_ClientWidth, (float)m_ClientHeight);
                newCam->SetFrustum(XM_PI / 3, AspectRatio(), 1.0f, 1000.0f);
                newCam->LookTo(m_pCamera->GetPosition(), m_pCamera->GetLookAxis(), XMFLOAT3(0.0f, 1.0f, 0.0f));
                m_pCamera = newCam;
                m_CameraMode = CameraMode::Free;
                m_BasicEffect.SetProjMatrix(m_pCamera->GetProjMatrixXM());
            }
        }
    }

    if (m_CameraMode == CameraMode::Focus && cam3rd)
    {
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Right))
        {
            cam3rd->RotateX(io.MouseDelta.y * 0.01f);
            cam3rd->RotateY(io.MouseDelta.x * 0.01f);
        }
        cam3rd->Approach(-io.MouseWheel * 1.0f);
    }
    else if (m_CameraMode == CameraMode::Free && cam1st)
    {
        XMFLOAT3 forward = cam1st->GetLookAxis();
        forward.y = 0.0f;
        XMFLOAT3 right = cam1st->GetRightAxis();
        right.y = 0.0f;

        if (ImGui::IsKeyDown(ImGuiKey_W))
            cam1st->Translate(forward, dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_S))
            cam1st->Translate(forward, -dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_A))
            cam1st->Translate(right, -dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_D))
            cam1st->Translate(right, dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_Q))
            cam1st->Translate(XMFLOAT3(0, -1, 0), dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_E))
            cam1st->Translate(XMFLOAT3(0, 1, 0), dt * 10.0f);

        if (ImGui::IsMouseDragging(ImGuiMouseButton_Right))
        {
            cam1st->Pitch(io.MouseDelta.y * 0.01f);
            cam1st->RotateY(io.MouseDelta.x * 0.01f);
        }

        if (io.MouseWheel != 0.0f)
            cam1st->Translate(cam1st->GetLookAxis(), io.MouseWheel * 2.0f);
    }
}
void CarSim::UpdateUI(float dt)
{
    GameApp::UpdateUI(dt);

    ImGui::SetNextWindowPos(ImVec2(0.0f, 100.0f), ImGuiCond_Always);
    if (ImGui::Begin("Objects"))
    {
        ImGui::Text("Mode : %s", m_CameraMode == CameraMode::Free ? "Free Camera" : "Focus Camera");

        if (m_PickedObjectName.empty())
            ImGui::Text("Picked: (none)");
        else
            ImGui::Text("Picked: %s", m_PickedObjectName.c_str());

        ImGui::Separator();
        for (auto &obj : m_CarObjects)
        {
            bool isSelected = (obj == m_pPickedObject.lock());
            if (ImGui::Selectable(obj->GetName().c_str(), isSelected))
                FocusOnObject(obj);
        }
    }
    ImGui::End();

    DebugConsole::Get().Draw();
}
void CarSim::DrawScene()
{
    // Create render target view for the back buffer
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
    for (auto &roadRender : m_RoadRenders)
        roadRender.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    for (auto &markingRender : m_MarkingRenders)
        markingRender.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);

    m_BasicEffect.SetRenderLines();
    if (m_ShowGridXZ)
        m_GridXZ.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    if (m_ShowGridXY)
        m_GridXY.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    if (m_ShowGridYZ)
        m_GridYZ.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    for (auto &edgeRender : m_RoadEdgeRenders)
        edgeRender.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    m_BasicEffect.SetRenderDefault();

    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

    HR(m_pSwapChain->Present(0, m_IsDxgiFlipModel ? DXGI_PRESENT_ALLOW_TEARING : 0));
}

bool CarSim::InitResource()
{
    // ******************
    // Initialize game objects
    //
    // Road
    {
        constexpr float ROAD_SIZE = 2000.0f;

        auto road = std::make_shared<GameObject>();
        road->SetName("Ground");
        Model *pGround = m_ModelManager.CreateFromGeometry("ground", Geometry::CreatePlane(ROAD_SIZE, ROAD_SIZE));
        // Model* pGround = m_ModelManager.CreateFromGeometry("road_ground", Geometry::CreatePlane(10.0f, ROAD_SIZE));
        pGround->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.7f, 0.8f, 0.6f, 1.0f));
        pGround->materials[0].Set<float>("$Opacity", 1.0f);
        road->SetModel(pGround);
        road->SetPosition(Vec3(0.0f, -0.01f, 0.0f));
        road->Init(JPH::Vec3(ROAD_SIZE * 0.5f, 0.05f, ROAD_SIZE * 0.5f), Rigidbody::Type::Static);
        m_GameObjects.push_back(road);

        InitRoadRenderer();
        InitMarkingRenderer();
    }

    // Car 1
    {
        auto car = std::make_shared<Car>();
        car->Init(GetCarSpec(CarType::Car0), &m_RoadDataManager, JPH::Vec3(-10.0f, 0.1f, -10.0f));
        car->SetDrawCollider(true);
        car->SetDestination(m_RoadDataManager.GetClosestLaneEnd(m_RoadDataManager.GetNode(1)->position));

        m_GameObjects.push_back(car);
        m_CarObjects.push_back(car);
    }

    // // Car 2
    // {
    //     auto car = std::make_shared<Car>();
    //     car->Init(GetCarSpec(CarType::Car1), &m_RoadDataManager, JPH::Vec3(-2.0f, 0.1f, 0.0f));
    //     car->SetDrawCollider(true);
    //     m_GameObjects.push_back(car);
    // }

    return true;
}

void CarSim::InitCamera()
{
    auto camera = std::make_shared<FocusCamera>();
    m_pCamera = camera;
    m_CameraMode = CameraMode::Focus;

    camera->SetViewPort(0.0f, 0.0f, (float)m_ClientWidth, (float)m_ClientHeight);
    camera->SetTarget(XMFLOAT3(0.0f, 0.5f, 0.0f));
    camera->SetDistance(15.0f);
    camera->SetDistanceMinMax(6.0f, 100.0f);
    camera->SetRotationX(XM_PIDIV4);
    camera->SetFrustum(XM_PI / 3, AspectRatio(), 1.0f, 1000.0f);

    m_BasicEffect.SetWorldMatrix(XMMatrixIdentity());
    m_BasicEffect.SetViewMatrix(camera->GetViewMatrixXM());
    m_BasicEffect.SetProjMatrix(camera->GetProjMatrixXM());
    m_BasicEffect.SetEyePos(camera->GetPosition());
}
void CarSim::InitRoadRenderer()
{
    constexpr float ROAD_WIDTH = 3.0f;
    constexpr float NODE_MARKER_RADIUS = 0.5f;
    constexpr float EDGE_LINE_HEIGHT = 0.1f;

    m_RoadRenders.clear();

    for (const auto &node : m_RoadDataManager.GetNodes())
    {
        Model *pMarker = m_ModelManager.CreateFromGeometry("node_marker" + std::to_string(node->id), Geometry::CreateSphere(NODE_MARKER_RADIUS));
        pMarker->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f));
        pMarker->materials[0].Set<float>("$Opacity", 1.0f);

        RenderObject &nodeRender = m_RoadRenders.emplace_back();
        nodeRender.SetModel(pMarker);
        nodeRender.GetTransform().SetPosition(ToXMFLOAT3(node->position));
    }

    // 레인 그래프의 successor 연결(레인 끝 -> 다음 레인 시작)을 노란 선으로 시각화한다.
    m_RoadEdgeRenders.clear();
    int linkIndex = 0;
    for (const auto &lane : m_RoadDataManager.GetLanes())
    {
        Vec3 from = lane->GetEndPoint() + Vec3(0.0f, EDGE_LINE_HEIGHT, 0.0f);
        for (const auto &weakSucc : lane->GetSuccessors())
        {
            auto succ = weakSucc.lock();
            if (!succ)
                continue;

            Vec3 to = succ->GetStartPoint() + Vec3(0.0f, EDGE_LINE_HEIGHT, 0.0f);

            Model *pLine = m_ModelManager.CreateFromGeometry("edge_line" + std::to_string(linkIndex++), Geometry::CreateLine(ToXMFLOAT3(from), ToXMFLOAT3(to)));
            pLine->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 1.0f, 0.0f, 1.0f));
            pLine->materials[0].Set<float>("$Opacity", 1.0f);

            RenderObject &edgeRender = m_RoadEdgeRenders.emplace_back();
            edgeRender.SetModel(pLine);
        }
    }
}

void CarSim::InitMarkingRenderer()
{
    // Mirrors EditApp::RebuildRenderObjects' marking loop so the sim renders exactly what was
    // authored: >=4 points get a Catmull-Rom spline centerline, 2-3 points are used as-is.
    m_MarkingRenders.clear();
    for (const auto &marking : m_MarkingDataManager.GetMarkings())
    {
        std::vector<XMFLOAT3> samples;
        if (marking.points.size() >= 4)
        {
            Spline spline(marking.points);
            for (const Vec3 &s : spline.GetSplinePoints())
                samples.push_back(ToXMFLOAT3(s));
        }
        else
        {
            samples.reserve(marking.points.size());
            for (const Vec3 &p : marking.points)
                samples.push_back(ToXMFLOAT3(p));
        }
        if (samples.size() < 2)
            continue;

        for (XMFLOAT3 &s : samples)
            s.y += 0.05f; // lift slightly above the road ribbon to avoid z-fighting

        GeometryData geo = marking.type == MarkingLineType::Dashed
                               ? Geometry::CreateDashedRibbon(samples, marking.width, marking.dashLength, marking.dashGap)
                               : Geometry::CreateRibbon(samples, marking.width);
        if (geo.vertices.empty())
            continue;

        Model *pMarking = m_ModelManager.CreateFromGeometry("marking_" + std::to_string(marking.id), geo);
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
}