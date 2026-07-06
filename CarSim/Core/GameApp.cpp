#include "GameApp.h"
#include <XUtil.h>
#include <DXTrace.h>
#include <Entities/Car.h>
#include <Nav/DataParser.h>
#include "DebugConsole.h"

using namespace DirectX;

GameApp::GameApp(HINSTANCE hInstance, const std::wstring &windowName, int initWidth, int initHeight)
    : D3DApp(hInstance, windowName, initWidth, initHeight)
{
}

GameApp::~GameApp()
{
    m_GameObjects.clear();
    m_Physics.Shutdown();
}

bool GameApp::Init()
{
    if (!D3DApp::Init())
        return false;

    m_Physics.Init();

    m_TextureManager.Init(m_pd3dDevice.Get());
    m_ModelManager.Init(m_pd3dDevice.Get());
    m_RoadDataManager.Init(NAV_DATA_DIR "/data.json");

    RenderStates::InitAll(m_pd3dDevice.Get());

    if (!m_BasicEffect.InitAll(m_pd3dDevice.Get()))
        return false;

    if (!InitResource())
        return false;

    return true;
}

void GameApp::OnResize()
{
    D3DApp::OnResize();

    m_pDepthTexture = std::make_unique<Depth2D>(m_pd3dDevice.Get(), m_ClientWidth, m_ClientHeight);
    m_pDepthTexture->SetDebugObjectName("DepthTexture");

    if (m_pCamera != nullptr)
    {
        m_pCamera->SetFrustum(XM_PI / 3, AspectRatio(), 1.0f, 1000.0f);
        m_pCamera->SetViewPort(0.0f, 0.0f, (float)m_ClientWidth, (float)m_ClientHeight);
        m_BasicEffect.SetProjMatrix(m_pCamera->GetProjMatrixXM());
    }
}

void GameApp::FocusOnObject(const std::shared_ptr<GameObject> &obj)
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
}

void GameApp::UpdateScene(float dt)
{
    // Step 1. Update game objects
    auto pickedObj = m_pPickedObject.lock();
    for (auto &obj : m_GameObjects)
    {
        if (auto car = std::dynamic_pointer_cast<Car>(obj))
            car->SetFocused(obj == pickedObj);
        obj->Update(dt);
    }

    // Step 2. Calculate physics System
    m_Physics.Update(dt);

    // Step 3. Update RenderObject
    for (auto &obj : m_GameObjects)
        obj->UpdateRender();

    // Step 4. Update camera by Input
    if (auto picked = m_pPickedObject.lock())
    {
        if (auto cam3rd = std::dynamic_pointer_cast<FocusCamera>(m_pCamera))
            cam3rd->SetTarget(picked->GetBoundingBox().Center);
    }

    auto cam3rd = std::dynamic_pointer_cast<FocusCamera>(m_pCamera);
    auto cam1st = std::dynamic_pointer_cast<FirstPersonCamera>(m_pCamera);

    ImGuiIO &io = ImGui::GetIO();

    // Left click: pick object or switch to free camera
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse)
    {
        Ray ray = Ray::ScreenToRay(*m_pCamera, io.MousePos.x, io.MousePos.y);

        float distObj = FLT_MAX;
        std::shared_ptr<GameObject> hitObj;
        for (auto &obj : m_GameObjects)
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
                auto newCam = std::make_shared<FirstPersonCamera>();
                newCam->SetViewPort(0.0f, 0.0f, (float)m_ClientWidth, (float)m_ClientHeight);
                newCam->SetFrustum(XM_PI / 3, AspectRatio(), 1.0f, 1000.0f);
                newCam->LookTo(m_pCamera->GetPosition(), m_pCamera->GetLookAxis(), XMFLOAT3(0.0f, 1.0f, 0.0f));
                m_pCamera = newCam;
                m_CameraMode = CameraMode::Free;
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
        if (ImGui::IsKeyDown(ImGuiKey_W))
            cam1st->MoveForward(dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_S))
            cam1st->MoveForward(-dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_A))
            cam1st->Strafe(-dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_D))
            cam1st->Strafe(dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_Q))
            cam1st->Translate(XMFLOAT3(0, -1, 0), dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_E))
            cam1st->Translate(XMFLOAT3(0, 1, 0), dt * 10.0f);

        if (ImGui::IsMouseDragging(ImGuiMouseButton_Right))
        {
            cam1st->Pitch(io.MouseDelta.y * 0.01f);
            cam1st->RotateY(io.MouseDelta.x * 0.01f);
        }
    }

    // Step 5. Set UI Window
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
    float debugGridWindowHeight = 0.0f;
    if (ImGui::Begin("Debug Grid"))
    {
        ImGui::Checkbox("XZ Plane", &m_ShowGridXZ);
        ImGui::Checkbox("XY Plane", &m_ShowGridXY);
        ImGui::Checkbox("YZ Plane", &m_ShowGridYZ);
        debugGridWindowHeight = ImGui::GetWindowSize().y;
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(0.0f, debugGridWindowHeight), ImGuiCond_Always);
    if (ImGui::Begin("Objects"))
    {
        ImGui::Text("Mode : %s", m_CameraMode == CameraMode::Free ? "Free Camera" : "Focus Camera");

        if (m_PickedObjectName.empty())
            ImGui::Text("Picked: (none)");
        else
            ImGui::Text("Picked: %s", m_PickedObjectName.c_str());

        ImGui::Separator();
        for (auto &obj : m_GameObjects)
        {
            bool isSelected = (obj->GetName() == m_PickedObjectName);
            if (ImGui::Selectable(obj->GetName().c_str(), isSelected))
                FocusOnObject(obj);
        }
    }
    ImGui::End();

    DebugConsole::Get().Draw();

    ImGui::Render();

    m_BasicEffect.SetViewMatrix(m_pCamera->GetViewMatrixXM());
    m_BasicEffect.SetEyePos(m_pCamera->GetPosition());
}

void GameApp::DrawScene()
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

    m_BasicEffect.SetRenderLines();
    if (m_ShowGridXZ)
        m_GridXZ.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    if (m_ShowGridXY)
        m_GridXY.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    if (m_ShowGridYZ)
        m_GridYZ.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    m_BasicEffect.SetRenderDefault();

    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

    HR(m_pSwapChain->Present(0, m_IsDxgiFlipModel ? DXGI_PRESENT_ALLOW_TEARING : 0));
}

bool GameApp::InitResource()
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

        UpdateSplineRender(m_RoadDataManager.GetSpline());
        UpdateRoadRender(m_RoadDataManager.GetSpline());
    }

    // Debug grid (XZ plane, rotated copies for XY / YZ)
    {
        Model *pGrid = m_ModelManager.CreateFromGeometry("debug_grid", Geometry::CreateLineGrid(500.0f, 1.0f));
        pGrid->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.4f, 0.4f, 0.4f, 1.0f));
        pGrid->materials[0].Set<float>("$Opacity", 1.0f);

        m_GridXZ.SetModel(pGrid);

        XMFLOAT4 rotXY, rotYZ;
        XMStoreFloat4(&rotXY, XMQuaternionRotationAxis(g_XMIdentityR0, XM_PIDIV2)); // XZ -> XY
        XMStoreFloat4(&rotYZ, XMQuaternionRotationAxis(g_XMIdentityR2, XM_PIDIV2)); // XZ -> YZ

        m_GridXY.SetModel(pGrid);
        m_GridXY.GetTransform().SetRotation(rotXY);

        m_GridYZ.SetModel(pGrid);
        m_GridYZ.GetTransform().SetRotation(rotYZ);
    }

    // Car 1
    {
        auto car = std::make_shared<Car>();
        car->Init(GetCarSpec(CarType::Car0), &m_RoadDataManager, JPH::Vec3(0.0f, 0.1f, 0.0f));
        car->SetDrawCollider(true);
        m_GameObjects.push_back(car);
    }

    // // Car 2
    // {
    //     auto car = std::make_shared<Car>();
    //     car->Init(GetCarSpec(CarType::Car1), &m_RoadDataManager, JPH::Vec3(-2.0f, 0.1f, 0.0f));
    //     car->SetDrawCollider(true);
    //     m_GameObjects.push_back(car);
    // }

    // ******************
    // Initialize camera
    //

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

    // ******************
    // Initialize constant values
    //

    // Directional light (ambient)
    DirectionalLight dirLight{};
    dirLight.ambient = XMFLOAT4(0.5f, 0.5f, 0.5f, 1.0f);
    dirLight.diffuse = XMFLOAT4(0.8f, 0.8f, 0.8f, 1.0f);
    dirLight.specular = XMFLOAT4(0.5f, 0.5f, 0.5f, 1.0f);
    dirLight.direction = XMFLOAT3(0.0f, -1.0f, 0.0f);
    m_BasicEffect.SetDirLight(0, dirLight);

    return true;
}

void GameApp::UpdateSplineRender(const Spline &spline)
{
    if (spline.GetControlPointCount() < 4)
        return;

    std::vector<Vec3> splinePoints = spline.GenerateSplinePoints();
    if (splinePoints.empty())
        return;

    // Lift the curve above the road plane (y = 0) so it doesn't z-fight with it.
    constexpr float CURVE_HEIGHT_OFFSET = 0.05f;

    std::vector<XMFLOAT3> renderPoints;
    renderPoints.reserve(splinePoints.size());
    for (const Vec3 &p : splinePoints)
    {
        XMFLOAT3 renderPoint = ToXMFLOAT3(p);
        renderPoint.y += CURVE_HEIGHT_OFFSET;
        renderPoints.push_back(renderPoint);
    }

    Model *pCurveModel = m_ModelManager.CreateFromGeometry("spline_curve", Geometry::CreatePolyline(renderPoints));
    pCurveModel->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 0.8f, 0.0f, 1.0f));
    pCurveModel->materials[0].Set<float>("$Opacity", 1.0f);
}

void GameApp::UpdateRoadRender(const Spline &spline)
{
    const auto &points = spline.GetControlPoints();
    if (points.size() < 2)
        return;

    constexpr float ROAD_WIDTH = 3.0f;

    // Spline::GenerateSplinePoints() only draws between its interior control points (see
    // Spline.cpp), so duplicating the first/last point as phantom endpoints makes the curve
    // actually reach the first and last clicked points instead of stopping short of them.

    std::vector<Vec3> splinePoints = spline.GenerateSplinePoints();

    std::vector<XMFLOAT3> centerline;
    centerline.reserve(splinePoints.size());
    for (const Vec3 &p : splinePoints)
        centerline.push_back(ToXMFLOAT3(p));

    Model *pGround = m_ModelManager.CreateFromGeometry("road", Geometry::CreateRibbon(centerline, ROAD_WIDTH));
    pGround->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.22f, 0.22f, 0.22f, 1.0f));
    pGround->materials[0].Set<float>("$Opacity", 1.0f);

    m_RoadRenders.clear();
    RenderObject &roadRender = m_RoadRenders.emplace_back();
    roadRender.SetModel(pGround);
}