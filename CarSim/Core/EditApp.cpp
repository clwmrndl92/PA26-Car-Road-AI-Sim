#include "EditApp.h"
#include <XUtil.h>
#include <DXTrace.h>

using namespace DirectX;

EditApp::EditApp(HINSTANCE hInstance, const std::wstring &windowName, int initWidth, int initHeight)
    : D3DApp(hInstance, windowName, initWidth, initHeight)
{
}

EditApp::~EditApp()
{
    m_GameObjects.clear();
    m_Physics.Shutdown();
}

bool EditApp::Init()
{
    if (!D3DApp::Init())
        return false;

    m_Physics.Init();

    m_TextureManager.Init(m_pd3dDevice.Get());
    m_ModelManager.Init(m_pd3dDevice.Get());

    RenderStates::InitAll(m_pd3dDevice.Get());

    if (!m_BasicEffect.InitAll(m_pd3dDevice.Get()))
        return false;

    if (!InitResource())
        return false;

    return true;
}

void EditApp::OnResize()
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

void EditApp::UpdateScene(float dt)
{
    // Step 1. Update game objects
    for (auto &obj : m_GameObjects)
        obj->Update(dt);

    // Step 2. Calculate physics System
    m_Physics.Update(dt);

    // Step 3. Update RenderObject
    for (auto &obj : m_GameObjects)
        obj->UpdateRender();

    // Step 4. Update top-down camera: mouse wheel zooms (height), WASD pans along X/Z.
    // Camera direction is fixed (straight down), so it never rotates.
    ImGuiIO &io = ImGui::GetIO();

    constexpr float PAN_SPEED = 20.0f;
    constexpr float ZOOM_SPEED = 5.0f;

    XMFLOAT3 pos = m_pCamera->GetPosition();

    if (ImGui::IsKeyDown(ImGuiKey_W))
        pos.z += dt * PAN_SPEED;
    if (ImGui::IsKeyDown(ImGuiKey_S))
        pos.z -= dt * PAN_SPEED;
    if (ImGui::IsKeyDown(ImGuiKey_A))
        pos.x -= dt * PAN_SPEED;
    if (ImGui::IsKeyDown(ImGuiKey_D))
        pos.x += dt * PAN_SPEED;

    if (!io.WantCaptureMouse && io.MouseWheel != 0.0f)
    {
        pos.y -= io.MouseWheel * ZOOM_SPEED;
        if (pos.y < m_TopDownHeightMin)
            pos.y = m_TopDownHeightMin;
        else if (pos.y > m_TopDownHeightMax)
            pos.y = m_TopDownHeightMax;
    }

    m_pCamera->SetPosition(pos);

    // Step 4.5. Left click on the ground plane (y = 0) adds a spline control point
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse)
    {
        Ray ray = Ray::ScreenToRay(*m_pCamera, io.MousePos.x, io.MousePos.y);
        if (ray.direction.y != 0.0f)
        {
            float t = -ray.origin.y / ray.direction.y;
            if (t > 0.0f)
            {
                float hitX = ray.origin.x + ray.direction.x * t;
                float hitZ = ray.origin.z + ray.direction.z * t;

                m_SplineControlPoints.push_back(Vec3(hitX, 0.0f, hitZ));
                m_Spline = Spline(m_SplineControlPoints);

                RenderObject &marker = m_SplineMarkers.emplace_back();
                marker.SetModel(m_pSplineMarkerModel);
                marker.GetTransform().SetPosition(hitX, 0.02f, hitZ);

                UpdateSplineRender(m_Spline);
                UpdateRoadRender(m_Spline);
            }
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

    ImGui::Render();

    m_BasicEffect.SetViewMatrix(m_pCamera->GetViewMatrixXM());
    m_BasicEffect.SetEyePos(m_pCamera->GetPosition());
}

void EditApp::DrawScene()
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
    for (auto &marker : m_SplineMarkers)
        marker.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);

    m_BasicEffect.SetRenderLines();
    if (m_ShowGridXZ)
        m_GridXZ.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    if (m_ShowGridXY)
        m_GridXY.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    if (m_ShowGridYZ)
        m_GridYZ.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    if (m_SplineCurveVisible)
        m_SplineCurve.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    m_BasicEffect.SetRenderDefault();

    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

    HR(m_pSwapChain->Present(0, m_IsDxgiFlipModel ? DXGI_PRESENT_ALLOW_TEARING : 0));
}

bool EditApp::InitResource()
{
    // m_Spline = DataParser::ParseSplineData(NAV_DATA_DIR "/data.json");
    // ******************
    // Initialize game objects
    //
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

    // Spline control point marker
    {
        constexpr float MARKER_SIZE = 0.5f;

        Model *pMarker = m_ModelManager.CreateFromGeometry("spline_marker", Geometry::CreatePlane(MARKER_SIZE, MARKER_SIZE));
        pMarker->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f));
        pMarker->materials[0].Set<float>("$Opacity", 1.0f);
        m_pSplineMarkerModel = pMarker;
    }

    // Markers for control points loaded from data.json
    for (const Vec3 &p : m_Spline.GetControlPoints())
    {
        RenderObject &marker = m_SplineMarkers.emplace_back();
        marker.SetModel(m_pSplineMarkerModel);
        marker.GetTransform().SetPosition(p.GetX(), 0.02f, p.GetZ());
    }

    UpdateSplineRender(m_Spline);
    UpdateRoadRender(m_Spline);

    // ******************
    // Initialize camera
    //

    auto camera = std::make_shared<FreeCamera>();
    m_pCamera = camera;

    camera->SetViewPort(0.0f, 0.0f, (float)m_ClientWidth, (float)m_ClientHeight);
    camera->SetFrustum(XM_PI / 3, AspectRatio(), 1.0f, 1000.0f);
    // Fixed top-down view looking straight at the origin from above; only position (pan/zoom) changes afterwards.
    camera->LookTo(XMFLOAT3(0.0f, 15.0f, 0.0f), XMFLOAT3(0.0f, -1.0f, 0.0f), XMFLOAT3(0.0f, 0.0f, 1.0f));

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
    // Point light
    PointLight pointLight{};
    pointLight.position = XMFLOAT3(0.0f, 20.0f, 0.0f);
    pointLight.ambient = XMFLOAT4(0.3f, 0.3f, 0.3f, 1.0f);
    pointLight.diffuse = XMFLOAT4(0.7f, 0.7f, 0.7f, 1.0f);
    pointLight.specular = XMFLOAT4(0.2f, 0.2f, 0.2f, 1.0f);
    pointLight.att = XMFLOAT3(0.0f, 0.1f, 0.0f);
    pointLight.range = 30.0f;
    m_BasicEffect.SetPointLight(0, pointLight);

    return true;
}

void EditApp::UpdateSplineRender(const Spline &spline)
{
    if (spline.GetControlPointCount() < 4)
        return;

    std::vector<Vec3> splinePoints = spline.GetSplinePoints();
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
    m_SplineCurve.SetModel(pCurveModel);
    m_SplineCurveVisible = true;
}

void EditApp::UpdateRoadRender(const Spline &spline)
{
    const auto &points = spline.GetControlPoints();
    if (points.size() < 2)
        return;

    constexpr float ROAD_WIDTH = 3.0f;

    // Spline::GenerateSplinePoints() only draws between its interior control points (see
    // Spline.cpp), so duplicating the first/last point as phantom endpoints makes the curve
    // actually reach the first and last clicked points instead of stopping short of them.

    std::vector<Vec3> splinePoints = spline.GetSplinePoints();

    std::vector<XMFLOAT3> centerline;
    centerline.reserve(splinePoints.size());
    for (const Vec3 &p : splinePoints)
        centerline.push_back(ToXMFLOAT3(p));

    Model *pGround = m_ModelManager.CreateFromGeometry("road_ground", Geometry::CreateRibbon(centerline, ROAD_WIDTH));
    pGround->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.22f, 0.22f, 0.22f, 1.0f));
    pGround->materials[0].Set<float>("$Opacity", 1.0f);

    m_RoadRenders.clear();
    RenderObject &roadRender = m_RoadRenders.emplace_back();
    roadRender.SetModel(pGround);
}
