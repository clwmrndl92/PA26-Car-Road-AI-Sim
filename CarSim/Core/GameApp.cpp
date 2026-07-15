#include "GameApp.h"
#include <XUtil.h>
#include <DXTrace.h>
#include "Car/Car.h"
#include "Utill/DebugConsole.h"

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

    m_TextureManager.Init(m_pd3dDevice.Get());
    m_ModelManager.Init(m_pd3dDevice.Get());

    m_Physics.Init();

    RenderStates::InitAll(m_pd3dDevice.Get());

    if (!m_BasicEffect.InitAll(m_pd3dDevice.Get()))
        return false;

    InitResource();
    InitCamera();
    InitLight();
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

void GameApp::UpdateScene(float dt)
{
    // Step 1. Update game objects
    for (auto &obj : m_GameObjects)
    {
        obj->Update(dt);
    }

    // Step 2. Calculate physics System
    m_Physics.Update(dt);

    // Step 3. Update RenderObject
    for (auto &obj : m_GameObjects)
        obj->UpdateRender();

    // Step 4. Update camera
    UpdateCamera(dt);

    // Step 5. Set UI Window
    UpdateUI(dt);
    ImGui::Render();

    m_BasicEffect.SetViewMatrix(m_pCamera->GetViewMatrixXM());
    m_BasicEffect.SetEyePos(m_pCamera->GetPosition());
}

void GameApp::UpdateCamera(float dt)
{
    auto cam1st = std::dynamic_pointer_cast<FreeCamera>(m_pCamera);

    ImGuiIO &io = ImGui::GetIO();

    if (cam1st)
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
}
void GameApp::UpdateUI(float dt)
{
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

    return true;
}

void GameApp::InitLight()
{
    DirectionalLight dirLight{};
    dirLight.ambient = XMFLOAT4(0.5f, 0.5f, 0.5f, 1.0f);
    dirLight.diffuse = XMFLOAT4(0.8f, 0.8f, 0.8f, 1.0f);
    dirLight.specular = XMFLOAT4(0.5f, 0.5f, 0.5f, 1.0f);
    dirLight.direction = XMFLOAT3(0.0f, -1.0f, 0.0f);
    m_BasicEffect.SetDirLight(0, dirLight);
}
void GameApp::InitCamera()
{
    auto camera = std::make_shared<FreeCamera>();
    m_pCamera = camera;

    camera->SetViewPort(0.0f, 0.0f, (float)m_ClientWidth, (float)m_ClientHeight);
    camera->SetFrustum(XM_PI / 3, AspectRatio(), 1.0f, 1000.0f);

    m_BasicEffect.SetWorldMatrix(XMMatrixIdentity());
    m_BasicEffect.SetViewMatrix(camera->GetViewMatrixXM());
    m_BasicEffect.SetProjMatrix(camera->GetProjMatrixXM());
    m_BasicEffect.SetEyePos(camera->GetPosition());
}