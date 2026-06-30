#include "GameApp.h"
#include <XUtil.h>
#include <DXTrace.h>

using namespace DirectX;

GameApp::GameApp(HINSTANCE hInstance, const std::wstring& windowName, int initWidth, int initHeight)
    : D3DApp(hInstance, windowName, initWidth, initHeight)
{
}

GameApp::~GameApp()
{
}

bool GameApp::Init()
{
    if (!D3DApp::Init())
        return false;

    m_TextureManager.Init(m_pd3dDevice.Get());
    m_ModelManager.Init(m_pd3dDevice.Get());

    // Initialize all render states first before effects
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

void GameApp::UpdateScene(float dt)
{
    auto cam3rd = std::dynamic_pointer_cast<ThirdPersonCamera>(m_pCamera);
    auto cam1st = std::dynamic_pointer_cast<FirstPersonCamera>(m_pCamera);

    ImGuiIO& io = ImGui::GetIO();

    // Left click: pick object or switch to free camera
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse)
    {
        Ray ray = Ray::ScreenToRay(*m_pCamera, io.MousePos.x, io.MousePos.y);

        float distHouse = FLT_MAX, distGround = FLT_MAX;
        bool hitHouse  = ray.Hit(m_House.GetBoundingBox(),  &distHouse);
        bool hitGround = ray.Hit(m_Ground.GetBoundingBox(), &distGround);

        XMFLOAT3 target;
        bool switchToThird = false;

        if (hitHouse && (!hitGround || distHouse <= distGround))
        {
            m_PickedObjectName = "House";
            target = m_House.GetBoundingBox().Center;
            switchToThird = true;
        }
        else if (hitGround)
        {
            m_PickedObjectName = "Ground";
            target = m_Ground.GetBoundingBox().Center;
            switchToThird = true;
        }
        else
        {
            m_PickedObjectName = "";
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

        if (switchToThird)
        {
            auto newCam = std::make_shared<ThirdPersonCamera>();
            newCam->SetViewPort(0.0f, 0.0f, (float)m_ClientWidth, (float)m_ClientHeight);
            newCam->SetFrustum(XM_PI / 3, AspectRatio(), 1.0f, 1000.0f);
            newCam->SetTarget(target);
            newCam->SetDistance(15.0f);
            newCam->SetDistanceMinMax(3.0f, 100.0f);
            newCam->SetRotationX(XM_PIDIV4);
            m_pCamera = newCam;
            m_CameraMode = CameraMode::ThirdPerson;
        }
    }

    if (m_CameraMode == CameraMode::ThirdPerson && cam3rd)
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
        // WASD movement
        if (ImGui::IsKeyDown(ImGuiKey_W)) cam1st->MoveForward(dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_S)) cam1st->MoveForward(-dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_A)) cam1st->Strafe(-dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_D)) cam1st->Strafe(dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_Q)) cam1st->Translate(XMFLOAT3(0, -1, 0), dt * 10.0f);
        if (ImGui::IsKeyDown(ImGuiKey_E)) cam1st->Translate(XMFLOAT3(0, 1, 0), dt * 10.0f);

        if (ImGui::IsMouseDragging(ImGuiMouseButton_Right))
        {
            cam1st->Pitch(io.MouseDelta.y * 0.01f);
            cam1st->RotateY(io.MouseDelta.x * 0.01f);
        }
    }

    if (ImGui::Begin("Camera"))
    {
        int curr_item = (m_CameraMode == CameraMode::Free) ? 1 : 0;
        static const char* modes[] = { "Third Person", "Free Camera" };
        if (ImGui::Combo("Mode", &curr_item, modes, ARRAYSIZE(modes)))
        {
            if (curr_item == 0 && m_CameraMode != CameraMode::ThirdPerson)
            {
                auto newCam = std::make_shared<ThirdPersonCamera>();
                newCam->SetViewPort(0.0f, 0.0f, (float)m_ClientWidth, (float)m_ClientHeight);
                newCam->SetFrustum(XM_PI / 3, AspectRatio(), 1.0f, 1000.0f);
                newCam->SetTarget(XMFLOAT3(0.0f, 0.5f, 0.0f));
                newCam->SetDistance(15.0f);
                newCam->SetDistanceMinMax(6.0f, 100.0f);
                newCam->SetRotationX(XM_PIDIV4);
                m_pCamera = newCam;
                m_CameraMode = CameraMode::ThirdPerson;
            }
            else if (curr_item == 1 && m_CameraMode != CameraMode::Free)
            {
                auto newCam = std::make_shared<FirstPersonCamera>();
                newCam->SetViewPort(0.0f, 0.0f, (float)m_ClientWidth, (float)m_ClientHeight);
                newCam->SetFrustum(XM_PI / 3, AspectRatio(), 1.0f, 1000.0f);
                XMFLOAT3 pos = m_pCamera->GetPosition();
                newCam->LookTo(pos, XMFLOAT3(0.0f, 0.0f, 1.0f), XMFLOAT3(0.0f, 1.0f, 0.0f));
                m_pCamera = newCam;
                m_CameraMode = CameraMode::Free;
            }
        }
        if (m_CameraMode == CameraMode::Free)
            ImGui::Text("WASD: Move  Q/E: Down/Up\nRight drag: Look");
        else
            ImGui::Text("Right drag: Rotate  Scroll: Zoom");

        ImGui::Separator();
        if (m_PickedObjectName.empty())
            ImGui::Text("Picked: (none)");
        else
            ImGui::Text("Picked: %s", m_PickedObjectName.c_str());

        auto pos = m_pCamera->GetPosition();
        ImGui::Text("Pos: %.1f %.1f %.1f", pos.x, pos.y, pos.z);
    }
    ImGui::End();
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


    float black[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
    m_pd3dImmediateContext->ClearRenderTargetView(GetBackBufferRTV(), black);
    m_pd3dImmediateContext->ClearDepthStencilView(m_pDepthTexture->GetDepthStencil(), D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);
    ID3D11RenderTargetView* pRTVs[1] = { GetBackBufferRTV() };
    m_pd3dImmediateContext->OMSetRenderTargets(1, pRTVs, m_pDepthTexture->GetDepthStencil());
    D3D11_VIEWPORT viewport = m_pCamera->GetViewPort();
    m_pd3dImmediateContext->RSSetViewports(1, &viewport);

    m_BasicEffect.SetRenderDefault();
    m_Ground.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    m_House.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    
    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

    HR(m_pSwapChain->Present(0, m_IsDxgiFlipModel ? DXGI_PRESENT_ALLOW_TEARING : 0));
}



bool GameApp::InitResource()
{
    // ******************
    // Initialize game objects
    //

    // Initialize ground
    Model* pModel = m_ModelManager.CreateFromFile("Model\\ground_19.obj");
    m_Ground.SetModel(pModel);
    pModel->SetDebugObjectName("ground_19");

    // Initialize house model
    pModel = m_ModelManager.CreateFromFile("Model\\house.obj");
    m_House.SetModel(pModel);
    pModel->SetDebugObjectName("house");

    // Get house bounding box
    XMMATRIX S = XMMatrixScaling(0.015f, 0.015f, 0.015f);
    BoundingBox houseBox = m_House.GetModel()->boundingbox;
    houseBox.Transform(houseBox, S);
    // Place house flush with the ground
    Transform& houseTransform = m_House.GetTransform();
    houseTransform.SetScale(0.015f, 0.015f, 0.015f);
    houseTransform.SetPosition(0.0f, -(houseBox.Center.y - houseBox.Extents.y + 1.0f), 0.0f);
    
    // ******************
    // Initialize camera
    //

    auto camera = std::make_shared<ThirdPersonCamera>();
    m_pCamera = camera;
    m_CameraMode = CameraMode::ThirdPerson;

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


