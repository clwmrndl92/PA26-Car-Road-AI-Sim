#include "GameApp.h"
#include <XUtil.h>
#include <DXTrace.h>
#include <Entities/Car.h>

using namespace DirectX;

GameApp::GameApp(HINSTANCE hInstance, const std::wstring& windowName, int initWidth, int initHeight)
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
    // Step 1. Update game objects
    for (auto& obj : m_GameObjects) obj->Update(dt);

    // Step 2. Calculate physics System
    m_Physics.Update(dt);

    // Step 3. Update RenderObject
    for (auto& obj : m_GameObjects) obj->UpdateRender();

    // Step 4. Update camera by Input
    if (auto picked = m_pPickedObject.lock())
    {
        if (auto cam3rd = std::dynamic_pointer_cast<ThirdPersonCamera>(m_pCamera))
            cam3rd->SetTarget(picked->GetRender().GetBoundingBox().Center);
    }

    auto cam3rd = std::dynamic_pointer_cast<ThirdPersonCamera>(m_pCamera);
    auto cam1st = std::dynamic_pointer_cast<FirstPersonCamera>(m_pCamera);

    ImGuiIO& io = ImGui::GetIO();

    // Left click: pick object or switch to free camera
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse)
    {
        Ray ray = Ray::ScreenToRay(*m_pCamera, io.MousePos.x, io.MousePos.y);

        float distObj = FLT_MAX;
        std::shared_ptr<GameObject> hitObj;
        for (auto& obj : m_GameObjects)
        {
            float d = FLT_MAX;
            if (ray.Hit(obj->GetRender().GetBoundingBox(), &d) && d < distObj)
            {
                distObj = d;
                hitObj = obj;
            }
        }

        if (hitObj)
        {
            m_PickedObjectName = hitObj->GetName();
            m_pPickedObject = hitObj;
            XMFLOAT3 target = hitObj->GetRender().GetBoundingBox().Center;

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

    // Step 5. Set UI Window
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
                newCam->LookTo(m_pCamera->GetPosition(), XMFLOAT3(0.0f, 0.0f, 1.0f), XMFLOAT3(0.0f, 1.0f, 0.0f));
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

        if (auto pickedCar = std::dynamic_pointer_cast<Car>(m_pPickedObject.lock()))
        {
            ImGui::Text("Speed: %.2f", pickedCar->GetSpeed());
            ImGui::Text("Accel: %.2f", pickedCar->GetAcceleration());
        }

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
    for (auto& obj : m_GameObjects)
        obj->Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);

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
        road->SetName("Road");
        Model* pGround = m_ModelManager.CreateFromGeometry("road_ground", Geometry::CreatePlane(ROAD_SIZE, ROAD_SIZE));
        // Model* pGround = m_ModelManager.CreateFromGeometry("road_ground", Geometry::CreatePlane(10.0f, ROAD_SIZE));
        pGround->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.22f, 0.22f, 0.22f, 1.0f));
        pGround->materials[0].Set<float>("$Opacity", 1.0f);
        road->GetRender().SetModel(pGround);
        road->GetTransform().SetPosition(0.0f, 0.0f, 0.0f);
        road->Init(JPH::Vec3(ROAD_SIZE * 0.5f, 0.05f, ROAD_SIZE * 0.5f), Rigidbody::Type::Static);
        m_GameObjects.push_back(road);

        Model* pDash = m_ModelManager.CreateFromGeometry("road_dash", Geometry::CreatePlane(0.3f, 3.0f));
        pDash->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f));
        pDash->materials[0].Set<float>("$Opacity", 1.0f);

        constexpr float DASH_SPACING = 6.0f;
        int dashCount = static_cast<int>(ROAD_SIZE / DASH_SPACING + 0.5f) + 1;
        float startZ = -(dashCount - 1) * DASH_SPACING * 0.5f;
        for (int i = 0; i < dashCount; i++)
        {
            RenderObject& dash = road->AddSubRender();
            dash.SetModel(pDash);
            dash.GetTransform().SetPosition(0.0f, 0.01f, startZ + i * DASH_SPACING);
        }
    }

    // Car 1
    {
        auto car = std::make_shared<Car>();
        car->SetName("Car_0");
        car->GetTransform().SetPosition(0.0f, 1.0f, 0.0f);
        car->Init(GetCarSpec(CarType::Car1));
        car->SetDrawCollider(true);
        m_GameObjects.push_back(car);
    }

    // // Car 2
    // {
    //     auto car = std::make_shared<GameObject>();
    //     car->SetName("Car_1");
    //     car->GetRender().SetModel(m_ModelManager.CreateFromFile("Model\\car_2.obj"));
    //     car->GetTransform().SetPosition(2.0f, 0.0f, 0.0f);
    //     car->Init(JPH::Vec3(1.3421f, 0.9073f, 2.8342f), Rigidbody::Type::Static);
    //     m_GameObjects.push_back(car);
    // }

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
    dirLight.ambient   = XMFLOAT4(0.5f, 0.5f, 0.5f, 1.0f);
    dirLight.diffuse   = XMFLOAT4(0.8f, 0.8f, 0.8f, 1.0f);
    dirLight.specular  = XMFLOAT4(0.5f, 0.5f, 0.5f, 1.0f);
    dirLight.direction = XMFLOAT3(0.0f, -1.0f, 0.0f);
    m_BasicEffect.SetDirLight(0, dirLight);
    // Point light
    PointLight pointLight{};
    pointLight.position = XMFLOAT3(0.0f, 20.0f, 0.0f);
    pointLight.ambient  = XMFLOAT4(0.3f, 0.3f, 0.3f, 1.0f);
    pointLight.diffuse  = XMFLOAT4(0.7f, 0.7f, 0.7f, 1.0f);
    pointLight.specular = XMFLOAT4(0.2f, 0.2f, 0.2f, 1.0f);
    pointLight.att      = XMFLOAT3(0.0f, 0.1f, 0.0f);
    pointLight.range    = 30.0f;
    m_BasicEffect.SetPointLight(0, pointLight);

    return true;
}
