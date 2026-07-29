#include "CarSim.h"
#include <XUtil.h>
#include <DXTrace.h>
#include <Geometry.h>
#include "Car/Car.h"
#include "Utill/DebugConsole.h"
#include "Nav/Spline.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdlib>

using namespace DirectX;

namespace
{
    // 박스의 로컬 +Z(길이축)를 forward 방향에 정렬하는 쿼터니언. 경사(오르막/내리막)도 반영.
    XMFLOAT4 QuatFromForward(const Vec3 &forwardIn)
    {
        XMVECTOR fwd = XMVector3Normalize(XMVectorSet(forwardIn.GetX(), forwardIn.GetY(), forwardIn.GetZ(), 0.0f));
        XMVECTOR up = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
        if (fabsf(XMVectorGetX(XMVector3Dot(fwd, up))) > 0.999f) // 거의 수직이면 up 평행 -> 대체축
            up = XMVectorSet(0.0f, 0.0f, 1.0f, 0.0f);
        XMVECTOR right = XMVector3Normalize(XMVector3Cross(up, fwd));
        XMVECTOR trueUp = XMVector3Cross(fwd, right);
        XMMATRIX rot(right, trueUp, fwd, XMVectorSet(0.0f, 0.0f, 0.0f, 1.0f)); // 행 = X/Y/Z축 상(image)
        XMFLOAT4 q;
        XMStoreFloat4(&q, XMQuaternionRotationMatrix(rot));
        return q;
    }
}

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
    m_MarkingDataManager.Init(NAV_DATA_DIR "/marking2.json");

    if (!InitResource())
        return false;

    return true;
}

void CarSim::UpdateScene(float dt)
{
    m_SimState.Tick(GetSimDt(dt));
    UpdateSignalMarkers();
    GameApp::UpdateScene(dt);
}

void CarSim::UpdateSignalMarkers()
{
    for (const SignalMarker &marker : m_SignalMarkers)
    {
        XMFLOAT4 color;
        switch (m_SimState.GetSignalColor(marker.phaseOffset))
        {
        case TrafficSignal::Color::Red:
            color = XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f);
            break;
        case TrafficSignal::Color::Yellow:
            color = XMFLOAT4(1.0f, 1.0f, 0.0f, 1.0f);
            break;
        default:
            color = XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f);
            break;
        }
        marker.model->materials[0].Set<XMFLOAT4>("$DiffuseColor", color);
    }
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
        pGround->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.7f, 0.8f, 0.6f, 1.0f));
        pGround->materials[0].Set<float>("$Opacity", 1.0f);
        road->SetModel(pGround);
        road->SetPosition(Vec3(0.0f, -0.01f, 0.0f));
        road->Init(JPH::Vec3(ROAD_SIZE * 0.5f, 0.05f, ROAD_SIZE * 0.5f), Rigidbody::Type::Static);
        m_GameObjects.push_back(road);

        InitRoadRenderer();
        InitMarkingRenderer();
        InitRoadColliders();
        InitObstacleColliders();
    }

    // Car 1
    {
        auto car = std::make_shared<Car>();
        car->Init(GetCarSpec(CarType::Car0), &m_SimState, JPH::Vec3(0.0f, 0.1f, -30.0f));

        // car->SetDestination(m_RoadDataManager.GetNode(1));
        // std::shared_ptr<RoadNode> dest = m_RoadDataManager.GetRandomDestNode();
        // if (dest)
        //     car->SetDestination(dest);
        car->SetRoaming(true); // 목적지 없이 스플라인 따라 배회
        car->SetRotation(Vec3(-1, 0, 0));

        m_GameObjects.push_back(car);
        m_CarObjects.push_back(car);
        FocusOnObject(car);
    }

    // // Car 2
    // {
    //     auto car = std::make_shared<Car>();
    //     car->Init(GetCarSpec(CarType::Car1), &m_SimState, JPH::Vec3(-25.0f, 0.1f, 8.0f));
    //     car->SetDestination(m_RoadDataManager.GetNode(1));
    //     car->SetRotation(Vec3(-1, 0, 0));

    //     m_GameObjects.push_back(car);
    //     m_CarObjects.push_back(car);
    // }

    return true;
}

void CarSim::SpawnCar(CarType type)
{
    auto spawnNode = m_RoadDataManager.GetRandomDestNode();
    auto destNode = m_RoadDataManager.GetRandomDestNode();
    if (!spawnNode || !destNode)
        return;

    float yaw = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * XM_2PI;
    Vec3 direction(std::sin(yaw), 0.0f, std::cos(yaw));

    auto car = std::make_shared<Car>();
    car->Init(GetCarSpec(type), &m_SimState,
              JPH::Vec3(spawnNode->position.GetX(), 0.1f, spawnNode->position.GetZ()));
    // car->SetRotation(direction);
    car->SetRotation(spawnNode->direction);
    car->SetDestination(destNode);
    car->SetName(car->GetName() + ToString(m_carIDCounter++));

    m_GameObjects.push_back(car);
    m_CarObjects.push_back(car);
}

void CarSim::RemoveCar(const std::shared_ptr<Car> &car)
{
    if (m_pPickedObject.lock() == car)
    {
        m_pPickedObject.reset();
        m_PickedObjectName = "";
    }

    car->Destroy();
    m_CarObjects.erase(std::remove(m_CarObjects.begin(), m_CarObjects.end(), car), m_CarObjects.end());
    m_GameObjects.erase(std::remove(m_GameObjects.begin(), m_GameObjects.end(), car), m_GameObjects.end());
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

        ImGui::Separator();
        ImGui::Text("Simulation Speed");
        float timeScale = GetTimeScale();
        const float kSpeedOptions[] = {0.5f, 1.0f, 2.0f, 3.0f};
        for (int i = 0; i < IM_ARRAYSIZE(kSpeedOptions); ++i)
        {
            if (i > 0)
                ImGui::SameLine();
            bool isActive = std::fabs(timeScale - kSpeedOptions[i]) < 0.01f;
            if (isActive)
                ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
            if (ImGui::Button((std::to_string(kSpeedOptions[i]).substr(0, 3) + "x").c_str()))
                SetTimeScale(kSpeedOptions[i]);
            if (isActive)
                ImGui::PopStyleColor();
        }

        if (m_PickedObjectName.empty())
            ImGui::Text("Picked: (none)");
        else
            ImGui::Text("Picked: %s", m_PickedObjectName.c_str());

        ImGui::Separator();
        ImGui::Text("Spawn Car");
        for (int i = 0; i < static_cast<int>(CarType::Count); ++i)
        {
            CarType type = static_cast<CarType>(i);
            ImGui::Text("%s", GetCarSpec(type).name);
            ImGui::SameLine();
            if (ImGui::Button(("Spawn##spawnCar" + std::to_string(i)).c_str()))
                SpawnCar(type);
        }
        ImGui::Separator();
        std::shared_ptr<Car> carToRemove;
        for (auto &obj : m_CarObjects)
        {
            ImGui::PushID(obj.get());
            if (ImGui::Button("X"))
                carToRemove = obj;
            ImGui::SameLine();

            bool isSelected = (obj == m_pPickedObject.lock());
            if (ImGui::Selectable(obj->GetName().c_str(), isSelected))
                FocusOnObject(obj);
            ImGui::PopID();
        }
        if (carToRemove)
            RemoveCar(carToRemove);
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
    for (auto &obstacleRender : m_ObstacleRenders)
        obstacleRender.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
    for (auto &signalRender : m_SignalRenders)
        signalRender.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);

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
    constexpr float ROAD_WIDTH = RoadDataManager::ROAD_WIDTH;
    constexpr float NODE_MARKER_RADIUS = 0.5f;
    constexpr float EDGE_LINE_HEIGHT = 0.1f;

    m_RoadRenders.clear();

    // 각 road의 아스팔트(회색, 밴드 전체 d범위) + 차선 마킹(중앙선/밴드 바깥경계) 렌더.
    // EditApp::RebuildRenderObjects의 addMark/아스팔트 로직과 동일 규약(참조선 기준 오프셋 리본).
    auto roadMarkColor = [](BoundaryMark::Color c) -> XMFLOAT4
    {
        return c == BoundaryMark::Color::Yellow ? XMFLOAT4(1.0f, 0.85f, 0.0f, 1.0f) : XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
    };
    for (const auto &road : m_RoadDataManager.GetRoads())
    {
        const std::vector<Vec3> &refPts = road->GetReferenceLine().GetSplinePoints();
        if (refPts.size() < 2)
            continue;

        const LaneSection *section = m_RoadDataManager.GetLateralProfile(road, 0.0f);

        // 아스팔트: 밴드 전체 d범위를 덮는 리본.
        if (section != nullptr && !section->bands.empty())
        {
            float dMin = FLT_MAX, dMax = -FLT_MAX;
            for (const LaneBand &b : section->bands)
            {
                dMin = std::min(dMin, b.centerOffset - b.width * 0.5f);
                dMax = std::max(dMax, b.centerOffset + b.width * 0.5f);
            }
            if (dMax > dMin)
            {
                Spline asphaltSpline = m_RoadDataManager.BuildOffsetSpline(road, (dMin + dMax) * 0.5f);
                std::vector<XMFLOAT3> asphaltLine;
                for (const Vec3 &p : asphaltSpline.GetSplinePoints())
                {
                    XMFLOAT3 f = ToXMFLOAT3(p);
                    f.y += 0.02f;
                    asphaltLine.push_back(f);
                }
                if (asphaltLine.size() >= 2)
                {
                    GeometryData geo = Geometry::CreateRibbon(asphaltLine, dMax - dMin);
                    if (!geo.vertices.empty())
                    {
                        Model *pAsphalt = m_ModelManager.CreateFromGeometry("road_asphalt_" + std::to_string(road->GetId()), geo);
                        pAsphalt->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.18f, 0.18f, 0.18f, 1.0f));
                        pAsphalt->materials[0].Set<float>("$Opacity", 1.0f);
                        RenderObject &ro = m_RoadRenders.emplace_back();
                        ro.SetModel(pAsphalt);
                    }
                }
            }
        }

        // 마킹 리본(참조선 기준 오프셋 d, 복선이면 두 줄).
        auto addMark = [&](float d, const BoundaryMark &mk, const std::string &tag)
        {
            if (mk.type == BoundaryMark::Type::None)
                return;
            auto ribbon = [&](float dd, const std::string &name)
            {
                Spline markSpline = m_RoadDataManager.BuildOffsetSpline(road, dd);
                std::vector<XMFLOAT3> poly;
                for (const Vec3 &p : markSpline.GetSplinePoints())
                {
                    XMFLOAT3 f = ToXMFLOAT3(p);
                    f.y += 0.06f;
                    poly.push_back(f);
                }
                if (poly.size() < 2)
                    return;
                GeometryData geo = mk.type == BoundaryMark::Type::Broken
                                       ? Geometry::CreateDashedRibbon(poly, mk.width, 3.0f, 5.0f)
                                       : Geometry::CreateRibbon(poly, mk.width);
                if (geo.vertices.empty())
                    return;
                Model *pm = m_ModelManager.CreateFromGeometry(name, geo);
                pm->materials[0].Set<XMFLOAT4>("$DiffuseColor", roadMarkColor(mk.color));
                pm->materials[0].Set<float>("$Opacity", 1.0f);
                RenderObject &ro = m_RoadRenders.emplace_back();
                ro.SetModel(pm);
            };
            if (mk.type == BoundaryMark::Type::DoubleSolid)
            {
                ribbon(d - 0.12f, tag + "_a");
                ribbon(d + 0.12f, tag + "_b");
            }
            else
            {
                ribbon(d, tag);
            }
        };

        if (const BoundaryMark *centerMark = m_RoadDataManager.GetCenterMark(road))
            addMark(0.0f, *centerMark, "road_centermark_" + std::to_string(road->GetId()));

        if (section != nullptr)
        {
            for (int bi = 0; bi < (int)section->bands.size(); ++bi)
            {
                const LaneBand &b = section->bands[bi];
                float outer = b.centerOffset + (b.centerOffset >= 0.0f ? 1.0f : -1.0f) * b.width * 0.5f;
                addMark(outer, b.boundaryMark, "road_bandmark_" + std::to_string(road->GetId()) + "_" + std::to_string(bi));
            }
        }
    }

    for (const auto &[id, node] : m_RoadDataManager.GetNodes())
    {
        Model *pMarker = m_ModelManager.CreateFromGeometry("node_marker" + std::to_string(node->id), Geometry::CreateSphere(NODE_MARKER_RADIUS));
        pMarker->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f));
        pMarker->materials[0].Set<float>("$Opacity", 1.0f);

        RenderObject &nodeRender = m_RoadRenders.emplace_back();
        nodeRender.SetModel(pMarker);
        nodeRender.GetTransform().SetPosition(ToXMFLOAT3(node->position));
    }

    // 신호(traffic_light) 노드: 채워진 원 마커. phaseOffset이 신호마다 달라 동시에 다른 색일 수
    // 있으므로 Model을 공유하지 않고 노드마다 따로 만든다 (UpdateSignalMarkers가 매 프레임 각자의
    // phaseOffset으로 자기 Model의 색만 갈아끼움).
    {
        constexpr float SIGNAL_MARKER_RADIUS = 1.0f;
        constexpr float SIGNAL_MARKER_LIFT = 0.15f; // 도로면/노드 마커와 겹치지 않게 살짝 띄운다.

        m_SignalRenders.clear();
        m_SignalMarkers.clear();

        for (const auto &[id, node] : m_RoadDataManager.GetNodes())
        {
            if (node->nodeType != RoadNodeType::TrafficLight)
                continue;

            Model *pMarker = m_ModelManager.CreateFromGeometry("signal_marker" + std::to_string(node->id), Geometry::CreateCircle(SIGNAL_MARKER_RADIUS));
            pMarker->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f));
            pMarker->materials[0].Set<float>("$Opacity", 1.0f);

            RenderObject &signalRender = m_SignalRenders.emplace_back();
            signalRender.SetModel(pMarker);
            XMFLOAT3 pos = ToXMFLOAT3(node->position);
            pos.y += SIGNAL_MARKER_LIFT;
            signalRender.GetTransform().SetPosition(pos);

            m_SignalMarkers.push_back({pMarker, node->signalPhaseOffset});
        }
    }

    // road 그래프의 successor 연결(road 참조선 끝 -> 다음 road 시작)을 노란 선으로 시각화한다.
    m_RoadEdgeRenders.clear();
    int linkIndex = 0;
    for (const auto &road : m_RoadDataManager.GetRoads())
    {
        const std::vector<Vec3> &fromPts = road->GetReferenceLine().GetSplinePoints();
        if (fromPts.size() < 2)
            continue;
        Vec3 from = fromPts.back() + Vec3(0.0f, EDGE_LINE_HEIGHT, 0.0f);
        for (const auto &succ : m_RoadDataManager.GetRoadSuccessors(road->GetId()))
        {
            const std::vector<Vec3> &toPts = succ->GetReferenceLine().GetSplinePoints();
            if (toPts.size() < 2)
                continue;
            Vec3 to = toPts.front() + Vec3(0.0f, EDGE_LINE_HEIGHT, 0.0f);

            Model *pLine = m_ModelManager.CreateFromGeometry("edge_line" + std::to_string(linkIndex++), Geometry::CreateLine(ToXMFLOAT3(from), ToXMFLOAT3(to)));
            pLine->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 1.0f, 0.0f, 1.0f));
            pLine->materials[0].Set<float>("$Opacity", 1.0f);

            RenderObject &edgeRender = m_RoadEdgeRenders.emplace_back();
            edgeRender.SetModel(pLine);
        }
    }

    // 장애물(data.json의 obstacles)을 위치/크기/회전각 그대로 파란 평면으로 시각화.
    m_ObstacleRenders.clear();
    int obstacleIndex = 0;
    for (const VehicleCollision::Obstacle &obstacle : m_RoadDataManager.GetObstacles())
    {
        float headingRad = obstacle.headingRad;
        Vec3 forward(cosf(headingRad), 0.0f, sinf(headingRad));
        Vec3 right(-forward.GetZ(), 0.0f, forward.GetX());

        auto corner = [&](float alongSign, float acrossSign)
        {
            XMFLOAT3 p = ToXMFLOAT3(obstacle.center + forward * (obstacle.halfLength * alongSign) + right * (obstacle.halfWidth * acrossSign));
            p.y += EDGE_LINE_HEIGHT;
            return p;
        };

        Model *pObstacle = m_ModelManager.CreateFromGeometry(
            "obstacle_marker" + std::to_string(obstacleIndex++),
            Geometry::CreateQuad(corner(1.0f, 1.0f), corner(1.0f, -1.0f), corner(-1.0f, -1.0f), corner(-1.0f, 1.0f)));
        pObstacle->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.0f, 0.4f, 1.0f, 1.0f));
        pObstacle->materials[0].Set<float>("$Opacity", 1.0f);

        RenderObject &obstacleRender = m_ObstacleRenders.emplace_back();
        obstacleRender.SetModel(pObstacle);
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

void CarSim::InitRoadColliders()
{
    constexpr float FALLBACK_ROAD_WIDTH = RoadDataManager::ROAD_WIDTH; // 밴드 정보 없는 road용 폴백
    constexpr float ROAD_THICKNESS = 0.2f;                             // 도로 판 두께
    constexpr float SEGMENT_LENGTH = 1.0f;                             // 콜라이더 박스 하나가 덮는 목표 길이(짧을수록 경사 이음새 각이 작아 튐이 준다)

    int segIndex = 0;
    for (const auto &roadData : m_RoadDataManager.GetRoads())
    {
        // 컨트롤 포인트 y가 전부 0.01 이하인 지면 도로는 ground plane이 이미 받쳐주므로 만들지 않는다.
        const std::vector<Vec3> &ctrl = roadData->GetReferenceLine().GetControlPoints();
        bool elevated = std::any_of(ctrl.begin(), ctrl.end(),
                                    [](const Vec3 &p)
                                    { return std::fabs(p.GetY()) > 0.01f; });
        if (!elevated)
            continue;

        // 밴드 전체 d범위(InitRoadRenderer의 아스팔트 리본과 동일 규약)로 폭/중심오프셋을 구해,
        // 콜라이더가 회색 아스팔트와 정확히 같은 폭/위치를 덮게 한다. 밴드 없으면 참조선 폭 폴백.
        const LaneSection *section = m_RoadDataManager.GetLateralProfile(roadData, 0.0f);
        float roadWidth = FALLBACK_ROAD_WIDTH;
        float centerOffset = 0.0f;
        if (section != nullptr && !section->bands.empty())
        {
            float dMin = FLT_MAX, dMax = -FLT_MAX;
            for (const LaneBand &b : section->bands)
            {
                dMin = std::min(dMin, b.centerOffset - b.width * 0.5f);
                dMax = std::max(dMax, b.centerOffset + b.width * 0.5f);
            }
            if (dMax > dMin)
            {
                roadWidth = dMax - dMin;
                centerOffset = (dMin + dMax) * 0.5f;
            }
        }

        Spline colliderSpline = m_RoadDataManager.BuildOffsetSpline(roadData, centerOffset);
        const std::vector<Vec3> &pts = colliderSpline.GetSplinePoints();
        if (pts.size() < 2)
            continue;

        // 스플라인을 호길이 기준으로 걸어가며 SEGMENT_LENGTH마다(또는 끝에서) 박스 하나로 끊는다.
        size_t start = 0;
        float accum = 0.0f;
        for (size_t i = 1; i < pts.size(); ++i)
        {
            accum += (pts[i] - pts[i - 1]).Length();
            if (accum < SEGMENT_LENGTH && i + 1 < pts.size())
                continue;

            const Vec3 &a = pts[start];
            const Vec3 &b = pts[i];
            start = i;
            accum = 0.0f;

            Vec3 delta = b - a;
            float length = delta.Length();
            if (length < 1e-3f)
                continue;

            Vec3 mid = (a + b) * 0.5f - Vec3(0.0f, ROAD_THICKNESS * 0.5f, 0.0f); // 박스 윗면 = 레인 높이

            Model *pRoad = m_ModelManager.CreateFromGeometry(
                "road_seg" + std::to_string(segIndex++),
                Geometry::CreateBox(roadWidth, ROAD_THICKNESS, length));
            pRoad->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.25f, 0.25f, 0.27f, 1.0f));
            pRoad->materials[0].Set<float>("$Opacity", 1.0f);

            auto road = std::make_shared<GameObject>();
            road->SetName("Road_L" + std::to_string(roadData->GetId()));
            road->SetModel(pRoad);
            road->SetPosition(mid);
            road->Init(JPH::Vec3(roadWidth * 0.5f, ROAD_THICKNESS * 0.5f, length * 0.5f), Rigidbody::Type::Static);
            road->SetRotation(QuatFromForward(delta));
            m_GameObjects.push_back(road);
        }
    }
}

void CarSim::InitObstacleColliders()
{
    constexpr float OBSTACLE_HEIGHT = 1.5f; // 큐브 높이(장애물 데이터엔 높이가 없어 고정값 사용)

    int obstacleIndex = 0;
    for (const VehicleCollision::Obstacle &obstacle : m_RoadDataManager.GetObstacles())
    {
        // heading 기준 forward축을 박스 로컬 +Z(길이축)에 정렬. 길이=2*halfLength(Z), 폭=2*halfWidth(X).
        Vec3 forward(cosf(obstacle.headingRad), 0.0f, sinf(obstacle.headingRad));

        Model *pCube = m_ModelManager.CreateFromGeometry(
            "obstacle_cube" + std::to_string(obstacleIndex++),
            Geometry::CreateBox(obstacle.halfWidth * 2.0f, OBSTACLE_HEIGHT, obstacle.halfLength * 2.0f));
        pCube->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.0f, 0.4f, 1.0f, 1.0f));
        pCube->materials[0].Set<float>("$Opacity", 1.0f);

        auto cube = std::make_shared<GameObject>();
        cube->SetName("Obstacle" + std::to_string(obstacleIndex));
        cube->SetModel(pCube);
        cube->SetPosition(obstacle.center + Vec3(0.0f, OBSTACLE_HEIGHT * 0.5f, 0.0f)); // 바닥면을 center.y에 맞춘다
        cube->Init(JPH::Vec3(obstacle.halfWidth, OBSTACLE_HEIGHT * 0.5f, obstacle.halfLength), Rigidbody::Type::Static);
        cube->SetRotation(QuatFromForward(forward));
        m_GameObjects.push_back(cube);
    }
}