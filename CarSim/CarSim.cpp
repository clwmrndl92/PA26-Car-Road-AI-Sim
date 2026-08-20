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
#include <limits>

using namespace DirectX;

namespace
{
    // weights 누적합 위에 [0, total) 균등난수를 던져 인덱스를 고른다 (비율 기반 랜덤 선택).
    int PickWeightedIndex(const float *weights, int count)
    {
        float total = 0.0f;
        for (int i = 0; i < count; ++i)
            total += weights[i];

        float r = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * total;
        float accum = 0.0f;
        for (int i = 0; i < count; ++i)
        {
            accum += weights[i];
            if (r < accum)
                return i;
        }
        return count - 1; // 부동소수 오차로 못 걸렸을 때의 폴백
    }

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

    m_RoadDataManager.Init(NAV_DATA_DIR "/data6.json");
    m_MarkingDataManager.Init(NAV_DATA_DIR "/marking.json");

    if (!InitResource())
        return false;

    return true;
}

void CarSim::UpdateScene(float dt)
{
    float simDt = GetSimDt(dt);
    m_SimState.Tick(simDt);
    m_RoadDataManager.UpdateDynamicObstacles(simDt);
    UpdateSignalMarkers();
    GameApp::UpdateScene(dt);

    // 왕복 동적 장애물 렌더 위치 갱신 -- 개수/순서는 InitDynamicObstacleRenders가 만든 것과 GetDynamicObstacles()가 항상 같다.
    const auto &dynamicObstacles = m_RoadDataManager.GetDynamicObstacles();
    for (size_t i = 0; i < m_DynamicObstacleRenders.size() && i < dynamicObstacles.size(); ++i)
    {
        const VehicleCollision::Obstacle &obstacle = dynamicObstacles[i];
        Vec3 forward(cosf(obstacle.headingRad), 0.0f, sinf(obstacle.headingRad));
        m_DynamicObstacleRenders[i].GetTransform().SetPosition(ToXMFLOAT3(obstacle.center));
        m_DynamicObstacleRenders[i].GetTransform().SetRotation(QuatFromForward(forward));
    }
}

void CarSim::UpdateSignalMarkers()
{
    for (const SignalMarker &marker : m_SignalMarkers)
    {
        XMFLOAT4 color;
        switch (m_SimState.GetSignalColor(marker.node->signalPhaseOffset, marker.node->signalGreenDuration,
                                          marker.node->signalYellowDuration, marker.node->signalRedDuration))
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
        InitDynamicObstacleRenders();
    }

    // // Car 1
    // {
    //     auto car = std::make_shared<Car>();
    //     car->Init(GetCarSpec(CarType::Jeep), GetCarPersonality(CarPersonalityType::Siren),
    //               &m_SimState, JPH::Vec3(133.0f, 0.0, -74.0f), 100.0f);
    //     car->SetSpeed(20.0f);
    //     car->SetRotation(Vec3(0, 0, 1));
    //     car->SetRoaming(true);
    //     car->SetFleeOn(false);

    //     m_GameObjects.push_back(car);
    //     m_CarObjects.push_back(car);
    //     FocusOnObject(car);
    // }

    return true;
}

void CarSim::SpawnCar(CarType type, CarPersonalityType personality, bool roaming)
{
    auto spawnNode = m_RoadDataManager.GetRandomDestNode();
    if (!spawnNode)
        return;

    std::shared_ptr<RoadNode> destNode;
    if (!roaming)
    {
        destNode = m_RoadDataManager.GetRandomParkNode();
        if (!destNode)
            return;
    }

    auto car = SpawnCarAt(spawnNode->position, spawnNode->direction, type, personality, roaming);
    if (!roaming)
        car->SetDestination(destNode);
}

std::shared_ptr<Car> CarSim::SpawnCarAt(const Vec3 &position, const Vec3 &direction, CarType type,
                                        CarPersonalityType personality, bool roaming)
{
    auto car = std::make_shared<Car>();
    car->Init(GetCarSpec(type), GetCarPersonality(personality), &m_SimState,
              JPH::Vec3(position.GetX(), 0.1f, position.GetZ()));
    car->SetRotation(direction);
    car->SetRoaming(roaming);
    car->SetId(m_carIDCounter);
    car->SetName(car->GetName() + ToString(m_carIDCounter++));

    m_GameObjects.push_back(car);
    m_CarObjects.push_back(car);
    return car;
}

void CarSim::SpawnAllCars()
{
    // 차종/성격 비율 -- 사용자가 지정한 3:3:1:1.5:1.5 / 7:1.5:1.5
    static const float kCarTypeWeights[] = {3.0f, 3.0f, 1.0f, 1.5f, 1.5f}; // Car0:Car1:Jeep:LittleTruck:Van
    static const float kPersonalityWeights[] = {7.0f, 1.5f, 1.5f};         // Normal:Aggressive:Cautious
    static_assert(IM_ARRAYSIZE(kCarTypeWeights) == static_cast<int>(CarType::Count));

    constexpr float kSpawnRange = 150.0f; // x, z 각각 [-150, 150] 범위에 대충 흩뿌린다
    for (int i = 0; i < kMaxSpawnAllCount; ++i)
    {
        float x = (static_cast<float>(rand()) / RAND_MAX) * 2.0f * kSpawnRange - kSpawnRange;
        float z = (static_cast<float>(rand()) / RAND_MAX) * 2.0f * kSpawnRange - kSpawnRange;
        float yaw = (static_cast<float>(rand()) / RAND_MAX) * XM_2PI;
        Vec3 position(x, 0.1f, z);
        Vec3 direction(cosf(yaw), 0.0f, sinf(yaw));

        CarType type = static_cast<CarType>(PickWeightedIndex(kCarTypeWeights, IM_ARRAYSIZE(kCarTypeWeights)));
        CarPersonalityType personality =
            static_cast<CarPersonalityType>(PickWeightedIndex(kPersonalityWeights, IM_ARRAYSIZE(kPersonalityWeights)));
        SpawnCarAt(position, direction, type, personality, /*roaming=*/true);
    }
}

void CarSim::SpawnAllNodes()
{
    static const float kCarTypeWeights[] = {3.0f, 3.0f, 1.0f, 1.5f, 1.5f}; // Car0:Car1:Jeep:LittleTruck:Van
    static const float kPersonalityWeights[] = {7.0f, 1.5f, 1.5f};         // Normal:Aggressive:Cautious

    for (const auto &[id, node] : m_RoadDataManager.GetNodes())
    {
        if (node->nodeType == RoadNodeType::TrafficLight)
            continue;

        CarType type = static_cast<CarType>(PickWeightedIndex(kCarTypeWeights, IM_ARRAYSIZE(kCarTypeWeights)));
        CarPersonalityType personality =
            static_cast<CarPersonalityType>(PickWeightedIndex(kPersonalityWeights, IM_ARRAYSIZE(kPersonalityWeights)));
        SpawnCarAt(node->position, node->direction, type, personality, /*roaming=*/true);
    }
}

void CarSim::RemoveCar(const std::shared_ptr<Car> &car)
{
    if (m_pPickedObject.lock() == car)
    {
        m_pPickedObject.reset();
        m_PickedObjectName = "";
    }
    if (m_ManualCar == car)
        m_ManualCar.reset();

    car->Destroy();
    m_CarObjects.erase(std::remove(m_CarObjects.begin(), m_CarObjects.end(), car), m_CarObjects.end());
    m_GameObjects.erase(std::remove(m_GameObjects.begin(), m_GameObjects.end(), car), m_GameObjects.end());
}

void CarSim::RemoveAllCars()
{
    std::vector<std::shared_ptr<Car>> cars = m_CarObjects;
    for (const std::shared_ptr<Car> &car : cars)
    {
        if (car == m_ManualCar)
            continue;
        RemoveCar(car);
    }
}

void CarSim::SpawnManualCar(CarType type)
{
    // 기존 사용자 조작 차가 있으면 그 자리(위치/방향)를 기억해두고 제거한 뒤 같은 자리에 새 차종으로 소환한다.
    // 처음 소환하는 경우엔 초기 AI 차(Car 1, (0,0.1,-30) 부근에서 -X 방향)와 안 겹치도록 그 뒤(+X쪽)에 둔다.
    JPH::Vec3 spawnPos(15.0f, 0.1f, -30.0f);
    Vec3 spawnDir(-1.0f, 0.0f, 0.0f);
    if (m_ManualCar)
    {
        Vec3 pos = m_ManualCar->GetPosition();
        spawnPos = JPH::Vec3(pos.GetX(), pos.GetY(), pos.GetZ());
        spawnDir = m_ManualCar->GetForwardAxis();
        // 복사본으로 넘긴다: RemoveCar가 내부에서 m_ManualCar를 reset하므로, 멤버를 그대로 참조로 넘기면
        // reset 직후 같은 객체를 가리키는 참조가 null이 되어 곧이은 car->Destroy()가 크래시난다.
        std::shared_ptr<Car> oldManualCar = m_ManualCar;
        RemoveCar(oldManualCar);
    }

    auto car = std::make_shared<Car>();
    car->Init(GetCarSpec(type), GetCarPersonality(CarPersonalityType::Normal), &m_SimState, spawnPos);
    car->SetRotation(spawnDir);
    car->SetControl(true);
    car->SetId(m_carIDCounter++);
    car->SetName(car->GetName() + "_User");

    m_GameObjects.push_back(car);
    m_CarObjects.push_back(car);
    m_ManualCar = car;
    FocusOnObject(car);
}

void CarSim::RemoveManualCar()
{
    if (!m_ManualCar)
        return;
    // 복사본으로 넘긴다 (SpawnManualCar와 같은 이유: RemoveCar가 내부에서 m_ManualCar를 reset한다).
    std::shared_ptr<Car> oldManualCar = m_ManualCar;
    RemoveCar(oldManualCar);
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
        car->SetHighlighted(false);
        car->SetMobilHighlighted(false);
    }
    if (pickedObj != nullptr)
    {
        if (Car *leader = pickedObj->GetIdmLeader())
            leader->SetHighlighted(true);
        Car *laneLeader = nullptr;
        Car *laneFollower = nullptr;
        pickedObj->GetLaneChangeNeighbors(laneLeader, laneFollower);
        if (laneLeader != nullptr)
            laneLeader->SetMobilHighlighted(true);
        if (laneFollower != nullptr)
            laneFollower->SetMobilHighlighted(true);
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

    ImGui::SetNextWindowPos(ImVec2((float)m_ClientWidth * 0.5f, 10.0f), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
    if (ImGui::Begin("Simulation Speed", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize))
    {
        float timeScale = GetTimeScale();
        const float kSpeedOptions[] = {0.0f, 0.5f, 1.0f, 2.0f, 3.0f};
        for (int i = 0; i < IM_ARRAYSIZE(kSpeedOptions); ++i)
        {
            if (i > 0)
                ImGui::SameLine();
            bool isActive = std::fabs(timeScale - kSpeedOptions[i]) < 0.01f;
            if (isActive)
                ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
            std::string label = (kSpeedOptions[i] == 0.0f) ? "Pause" : (std::to_string(kSpeedOptions[i]).substr(0, 3) + "x");
            if (ImGui::Button(label.c_str()))
                SetTimeScale(kSpeedOptions[i]);
            if (isActive)
                ImGui::PopStyleColor();
        }
        ImGui::Text("Elapsed: %.1fs", m_SimState.GetSimTime());
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(0.0f, 100.0f), ImGuiCond_Always);
    if (ImGui::Begin("Objects"))
    {
        ImGui::Text("Mode : %s", m_CameraMode == CameraMode::Free ? "Free Camera" : "Focus Camera");

        ImGui::Separator();

        if (m_PickedObjectName.empty())
            ImGui::Text("Picked: (none)");
        else
            ImGui::Text("Picked: %s", m_PickedObjectName.c_str());

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

    ImGui::SetNextWindowPos(ImVec2(280.0f, 100.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Spawn Car"))
    {
        static const char *kPersonalityNames[] = {"Normal", "Aggressive", "Cautious"};
        ImGui::Combo("Personality", &m_SpawnPersonalityIndex, kPersonalityNames, IM_ARRAYSIZE(kPersonalityNames));
        CarPersonalityType personality = static_cast<CarPersonalityType>(m_SpawnPersonalityIndex);
        ImGui::Checkbox("Roaming (no destination)", &m_SpawnRoaming);

        ImGui::Separator();
        for (int i = 0; i < static_cast<int>(CarType::Count); ++i)
        {
            CarType type = static_cast<CarType>(i);
            ImGui::Text("%s", GetCarSpec(type).name);
            ImGui::SameLine();
            if (ImGui::Button(("Spawn##spawnCar" + std::to_string(i)).c_str()))
                SpawnCar(type, personality, m_SpawnRoaming);
        }

        ImGui::Separator();
        if (ImGui::Button("Remove All"))
            RemoveAllCars();
        if (ImGui::Button(("Spawn All (" + std::to_string(kMaxSpawnAllCount) + ")").c_str()))
            SpawnAllCars();
        ImGui::SameLine();
        if (ImGui::Button("Spawn All Nodes"))
            SpawnAllNodes();
    }
    ImGui::End();

    DrawManualCarWindow();
    DrawRaceWindow();

    DebugConsole::Get().Draw();
}

void CarSim::DrawManualCarWindow()
{
    ImGui::SetNextWindowPos(ImVec2(280.0f, 320.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("User Car"))
    {
        std::vector<const char *> typeNames;
        for (int i = 0; i < static_cast<int>(CarType::Count); ++i)
            typeNames.push_back(GetCarSpec(static_cast<CarType>(i)).name);
        ImGui::Combo("Car Type", &m_ManualCarTypeIndex, typeNames.data(), static_cast<int>(typeNames.size()));

        CarType selectedType = static_cast<CarType>(m_ManualCarTypeIndex);
        if (ImGui::Button(m_ManualCar ? "Change" : "Spawn"))
            SpawnManualCar(selectedType);
        ImGui::SameLine();
        if (ImGui::Button("Remove"))
            RemoveManualCar();

        ImGui::Separator();
        if (m_ManualCar)
            ImGui::Text("Driving: %s", m_ManualCar->GetName().c_str());
        else
            ImGui::Text("Driving: (none)");
        ImGui::TextWrapped("Controls: Up/Down = accel/brake, Left/Right = steer, Z = gear, Space = reset");
    }
    ImGui::End();
}

// QP는 전략당 수백 ms라 게임 시작 때 항상 돌릴 수는 없다. 레이스를 실제로 켤 때
// 한 번만 풀고 SimulationState가 소유한다.
bool CarSim::EnsureRaceLines()
{
    if (m_SimState.HasRaceLines())
        return true;

    Car::BuildSharedRaceLines(m_SimState);
    if (!m_SimState.HasRaceLines())
    {
        DebugConsole::Log("Race: failed to build racing lines (no road loop?)");
        return false;
    }

    // 차 선택과 무관하게 기준 라인(Apex)을 항상 그려 둔다.
    if (const RaceLine *line = m_SimState.GetRaceLine(static_cast<int>(Car::ApexStrategy::Apex)))
        Car::BuildRaceLineRenderObject(*line, "shared", m_RaceLineRender);
    return true;
}

void CarSim::SetRaceModeForAll(bool on)
{
    if (on && !EnsureRaceLines())
        return;

    for (auto &car : m_CarObjects)
        if (car != m_ManualCar) // 수동조작 차는 AI FSM을 타지 않는다
            car->SetRaceMode(on);

    if (!on)
        m_RaceLineRender.SetModel(nullptr);
}

// 레이싱 라인 위에 일정 간격으로 세운다. 무작위 스폰을 쓰면 출발과 동시에 서로를 향해
// 수렴해 첫 코너 전에 전부 엉킨다.
void CarSim::SpawnRaceGrid(int carCount)
{
    if (!EnsureRaceLines())
        return;

    const RaceLine *line = m_SimState.GetRaceLine(static_cast<int>(Car::ApexStrategy::Apex));
    if (line == nullptr || line->lapPoints < 2)
        return;

    RemoveAllCars();

    constexpr float GRID_SPACING = 12.0f; // 앞뒤 간격(m)
    constexpr float GRID_STAGGER = 1.6f;  // 좌우 엇갈림(m). 정렬 그리드처럼 두 줄로 세운다
    static const float kCarTypeWeights[] = {3.0f, 3.0f, 1.0f, 1.5f, 1.5f};

    const size_t lapPoints = line->lapPoints;

    // 그리드 기준은 월드 원점에서 가장 가까운 라인 지점. 랩 시작점(arcLength[0])을 쓰면
    // JSON에서 어느 도로가 먼저 읽히냐에 따라 스타트 위치가 트랙 반대편으로 튄다.
    // 차는 레이싱 라인 위에 있어야 주행이 성립하므로 라인에 붙인 채로 원점에 맞춘다.
    float anchorS = 0.0f;
    {
        float bestDistance = std::numeric_limits<float>::max();
        for (size_t k = 0; k < lapPoints; ++k)
        {
            const Vec3 &p = line->points[k];
            float distance = p.GetX() * p.GetX() + p.GetZ() * p.GetZ(); // 원점까지 (y는 무시)
            if (distance < bestDistance)
            {
                bestDistance = distance;
                anchorS = line->arcLength[k];
            }
        }
    }

    for (int i = 0; i < carCount; ++i)
    {
        // 폴포지션이 기준점에 서고, i가 커질수록 뒤(음의 s)에 놓는다.
        float s = anchorS - static_cast<float>(i) * GRID_SPACING;
        while (s < 0.0f)
            s += line->lapLength;

        size_t index = 0;
        for (size_t k = 0; k + 1 < lapPoints; ++k)
        {
            if (line->arcLength[k] <= s && s < line->arcLength[k + 1])
            {
                index = k;
                break;
            }
        }
        size_t next = (index + 1) % lapPoints;

        Vec3 position = line->points[index];
        Vec3 direction = line->points[next] - line->points[index];
        if (direction.LengthSq() < 1e-6f)
            continue;
        direction = direction.Normalized();

        // 좌우 엇갈림. 라인 접선의 오른쪽 법선으로 민다.
        Vec3 right(direction.GetZ(), 0.0f, -direction.GetX());
        position = position + right * ((i % 2 == 0) ? GRID_STAGGER : -GRID_STAGGER);

        CarType type = static_cast<CarType>(PickWeightedIndex(kCarTypeWeights, IM_ARRAYSIZE(kCarTypeWeights)));
        auto car = std::make_shared<Car>();
        // 앞줄일수록 잘 타는 드라이버. 스킬이 뒤로 갈수록 낮아져야 추월 압력이 생긴다.
        float skill = carCount > 1 ? 1.0f - static_cast<float>(i) / static_cast<float>(carCount - 1) : 1.0f;
        car->Init(GetCarSpec(type), MakeRacerPersonality(skill, static_cast<unsigned int>(i + 1)),
                  &m_SimState, JPH::Vec3(position.GetX(), 0.1f, position.GetZ()));
        car->SetRotation(direction);
        car->SetRoaming(true);
        car->SetId(m_carIDCounter);
        car->SetName(car->GetName() + ToString(m_carIDCounter++));
        car->SetRaceMode(true);

        m_GameObjects.push_back(car);
        m_CarObjects.push_back(car);
    }

    m_RaceMode = true;
    DebugConsole::Log("Race: grid of " + ToString(carCount) + " cars");
}

void CarSim::DrawRaceWindow()
{
    ImGui::SetNextWindowPos(ImVec2(280.0f, 520.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Race"))
    {
        if (ImGui::Checkbox("Race Mode", &m_RaceMode))
            SetRaceModeForAll(m_RaceMode);
        ImGui::TextWrapped("Race mode replaces signals/lane rules with a shared minimum-curvature "
                           "racing line, slipstream, and an overtake FSM.");

        ImGui::Separator();
        ImGui::SliderInt("Grid Size", &m_RaceGridCount, 2, kMaxSpawnAllCount);
        if (ImGui::Button("Spawn Race Grid"))
            SpawnRaceGrid(m_RaceGridCount);
        ImGui::SameLine();
        if (ImGui::Button("Build Lines"))
            EnsureRaceLines();

        ImGui::Separator();
        if (!m_SimState.HasRaceLines())
        {
            ImGui::Text("Racing lines: not built");
        }
        else
        {
            const RaceLine *line = m_SimState.GetRaceLine(static_cast<int>(Car::ApexStrategy::Apex));
            ImGui::Text("Lap length: %.0f m", line != nullptr ? line->lapLength : 0.0f);

            // 순위표. SimulationState가 이미 진행도 순으로 갱신해 두므로 여기선 정렬만 한다.
            std::vector<std::shared_ptr<Car>> ordered;
            for (const auto &car : m_CarObjects)
                if (car->IsRaceMode())
                    ordered.push_back(car);
            std::sort(ordered.begin(), ordered.end(),
                      [](const std::shared_ptr<Car> &a, const std::shared_ptr<Car> &b)
                      { return a->GetRaceProgress() > b->GetRaceProgress(); });

            if (ImGui::BeginTable("standings", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
            {
                ImGui::TableSetupColumn("P");
                ImGui::TableSetupColumn("Car");
                ImGui::TableSetupColumn("Lap");
                ImGui::TableSetupColumn("Best");
                ImGui::TableSetupColumn("+/-");
                ImGui::TableHeadersRow();
                for (size_t i = 0; i < ordered.size(); ++i)
                {
                    const std::shared_ptr<Car> &car = ordered[i];
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::Text("%d", static_cast<int>(i) + 1);
                    ImGui::TableNextColumn();
                    if (ImGui::Selectable(car->GetName().c_str(), car == m_pPickedObject.lock()))
                        FocusOnObject(car);
                    ImGui::TableNextColumn();
                    ImGui::Text("%d", car->GetLap());
                    ImGui::TableNextColumn();
                    ImGui::Text("%.2f", car->GetBestLapTime());
                    ImGui::TableNextColumn();
                    ImGui::Text("+%d/-%d", car->GetOvertakes(), car->GetOvertakenBy());
                }
                ImGui::EndTable();
            }
        }
    }
    ImGui::End();
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
    for (auto &dynamicObstacleRender : m_DynamicObstacleRenders)
        dynamicObstacleRender.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
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
    if (m_RaceLineRender.GetModel())
        m_RaceLineRender.Draw(m_pd3dImmediateContext.Get(), m_BasicEffect);
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

            m_SignalMarkers.push_back({pMarker, node});
        }
    }

    // road 그래프의 successor 연결(진행방향 끝 -> 다음 road의 진행방향 시작)을 노란 선으로 시각화한다.
    // 왕복 도로는 방향마다 나가는 곳이 다르므로 두 방향을 모두 그린다.
    m_RoadEdgeRenders.clear();
    int linkIndex = 0;
    for (const auto &road : m_RoadDataManager.GetRoads())
    {
        if (road->GetReferenceLine().GetSplinePoints().size() < 2)
            continue;
        for (LaneDirection travel : {LaneDirection::Forward, LaneDirection::Backward})
        {
            Vec3 from = m_RoadDataManager.GetTravelEnd(road, travel) + Vec3(0.0f, EDGE_LINE_HEIGHT, 0.0f);
            for (const RoadRef &succ : m_RoadDataManager.GetRoadSuccessors(road->GetId(), travel))
            {
                if (succ.road->GetReferenceLine().GetSplinePoints().size() < 2)
                    continue;
                // 다음 road의 '진행방향 시작' = 반대 방향으로 봤을 때의 끝.
                Vec3 to = m_RoadDataManager.GetTravelEnd(succ.road, GetOppositeDirection(succ.direction)) +
                          Vec3(0.0f, EDGE_LINE_HEIGHT, 0.0f);

                Model *pLine = m_ModelManager.CreateFromGeometry("edge_line" + std::to_string(linkIndex++), Geometry::CreateLine(ToXMFLOAT3(from), ToXMFLOAT3(to)));
                pLine->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 1.0f, 0.0f, 1.0f));
                pLine->materials[0].Set<float>("$Opacity", 1.0f);

                RenderObject &edgeRender = m_RoadEdgeRenders.emplace_back();
                edgeRender.SetModel(pLine);
            }
        }
    }

    // 장애물은 InitObstacleColliders가 만드는 텍스처 박스로 시각화한다(별도 평면 마커 없음).
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

void CarSim::InitDynamicObstacleRenders()
{
    // 정적 장애물(InitObstacleColliders)과 달리 물리 콜라이더는 안 만든다 -- 지금은 Car의 OBB 감지
    // (UpdateSensors/m_obstacles)만 검증하는 테스트용이라, 실제로 부딪혀도 뚫고 지나간다.
    constexpr float OBSTACLE_HEIGHT = 1.5f;

    m_DynamicObstacleRenders.clear();
    int obstacleIndex = 0;
    for (const VehicleCollision::Obstacle &obstacle : m_RoadDataManager.GetDynamicObstacles())
    {
        Model *pCube = m_ModelManager.CreateFromGeometry(
            "dynamic_obstacle_cube" + std::to_string(obstacleIndex++),
            Geometry::CreateBox(obstacle.halfWidth * 2.0f, OBSTACLE_HEIGHT, obstacle.halfLength * 2.0f));
        pCube->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 0.5f, 0.0f, 1.0f));
        pCube->materials[0].Set<float>("$Opacity", 1.0f);

        RenderObject &render = m_DynamicObstacleRenders.emplace_back();
        render.SetModel(pCube);
        render.GetTransform().SetPosition(ToXMFLOAT3(obstacle.center + Vec3(0.0f, OBSTACLE_HEIGHT * 0.5f, 0.0f)));
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

        bool isFirstObstacle = (obstacleIndex == 0);
        Model *pCube = m_ModelManager.CreateFromGeometry(
            "obstacle_cube" + std::to_string(obstacleIndex++),
            Geometry::CreateBox(obstacle.halfWidth * 2.0f, obstacle.height, obstacle.halfLength * 2.0f));
        pCube->materials[0].Set<float>("$Opacity", 1.0f);
        if (isFirstObstacle)
        {
            pCube->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f));
            pCube->materials[0].Set<XMFLOAT4>("$AmbientColor", XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f));
            TextureManager::Get().CreateFromFile("Model\\office.png");
            pCube->materials[0].Set<std::string>("$Diffuse", std::string("Model\\office.png"));
        }
        else
        {
            pCube->materials[0].Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(0.0f, 0.4f, 1.0f, 1.0f));
        }

        auto cube = std::make_shared<GameObject>();
        cube->SetName("Obstacle" + std::to_string(obstacleIndex));
        cube->SetModel(pCube);
        cube->SetPosition(obstacle.center + Vec3(0.0f, obstacle.height * 0.5f, 0.0f)); // 바닥면을 center.y에 맞춘다
        cube->Init(JPH::Vec3(obstacle.halfWidth, obstacle.height * 0.5f, obstacle.halfLength), Rigidbody::Type::Static);
        cube->SetRotation(QuatFromForward(forward));
        m_GameObjects.push_back(cube);
    }
}