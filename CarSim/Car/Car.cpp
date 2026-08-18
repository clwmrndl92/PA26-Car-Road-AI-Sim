#include "Car.h"
#include "Core/Physics/PhysicsSystem.h"
#include "Rendering/Effects.h"
#include "Nav/SimulationState.h"
#include "Nav/TrafficSignal.h"
#include <ModelManager.h>
#include <algorithm>
#include <cmath>
#include <utility>
#include <imgui.h>
#include "Utill/DebugConsole.h"

void Car::Init(const CarSpec &spec, const CarPersonality &personality, SimulationState *simState, JPH::Vec3 position,
                float aiStartDelay)
{
    m_aiStartDelay = aiStartDelay;
    m_aiDelayTimer = 0.0f;

    SetName(spec.name);
    m_carModel = ModelManager::Get().CreateFromFile(spec.modelPath);
    m_render.SetModel(m_carModel);
    SetRenderOffset(ToXMFLOAT3(spec.renderOffset));
    m_wheelbase = spec.wheelbase;
    m_halfExtents = spec.halfExtents;
    m_personality = personality;

    DirectX::XMFLOAT3 fwd = m_transform.GetForwardAxis();
    m_transform.SetPosition(position.GetX() - fwd.x * m_wheelbase,
                            position.GetY() - fwd.y * m_wheelbase,
                            position.GetZ() - fwd.z * m_wheelbase);

    constexpr JPH::EAllowedDOFs carDOFs = JPH::EAllowedDOFs::TranslationX | JPH::EAllowedDOFs::TranslationY |
                                          JPH::EAllowedDOFs::TranslationZ | JPH::EAllowedDOFs::RotationY;
    GameObject::Init(spec.halfExtents, Rigidbody::Type::Dynamic, spec.colliderOffset, spec.mass, carDOFs);

    m_spawnPosition = m_transform.GetPosition();
    m_spawnRotation = m_transform.GetRotationQuat();
    m_mass = spec.mass;

    m_SimState = simState;
    m_SimState->RegisterCar(this);

    DebugInit();
}

void Car::Update(float dt)
{
    m_deltaTime = dt;
    m_currentTime += dt;

    if (m_isControl)
    {
        UpdateWithControl();
        return;
    }

    if (IsAiDelayed())
    {
        m_aiDelayTimer += dt;
        return;
    }

    UpdateMode();
    m_wantSegmentTick = false;
    switch (m_mode)
    {
    case Mode::Stop:
        UpdateStop();
        break;
    case Mode::Drive:
        UpdateDrive();
        break;
    }
    UpdateHorn(dt);
}

void Car::UpdatePhysics(float dt)
{
    m_deltaTime = dt;
    if (IsAiDelayed())
        return;
    if (m_wantSegmentTick)
        m_vehicleController.Tick(*this);
    UpdateCar();
    ApplyMotion();
}

void Car::UpdateUI(float dt)
{
    if (IsAiDelayed())
        return;
    UpdateTrail();
    UpdateDebugWindow();
}

void Car::Draw(ID3D11DeviceContext *context, IEffect &effect)
{
    using namespace DirectX;

    bool honking = m_hornFlashTimer > 0.0f && m_carModel != nullptr;
    std::vector<std::pair<size_t, XMFLOAT4>> savedDiffuse;
    if (honking)
    {
        for (size_t i = 0; i < m_carModel->materials.size(); ++i)
        {
            Material &mat = m_carModel->materials[i];
            if (!mat.Has<XMFLOAT4>("$DiffuseColor"))
                continue;
            savedDiffuse.emplace_back(i, mat.Get<XMFLOAT4>("$DiffuseColor"));
            mat.Set<XMFLOAT4>("$DiffuseColor", XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f));
        }
    }
    GameObject::Draw(context, effect);
    for (const std::pair<size_t, XMFLOAT4> &saved : savedDiffuse)
        m_carModel->materials[saved.first].Set<XMFLOAT4>("$DiffuseColor", saved.second);

    if (!m_drawCollider)
        return;

    if (m_debugBox.GetModel())
    {
        XMVECTOR colliderOffsetWorld = XMVector3Rotate(XMLoadFloat3(&m_colliderOffset), m_transform.GetRotationQuatXM());
        XMFLOAT3 colliderPos;
        XMStoreFloat3(&colliderPos, XMVectorAdd(m_transform.GetPositionXM(), colliderOffsetWorld));

        m_debugBox.GetTransform().SetPosition(colliderPos);
        m_debugBox.GetTransform().SetRotation(m_transform.GetRotationQuat());

        if (auto *pBasic = dynamic_cast<BasicEffect *>(&effect))
        {
            pBasic->SetRenderWireframe();
            m_debugBox.Draw(context, effect);
            pBasic->SetRenderDefault();
        }
    }

    if ((m_rearTrailRender.GetModel() || m_frontTrailRender.GetModel() || m_splineRender.GetModel() ||
         m_ctxSteerOpenRender.GetModel() || m_ctxSteerBlockedRender.GetModel() ||
         m_racingLineRender.GetModel()))
    {
        if (auto *pBasic = dynamic_cast<BasicEffect *>(&effect))
        {
            pBasic->SetRenderLines();
            if (m_rearTrailRender.GetModel())
                m_rearTrailRender.Draw(context, effect);
            if (m_frontTrailRender.GetModel())
                m_frontTrailRender.Draw(context, effect);
            if (m_splineRender.GetModel())
                m_splineRender.Draw(context, effect);
            if (m_ctxSteerOpenRender.GetModel())
                m_ctxSteerOpenRender.Draw(context, effect);
            if (m_ctxSteerBlockedRender.GetModel())
                m_ctxSteerBlockedRender.Draw(context, effect);
            if (m_racingLineRender.GetModel())
                m_racingLineRender.Draw(context, effect);
            pBasic->SetRenderDefault();
        }
    }

    if (m_steerLine.GetModel())
    {
        XMFLOAT4 carRotF = m_transform.GetRotationQuat();
        XMVECTOR carRot = XMLoadFloat4(&carRotF);
        XMVECTOR steerYaw = XMQuaternionRotationAxis(g_XMIdentityR1, m_steerAngle);
        XMVECTOR lineRot = XMQuaternionNormalize(XMQuaternionMultiply(carRot, steerYaw));

        XMFLOAT4 lineRotF;
        XMStoreFloat4(&lineRotF, lineRot);
        m_steerLine.GetTransform().SetPosition(ToXMFLOAT3(GetPosition()));
        m_steerLine.GetTransform().SetRotation(lineRotF);

        if (auto *pBasic = dynamic_cast<BasicEffect *>(&effect))
        {
            pBasic->SetRenderLines();
            m_steerLine.Draw(context, effect);
            pBasic->SetRenderDefault();
        }
    }

    if (m_targetMarker.GetModel())
        m_targetMarker.Draw(context, effect);
}

Vec3 Car::GetPosition() const
{
    DirectX::XMFLOAT3 rear = m_transform.GetPosition();
    DirectX::XMFLOAT3 fwd = m_transform.GetForwardAxis();
    return Vec3(rear.x + fwd.x * m_wheelbase, rear.y + fwd.y * m_wheelbase, rear.z + fwd.z * m_wheelbase);
}
Vec3 Car::GetForwardAxis() const
{
    return ToVec3(m_transform.GetForwardAxis());
}

void Car::SetPosition(Vec3 position)
{
    DirectX::XMFLOAT3 fwd = m_transform.GetForwardAxis();
    GameObject::SetPosition(Vec3(position.GetX() - fwd.x * m_wheelbase,
                                 position.GetY() - fwd.y * m_wheelbase,
                                 position.GetZ() - fwd.z * m_wheelbase));
}

void Car::SetRotation(Vec3 direction)
{
    float yaw = std::atan2(direction.GetX(), direction.GetZ());
    DirectX::XMFLOAT4 rotation;
    DirectX::XMStoreFloat4(&rotation, DirectX::XMQuaternionRotationRollPitchYaw(0.0f, yaw, 0.0f));
    Vec3 frontAxle = GetPosition();
    GameObject::SetRotation(rotation);
    SetPosition(frontAxle);
}

void Car::AccelerateVel(float desiredVelocity)
{
    Accelerate(m_speedGain * (desiredVelocity - m_speed));
}
void Car::Accelerate(float desiredAccel)
{
    float aTarget = std::clamp(desiredAccel, -m_maxBrake, m_maxAccel);
    if ((aTarget >= 0.0f) != (m_acceleration >= 0.0f))
        m_acceleration = 0.0f;

    float jerkLimit = (aTarget > m_acceleration) ? m_jerkUp : m_jerkDown;
    float maxStep = jerkLimit * m_deltaTime;
    m_acceleration += std::clamp(aTarget - m_acceleration, -maxStep, maxStep);
    m_planAccelDebug = m_acceleration;
}

void Car::Steer(float radian, float steerRamp)
{
    float maxDelta = steerRamp * m_deltaTime;
    if (m_steerAngle > radian)
        m_steerAngle = std::max(m_steerAngle - maxDelta, radian);
    else if (m_steerAngle < radian)
        m_steerAngle = std::min(m_steerAngle + maxDelta, radian);
}

void Car::ChangeGear()
{
    constexpr float GEAR_SWITCH_SPEED_THRESHOLD = 2.0f / 3.6f;

    if (m_speed <= GEAR_SWITCH_SPEED_THRESHOLD)
        m_isReverse = !m_isReverse;
}

void Car::Destroy()
{
    if (m_SimState != nullptr)
        m_SimState->UnregisterCar(this);
    GameObject::Destroy();
}

void Car::SetCurrentRoad(const shared_ptr<Road> &road, float offset)
{
    if (m_currentRoad == road && m_currentOffset == offset)
        return;

    m_currentRoad = road;
    SetCurrentOffset(offset);
    m_currentSpline = RoadDataManager::Get().BuildOffsetSpline(road, offset);

    RebuildSplineRender();
}

void Car::SetCurrentOffset(float offset)
{
    m_currentOffset = offset;
    m_currentBand = RoadDataManager::Get().FindNearestBand(m_currentRoad, m_currentOffset);
}

void Car::UpdateCar()
{
    constexpr float FRICT_DECEL_RATE = 0.1f;

    if (m_acceleration == 0.0f)
    {
        m_speed -= m_speed * FRICT_DECEL_RATE * m_deltaTime;

        if (m_speed < 0.1f)
            m_speed = 0.0f;
    }
    else
        m_speed += m_acceleration * m_deltaTime;

    m_speed = std::clamp(m_speed, 0.0f, m_maxSpeed);

    m_maxSteerAngle = CalcMaxSteerAngle(m_speed, m_wheelbase);
    m_steerAngle = std::clamp(m_steerAngle, -m_maxSteerAngle, m_maxSteerAngle);
}

void Car::UpdateWithControl()
{
    if (!m_isFocused)
        return;

    if (ImGui::IsKeyPressed(ImGuiKey_Space, false))
    {
        m_rigidbody.SetPositionAndRotation(
            JPH::Vec3(m_spawnPosition.x, m_spawnPosition.y, m_spawnPosition.z),
            JPH::Quat(m_spawnRotation.x, m_spawnRotation.y, m_spawnRotation.z, m_spawnRotation.w));
        m_rigidbody.SetLinearVelocity(JPH::Vec3::sZero());
        m_rigidbody.SetAngularVelocity(JPH::Vec3::sZero());
        m_speed = 0.0f;
        m_acceleration = 0.0f;

        m_rearTrail.clear();
        m_frontTrail.clear();
        m_rearTrailRender.SetModel(nullptr);
        m_frontTrailRender.SetModel(nullptr);
    }

    if (ImGui::IsKeyPressed(ImGuiKey_Z, false))
        ChangeGear();

    if (m_isFocused && ImGui::IsKeyDown(ImGuiKey_DownArrow))
        Accelerate(-m_maxBrake);
    else if (m_isFocused && ImGui::IsKeyDown(ImGuiKey_UpArrow))
        Accelerate(m_maxAccel);
    else
        AccelerateVel(0);

    if (m_isFocused && ImGui::IsKeyDown(ImGuiKey_LeftArrow))
        Steer(-1);
    else if (m_isFocused && ImGui::IsKeyDown(ImGuiKey_RightArrow))
        Steer(1);
    else
        Steer(0);
}

void Car::ApplyMotion()
{
    JPH::BodyID otherId;
    if (PhysicsSystem::Get().GetNewContact(m_rigidbody.GetBodyID(), otherId))
    {
        float vy = m_rigidbody.GetLinearVelocity().GetY();
        m_rigidbody.SetLinearVelocity(JPH::Vec3(0.0f, vy, 0.0f));
        m_rigidbody.SetAngularVelocity(JPH::Vec3::sZero());
        m_acceleration = 0.0f;
        m_speed = 0.0f;
        DebugConsole::Log(GetName() + ": CRASH!!");
        return;
    }

    float angularVelocity = GetSignedSpeed() * tan(m_steerAngle) / m_wheelbase;
    m_rigidbody.SetAngularVelocity(JPH::Vec3(0.0f, angularVelocity, 0.0f));
    m_rigidbody.SetLinearVelocity(ComputeDesiredVelocity());
}

JPH::Vec3 Car::ComputeDesiredVelocity() const
{
    float signedSpeed = GetSignedSpeed();
    DirectX::XMFLOAT3 fwd = m_transform.GetForwardAxis();
    float vy = m_rigidbody.GetLinearVelocity().GetY();
    return JPH::Vec3(fwd.x * signedSpeed, vy > 0.0f ? 0.0f : vy, fwd.z * signedSpeed);
}

float Car::Stanley(Vec3 pathPoint, Vec3 pathDirection)
{
    Vec3 frontAxle = GetPosition();
    Vec3 carFwd = ToVec3(m_transform.GetForwardAxis()).Normalized();
    Vec3 carRight = ToVec3(m_transform.GetRightAxis()).Normalized();
    Vec3 pathDir = pathDirection.LengthSq() > 0.000001f ? pathDirection.Normalized() : carFwd;

    float headingError = atan2f(carRight.Dot(pathDir), carFwd.Dot(pathDir));

    float crossTrackRight = (frontAxle - pathPoint).Dot(carRight);
    float crossTrackTerm = atanf(m_stanleyGain * -crossTrackRight / (m_speed + m_stanleySoft));

    return std::clamp(headingError + crossTrackTerm, -m_maxSteerAngle, m_maxSteerAngle);
}

void Car::UpdateHorn(float dt)
{
    constexpr float HORN_INTERVAL = 5.0f;
    constexpr float HORN_FLASH_DURATION = 2.0f;

    if (IsHornSituation())
    {
        m_hornStoppedDuration += dt;
        if (m_hornStoppedDuration >= HORN_INTERVAL)
        {
            m_hornStoppedDuration = 0.0f;
            m_hornFlashTimer = HORN_FLASH_DURATION;
        }
    }
    else
        m_hornStoppedDuration = 0.0f;

    if (m_hornFlashTimer > 0.0f)
        m_hornFlashTimer -= dt;
}

bool Car::IsHornSituation() const
{
    if (m_mode != Mode::Drive)
        return false;
    if (m_speed > HORN_STOP_SPEED)
        return false;
    return !KnowsRedSignalAhead();
}

bool Car::KnowsRedSignalAhead() const
{
    constexpr float HORN_SIGNAL_AWARE_DISTANCE = 40.0f;

    for (size_t i = m_pathIndex; i < m_path.size(); ++i)
    {
        const RoadRef &road = m_path[i];
        int nextRoadId = (i + 1 < m_path.size()) ? m_path[i + 1].road->GetId() : -1;
        shared_ptr<RoadNode> signalNode = RoadDataManager::Get().GetSignalNodeForRoad(road.road->GetId(), nextRoadId);
        if (signalNode == nullptr)
            continue;

        if (i == m_pathIndex)
        {
            const Spline &spline = road.road->GetReferenceLine();
            if (spline.GetSplinePosition(GetPosition()) > spline.GetSplinePosition(signalNode->position))
                continue;
        }

        if ((signalNode->position - GetPosition()).Length() > HORN_SIGNAL_AWARE_DISTANCE)
            return false;
        return m_SimState->GetSignalColor(signalNode->signalPhaseOffset, signalNode->signalGreenDuration,
                                          signalNode->signalYellowDuration, signalNode->signalRedDuration) ==
               TrafficSignal::Color::Red;
    }
    return false;
}
#pragma region Debug
void Car::UpdateDebugWindow()
{
    if (!m_drawCollider || !m_isFocused)
        return;

    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x, 0.0f), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
    if (ImGui::Begin(("Car: " + GetName()).c_str()))
    {
        DirectX::XMFLOAT3 pos = m_transform.GetPosition();
        ImGui::Text("Pos: %.1f %.1f %.1f", pos.x, pos.y, pos.z);
        ImGui::Text("Speed: %.1f km/h", m_speed * 3.6f);
        ImGui::Text("Accel: %.1f km/h/s", m_acceleration * 3.6f);
        ImGui::Text("Steer: %.2f / %.2f", m_steerAngle, m_maxSteerAngle);
        ImGui::Text("Current Road: %d", m_currentRoad != nullptr ? m_currentRoad->GetId() : -1);
        RoadRef nextRoad = (m_pathIndex + 1 < m_path.size()) ? m_path[m_pathIndex + 1] : RoadRef{};
        ImGui::Text("Next Road: %d", nextRoad.road != nullptr ? nextRoad.road->GetId() : -1);
        if (m_mode == Mode::Drive)
            ImGui::Text("Mode: %s / %s", StateToString(m_mode), SubStateToString(m_subMode));
        else
            ImGui::Text("Mode: %s", StateToString(m_mode));

        ImGui::Text("Plan accel: %.2f m/s^2", m_planAccelDebug);
        ImGui::Text("Cur offset d: %.2f m", m_currentOffset);

        if (UsesReactiveSteer())
            ImGui::Text("ReactiveSteer: slot %.0f deg, danger %.2f", ToDegrees(m_ctxSteerDebugRad),
                        m_ctxSteerDebugDanger);

        ImGui::Separator();
        ImGui::Text("Racing Strategy");
        static const char *apexNames[] = {"1. Early Apex", "2. Geometric Apex", "3. Late Apex"};
        int apexIndex = static_cast<int>(m_apexStrategy);
        if (ImGui::Combo("Apex", &apexIndex, apexNames, IM_ARRAYSIZE(apexNames)))
            m_apexStrategy = static_cast<ApexStrategy>(apexIndex);

        ImGui::Separator();
        ImGui::Text("Personality (notes/accel.txt A~D)");
        ImGui::SliderFloat("Jerk Up Max", &m_jerkUp, 5.0f, 50.0f);
        ImGui::SliderFloat("Jerk Down Max", &m_jerkDown, 10.0f, 100.0f);
    }
    ImGui::End();
}

void Car::UpdateTrail()
{
    if (!m_drawCollider)
        return;

    constexpr float TRAIL_SAMPLE_DISTANCE = 0.5f;
    constexpr size_t TRAIL_MAX_POINTS = 100;

    using namespace DirectX;

    XMFLOAT3 rearPos = m_transform.GetPosition();
    XMFLOAT3 fwd = m_transform.GetForwardAxis();
    XMFLOAT3 frontPos(rearPos.x + fwd.x * m_wheelbase, rearPos.y + fwd.y * m_wheelbase, rearPos.z + fwd.z * m_wheelbase);

    auto recordPoint = [TRAIL_SAMPLE_DISTANCE, TRAIL_MAX_POINTS](std::deque<XMFLOAT3> &trail, const XMFLOAT3 &pos)
    {
        if (!trail.empty())
        {
            XMVECTOR diff = XMVectorSubtract(XMLoadFloat3(&pos), XMLoadFloat3(&trail.back()));
            if (XMVectorGetX(XMVector3LengthSq(diff)) < TRAIL_SAMPLE_DISTANCE * TRAIL_SAMPLE_DISTANCE)
                return false;
        }
        trail.push_back(pos);
        if (trail.size() > TRAIL_MAX_POINTS)
            trail.pop_front();
        return true;
    };

    if (recordPoint(m_rearTrail, rearPos) && m_rearTrail.size() >= 2)
        RebuildTrailRender(m_rearTrailRender, m_rearTrail, "__rear_trail__:" + GetName(), XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f));

    if (recordPoint(m_frontTrail, frontPos) && m_frontTrail.size() >= 2)
        RebuildTrailRender(m_frontTrailRender, m_frontTrail, "__front_trail__:" + GetName(), XMFLOAT4(0.0f, 1.0f, 1.0f, 1.0f));
}

void Car::RebuildTrailRender(RenderObject &render, const std::deque<DirectX::XMFLOAT3> &trail,
                             const std::string &name, const DirectX::XMFLOAT4 &color)
{
    constexpr float TRAIL_LINE_HEIGHT = 0.15f;

    std::vector<DirectX::XMFLOAT3> points(trail.begin(), trail.end());
    for (DirectX::XMFLOAT3 &point : points)
        point.y += TRAIL_LINE_HEIGHT;

    Model *pModel = ModelManager::Get().CreateFromGeometry(name, Geometry::CreatePolyline(points));
    pModel->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", color);
    pModel->materials[0].Set<float>("$Opacity", 1.0f);
    render.SetModel(pModel);
}

void Car::RebuildSplineRender()
{
    if (m_currentRoad == nullptr)
    {
        m_splineRender.SetModel(nullptr);
        return;
    }

    const std::vector<Vec3> &splinePoints = m_currentSpline.GetSplinePoints();
    if (splinePoints.size() < 2)
    {
        m_splineRender.SetModel(nullptr);
        return;
    }

    constexpr float SPLINE_LINE_HEIGHT = 0.15f;

    std::vector<DirectX::XMFLOAT3> points;
    points.reserve(splinePoints.size());
    for (const Vec3 &point : splinePoints)
    {
        DirectX::XMFLOAT3 p = ToXMFLOAT3(point);
        p.y += SPLINE_LINE_HEIGHT;
        points.push_back(p);
    }

    Model *pModel = ModelManager::Get().CreateFromGeometry("__current_spline__:" + GetName(), Geometry::CreatePolyline(points));
    pModel->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f));
    pModel->materials[0].Set<float>("$Opacity", 1.0f);
    m_splineRender.SetModel(pModel);
}

void Car::RebuildRacingLineRender()
{
    if (m_raceLine.points.size() < 2)
    {
        m_racingLineRender.SetModel(nullptr);
        return;
    }

    constexpr float RACING_LINE_HEIGHT = 0.35f; // 다른 디버그선 위로

    // 문맥용으로만 붙인 앞뒤 도로는 빼고 그린다
    std::vector<DirectX::XMFLOAT3> points;
    points.reserve(m_raceLine.points.size());
    for (size_t i = 0; i < m_raceLine.points.size(); ++i)
    {
        float s = m_raceLine.arcLength[i];
        if (s < m_raceLine.drawFromS || s > m_raceLine.drawToS)
            continue;
        DirectX::XMFLOAT3 p = ToXMFLOAT3(m_raceLine.points[i]);
        p.y += RACING_LINE_HEIGHT;
        points.push_back(p);
    }
    if (points.size() < 2)
    {
        m_racingLineRender.SetModel(nullptr);
        return;
    }

    Model *pModel = ModelManager::Get().CreateFromGeometry("__racing_line__:" + GetName(),
                                                           Geometry::CreatePolyline(points));
    pModel->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(0.0f, 0.3f, 1.0f, 1.0f));
    pModel->materials[0].Set<float>("$Opacity", 1.0f);
    m_racingLineRender.SetModel(pModel);
}

namespace
{
    void BuildLineListModel(RenderObject &render, const std::vector<Vec3> &points, const std::string &modelKey,
                            DirectX::XMFLOAT4 color, float debugLineHeight)
    {
        if (points.empty())
        {
            render.SetModel(nullptr);
            return;
        }

        GeometryData geoData;
        geoData.vertices.reserve(points.size());
        for (const Vec3 &point : points)
        {
            DirectX::XMFLOAT3 p = ToXMFLOAT3(point);
            p.y += debugLineHeight;
            geoData.vertices.push_back(p);
        }
        geoData.normals.assign(geoData.vertices.size(), DirectX::XMFLOAT3(0.0f, 1.0f, 0.0f));
        geoData.texcoords.assign(geoData.vertices.size(), DirectX::XMFLOAT2(0.0f, 0.0f));

        std::vector<uint32_t> indices;
        size_t lineCount = points.size() / 2;
        indices.reserve(lineCount * 2);
        for (size_t line = 0; line < lineCount; ++line)
        {
            indices.push_back(static_cast<uint32_t>(line * 2));
            indices.push_back(static_cast<uint32_t>(line * 2 + 1));
        }
        geoData.indices16.assign(indices.begin(), indices.end());

        Model *pModel = ModelManager::Get().CreateFromGeometry(modelKey, geoData);
        pModel->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", color);
        pModel->materials[0].Set<float>("$Opacity", 1.0f);
        render.SetModel(pModel);
    }
}

void Car::RebuildCtxSteerRender()
{
    constexpr float DEBUG_LINE_HEIGHT = 0.25f; // 다른 디버그 레이보다 살짝 위(겹침 방지)
    BuildLineListModel(m_ctxSteerOpenRender, m_ctxSteerOpenLines, "__ctxsteer_open__:" + GetName(),
                       DirectX::XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f), DEBUG_LINE_HEIGHT);
    BuildLineListModel(m_ctxSteerBlockedRender, m_ctxSteerBlockedLines, "__ctxsteer_blocked__:" + GetName(),
                       DirectX::XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f), DEBUG_LINE_HEIGHT);
}

void Car::DebugInit()
{
    float w = m_halfExtents.GetX() * 2.0f;
    float h = m_halfExtents.GetY() * 2.0f;
    float d = m_halfExtents.GetZ() * 2.0f;
    Model *pBox = ModelManager::Get().CreateFromGeometry("__collider__:" + GetName(), Geometry::CreateBox(w, h, d));
    pBox->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f));
    pBox->materials[0].Set<float>("$Opacity", 1.0f);
    m_debugBox.SetModel(pBox);

    Model *pLine = ModelManager::Get().CreateFromGeometry("__steer_line__:" + GetName(),
                                                          Geometry::CreateLine(DirectX::XMFLOAT3(0.0f, 0.15f, 0.0f), DirectX::XMFLOAT3(0.0f, 0.15f, 6.0f)));
    pLine->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(0.0f, 0.0f, 1.0f, 1.0f));
    pLine->materials[0].Set<float>("$Opacity", 1.0f);
    m_steerLine.SetModel(pLine);

    constexpr float TARGET_MARKER_SIZE = 0.5f;
    Model *pTargetMarker = ModelManager::Get().CreateFromGeometry("__target_marker__:" + GetName(),
                                                                  Geometry::CreatePlane(TARGET_MARKER_SIZE, TARGET_MARKER_SIZE));
    pTargetMarker->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f));
    pTargetMarker->materials[0].Set<float>("$Opacity", 1.0f);
    m_targetMarker.SetModel(pTargetMarker);
}
#pragma endregion
