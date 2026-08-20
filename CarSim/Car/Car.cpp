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
    // 개체별 성능 편차를 여기서 한 번만 굽는다. 이 차이가 랩타임 차 -> 갭 변화 -> 추월 기회가 된다.
    m_personality = personality;
    m_maxSpeed = BASE_MAX_SPEED * personality.topSpeedFactor;
    m_maxAccel = BASE_MAX_ACCEL * personality.accelFactor;
    m_maxBrake = BASE_MAX_BRAKE * personality.brakeFactor;
    m_gripAccel = BASE_GRIP_ACCEL * personality.gripFactor;
    m_speedCap = m_maxSpeed;
    m_jerkUp = personality.jerkUp;
    m_jerkDown = personality.jerkDown;

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
    // 접촉 목록은 PhysicsSystem::Update가 매 스텝 지운다. 그 Update보다 먼저 도는
    // 여기서 읽어야 직전 스텝의 접촉을 놓치지 않는다.
    DetectContact();
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

    // 레이싱 라인은 포커스 여부와 무관하게 항상 표시한다. 나머지 디버그 표시(트레일,
    // 콜라이더, 반응형 조향 레이 등)는 여전히 포커스된 차만 그린다.
    if (m_racingLineRender.GetModel())
    {
        if (auto *pBasic = dynamic_cast<BasicEffect *>(&effect))
        {
            pBasic->SetRenderLines();
            m_racingLineRender.Draw(context, effect);
            pBasic->SetRenderDefault();
        }
    }

    if (!m_drawCollider)
        return;

    // 콜라이더 와이어프레임 박스 표시를 껐다. 트레일/레이싱 라인/조향 표시는 그대로 켜져 있다.
    constexpr bool DRAW_COLLIDER_BOX = false;
    if (DRAW_COLLIDER_BOX && m_debugBox.GetModel())
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
         m_ctxSteerOpenRender.GetModel() || m_ctxSteerBlockedRender.GetModel()))
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

    // m_maxSpeed가 아니라 m_speedCap이다 -- 슬립스트림이 얹힌 상한을 여기서 자르면
    // 와류 이득이 통째로 사라진다. AI가 아닐 땐 m_speedCap == m_maxSpeed.
    m_speed = std::clamp(m_speed, 0.0f, m_speedCap);

    m_maxSteerAngle = CalcMaxSteerAngle(m_speed, m_wheelbase, m_gripAccel);
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
    // 접촉 시 속도를 0으로 만들던 처리를 뺐다.
    //
    // 한 프레임 만에 100km/h에서 정지해 버리면 (1) 뒤차는 물리적으로 피할 방법이 없어
    // 연쇄 추돌이 나고, (2) 서로 닿은 채 멈춘 두 대가 영원히 못 벗어나 랩을 끝내지 못했다.
    // 접촉 자체는 DetectContact가 기록하므로 여기서 주행을 멈출 이유가 없다.
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

        ImGui::Text("Overtake: %s  side %+.0f  offset %+.2f m", OvertakeStateToString(m_overtakeState),
                    m_overtakeSide, m_raceLateralOffset);
        if (m_overtakeState == OvertakeState::Follow && m_overtakeBlock[0] != 0)
            ImGui::Text("  blocked by: %s", m_overtakeBlock);
        if (m_overtakeState == OvertakeState::Setup || m_overtakeState == OvertakeState::Attack)
            ImGui::Text("  t %.1fs  stall %.1fs", m_overtakeTimer, m_overtakeStall);
        if (m_overtakeHoldOffset)
            ImGui::Text("  holding offset (still overlapping)%s", m_overtakeBackOff ? " + backing off" : "");
        if (m_overtakeCooldown > 0.0f)
            ImGui::Text("  cooldown %.1f s", m_overtakeCooldown);

        if (m_leader.IsValid() && m_SimState->IsCarAlive(m_leader.car))
            ImGui::Text("Leader: %s  gap %.1f m  closing %+.1f km/h  dOffset %+.2f m",
                        m_leader.car->GetName().c_str(), m_leader.gap, m_leader.closingSpeed * 3.6f,
                        m_leader.lateralOffset - m_leader.myLateralOffset);
        else
            ImGui::Text("Leader: none");

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
        ImGui::Text("Driver / Car");
        ImGui::Text("Top speed: %.0f km/h (x%.3f)", m_maxSpeed * 3.6f, m_personality.topSpeedFactor);
        ImGui::Text("Grip: %.2f m/s^2 (x%.3f)", m_gripAccel, m_personality.gripFactor);
        ImGui::Text("Accel: %.2f / Brake: %.2f m/s^2", m_maxAccel, m_maxBrake);
        ImGui::Text("Headway: x%.2f", m_personality.headwayFactor);
        if (m_slipstream > 0.005f)
            ImGui::Text("Slipstream: %.0f%% (gap %.1f m) -> cap %.0f km/h", m_slipstream * 100.0f,
                        m_slipstreamGapDebug, m_speedCap * 3.6f);
        else
            ImGui::Text("Slipstream: clean air");
        ImGui::SliderFloat("Jerk Up Max", &m_jerkUp, 5.0f, 50.0f);
        ImGui::SliderFloat("Jerk Down Max", &m_jerkDown, 10.0f, 100.0f);
    }
    ImGui::End();
}

void Car::UpdateRacePosition(int position, float now)
{
    if (position != m_pendingPosition)
    {
        m_pendingPosition = position;
        m_pendingPositionSince = now;
        return;
    }
    if (position == m_racePosition)
        return;

    // 접전 중에는 순위가 프레임마다 뒤집힌다. 잠깐 앞섰다 돌아온 걸 추월로 세면
    // 카운트가 의미 없이 부풀려지므로, 일정 시간 유지될 때만 확정한다.
    constexpr float POSITION_COMMIT_TIME = 0.75f;
    if (now - m_pendingPositionSince < POSITION_COMMIT_TIME)
        return;

    if (m_racePosition > 0)
    {
        const int gained = m_racePosition - position;
        if (gained > 0)
            m_overtakes += gained;
        else
            m_overtakenBy += -gained;
    }
    m_racePosition = position;
}

void Car::OnLapCompleted()
{
    const float now = m_SimState->GetSimTime();
    if (m_lapStartTime >= 0.0f) // 스폰 지점이 결승선이 아니므로 첫 통과는 계측하지 않는다
    {
        m_lastLapTime = now - m_lapStartTime;
        if (m_bestLapTime <= 0.0f || m_lastLapTime < m_bestLapTime)
            m_bestLapTime = m_lastLapTime;
        ++m_lap;
        m_SimState->LogLap(*this, m_lastLapTime);
    }
    else
    {
        ++m_lap;
    }
    m_lapStartTime = now;
}

void Car::DetectContact()
{
    JPH::BodyID otherBody;
    if (!PhysicsSystem::Get().GetNewContact(m_rigidbody.GetBodyID(), otherBody))
        return;

    Car *other = m_SimState->FindCarByBody(otherBody);
    if (other == nullptr || other == this)
        return; // 도로/장애물 접촉은 여기 관심사가 아니다

    m_lastContactCar = other;
    m_lastContactTime = m_SimState->GetSimTime();
    if (m_SimState->ShouldLogContact(GetId(), other->GetId()))
        m_SimState->LogContact(*this, *other);
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

void Car::BuildRaceLineRenderObject(const RaceLine &line, const std::string &modelKey, RenderObject &out,
                                    const DirectX::XMFLOAT4 &color)
{
    if (line.points.size() < 2 || line.lapPoints < 2)
    {
        out.SetModel(nullptr);
        return;
    }

    constexpr float RACING_LINE_HEIGHT = 0.35f; // 다른 디버그선 위로

    // 라인이 전략별로 공유되므로 모델도 전략마다 하나면 된다. 이름을 전략으로 잡아
    // ModelManager가 재사용하게 한다 -- 차마다(그리고 차 없이 그리는 CarSim도) 트랙
    // 전체를 다시 만들 이유가 없다.
    std::vector<DirectX::XMFLOAT3> points;
    points.reserve(line.lapPoints);
    for (size_t i = 0; i < line.lapPoints; ++i)
    {
        DirectX::XMFLOAT3 p = ToXMFLOAT3(line.points[i]);
        p.y += RACING_LINE_HEIGHT;
        points.push_back(p);
    }
    if (points.size() < 2)
    {
        out.SetModel(nullptr);
        return;
    }

    Model *pModel = ModelManager::Get().CreateFromGeometry("__racing_line__:" + modelKey,
                                                           Geometry::CreatePolyline(points));
    pModel->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", color);
    pModel->materials[0].Set<float>("$Opacity", 1.0f);
    out.SetModel(pModel);
}

void Car::RebuildRacingLineRender()
{
    if (m_raceLine == nullptr)
    {
        m_racingLineRender.SetModel(nullptr);
        return;
    }
    BuildRaceLineRenderObject(*m_raceLine, std::to_string(static_cast<int>(m_boundStrategy)),
                              m_racingLineRender);
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
    // 컨텍스트 조향 안전/위험 레이 디버그 표시를 껐다. 필요할 때 이 return만 지우면 된다.
    return;
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
