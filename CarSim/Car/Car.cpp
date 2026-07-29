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
#include "Utill/Assert.h"

void Car::Init(const CarSpec &spec, SimulationState *simState, JPH::Vec3 position)
{
    SetName(spec.name);
    m_carModel = ModelManager::Get().CreateFromFile(spec.modelPath);
    m_render.SetModel(m_carModel);
    SetRenderOffset(ToXMFLOAT3(spec.renderOffset));
    m_wheelbase = spec.wheelbase;
    m_halfExtents = spec.halfExtents;
    m_behaviorWeights = spec.behaviorWeights;

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

    m_parkSpot = make_shared<RoadNode>();
    m_parkSpot->id = -1;
    m_parkSpot->position = GetPosition();
    m_parkSpot->direction = GetForwardAxis();
    m_parkSpot->nodeType = RoadNodeType::ParkSpot;

    // DEBUG
    DebugInit();
}

void Car::Update(float dt)
{
    m_deltaTime = dt;
    m_currentTime += dt;
    UpdateMode();
    m_wantSegmentTick = false;
    switch (m_mode)
    {
    case Mode::Stop:
        UpdateStop();
        break;
    case Mode::Park:
        UpdatePark();
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
    if (m_wantSegmentTick)
        m_vehicleController.Tick(*this);
    UpdateCar();
    ApplyMotion();
}

void Car::UpdateUI(float dt)
{
    UpdateTrail();
    UpdateDebugWindow();
}

void Car::Draw(ID3D11DeviceContext *context, IEffect &effect)
{
    using namespace DirectX;

    // 경적 중이면 차체 재질을 잠깐 빨갛게 칠해 그리고 곧바로 원복한다. 모델은 같은 modelPath끼리
    // 공유되지만 Draw는 차마다 순차 호출이라, 이 차의 GameObject::Draw 동안만 빨간색이 적용된다.
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
         m_parkPathRender.GetModel() || m_parkTargetLine.GetModel()))
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
            if (m_parkPathRender.GetModel())
                m_parkPathRender.Draw(context, effect);
            if (m_parkTargetLine.GetModel())
                m_parkTargetLine.Draw(context, effect);
            pBasic->SetRenderDefault();
        }
    }

    if (m_steerLine.GetModel())
    {
        // Car only ever yaws around world Y, so the steer-angle offset and the car's own
        // rotation share an axis and can be combined in either order.
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
    if (m_parkTargetMarker.GetModel())
        m_parkTargetMarker.Draw(context, effect);
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
    Vec3 frontAxle = GetPosition(); // capture using the OLD rotation, before it changes
    GameObject::SetRotation(rotation);
    SetPosition(frontAxle); // re-derive the rear axle using the NEW rotation
}

void Car::EmergBrake()
{
    m_acceleration = -m_maxEmergBrake;
}

void Car::Accelerate(float desiredVelocity, float aFF)
{
    // 계획감속 피드포워드(접근 중일 때만) + 속도오차 비례 피드백. FF가 기준 감속을 미리 넣어 접근 시 P제어 lag를 없앤다.
    float ff = (desiredVelocity < m_speed) ? aFF : 0.0f; // vRef 도달 순간 FF를 꺼 아래로 오버슈트 방지
    float aTarget = std::clamp(ff + m_speedGain * (desiredVelocity - m_speed), -m_maxBrake, m_maxAccel);
    float jerkLimit = (aTarget > m_acceleration) ? m_jerkUp : m_jerkDown;
    float maxStep = jerkLimit * m_deltaTime;
    m_acceleration += std::clamp(aTarget - m_acceleration, -maxStep, maxStep);
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
    constexpr float GEAR_SWITCH_SPEED_THRESHOLD = 2.0f / 3.6f; // 2 km/h

    if (m_speed <= GEAR_SWITCH_SPEED_THRESHOLD) // Toggle Drive / Reverse gear
        m_isReverse = !m_isReverse;
}

void Car::Destroy()
{
    if (m_SimState != nullptr)
    {
        if (m_parkSpot != nullptr)
            m_SimState->ReleaseParkSpot(m_parkSpot->id); // 예약된 자리를 든 채로 제거되면 그 자리가 영원히 잠기므로 먼저 반환한다.
        m_SimState->UnregisterCar(this);                 // 전역 차량 레지스트리에서 자신을 빼둔다.
    }
    GameObject::Destroy();
}

void Car::SetCurrentRoad(const shared_ptr<Road> &road, float offset)
{
    if (m_currentRoad == road && m_currentOffset == offset)
        return;
    m_currentRoad = road;
    m_currentOffset = offset;
    m_currentSpline = RoadDataManager::Get().BuildOffsetSpline(road, offset);

    RebuildSplineRender();
}

bool Car::ShouldStopForSignal(const shared_ptr<Road> &road) const
{
    if (!road)
        return false;
    shared_ptr<RoadNode> signalNode = RoadDataManager::Get().GetSignalNodeForRoad(road->GetId());
    if (!signalNode)
        return false;

    const Spline &spline = road->GetReferenceLine();
    float nodeT = spline.GetSplinePosition(signalNode->position);
    float myT = spline.GetSplinePosition(GetPosition());
    if (myT > nodeT)
    {
        if (m_committedYellowNodeId == signalNode->id)
            m_committedYellowNodeId = -1;
        return false;
    }

    TrafficSignal::Color color = m_SimState->GetSignalColor(signalNode->signalPhaseOffset);
    float gap = (signalNode->position - GetPosition()).Length();
    float emergStopDistance = (m_speed * m_speed) / (2.0f * m_maxEmergBrake);
    if (color == TrafficSignal::Color::Green)
    {
        if (m_committedYellowNodeId == signalNode->id)
            m_committedYellowNodeId = -1; // 다음 사이클 대비 리셋
    }
    else if (color == TrafficSignal::Color::Yellow && m_committedYellowNodeId != signalNode->id)
    {
        if (gap <= emergStopDistance)
            m_committedYellowNodeId = signalNode->id; // 최대 제동으로도 못 서는 거리 -- 통과 확정
    }
    else if (color == TrafficSignal::Color::Red && m_committedYellowNodeId == signalNode->id)
    {
        // 아직 설 수 있으면(최대 제동 거리 밖) 커밋 취소하고 선다
        if (gap > emergStopDistance)
            m_committedYellowNodeId = -1;
    }
    return color != TrafficSignal::Color::Green && m_committedYellowNodeId != signalNode->id;
}

void Car::UpdateCar()
{
    constexpr float FRICT_DECEL_RATE = 0.1f;
    if (m_acceleration == 0.0f)
    {
        // natural deceleration (drag) when coasting
        m_speed -= m_speed * FRICT_DECEL_RATE * m_deltaTime;

        if (m_speed < 0.1f)
            m_speed = 0.0f;
    }
    else
        m_speed += m_acceleration * m_deltaTime;

    m_speed = std::clamp(m_speed, 0.0f, m_maxSpeed);

    // steer
    m_maxSteerAngle = CalcMaxSteerAngle(m_speed);
    m_steerAngle = std::clamp(m_steerAngle, -m_maxSteerAngle, m_maxSteerAngle);
}

void Car::UpdateWithControl()
{
    if (!m_isFocused)
        return;

    // Reset
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

    // Change Gear
    if (ImGui::IsKeyPressed(ImGuiKey_Z, false)) // Toggle Drive / Reverse gear
        ChangeGear();

    // Acceleration / Brake

    if (m_isFocused && ImGui::IsKeyDown(ImGuiKey_DownArrow)) // Brake
        Accelerate(-1);
    else if (m_isFocused && ImGui::IsKeyDown(ImGuiKey_UpArrow)) // Accelerate
        Accelerate(1);
    else
        Accelerate(0);

    // Steering
    if (m_isFocused && ImGui::IsKeyDown(ImGuiKey_LeftArrow))
        Steer(-1);
    else if (m_isFocused && ImGui::IsKeyDown(ImGuiKey_RightArrow))
        Steer(1);
    else
        Steer(0);
}

void Car::ApplyMotion()
{
    if (PhysicsSystem::Get().HasNewContact(m_rigidbody.GetBodyID()))
    {
        float vy = m_rigidbody.GetLinearVelocity().GetY();
        m_rigidbody.SetLinearVelocity(JPH::Vec3(0.0f, vy, 0.0f));
        m_rigidbody.SetAngularVelocity(JPH::Vec3::sZero());
        m_acceleration = 0.0f;
        m_speed = 0.0f;
        DebugConsole::Log("CRASH!!");
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
    float vy = m_rigidbody.GetLinearVelocity().GetY(); // 수직 속도는 물리(중력/지면)가 정한 값을 그대로 유지
    return JPH::Vec3(fwd.x * signedSpeed, vy > 0.0f ? 0.0f : vy, fwd.z * signedSpeed);
}

float Car::PurePursuit(Vec3 target)
{
    Vec3 rigidPosition = m_rigidbody.GetPosition();
    Vec3 targetVec = target - rigidPosition;

    float distance = targetVec.Length();

    // [방어 코드] 혹시라도 타겟과 내 차의 위치가 완벽히 겹치면 조향하지 않음
    if (distance < 0.001f)
        return 0.0f;

    Vec3 carFwd = ToVec3(m_transform.GetForwardAxis()).Normalized();
    Vec3 carRight = ToVec3(m_transform.GetRightAxis()).Normalized(); // 좌우 판별용

    float dotProd = carFwd.Dot(targetVec) / distance;
    dotProd = std::clamp(dotProd, -1.0f, 1.0f);
    float headingError = acosf(dotProd);

    float directionSign = (carRight.Dot(targetVec) > 0.0f) ? 1.0f : -1.0f;

    float steeringAngle = atanf((2.0f * m_wheelbase * sinf(headingError)) / distance);

    return steeringAngle * directionSign;
}

float Car::Stanley(const Spline &spline)
{
    Vec3 frontAxle = GetPosition(); // Stanley는 앞축 기준
    float t = spline.GetSplinePosition(frontAxle);
    Vec3 pathPoint = spline.GetPositionAt(t);
    Vec3 pathDir = spline.GetDirectionAt(t).Normalized();

    Vec3 carFwd = ToVec3(m_transform.GetForwardAxis()).Normalized();
    Vec3 carRight = ToVec3(m_transform.GetRightAxis()).Normalized();

    // 헤딩오차: 경로 진행방향이 내 오른쪽을 향할수록 +(우조향)
    float headingError = atan2f(carRight.Dot(pathDir), carFwd.Dot(pathDir));

    // 횡오차: 앞축이 경로 오른쪽에 있으면 +, 되돌리려면 좌조향이므로 부호 반전. 저속 발산은 분모 소프트닝으로 막는다.
    float crossTrackRight = (frontAxle - pathPoint).Dot(carRight);
    float crossTrackTerm = atanf(m_stanleyGain * -crossTrackRight / (m_speed + m_stanleySoft));

    return std::clamp(headingError + crossTrackTerm, -m_maxSteerAngle, m_maxSteerAngle);
}

void Car::SetDestination(const shared_ptr<RoadNode> &destNode)
{
    m_destRoad = RoadDataManager::Get().GetClosestRoad(destNode->position).road;
    DebugConsole::Log(GetName() + ": SetDestination -> node " + std::to_string(destNode->id) +
                      " (road " + std::to_string(m_destRoad ? m_destRoad->GetId() : -1) + ")");
    if (destNode->nodeType == RoadNodeType::Park)
    {
        m_pendingParkNode = destNode;
    }

    if (m_currentRoad != nullptr)
        TryFindPathAndSetRoad();
}

void Car::UpdateHorn(float dt)
{
    if (IsHornSituation())
    {
        m_hornStoppedDuration += dt;
        if (m_hornStoppedDuration >= HORN_INTERVAL)
        {
            m_hornStoppedDuration = 0.0f;           // 다음 5초 카운트 시작
            m_hornFlashTimer = HORN_FLASH_DURATION; // 경적: 2초간 빨갛게
            DebugConsole::Log(GetName() + ": HONK!");
        }
    }
    else
        m_hornStoppedDuration = 0.0f; // 다시 움직이면(또는 신호대기면) 리셋

    if (m_hornFlashTimer > 0.0f)
        m_hornFlashTimer -= dt;
}

// Drive 중 장애물/차에 막혀 멈춘 상태인가. 빨간불 대기(그걸 아는 차)와 정상 정차(도착 등 Drive 외)는 제외한다.
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
    for (size_t i = m_pathIndex; i < m_path.size(); ++i)
    {
        const shared_ptr<Road> &road = m_path[i];
        shared_ptr<RoadNode> signalNode = RoadDataManager::Get().GetSignalNodeForRoad(road->GetId());
        if (signalNode == nullptr)
            continue;

        // 현재 road의 신호는 이미 정지선을 지났으면 건너뛴다(지난 신호엔 안 걸린다).
        if (i == m_pathIndex)
        {
            const Spline &spline = road->GetReferenceLine();
            if (spline.GetSplinePosition(GetPosition()) > spline.GetSplinePosition(signalNode->position))
                continue;
        }

        // 경로상 가장 가까운(앞선) 신호가 판정 기준 -- 그 너머 신호는 이 신호를 지나야 만난다.
        if ((signalNode->position - GetPosition()).Length() > HORN_SIGNAL_AWARE_DISTANCE)
            return false; // 가장 가까운 신호가 너무 멀다 -- 신호 때문에 선 게 아님
        return m_SimState->GetSignalColor(signalNode->signalPhaseOffset) == TrafficSignal::Color::Red;
    }
    return false;
}
#pragma region Debug
// Debug / Rendering Helpers
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
        ImGui::Text("ActualVel: %.2f", m_rigidbody.GetLinearVelocity().Length());
        ImGui::Text("DesiredVel: %.2f", ComputeDesiredVelocity().Length());
        if (m_mode == Mode::Drive)
            ImGui::Text("Mode: %s / %s", StateToString(m_mode), SubStateToString(m_subMode));
        else
            ImGui::Text("Mode: %s", StateToString(m_mode));

        // 현재 채택된 행동계획(0.2초마다 UpdateBehaviorPlan이 갱신)과, 그때 계산된 지금 위치 기준 속도 캡.
        const BehaviorCandidate &plan = m_currentBehaviorPlan;
        ImGui::Text("Plan: %s / %s (target %.1f, end %.1f km/h)",
                    LaneChoiceToString(plan.laneChoice), SpeedActionToString(plan.speedAction),
                    plan.targetSpeed * 3.6f, plan.horizonEndSpeed * 3.6f);
        ImGui::Text("Speed Cap: %.1f km/h", m_lastDesiredSpeed * 3.6f);
        if (m_laneChangeActive)
            ImGui::Text("Lane change: in progress");
        if (plan.minApproachGap < 1e6f)
            ImGui::Text("Lead gap: %.1f m  headway: %.2f s", plan.minApproachGap,
                        plan.minTimeHeadway < 1e6f ? plan.minTimeHeadway : 0.0f);
        else
            ImGui::Text("Lead: none");
        ImGui::Text("Max lat accel: %.2f m/s^2", plan.maxLateralAccel);

        ImGui::Separator();
        ImGui::Text("Behavior Plan Weights");
        ImGui::SliderFloat("Speed Weight w1", &m_behaviorWeights.speed, 0.0f, 10.0f);
        ImGui::SliderFloat("Lane Change Weight w2", &m_behaviorWeights.laneChange, 0.0f, 20.0f);
        ImGui::SliderFloat("Lateral Accel Weight w3", &m_behaviorWeights.lateralAccel, 0.0f, 5.0f);
        ImGui::SliderFloat("Inertia Weight w4", &m_behaviorWeights.inertia, 0.0f, 10.0f);
        ImGui::SliderFloat("Signal Violation Weight w5", &m_behaviorWeights.signalViolation, 0.0f, 50.0f);
        ImGui::SliderFloat("Following Weight w6", &m_behaviorWeights.following, 0.0f, 20.0f);
    }
    ImGui::End();
}

void Car::UpdateTrail()
{
    if (!m_drawCollider) // only track/rebuild the trail for cars actually shown in debug view
        return;

    using namespace DirectX;

    XMFLOAT3 rearPos = m_transform.GetPosition();
    XMFLOAT3 fwd = m_transform.GetForwardAxis();
    XMFLOAT3 frontPos(rearPos.x + fwd.x * m_wheelbase, rearPos.y + fwd.y * m_wheelbase, rearPos.z + fwd.z * m_wheelbase);

    auto recordPoint = [](std::deque<XMFLOAT3> &trail, const XMFLOAT3 &pos)
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
    constexpr float TRAIL_LINE_HEIGHT = 0.15f; // lift above the road edge lines (y = 0.1f)

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

    constexpr float SPLINE_LINE_HEIGHT = 0.15f; // lift above the road edge lines (y = 0.1f)

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

void Car::RebuildRSDebugRender(const ReedsShepp::Path &path, const Vec3 &startPos, float startAngleRad,
                               float turningRadius, const Vec3 &targetPos, float targetAngleRad)
{
    constexpr float DEBUG_LINE_HEIGHT = 0.15f;

    // RS 경로 폴리라인 (보라색)
    std::vector<Vec3> pathPoints = ReedsShepp::GetDebugPath(path, startPos, startAngleRad, turningRadius);
    if (pathPoints.size() < 2)
    {
        m_parkPathRender.SetModel(nullptr);
    }
    else
    {
        std::vector<DirectX::XMFLOAT3> points;
        points.reserve(pathPoints.size());
        for (const Vec3 &point : pathPoints)
        {
            DirectX::XMFLOAT3 p = ToXMFLOAT3(point);
            p.y += DEBUG_LINE_HEIGHT;
            points.push_back(p);
        }

        Model *pPathModel = ModelManager::Get().CreateFromGeometry("__park_path__:" + GetName(), Geometry::CreatePolyline(points));
        pPathModel->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(1.0f, 0.0f, 1.0f, 1.0f));
        pPathModel->materials[0].Set<float>("$Opacity", 1.0f);
        m_parkPathRender.SetModel(pPathModel);
    }

    // 목표 위치 마커 (초록 평면, 모델은 Init에서 이미 만들어둠)
    DirectX::XMFLOAT3 markerPos = ToXMFLOAT3(targetPos);
    markerPos.y += DEBUG_LINE_HEIGHT;
    m_parkTargetMarker.GetTransform().SetPosition(markerPos);

    // 목표 방향 선 (초록 선) — 매번 targetPos/targetAngleRad가 바뀌므로 그때그때 새로 만든다.
    constexpr float TARGET_LINE_LENGTH = 6.0f;
    Vec3 targetDir(cosf(targetAngleRad), 0.0f, sinf(targetAngleRad));
    DirectX::XMFLOAT3 lineStart = ToXMFLOAT3(targetPos);
    DirectX::XMFLOAT3 lineEnd = ToXMFLOAT3(targetPos + targetDir * TARGET_LINE_LENGTH);
    lineStart.y += DEBUG_LINE_HEIGHT;
    lineEnd.y += DEBUG_LINE_HEIGHT;

    Model *pTargetLineModel = ModelManager::Get().CreateFromGeometry("__park_target_line__:" + GetName(),
                                                                     Geometry::CreateLine(lineStart, lineEnd));
    pTargetLineModel->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f));
    pTargetLineModel->materials[0].Set<float>("$Opacity", 1.0f);
    m_parkTargetLine.SetModel(pTargetLineModel);
}

void Car::DebugInit()
{
    // Per-object model names: CreateFromGeometry() overwrites any existing
    // model stored under the same name, so a shared name would make every
    // car's debug bbox end up showing whichever car initialized last.
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

    // Park(RS) 목표 위치 마커 — 위치만 바뀌므로 모델은 한 번만 만들어두고 재사용한다.
    Model *pParkTargetMarker = ModelManager::Get().CreateFromGeometry("__park_target_marker__:" + GetName(),
                                                                      Geometry::CreatePlane(TARGET_MARKER_SIZE, TARGET_MARKER_SIZE));
    pParkTargetMarker->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f));
    pParkTargetMarker->materials[0].Set<float>("$Opacity", 1.0f);
    m_parkTargetMarker.SetModel(pParkTargetMarker);
}
#pragma endregion