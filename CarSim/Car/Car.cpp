#include "Car.h"
#include "Core/Physics/PhysicsSystem.h"
#include "Rendering/Effects.h"
#include <ModelManager.h>
#include <algorithm>
#include <cmath>
#include <imgui.h>
#include "Utill/DebugConsole.h"

namespace
{
    // 저크 완화용 S자(smoothstep) 보간. 가속/제동 램프와 속도 프로파일 계산에서 공유.
    float SmoothStep(float t)
    {
        t = std::clamp(t, 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }
} // namespace

void Car::Init(const CarSpec &spec, RoadDataManager *roadDataManager, JPH::Vec3 position)
{
    SetName(spec.name);
    m_render.SetModel(ModelManager::Get().CreateFromFile(spec.modelPath));
    SetRenderOffset(ToXMFLOAT3(spec.renderOffset));
    m_wheelbase = spec.wheelbase;
    m_halfExtents = spec.halfExtents;

    DirectX::XMFLOAT3 fwd = m_transform.GetForwardAxis();
    m_transform.SetPosition(position.GetX() - fwd.x * m_wheelbase,
                            position.GetY() - fwd.y * m_wheelbase,
                            position.GetZ() - fwd.z * m_wheelbase);

    GameObject::Init(spec.halfExtents, Rigidbody::Type::Dynamic, spec.colliderOffset, spec.mass);

    m_spawnPosition = m_transform.GetPosition();
    m_spawnRotation = m_transform.GetRotationQuat();
    m_mass = spec.mass;

    m_RoadDataManager = roadDataManager;

    // todo : 출차시 예약자리 release
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
    UpdateFindPath(); // Update Mode 보다 먼저 실행돼야함
    UpdateMode();
    switch (m_mode)
    {
    case DriveMode::Stop:
        UpdateStop();
        break;
    case DriveMode::Park:
        UpdatePark();
        break;
    case DriveMode::Drive:
        UpdateDrive();
        break;
    case DriveMode::Avoid:
        UpdateAvoid();
        break;
    }
    UpdateCar();
    UpdateDebugWindow();
    ApplyMotion();
    UpdateTrail();
}

void Car::Draw(ID3D11DeviceContext *context, IEffect &effect)
{
    GameObject::Draw(context, effect);

    if (m_drawCollider && (m_rearTrailRender.GetModel() || m_frontTrailRender.GetModel() || m_splineRender.GetModel() ||
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

    if (!m_drawCollider || !m_steerLine.GetModel())
        return;

    using namespace DirectX;

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

void Car::SetRotation(const DirectX::XMFLOAT4 &rotation)
{
    Vec3 frontAxle = GetPosition(); // capture using the OLD rotation, before it changes
    GameObject::SetRotation(rotation);
    SetPosition(frontAxle); // re-derive the rear axle using the NEW rotation
}

void Car::EmergBrake()
{
    m_acceleration = -m_maxEmergBrake;
    m_accelMode = AccelMode::Braking;
    m_accelRampTime = 0.0f;
}
void Car::Accelerate(float desiredVelocity)
{
    // 저크 완화를 위해 가속도를 시간에 따라 선형이 아닌 S자(smoothstep)로 보간한다.
    // desiredVelocity는 매 프레임 DriveControl이 조향각 기반 순간 속도상한(maxSteerSpeed)까지
    // 반영해서 넘기므로 아주 미세하게 흔들릴 수 있다 — 데드밴드 없이 m_speed와 바로 비교하면 그
    // 흔들림만으로 모드가 매번 뒤집혀 아래 램프가 계속 리셋되고, 실제 제동력이 거의 못 올라간 채로
    // (SmoothStep(0)≈0) 코너를 절반 넘게 지나서야 목표 속도에 닿는 문제가 있었다.
    constexpr float ACCEL_MODE_DEADBAND = 0.3f / 3.6f; // 0.3km/h 이내 차이로는 모드(=램프)를 유지

    float diff = desiredVelocity - m_speed;
    AccelMode targetMode = m_accelMode;
    if (diff > ACCEL_MODE_DEADBAND)
        targetMode = AccelMode::Accelerating;
    else if (diff < -ACCEL_MODE_DEADBAND)
        targetMode = AccelMode::Braking;

    if (m_accelMode != targetMode)
    {
        m_accelMode = targetMode;
        m_accelRampTime = 0.0f;
    }

    if (m_accelMode == AccelMode::None)
    {
        m_acceleration = 0.0f;
        return;
    }

    float duration = (m_accelMode == AccelMode::Accelerating) ? ACCEL_RAMP_DURATION : BRAKE_RAMP_DURATION;
    float maxLimit = (m_accelMode == AccelMode::Accelerating) ? m_maxAccel : -m_maxBrake;

    m_accelRampTime += m_deltaTime;
    m_acceleration = SmoothStep(m_accelRampTime / duration) * maxLimit;
}

void Car::Steer(float radian, float steerRamp)
{
    if (m_steerAngle > radian)
        m_steerAngle -= steerRamp * m_deltaTime;
    else if (m_steerAngle < radian)
        m_steerAngle += steerRamp * m_deltaTime;
    else if (m_steerAngle > 0.0f) // Return to center
        m_steerAngle = std::max(m_steerAngle - m_deltaTime, 0.0f);
    else
        m_steerAngle = std::min(m_steerAngle + m_deltaTime, 0.0f);
}
void Car::ChangeGear()
{
    constexpr float GEAR_SWITCH_SPEED_THRESHOLD = 2.0f / 3.6f; // 2 km/h

    if (m_speed <= GEAR_SWITCH_SPEED_THRESHOLD) // Toggle Drive / Reverse gear
        m_isReverse = !m_isReverse;
}

bool Car::IsOffCourse()
{
    constexpr float OFF_COURSE_DISTANCE = 5.0f;          // 스플라인과의 최대 허용 거리 (m)
    constexpr float OFF_COURSE_ANGLE = ToRadians(90.0f); // 순간 조향각이 아니라 '회복 불가(역주행)' 감지용 고정 임계값

    Vec3 position = GetPosition();
    Vec3 closestPoint = m_currentSpline.GetLookaheadPoint(position, 0.0f);
    float distance = (closestPoint - position).Length();
    if (distance > OFF_COURSE_DISTANCE)
        return true;

    // pure-pursuit가 실제 조향하는 기준인 lookahead 점 방향과 비교한다. (스플라인 순간 접선은 merge cusp 등에 휘둘림)
    // 순간 heading 오차는 조향으로 흡수되므로, 90도를 넘는 '경로를 등진' 상태만 이탈로 본다.
    float lookaheadDistance = std::max(5.0f, m_speed * 0.5f);
    Vec3 toLookahead = m_currentSpline.GetLookaheadPoint(position, lookaheadDistance) - position;
    if (toLookahead.Length() < 1e-4f)
        return false;

    float dot = std::clamp(GetForwardAxis().Dot(toLookahead.Normalized()), -1.0f, 1.0f);
    return std::acos(dot) > OFF_COURSE_ANGLE;
}

void Car::MergeOntoLane(const shared_ptr<Lane> &lane, const Vec3 &position)
{
    m_currentLane = lane;
    m_currentSpline = lane->GetSpline();

    // 현재 위치에서 레인 위 전방 지점까지 짧은 연결 스플라인을 만들어 앞에 이어 붙여 부드럽게 합류한다.
    float minRadius = powf(m_speed / CURVE_SPEED_COEFF, 2);
    float width = m_RoadDataManager->ROAD_WIDTH;
    float insideRoot = (4 * minRadius * width) - (width * width);
    float L = insideRoot > 0 ? sqrt(insideRoot) : 10.0f;

    // L이 현재 레인 안에서 안 나오면(레인이 짧으면) 경로상 다음 레인까지 봐서 병합 목표 지점/방향을 잡는다.
    const Spline *spline = &m_currentSpline;
    size_t pathIndex = m_pathIndex;
    Vec3 segmentStart = position;
    float remaining = L;
    Vec3 mergePoint = position;
    Vec3 mergeDirection = GetForwardAxis();

    while (true)
    {
        float startT = spline->GetSplinePosition(segmentStart);
        float splineLength = spline->GetLength();
        float distanceToEnd = splineLength > 0.0f ? (1.0f - startT) * splineLength : 0.0f;

        if (splineLength <= 0.0f || remaining <= distanceToEnd)
        {
            float t = splineLength > 0.0f ? std::clamp(startT + remaining / splineLength, 0.0f, 1.0f) : startT;
            mergePoint = spline->GetPositionAt(t);
            mergeDirection = spline->GetDirectionAt(t);
            break;
        }

        remaining -= distanceToEnd;
        shared_ptr<Lane> nextLane = (pathIndex + 1 < m_path.size()) ? m_path[pathIndex + 1].lane : nullptr;
        if (!nextLane)
        {
            mergePoint = spline->GetPositionAt(1.0f);
            mergeDirection = spline->GetDirectionAt(1.0f);
            break;
        }

        spline = &nextLane->GetSpline();
        segmentStart = nextLane->GetStartPoint();
        ++pathIndex;
    }

    float mergeT = m_currentSpline.GetSplinePosition(mergePoint);

    // phantom은 방향만 주면 된다 — 접선 세기(길이)는 Spline이 꺾임 각에 따라 tan로 자동 조정한다.
    Spline connector({position - GetForwardAxis(),
                      position,
                      mergePoint,
                      mergePoint + mergeDirection});
    m_currentSpline.AddSplinePointsFront(connector.GetSplinePoints(), mergeT);
    RebuildSplineRender();
}

void Car::BakePathSpeedProfile()
{
    m_pathSpeedProfile.clear();
    m_pathLaneStartDistance.clear();
    if (m_path.empty())
        return;

    struct RawPoint
    {
        float distance;
        float curveSpeed; // 곡률/제한속도만 반영한 로컬 최대속도 (제동램프 전파 전)
    };
    std::vector<RawPoint> raw;

    float cumulative = 0.0f;
    for (const LaneStep &step : m_path)
    {
        m_pathLaneStartDistance.push_back(cumulative);

        const Spline &spline = step.lane->GetSpline();
        const std::vector<Vec3> &points = spline.GetSplinePoints();
        float splineLength = spline.GetLength();
        if (points.size() < 2 || splineLength <= 0.0f)
            continue;

        float curveSpeed = m_maxSpeed;
        float minPosT = 0.0f;
        float radius = spline.GetMinRadiusAhead(0.0f, 1.0f, &minPosT);
        minPosT = std::max(minPosT, 0.4f);
        if (radius < std::numeric_limits<float>::max())
            curveSpeed = RoadDataManager::CURVE_SPEED_COEFF * std::sqrt(radius);
        DebugConsole::Log("Bake ID " + ToString(step.lane->GetId()) + " / curve R " + ToString(radius) + " / max Speed " + ToString(curveSpeed * 3.6f));
        DebugConsole::Log("Bake minPos " + ToString(minPosT));
        float laneSpeedLimit = std::min(step.lane->GetLimitSpeed(), m_maxSpeed);
        size_t lastIndex = points.size() - 1;
        for (size_t i = 0; i < points.size(); ++i)
        {
            float pointDistance = cumulative + (static_cast<float>(i) / lastIndex) * splineLength;
            if (static_cast<float>(i) / lastIndex <= minPosT)
            {
                raw.push_back({pointDistance, std::min({curveSpeed, laneSpeedLimit})});
            }
            else
            {
                raw.push_back({pointDistance, laneSpeedLimit});
            }
        }

        cumulative += splineLength;
    }

    if (raw.empty())
        return;

    // 경로 끝(목적지 레인 끝)은 정지가 목표.
    raw.back().curveSpeed = 0.0f;

    m_pathSpeedProfile.resize(raw.size());
    m_pathSpeedProfile.back() = {raw.back().distance, raw.back().curveSpeed};

    // 뒤(목적지)에서부터 역방향으로 걸으며 "제동 램프가 얼마나 진행됐는지"(brakingElapsed)를
    // 상태로 들고 다닌다. 로컬 곡률/제한속도가 더 낮으면(=거기서부터 새로 브레이크를 밟아도
    // 충분함) 램프를 리셋하고, 아니면 계속 이어진 제동으로 보고 램프를 누적한다.
    // 목적지에 도달하는 시점엔 이미 충분히 오래 제동해온 상태(램프 완료)로 가정한다.
    float brakingElapsed = BRAKE_RAMP_DURATION;
    for (size_t i = raw.size() - 1; i-- > 0;)
    {
        float distance = raw[i + 1].distance - raw[i].distance;
        float nextSpeed = m_pathSpeedProfile[i + 1].maxSpeed;

        float rampProgress = std::clamp(brakingElapsed / BRAKE_RAMP_DURATION, 0.0f, 1.0f);
        float decel = SmoothStep(rampProgress) * m_maxBrake;
        float brakeEntrySpeed = std::sqrt(nextSpeed * nextSpeed + 2.0f * decel * distance);

        float cap;
        if (raw[i].curveSpeed <= brakeEntrySpeed)
        {
            cap = raw[i].curveSpeed;
            brakingElapsed = 0.0f;
        }
        else
        {
            cap = brakeEntrySpeed;
            float avgSpeed = std::max(0.1f, (cap + nextSpeed) * 0.5f);
            brakingElapsed = std::min(BRAKE_RAMP_DURATION, brakingElapsed + distance / avgSpeed);
        }

        m_pathSpeedProfile[i] = {raw[i].distance, cap};
    }
}

float Car::GetPathDistance(size_t pathIndex, const Vec3 &position) const
{
    if (m_pathLaneStartDistance.empty())
        return 0.0f;
    pathIndex = std::min(pathIndex, m_pathLaneStartDistance.size() - 1);

    // m_currentSpline은 차선변경 시 MergeOntoLane이 붙인 커넥터 때문에 원본 레인과 파라미터화가
    // 다를 수 있으므로, 항상 경로상 원본 레인 스플라인에 직접 투영해서 t를 구한다. 차가 아직
    // 커넥터 위에 있어도 최근접점 탐색이라 자연스럽게 레인 시작점 근처로 잡힌다.
    const Spline &spline = m_path[pathIndex].lane->GetSpline();
    float t = std::clamp(spline.GetSplinePosition(position), 0.0f, 1.0f);
    float laneLength = m_path[pathIndex].lane->GetLength();
    return m_pathLaneStartDistance[pathIndex] + t * laneLength;
}

Vec3 Car::GetPathPosition(float pathDistance) const
{
    if (m_pathLaneStartDistance.empty())
        return m_rigidbody.GetPosition();

    size_t hi = std::upper_bound(m_pathLaneStartDistance.begin(), m_pathLaneStartDistance.end(), pathDistance) - m_pathLaneStartDistance.begin();
    size_t laneIndex = std::min(hi == 0 ? size_t(0) : hi - 1, m_path.size() - 1);

    const Spline &spline = m_path[laneIndex].lane->GetSpline();
    float laneLength = spline.GetLength();
    float localDistance = pathDistance - m_pathLaneStartDistance[laneIndex];
    float t = laneLength > 0.0f ? std::clamp(localDistance / laneLength, 0.0f, 1.0f) : 0.0f;
    return spline.GetPositionAt(t);
}

float Car::GetPathMaxSpeed(float pathDistance) const
{
    if (m_pathSpeedProfile.empty())
        return m_maxSpeed;
    if (pathDistance <= m_pathSpeedProfile.front().distance)
        return m_pathSpeedProfile.front().maxSpeed;
    if (pathDistance >= m_pathSpeedProfile.back().distance)
        return m_pathSpeedProfile.back().maxSpeed;

    auto it = std::lower_bound(m_pathSpeedProfile.begin(), m_pathSpeedProfile.end(), pathDistance,
                               [](const PathSpeedSample &s, float d)
                               { return s.distance < d; });
    const PathSpeedSample &hi = *it;
    const PathSpeedSample &lo = *(it - 1);
    float span = hi.distance - lo.distance;
    float ratio = span > 1e-4f ? (pathDistance - lo.distance) / span : 0.0f;
    return lo.maxSpeed + (hi.maxSpeed - lo.maxSpeed) * ratio;
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
        m_rigidbody.SetLinearVelocity(JPH::Vec3::sZero());
        m_rigidbody.SetAngularVelocity(JPH::Vec3::sZero());
        m_acceleration = 0.0f;
        m_speed = 0.0f;
        return;
    }

    // Steering stays kinematic -- the bicycle model already gives the correct yaw rate.
    float angularVelocity = GetSignedSpeed() * tan(m_steerAngle) / m_wheelbase;
    m_rigidbody.SetAngularVelocity(JPH::Vec3(0.0f, angularVelocity, 0.0f));
    m_rigidbody.SetLinearVelocity(ComputeDesiredVelocity());
}

JPH::Vec3 Car::ComputeDesiredVelocity() const
{
    float signedSpeed = GetSignedSpeed();
    DirectX::XMFLOAT3 fwd = m_transform.GetForwardAxis();
    float vy = m_rigidbody.GetLinearVelocity().GetY();
    return JPH::Vec3(fwd.x * signedSpeed, vy, fwd.z * signedSpeed);
}

float Car::PurePursuit(Vec3 target)
{
    Vec3 carPos = m_rigidbody.GetPosition();
    Vec3 targetVec = target - carPos;

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
        ImGui::Text("Mode: %s", DriveModeToString(m_mode));
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

void Car::RebuildParkDebugRender(const ReedsShepp::Path &path, const Vec3 &startPos, float startAngleDeg,
                                 float turningRadius, const Vec3 &targetPos, float targetAngleDeg)
{
    constexpr float DEBUG_LINE_HEIGHT = 0.15f;

    // RS 경로 폴리라인 (보라색)
    std::vector<Vec3> pathPoints = ReedsShepp::SamplePath(path, startPos, startAngleDeg, turningRadius);
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

    // 목표 방향 선 (초록 선) — 매번 targetPos/targetAngleDeg가 바뀌므로 그때그때 새로 만든다.
    constexpr float TARGET_LINE_LENGTH = 6.0f;
    float targetAngleRad = ToRadians(targetAngleDeg);
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

void Car::SetDestination(const shared_ptr<RoadNode> &destNode)
{
    m_destLane = m_RoadDataManager->GetClosestLaneEnd(destNode->position);
    if (destNode->nodeType == RoadNodeType::Park)
    {
        m_pendingParkNode = destNode;
    }
}

void Car::DebugInit()
{
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