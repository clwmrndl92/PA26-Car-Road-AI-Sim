#include "Car.h"
#include "Core/Physics/PhysicsSystem.h"
#include "Rendering/Effects.h"
#include "Nav/CarFollowing.h"
#include <ModelManager.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_set>
#include <imgui.h>
#include "Utill/DebugConsole.h"
#include "Utill/Assert.h"

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

    GameObject::Init(spec.halfExtents, Rigidbody::Type::Kinematic, spec.colliderOffset, spec.mass);

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

    m_roadConstraints.clear();
    m_lastConstraintScanTime = 0.0f;

    // DEBUG
    DebugInit();
}

void Car::UpdateAI(float dt)
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
}

void Car::Update(float dt)
{
    m_deltaTime = dt;
    if (m_wantSegmentTick)
        m_vehicleController.Tick(*this);
    UpdateCar();
    ApplyMotion();

    UpdateTrail();
}

void Car::UpdateUI(float dt)
{
    UpdateDebugWindow();
}

void Car::Draw(ID3D11DeviceContext *context, IEffect &effect)
{
    GameObject::Draw(context, effect);

    using namespace DirectX;

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

    if (m_originMarker.GetModel())
    {
        m_originMarker.GetTransform().SetPosition(m_transform.GetPosition());

        if (auto *pBasic = dynamic_cast<BasicEffect *>(&effect))
        {
            pBasic->SetRenderNoDepthTest();
            m_originMarker.Draw(context, effect);
            pBasic->SetRenderDefault();
        }
    }

    if ((m_rearTrailRender.GetModel() || m_frontTrailRender.GetModel() || m_splineRender.GetModel() ||
         m_parkPathRender.GetModel() || m_parkTargetLine.GetModel() || !m_bboxDebugRenders.empty()))
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
            for (RenderObject &bboxRender : m_bboxDebugRenders)
                if (bboxRender.GetModel())
                    bboxRender.Draw(context, effect);
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
    m_accelMode = AccelMode::Braking;
    m_accelRampTime = 0.0f;
}
void Car::Accelerate(float desiredVelocity)
{
    float DECEL_MODE_DEADBAND = (m_speed * 0.05f); // 감속(브레이킹) 중 언더슛을 코스팅으로 봐줄 허용치
    float ACCEL_MODE_DEADBAND = (m_speed * 0.15f); // 이 이상 부족해야 Accelerating으로 전환

    float diff = desiredVelocity - m_speed;
    AccelMode targetMode = m_accelMode;
    bool coast = false; // 모드는 유지한 채 이번 프레임 가속도만 0으로 냄

    if (diff > ACCEL_MODE_DEADBAND)
        targetMode = AccelMode::Accelerating;
    else if (diff < -DECEL_MODE_DEADBAND)
        targetMode = AccelMode::Braking;
    else if (diff >= 0.0f && m_accelMode == AccelMode::Braking)
        // 브레이킹 중 목표에 도달/언더슛(0~ACCEL_MODE_DEADBAND) — 아직 Accelerating 전환 임계값 전이면 코스팅
        coast = true;
    else if (diff < 0.0f && m_accelMode == AccelMode::Accelerating)
        // 가속 중 목표를 오버슈트(0~-DECEL_MODE_DEADBAND) — 아직 Braking 전환 임계값 전이면 코스팅
        coast = true;
    else if (m_accelMode == AccelMode::None)
        // None은 지켜줄 기존 램프가 없으므로 데드밴드 없이 diff 부호대로 바로 방향을 정한다.
        // (없으면 diff가 데드밴드 안에 머무는 한 영원히 None에 머물러 가속이 시작되지 않는다)
        targetMode = (diff > 0.0f) ? AccelMode::Accelerating : (diff < 0.0f ? AccelMode::Braking : AccelMode::None);

    if (m_accelMode != targetMode)
    {
        m_accelMode = targetMode;
        m_accelRampTime = 0.0f;
    }

    // 코스팅 중에는 m_accelRampTime을 증가시키지 않아, 다시 벗어나면 멈췄던 지점부터 램프가 이어진다.
    if (m_accelMode == AccelMode::None || (coast && m_accelMode == AccelMode::Accelerating))
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

void Car::Destroy()
{
    SetCurrentLane(nullptr); // 레인 레지스트리에서 자신을 빼둔다.
    GameObject::Destroy();
}

void Car::SetCurrentLane(const shared_ptr<Lane> &lane)
{
    if (m_currentLane == lane)
        return;
    if (m_currentLane != nullptr)
        m_currentLane->UnregisterCar(this);
    m_currentLane = lane;
    if (m_currentLane != nullptr)
        m_currentLane->RegisterCar(this);

    RebuildSplineRender();
}

std::vector<Car::RoadSpeedSample> Car::ScanRoadSpeedConstraints(float lookDistance) const
{
    constexpr float ROAD_SAMPLE_SPACING = 5.0f; // 도로 스캔 샘플 간격 (m)
    constexpr float LOCAL_WINDOW = 0.1f;        // 로컬 곡률 추정용 t-window

    Vec3 calPosition = GetPosition();

    // m_currentSpline -> (필요시) path상의 다음 노드들의 lane spline 순으로 lookDistance까지 훑으며
    // 커브 지점(로컬 곡률 기반 최대속도)과 노드 지점(제한속도)의 샘플을 모은다. 각 샘플이 곧 DriveSpeedIDM이
    // 쓰는 "가상 리더"의 (위치, 거리, 요구 속도) 후보가 된다.
    std::vector<RoadSpeedSample> samples;
    samples.reserve(static_cast<size_t>(lookDistance / ROAD_SAMPLE_SPACING) + 4);
    {
        float currentNodeT = m_currentSpline.GetSplinePosition(calPosition);
        Assert(currentNodeT >= 0.0f); // ScanRoadSpeedConstraints 호출 전엔 항상 m_currentSpline이 세팅되어 있어야 함
        float currentNodeDistance = m_currentSpline.GetLength() * (1.0f - currentNodeT);
        float currentNodeSpeed = (m_currentLane == m_destLane) ? 0.0f : std::min(m_currentLane->GetLimitSpeed(), m_maxSpeed);
        samples.push_back({m_currentLane->GetEndPoint(), currentNodeDistance, currentNodeSpeed});
    }
    {
        const Spline *spline = &m_currentSpline;
        shared_ptr<Lane> segmentLane = m_currentLane;
        size_t pathIndex = m_pathIndex;
        Vec3 segmentStart = calPosition;
        float traveledDistance = 0.0f;
        float remainingDistance = lookDistance;

        while (remainingDistance > 0.0f && spline)
        {
            float startT = spline->GetSplinePosition(segmentStart);
            float splineLength = spline->GetLength();
            float segmentDistance = splineLength > 0.0f ? (1.0f - startT) * splineLength : 0.0f;
            float walkDistance = std::min(segmentDistance, remainingDistance);

            // nodeT < startT면 이미 지나온 신호라 건너뛴다 (안 그러면 통과 직후 급제동).
            if (shared_ptr<RoadNode> signalNode = segmentLane->GetSignalNode())
            {
                if (splineLength > 0.0f && ShouldStopForSignal(segmentLane))
                {
                    float nodeT = spline->GetSplinePosition(signalNode->position);
                    if (nodeT >= startT)
                    {
                        float nodeDistance = traveledDistance + (nodeT - startT) * splineLength;
                        samples.push_back({signalNode->position, nodeDistance, 0.0f});
                    }
                }
            }

            const std::vector<Vec3> &points = spline->GetSplinePoints();
            if (!points.empty() && splineLength > 0.0f)
            {
                size_t lastIndex = points.size() - 1;

                float minRadius = spline->GetMinRadiusAhead(0.0f, 1.0f);

                size_t sampleCount = static_cast<size_t>(walkDistance / ROAD_SAMPLE_SPACING) + 1;
                for (size_t s = 1; s <= sampleCount; ++s)
                {
                    float localDistance = std::min(walkDistance, s * ROAD_SAMPLE_SPACING);
                    float t = startT + localDistance / splineLength;
                    size_t index = static_cast<size_t>(std::clamp(t, 0.0f, 1.0f) * lastIndex);

                    float radius = spline->GetMinRadiusAhead(std::max(0.0f, t - LOCAL_WINDOW), std::min(1.0f, t + LOCAL_WINDOW));

                    float maxSpeed = m_maxSpeed;
                    if (s < sampleCount / 1.8 && radius < std::numeric_limits<float>::max())
                    {
                        maxSpeed = CURVE_SPEED_COEFF * std::sqrt(radius);
                    }
                    samples.push_back({points[index], traveledDistance + localDistance, maxSpeed});
                }
            }

            traveledDistance += walkDistance;
            remainingDistance -= walkDistance;
            if (remainingDistance <= 0.0f)
                break;

            shared_ptr<Lane> nextLane = (pathIndex + 1 < m_path.size()) ? m_path[pathIndex + 1].lane : nullptr;
            if (!nextLane)
            {
                // 경로가 여기서 끝난다는 것은 이 레인이 destLane이라는 뜻: 진짜 정지 지점인 레인 끝에 0속도를 박는다.
                if (segmentLane == m_destLane)
                    samples.push_back({segmentLane->GetEndPoint(), traveledDistance, 0.0f});
                break;
            }

            float nextNodeSpeed = std::min(nextLane->GetLimitSpeed(), m_maxSpeed);
            samples.push_back({nextLane->GetStartPoint(), traveledDistance, nextNodeSpeed});

            segmentLane = nextLane;
            segmentStart = nextLane->GetStartPoint();
            spline = &nextLane->GetSpline();
            ++pathIndex;
        }
    }
    std::sort(samples.begin(), samples.end(), [](const RoadSpeedSample &a, const RoadSpeedSample &b)
              { return a.distance < b.distance; });
    return samples;
}

bool Car::ShouldStopForSignal(const shared_ptr<Lane> &lane) const
{
    shared_ptr<RoadNode> signalNode = lane->GetSignalNode();
    if (!signalNode)
        return false;

    TrafficSignal::Color color = m_RoadDataManager->GetSignalColor(signalNode->signalPhaseOffset);
    if (color == TrafficSignal::Color::Green)
    {
        if (m_committedYellowNodeId == signalNode->id)
            m_committedYellowNodeId = -1; // 다음 사이클 대비 리셋
    }
    else if (color == TrafficSignal::Color::Yellow && m_committedYellowNodeId != signalNode->id)
    {
        float gap = (signalNode->position - GetPosition()).Length();
        if (gap <= (m_speed * m_speed) / (2.0f * m_maxBrake))
            m_committedYellowNodeId = signalNode->id; // 정지거리 안쪽 -- 통과 확정
    }
    return color != TrafficSignal::Color::Green && m_committedYellowNodeId != signalNode->id;
}

void Car::RescanRoadSpeedConstraints()
{
    // 정지거리(현재 속도로 편안한 감속을 했을 때 필요한 거리)에 여유를 둔 만큼만 앞을 본다 -- 너무 멀리
    // 보면 스플라인 곡률 스캔 비용만 늘고, 그 범위 밖 제약은 다음 재스캔 전에 어차피 다시 잡힌다.
    constexpr float MIN_LOOK_DISTANCE = 100.0f;
    constexpr float LOOK_DISTANCE_MARGIN = 1.5f;
    float lookDistance = std::max(MIN_LOOK_DISTANCE, (m_speed * m_speed) / (2.0f * m_maxBrake) * LOOK_DISTANCE_MARGIN);

    m_roadConstraints = ScanRoadSpeedConstraints(lookDistance);
    m_lastConstraintScanTime = m_currentTime;
}

CarFollowing::Params Car::BuildIdmParams(float v0) const
{
    CarFollowing::Params idmParams;
    idmParams.v0 = v0;
    idmParams.T = IDM_TIME_HEADWAY;
    idmParams.s0 = IDM_STANDSTILL_DISTANCE;
    idmParams.a = m_maxAccel;
    idmParams.b = m_maxBrake;
    idmParams.delta = IDM_ACCEL_EXPONENT;
    idmParams.coolness = IDM_COOLNESS;
    return idmParams;
}

void Car::WalkConnectedLanes(const shared_ptr<Lane> &rootLane, float rootPosition,
                             const std::function<void(Car *, float)> &visitor) const
{
    // 어떤 레인 위의 차들을 스캔해서 u(root 기준 통일 거리)와 함께 visitor에 넘긴다.
    // backward=false: entryOffset은 lane 시작점의 u (u = entryOffset + p).
    // backward=true : entryOffset은 lane 끝점의 u (u = entryOffset - length + p).
    auto scanLaneCars = [&](const shared_ptr<Lane> &lane, float entryOffset, bool backward)
    {
        const Spline &spline = lane->GetSpline();
        float length = lane->GetLength();
        for (Car *other : lane->GetCars())
        {
            if (other == this)
                continue;

            Vec3 otherPos = other->GetPosition();
            float t = spline.GetSplinePosition(otherPos);
            if ((otherPos - spline.GetPositionAt(t)).Length() > LANE_ENTRY_THRESHOLD)
                continue;

            float p = t * length;
            float u = backward ? (entryOffset - length + p) : (entryOffset + p);
            visitor(other, u);
        }
    };

    std::unordered_set<Lane *> visited;
    visited.insert(rootLane.get());
    scanLaneCars(rootLane, -rootPosition, /*backward=*/false); // rootLane 자신: u = p - rootPosition

    // 정방향(successor)/역방향(predecessor) 예산 기반 그래프 탐색. 스택 기반이라 사이클/깊은 체인에도
    // 안전(재귀 깊이 제한 없음).
    struct WorkItem
    {
        shared_ptr<Lane> lane;
        float entryOffset;
        float remainingBudget;
        bool backward;
        Lane *excludeBranch; // backward 탐색 시 방금 거쳐온(되돌아가지 않을) 레인
    };
    std::vector<WorkItem> stack;

    float rootRemaining = rootLane->GetLength() - rootPosition;
    stack.push_back({rootLane, rootRemaining, MOBIL_SEARCH_FORWARD - rootRemaining, false, nullptr});
    stack.push_back({rootLane, rootPosition, MOBIL_SEARCH_BACKWARD, true, nullptr});

    while (!stack.empty())
    {
        WorkItem item = std::move(stack.back());
        stack.pop_back();
        if (item.remainingBudget <= 0.0f)
            continue;

        if (!item.backward)
        {
            for (const weak_ptr<Lane> &succWeak : item.lane->GetSuccessors())
            {
                shared_ptr<Lane> succ = succWeak.lock();
                if (!succ || visited.count(succ.get()))
                    continue;
                visited.insert(succ.get());
                scanLaneCars(succ, item.entryOffset, false);

                // 3-(b): succ로 들어오는 다른 predecessor(방금 온 item.lane 제외) -- 이 분기점에서 남은
                // forward 예산과 같은 값으로 역방향 탐색(합류 지점 차량을 내 진행경로 연장으로 취급).
                stack.push_back({succ, item.entryOffset, item.remainingBudget, true, item.lane.get()});

                float childRemaining = item.remainingBudget - succ->GetLength();
                stack.push_back({succ, item.entryOffset + succ->GetLength(), childRemaining, false, nullptr});
            }
        }
        else
        {
            for (const weak_ptr<Lane> &predWeak : item.lane->GetPredecessors())
            {
                shared_ptr<Lane> pred = predWeak.lock();
                if (!pred || pred.get() == item.excludeBranch || visited.count(pred.get()))
                    continue;
                visited.insert(pred.get());
                scanLaneCars(pred, item.entryOffset, true);

                float childRemaining = item.remainingBudget - pred->GetLength();
                stack.push_back({pred, item.entryOffset - pred->GetLength(), childRemaining, true, nullptr});
            }
        }
    }
}

void Car::FindGraphNeighbors(const shared_ptr<Lane> &rootLane, float rootPosition,
                             LaneNeighbor &outLeader, LaneNeighbor &outFollower) const
{
    outLeader = LaneNeighbor{};
    outFollower = LaneNeighbor{};

    WalkConnectedLanes(rootLane, rootPosition, [&](Car *car, float u)
                       {
        if (u >= 0.0f)
        {
            if (outLeader.car == nullptr || u < outLeader.position)
                outLeader = {car, u};
        }
        else
        {
            if (outFollower.car == nullptr || u > outFollower.position)
                outFollower = {car, u};
        } });
}

std::vector<Car *> Car::CollectNearbyCars(const shared_ptr<Lane> &rootLane, float rootPosition) const
{
    std::vector<Car *> cars;
    WalkConnectedLanes(rootLane, rootPosition, [&](Car *car, float /*u*/)
                       { cars.push_back(car); });
    return cars;
}

Mobil::VehicleState Car::ToVehicleState(const LaneNeighbor &n) const
{
    return Mobil::VehicleState{n.car->GetSpeed(), n.car->GetAcceleration(), n.position, n.car->GetLength()};
}

void Car::DriveSpeedIDM(float steerSpeedCap)
{
    // 주기 재스캔: 곡률 스캔은 비교적 무거우니 매 틱이 아니라 LOOK_PROFILE_TIME/SPEED_PROFILE_COUNT마다,
    // 또는 경로를 벗어난 것으로 감지되면(다음 재스캔까지 기다리지 않고) 갱신한다.
    constexpr float RESCAN_INTERVAL = LOOK_PROFILE_TIME / SPEED_PROFILE_COUNT;
    if (IsOffCourse() || m_currentTime - m_lastConstraintScanTime >= RESCAN_INTERVAL)
        RescanRoadSpeedConstraints();

    float v0 = std::min({m_maxSpeed, m_currentLane->GetLimitSpeed(), steerSpeedCap});
    CarFollowing::Params idmParams = BuildIdmParams(v0);

    // 개방도로(앞을 막는 지점이 전혀 없는) 기준선 -- gap을 아주 크게 줘서 CarFollowing이 자유흐름
    // 가속도(a_free)로 수렴하게 한다.
    constexpr float OPEN_ROAD_GAP = 100000.0f;
    Vec3 position = GetPosition();
    float accel = CarFollowing::CalculateAcceleration(m_speed, m_acceleration, v0, 0.0f, OPEN_ROAD_GAP, idmParams);

    // 캐시된 도로 제약(커브/레인 제한속도/정지점)을 각각 정지/저속 가상 리더로 보고, 그중 가장
    // 보수적인(작은) 가속도를 채택한다 -- 앞에 여러 제약이 겹쳐 있어도 가장 급한 것에 맞춰 미리 감속한다.
    for (const RoadSpeedSample &sample : m_roadConstraints)
    {
        float gap = (sample.position - position).Length();
        float constrainedAccel = CarFollowing::CalculateAcceleration(m_speed, m_acceleration, sample.speed, 0.0f, gap, idmParams);
        accel = std::min(accel, constrainedAccel);
    }

    // 연결된 차선까지 넓혀 찾은 실제 앞차(합류 지점 등 다른 레인 위의 차 포함): MOBIL이 이 차를 근거로
    // 차선변경을 판단하는 것과 일관되게, 실제로도 이 차를 향해 감속해야 뚫고 지나가지 않는다.
    float egoLanePos = m_currentLane->GetSpline().GetSplinePosition(position) * m_currentLane->GetLength();
    LaneNeighbor leader, follower;
    FindGraphNeighbors(m_currentLane, egoLanePos, leader, follower);
    if (leader.car != nullptr)
    {
        // leader.position은 이미 egoLanePos 기준 상대 거리(u)라 다시 뺄 필요 없음.
        float gap = leader.position - leader.car->GetLength();
        float leaderAccel = CarFollowing::CalculateAcceleration(m_speed, m_acceleration, leader.car->GetSpeed(),
                                                                leader.car->GetAcceleration(), gap, idmParams);
        accel = std::min(accel, leaderAccel);
    }

    // ScanBBoxObstacleGap이 경로 폭 안에서 찾아둔 최근접 정적/동적 장애물: 정지한 가상 선행차량으로
    // 취급해 감속한다. s0는 일반 차량 추종용(IDM_STANDSTILL_DISTANCE)보다 작게 따로 둔다 -- 정적
    // 장애물은 다른 차만큼 넉넉한 여유를 안 두고 더 붙어도 된다.
    if (m_obstacleAheadGap >= 0.0f)
    {
        CarFollowing::Params obstacleParams = idmParams;
        obstacleParams.s0 = AVOID_OBSTACLE_STANDSTILL_DISTANCE;
        float obstacleAccel = CarFollowing::CalculateAcceleration(m_speed, m_acceleration, 0.0f, 0.0f, m_obstacleAheadGap, obstacleParams);
        accel = std::min(accel, obstacleAccel);
    }

    m_acceleration = accel;
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

void Car::RebuildRSDebugRender(const ReedsShepp::Path &path, const Vec3 &startPos, float startAngleRad,
                               float turningRadius, const Vec3 &targetPos, float targetAngleRad)
{
    constexpr float DEBUG_LINE_HEIGHT = 0.15f;

    // RS 경로 폴리라인 (보라색)
    std::vector<Vec3> pathPoints = ReedsShepp::SamplePath(path, startPos, startAngleRad, turningRadius);
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

// TryAvoidObstacle이 SimulateBBTrajectory로 예측한 궤적을 그대로 훑어 전부 박스 윤곽선으로
// 그린다(첫 충돌에서 멈추지 않음) — 충돌한 샘플은 빨강, 통과한 샘플은 초록.
void Car::RebuildBBDebugRender(const std::vector<Vec3> &positions, const std::vector<Vec3> &directions,
                               const std::vector<HybridAStar::Obstacle> &obstacles,
                               const HybridAStar::VehicleShape &shape)
{
    constexpr float DEBUG_LINE_HEIGHT = 0.15f;

    m_bboxDebugRenders.clear();
    for (size_t sampleIndex = 0; sampleIndex < positions.size(); ++sampleIndex)
    {
        const Vec3 &samplePos = positions[sampleIndex];
        const Vec3 &sampleDir = directions[sampleIndex];
        float headingRad = atan2f(sampleDir.GetZ(), sampleDir.GetX());
        bool colliding = HybridAStar::IsColliding(samplePos, headingRad, obstacles, shape);

        // IsColliding과 동일하게, 실제 충돌판정 박스 중심은 pivot(samplePos)에서 heading 방향으로
        // pivotToCenter만큼 떨어진 지점이다 (HybridAStar.cpp IsPoseCollision 참고).
        Vec3 forward(cosf(headingRad), 0.0f, sinf(headingRad));
        Vec3 right(-forward.GetZ(), 0.0f, forward.GetX());
        Vec3 bodyCenter = samplePos + forward * shape.pivotToCenter;

        auto corner = [&](float alongSign, float acrossSign)
        {
            DirectX::XMFLOAT3 p = ToXMFLOAT3(bodyCenter + forward * (shape.halfLength * alongSign) + right * (shape.halfWidth * acrossSign));
            p.y += DEBUG_LINE_HEIGHT;
            return p;
        };
        std::vector<DirectX::XMFLOAT3> corners = {
            corner(1.0f, 1.0f), corner(1.0f, -1.0f), corner(-1.0f, -1.0f), corner(-1.0f, 1.0f), corner(1.0f, 1.0f)};

        Model *pModel = ModelManager::Get().CreateFromGeometry(
            "__bbox_bbox__:" + GetName() + ":" + std::to_string(sampleIndex), Geometry::CreatePolyline(corners));
        DirectX::XMFLOAT4 color = colliding ? DirectX::XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f) : DirectX::XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f);
        pModel->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", color);
        pModel->materials[0].Set<float>("$Opacity", 1.0f);

        RenderObject &render = m_bboxDebugRenders.emplace_back();
        render.SetModel(pModel);
    }
}

void Car::SetDestination(const shared_ptr<RoadNode> &destNode)
{
    m_destLane = m_RoadDataManager->GetClosestLaneEnd(destNode->position);
    DebugConsole::Log(GetName() + ": SetDestination -> node " + std::to_string(destNode->id) +
                      " (lane " + std::to_string(m_destLane ? m_destLane->GetId() : -1) + ")");
    if (destNode->nodeType == RoadNodeType::Park)
    {
        m_pendingParkNode = destNode;
    }
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

    Model *pMarker = ModelManager::Get().CreateFromGeometry("__origin__:" + GetName(), Geometry::CreateSphere(0.1f, 8, 8));
    pMarker->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f));
    pMarker->materials[0].Set<float>("$Opacity", 1.0f);
    m_originMarker.SetModel(pMarker);

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