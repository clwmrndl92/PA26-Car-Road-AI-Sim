#include "Car.h"
#include "Core/Physics/PhysicsSystem.h"
#include "Rendering/Effects.h"
#include <ModelManager.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_set>
#include <imgui.h>
#include "Utill/DebugConsole.h"
#include "Utill/Assert.h"

void Car::Init(const CarSpec &spec, RoadDataManager *roadDataManager, JPH::Vec3 position)
{
    SetName(spec.name);
    m_render.SetModel(ModelManager::Get().CreateFromFile(spec.modelPath));
    SetRenderOffset(ToXMFLOAT3(spec.renderOffset));
    m_wheelbase = spec.wheelbase;
    m_halfExtents = spec.halfExtents;
    m_behaviorWeights = spec.behaviorWeights;

    DirectX::XMFLOAT3 fwd = m_transform.GetForwardAxis();
    m_transform.SetPosition(position.GetX() - fwd.x * m_wheelbase,
                            position.GetY() - fwd.y * m_wheelbase,
                            position.GetZ() - fwd.z * m_wheelbase);

    GameObject::Init(spec.halfExtents, Rigidbody::Type::Kinematic, spec.colliderOffset, spec.mass);

    m_spawnPosition = m_transform.GetPosition();
    m_spawnRotation = m_transform.GetRotationQuat();
    m_mass = spec.mass;

    m_RoadDataManager = roadDataManager;
    m_RoadDataManager->RegisterCar(this);

    // todo : 출차시 예약자리 release
    m_parkSpot = make_shared<RoadNode>();
    m_parkSpot->id = -1;
    m_parkSpot->position = GetPosition();
    m_parkSpot->direction = GetForwardAxis();
    m_parkSpot->nodeType = RoadNodeType::ParkSpot;

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

// IDM의 자유가속 항 a*(1-(v/v0)^4)을 그대로 가져다, 여기서 v0는 "지금 목표로 하는 속도"
// (desiredVelocity: 도로제약/앞차 gap을 이미 다 반영한 값)로 쓴다. v<v0면 0~a_max 사이에서 목표에
// 가까워질수록 부드럽게 줄어들고, v>v0면 자연히 음수(감속)가 되며 초과분이 클수록 더 세게 감속한다.
// 이전의 모드 전환 + S자 램프(저크 완화) 방식은, 목표속도가 빠르게 낮아지는 상황(커브 진입 등)에서
// 램프가 다 차기 전엔 최대 제동력의 일부만 나가 감속이 늦어지는 문제가 있어 이 방식으로 대체했다.
void Car::Accelerate(float desiredVelocity)
{
    constexpr float MIN_TARGET_SPEED = 0.01f; // 0으로 나누기 방지 -- 사실상 "완전 정지"로 취급
    if (desiredVelocity < MIN_TARGET_SPEED)
    {
        m_acceleration = (m_speed > 0.0f) ? -m_maxBrake : 0.0f;
        return;
    }

    float ratio = m_speed / desiredVelocity;
    float ratioPow4 = ratio * ratio * ratio * ratio;
    m_acceleration = std::max(-m_maxBrake, m_maxAccel * (1.0f - ratioPow4));
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
    if (m_RoadDataManager != nullptr)
        m_RoadDataManager->UnregisterCar(this); // 전역 차량 레지스트리에서 자신을 빼둔다.
    GameObject::Destroy();
}

void Car::SetCurrentLane(const shared_ptr<Lane> &lane)
{
    if (m_currentLane == lane)
        return;
    m_currentLane = lane;

    RebuildSplineRender();
}

std::vector<Car::RoadSpeedSample> Car::ScanRoadSpeedConstraints(float lookDistance) const
{
    constexpr float ROAD_SAMPLE_SPACING = 5.0f; // 도로 스캔 샘플 간격 (m)
    constexpr float LOCAL_WINDOW = 0.1f;        // 로컬 곡률 추정용 t-window

    Vec3 calPosition = GetPosition();
    const std::vector<VehicleCollision::Obstacle> &obstacles = m_RoadDataManager->GetObstacles();

    // m_currentLane->GetSpline() -> (필요시) path상의 다음 노드들의 lane spline 순으로 lookDistance까지 훑으며
    // 커브 지점(로컬 곡률 기반 최대속도)과 노드 지점(제한속도)의 샘플을 모은다. 각 샘플이 곧
    // ComputeSpeedCapFromSamples가 쓰는 "가상 리더"의 (위치, 거리, 요구 속도) 후보가 된다.
    std::vector<RoadSpeedSample> samples;
    samples.reserve(static_cast<size_t>(lookDistance / ROAD_SAMPLE_SPACING) + 4);
    {
        float currentNodeT = m_currentLane->GetSpline().GetSplinePosition(calPosition);
        Assert(currentNodeT >= 0.0f); // ScanRoadSpeedConstraints 호출 전엔 항상 m_currentLane->GetSpline()이 세팅되어 있어야 함
        float currentNodeDistance = m_currentLane->GetSpline().GetLength() * (1.0f - currentNodeT);
        float currentNodeSpeed = (m_currentLane == m_destLane) ? 0.0f : std::min(m_currentLane->GetLimitSpeed(), m_maxSpeed);
        samples.push_back({m_currentLane->GetEndPoint(), currentNodeDistance, currentNodeSpeed});
    }
    {
        const Spline *spline = &m_currentLane->GetSpline();
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

                size_t sampleCount = static_cast<size_t>(walkDistance / ROAD_SAMPLE_SPACING) + 1;
                for (size_t s = 1; s <= sampleCount; ++s)
                {
                    float localDistance = std::min(walkDistance, s * ROAD_SAMPLE_SPACING);
                    float t = startT + localDistance / splineLength;
                    size_t index = static_cast<size_t>(std::clamp(t, 0.0f, 1.0f) * lastIndex);

                    float radius = spline->GetMinRadiusAhead(std::max(0.0f, t - LOCAL_WINDOW), std::min(1.0f, t + LOCAL_WINDOW));

                    float maxSpeed = m_maxSpeed;
                    if (radius < std::numeric_limits<float>::max())
                    {
                        maxSpeed = CURVE_SPEED_COEFF * std::sqrt(radius);
                        DebugConsole::Log(GetName() + ": [curve-scan] dist=" + std::to_string(traveledDistance + localDistance) +
                                          "m radius=" + std::to_string(radius) + " curveSpeedCap=" +
                                          std::to_string(maxSpeed * 3.6f) + "km/h");
                    }
                    samples.push_back({points[index], traveledDistance + localDistance, maxSpeed});

                    // 경로 코리도와 겹치는 정적 장애물이 있으면 그 앞에 0속도 샘플(가상 정지선)을
                    // 세운다 -- 신호 정지선과 같은 방식으로 미리 감속한다. 프로브 폭은 차선폭이 아니라
                    // "차폭 + 여유"다: 차선폭 절반(1.6m)으로 하면 실제로는 지나갈 수 있는 길가 벽
                    // (예: 도로 옆 1.5m의 주차장 벽)에 걸려 가짜 정지선이 생긴다.
                    if (!obstacles.empty())
                    {
                        constexpr float OBSTACLE_PROBE_MARGIN = 0.25f;
                        Vec3 probeDir = spline->GetDirectionAt(std::clamp(t, 0.0f, 1.0f));
                        VehicleCollision::VehicleShape probeShape;
                        probeShape.pivotToCenter = 0.0f;
                        probeShape.halfLength = ROAD_SAMPLE_SPACING * 0.5f;
                        probeShape.halfWidth = m_halfExtents.GetX() + OBSTACLE_PROBE_MARGIN;
                        float probeHeading = atan2f(probeDir.GetZ(), probeDir.GetX());
                        if (VehicleCollision::IsColliding(points[index], probeHeading, obstacles, probeShape))
                        {
                            // 프로브 안 어디에 걸렸는지는 모르니 프로브 반길이 + 안전마진만큼 앞에서 선다.
                            float stopDistance = traveledDistance + localDistance - ROAD_SAMPLE_SPACING * 0.5f - MIN_SAFE_GAP;
                            samples.push_back({points[index], std::max(stopDistance, 0.01f), 0.0f});
                        }
                    }
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

    // 정지선(신호 노드)을 이미 넘었으면 이 신호는 더 이상 나를 구속하지 않는다. 이때 커밋도 함께
    // 해제해야 한다 -- 통과 후엔 이 레인이 경로에서 빠져 아래 초록 리셋이 다시 불릴 기회가 없어서,
    // 커밋이 영원히 남아 "맵을 돌아 같은 신호에 빨간불로 재접근"할 때 그대로 통과해버린다.
    const Spline &spline = lane->GetSpline();
    float nodeT = spline.GetSplinePosition(signalNode->position);
    float myT = spline.GetSplinePosition(GetPosition());
    if (myT > nodeT)
    {
        if (m_committedYellowNodeId == signalNode->id)
            m_committedYellowNodeId = -1;
        return false;
    }

    TrafficSignal::Color color = m_RoadDataManager->GetSignalColor(signalNode->signalPhaseOffset);
    float gap = (signalNode->position - GetPosition()).Length();
    float brakingDistance = (m_speed * m_speed) / (2.0f * m_maxBrake);
    if (color == TrafficSignal::Color::Green)
    {
        if (m_committedYellowNodeId == signalNode->id)
            m_committedYellowNodeId = -1; // 다음 사이클 대비 리셋
    }
    else if (color == TrafficSignal::Color::Yellow && m_committedYellowNodeId != signalNode->id)
    {
        if (gap <= brakingDistance)
            m_committedYellowNodeId = signalNode->id; // 정지거리 안쪽 -- 통과 확정
    }
    else if (color == TrafficSignal::Color::Red && m_committedYellowNodeId == signalNode->id)
    {
        // 노란불에 통과 확정했지만 (앞차 등으로) 지연돼 아직 정지선 앞인데 빨간불이 됐다:
        // 지금은 설 수 있으면 커밋을 취소하고 선다 -- 안 하면 빨간불을 그대로 뚫는다.
        if (gap > brakingDistance)
            m_committedYellowNodeId = -1;
    }
    return color != TrafficSignal::Color::Green && m_committedYellowNodeId != signalNode->id;
}

// samples 각각을, distanceOffset만큼 이미 다가간 지점 기준으로 다시 평가한다 -- distanceOffset=0이면
// "지금 이 순간" 기준(=예전 ComputeDesiredCruiseSpeed)과 같고, distanceOffset>0이면 궤적을 따라 그만큼
// 전진한 미래 시점 기준의 국소 안전속도 상한이 된다 (desiredSpeed는 계속 낮아지는데 그 사실을
// "지금 시점 값 하나"로만 비교하면 접근 구간에서 감속 판단이 늦어지는 문제를 피하기 위함).
float Car::ComputeSpeedCapFromSamples(const std::vector<RoadSpeedSample> &samples, float distanceOffset) const
{
    float speedCap = m_maxSpeed;
    for (const RoadSpeedSample &sample : samples)
    {
        float remaining = sample.distance - distanceOffset;
        if (remaining <= 0.0f)
            continue; // 이미 지난(또는 그 시점에서 지날) 샘플 -- 감속 대상 아님
        float allowedSpeed = std::sqrt(sample.speed * sample.speed + 2.0f * m_maxBrake * remaining);
        speedCap = std::min(speedCap, allowedSpeed);
    }
    return speedCap;
}

// 레인 등록 없이 위치만으로 주변 차를 모은다. 컬링 반경은 "3초(BEHAVIOR_SAFETY_HORIZON) 동안
// 나와 상대가 정면으로 마주 달려도 닿을 수 없는 거리"로 차마다 계산한다 -- 이 밖의 차는 어떤
// 후보 궤적으로도 시뮬레이션 중 겹칠 수 없으므로 EvaluateTrajectorySafety에 넘길 필요가 없다.
std::vector<Car::NearbyCar> Car::CollectNearbyCars() const
{
    std::vector<NearbyCar> nearby;
    Vec3 egoPosition = GetPosition();
    Vec3 egoFwd = GetForwardAxis();
    constexpr float SAME_DIRECTION_COS = 0.7071f; // 45도
    for (Car *other : m_RoadDataManager->GetCars())
    {
        if (other == this)
            continue;
        // 가속 후보(+m_maxAccel)로 3초 내 더 갈 수 있는 거리(0.5*a*t^2)까지 더해 안전측으로 잡는다.
        float reachDistance = (m_speed + other->GetSpeed()) * BEHAVIOR_SAFETY_HORIZON +
                              0.5f * m_maxAccel * BEHAVIOR_SAFETY_HORIZON * BEHAVIOR_SAFETY_HORIZON +
                              GetLength() * 0.5f + other->GetLength() * 0.5f + MIN_SAFE_GAP;
        if ((other->GetPosition() - egoPosition).Length() > reachDistance)
            continue;

        // 교차/합류 방향(진행방향 차 45도 초과) 차 중 내가 우선권을 가진 차만 "양보 예정"으로 본다.
        // 같은 방향 차(앞차/뒷차)는 우선순위 대상이 아니다 -- 코리도 리더 샘플로 차간을 유지한다.
        bool crossing = egoFwd.Dot(other->GetForwardAxis()) < SAME_DIRECTION_COS;
        nearby.push_back({other, crossing && HasPriorityOver(other)});
    }
    return nearby;
}

// 전방 약 15m 경로(현재 레인 잔여 + 다음 레인)의 최소 곡률반경이 회전 매뉴버 수준(<20m)인가.
bool Car::IsTurningAhead() const
{
    constexpr float TURN_LOOK_DISTANCE = 15.0f;
    constexpr float TURN_RADIUS_THRESHOLD = 20.0f;
    if (m_currentLane == nullptr)
        return false;

    const Spline &spline = m_currentLane->GetSpline();
    float length = spline.GetLength();
    float t0 = spline.GetSplinePosition(GetPosition());
    float t1 = (length > 0.0f) ? std::min(1.0f, t0 + TURN_LOOK_DISTANCE / length) : 1.0f;
    float minRadius = spline.GetMinRadiusAhead(t0, t1);

    float covered = (t1 - t0) * length;
    if (covered < TURN_LOOK_DISTANCE && m_pathIndex + 1 < m_path.size())
    {
        const Spline &next = m_path[m_pathIndex + 1].lane->GetSpline();
        float nextLength = next.GetLength();
        float nextT1 = (nextLength > 0.0f) ? std::min(1.0f, (TURN_LOOK_DISTANCE - covered) / nextLength) : 1.0f;
        minRadius = std::min(minRadius, next.GetMinRadiusAhead(0.0f, nextT1));
    }
    return minRadius < TURN_RADIUS_THRESHOLD;
}

// 교차 상황의 통행 우선권: 직진 > 회전(우회전/좌회전 등 작은 곡률 매뉴버), 같으면 이름 비교로
// 결정적 tie-break -- 규칙이 상보적(내가 우선이면 상대는 양보)이어야 교착이 안 생긴다.
bool Car::HasPriorityOver(const Car *other) const
{
    bool meTurning = IsTurningAhead();
    bool otherTurning = other->IsTurningAhead();
    if (meTurning != otherTurning)
        return !meTurning;
    return GetName() < other->GetName();
}

// 내 예정 경로(현재 레인 -> path의 다음 레인들)를 걸으며, 각 차를 그 세그먼트 스플라인에 투영해
// 횡 오프셋이 코리도 폭(차선폭/2 + 상대 반폭) 안이면 가상 리더 샘플로 추가한다.
// - 속도는 내 경로 방향 성분만 쓴다(dot). 교차/역방향 차는 0으로 클램프되어 그 지점의 정지
//   장애물처럼 취급되고, 코리도를 벗어나면 다음 플랜(0.2초)에서 샘플이 자연히 사라진다.
// - 옆 차선을 평행하게 달리는 차는 코리도 밖이라 여기서 걸러진다 -- 그런 차와의 간섭(차선변경 등)은
//   EvaluateTrajectorySafety의 OBB 시뮬이 담당한다.
void Car::AppendCarConstraintSamples(std::vector<RoadSpeedSample> &samples,
                                     const std::vector<NearbyCar> &nearbyCars, float lookDistance) const
{
    const Spline *spline = &m_currentLane->GetSpline();
    size_t pathIndex = m_pathIndex;
    float baseDistance = 0.0f; // 내 위치에서 이 세그먼트 시작(startT)까지의 누적 경로거리
    float startT = spline->GetSplinePosition(GetPosition());

    while (spline != nullptr && baseDistance <= lookDistance)
    {
        float splineLength = spline->GetLength();
        for (const NearbyCar &nearbyCar : nearbyCars)
        {
            // 나에게 양보할 차는 가상 리더로 세우지 않는다 -- 세우면 서로가 서로 앞에 정지선을
            // 만들어 교차로에서 대칭 교착이 생긴다. 실제 차체 회피는 OBB 검사가 담당.
            if (nearbyCar.yieldsToMe)
                continue;
            Car *other = nearbyCar.car;
            float otherT = spline->GetSplinePosition(other->GetPosition());
            if (otherT < startT)
                continue; // 이 세그먼트 기준 내 뒤 -- 리더 아님
            Vec3 projected = spline->GetPositionAt(otherT);
            float lateralOffset = (other->GetPosition() - projected).Length();
            if (lateralOffset > RoadDataManager::ROAD_WIDTH * 0.5f + other->GetHalfWidth())
                continue; // 코리도 밖

            Vec3 pathDir = spline->GetDirectionAt(otherT);
            float alongSpeed = std::max(0.0f, pathDir.Dot(other->GetForwardAxis()) * other->GetSpeed());

            // 앞축끼리의 경로거리에서 상대 차체(앞축 뒤로 뻗은 길이)와 안전마진을 뺀 지점에
            // "이 속도까지 줄여야 하는" 제약을 세운다. 이미 그보다 가까우면 0 근처로 클램프
            // -> ComputeSpeedCapFromSamples가 즉시 리더 속도로 cap을 내린다.
            float gap = baseDistance + (otherT - startT) * splineLength - other->GetLength() - MIN_SAFE_GAP;
            samples.push_back({other->GetPosition(), std::max(gap, 0.01f), alongSpeed});
        }

        baseDistance += (1.0f - startT) * splineLength;
        shared_ptr<Lane> nextLane = (pathIndex + 1 < m_path.size()) ? m_path[pathIndex + 1].lane : nullptr;
        if (nextLane == nullptr)
            break;
        spline = &nextLane->GetSpline();
        startT = 0.0f;
        ++pathIndex;
    }
}

// lane의 스플라인을 따라 Pure Pursuit + 자전거 모델로 BEHAVIOR_SAFETY_HORIZON 동안 BEHAVIOR_SIM_STEP
// 간격으로 전진시켜본다. simAccel(가정 가속도)로 speedAction의 종방향 프로파일을 흉내낸다.
// 가속 후보는 각 지점의 국소 허용속도(cap)를 추종한다: "3초 내내 풀가속" 그대로 두면 커브/정지
// 지점이 지평선 안에 있는 동안 Accel 후보가 항상 overshoot 페널티를 받아, 허용속도보다 한참 느린
// 저속에서도 가속을 못 골라 기어가거나 얼어붙는다 (실제 DriveControl도 매 틱 cap으로 클램프한
// desiredSpeed를 추종하므로 이게 현실적인 후보다). cap이 maxBrake보다 빨리 떨어지는 "진짜 못
// 멈추는" 상황만 overshoot로 남아 나쁘게 평가된다.
std::vector<Car::TrajectorySample> Car::SimulateEgoTrajectory(const shared_ptr<Lane> &lane, float simAccel,
                                                              const std::vector<RoadSpeedSample> &roadSamples) const
{
    std::vector<TrajectorySample> samples;
    if (lane == nullptr)
        return samples;

    const Spline &spline = lane->GetSpline();
    Vec3 pos = GetRigidbodyPosition();
    Vec3 dir = GetForwardAxis();
    float speed = m_speed;
    float lookaheadDistance = ComputeLookaheadDistance();
    float distanceTraveled = 0.0f;

    size_t stepCount = static_cast<size_t>(BEHAVIOR_SAFETY_HORIZON / BEHAVIOR_SIM_STEP + 0.5f);
    samples.reserve(stepCount);

    for (size_t i = 0; i < stepCount; ++i)
    {
        Vec3 targetPos = spline.GetLookaheadPoint(pos, lookaheadDistance);
        Vec3 toTarget = targetPos - pos;
        float distance = toTarget.Length();
        float maxSteer = CalcMaxSteerAngle(speed);
        float steerAngle = 0.0f;
        if (distance > 0.001f)
        {
            // Car::PurePursuit와 같은 조향각 공식을, carRight 없이 dir/toTarget만으로 부호까지 나오는
            // atan2 기반 signed heading error로 다시 쓴 것 (결과는 동일).
            float headingError = atan2f(dir.GetX() * toTarget.GetZ() - dir.GetZ() * toTarget.GetX(), dir.Dot(toTarget));
            steerAngle = std::clamp(-atanf(2.0f * m_wheelbase * sinf(headingError) / distance), -maxSteer, maxSteer);
        }

        float nextSpeed = speed + simAccel * BEHAVIOR_SIM_STEP;
        if (simAccel > 0.0f)
        {
            // 가속 후보는 국소 cap을 "추종"한다: cap 아래면 가속, cap을 넘으면 maxBrake 한도 내에서
            // cap까지 감속 (실제 DriveControl이 하는 일과 같다). 이래야 정지 제약이 지평선 안에 있어도
            // "다가가서 그 앞에 선다"가 가속 후보로 표현된다 -- 없으면 제약 앞 수십 m에서 얼어붙는다.
            // cap이 maxBrake보다 빨리 떨어지면(진짜 못 멈추는 상황) 초과분이 남아 overshoot로 잡힌다.
            float localCap = ComputeSpeedCapFromSamples(roadSamples, distanceTraveled);
            if (nextSpeed > localCap)
                nextSpeed = std::max(localCap, speed - m_maxBrake * BEHAVIOR_SIM_STEP);
        }
        speed = std::clamp(nextSpeed, 0.0f, m_maxSpeed);

        // ApplyMotion의 자전거 모델과 동일: angularVelocity = speed*tan(steerAngle)/wheelbase.
        float currentAngle = atan2f(dir.GetZ(), dir.GetX());
        float nextAngle = currentAngle - BEHAVIOR_SIM_STEP * speed * tanf(steerAngle) / m_wheelbase;
        pos = pos + dir * (speed * BEHAVIOR_SIM_STEP);
        dir = Vec3(cosf(nextAngle), 0.0f, sinf(nextAngle));
        distanceTraveled += speed * BEHAVIOR_SIM_STEP;

        samples.push_back({pos, dir, speed, distanceTraveled});
    }
    return samples;
}

// 상대가 "자기 레인을 따라간다"는 예측 정보를 만든다. 내 궤적은 Pure Pursuit로 차선을 따르게
// 시뮬레이션하면서 상대만 등속 직진 외삽하면 비대칭이 생긴다 -- 커브에서 상대 전방벡터(곡선의
// 접선)를 그대로 연장한 "직진 유령"이 내 차선을 가로질러, 실제로는 스쳐 지나갈 반대편 차선 차와
// 가짜 정면충돌이 예측되어 멈춰버린다. 상대도 자기 스플라인을 따라 전진시켜 이 비대칭을 없앤다.
Car::OtherPrediction Car::BuildOtherPrediction(const Car *other) const
{
    OtherPrediction pred;
    pred.basePos = other->GetRigidbodyPosition();
    pred.baseFwd = other->GetForwardAxis();

    const shared_ptr<Lane> &lane = other->m_currentLane;
    if (lane == nullptr)
        return pred; // 레인 없음(주차 완료/도착 등) -> 직진 폴백

    const Spline &spline = lane->GetSpline();
    float t0 = spline.GetSplinePosition(pred.basePos);
    Vec3 projected = spline.GetPositionAt(t0);
    Vec3 dir0 = spline.GetDirectionAt(t0);

    // 레인 중심에서 차선폭 이상 벗어났거나 레인 진행방향과 반대를 보고 있으면(RS 매뉴버 등)
    // "레인을 따르는 중"이라는 가정이 안 맞으니 직진 폴백.
    if ((pred.basePos - projected).Length() > RoadDataManager::ROAD_WIDTH || dir0.Dot(pred.baseFwd) < 0.0f)
        return pred;

    Vec3 left0(-dir0.GetZ(), 0.0f, dir0.GetX());
    pred.lateralOffset = (pred.basePos - projected).Dot(left0);

    // 지평선(3초) 동안 갈 수 있는 거리만큼 현재 레인 + 상대 path의 다음 레인들을 이어 붙인다.
    // 상대의 path가 현재 레인과 안 맞으면(방금 레인 전환 등) 현재 레인까지만 쓰고 넘어선 직진 외삽.
    float needDistance = other->GetSpeed() * BEHAVIOR_SAFETY_HORIZON + 5.0f;
    bool pathAligned = other->m_pathIndex < other->m_path.size() &&
                       other->m_path[other->m_pathIndex].lane == lane;
    const Spline *segSpline = &spline;
    float segStartT = t0;
    size_t pathIndex = other->m_pathIndex;
    float covered = 0.0f;
    while (covered < needDistance)
    {
        float remainingArc = (1.0f - segStartT) * segSpline->GetLength();
        pred.segments.push_back({segSpline, segStartT, remainingArc});
        covered += remainingArc;
        if (!pathAligned || pathIndex + 1 >= other->m_path.size())
            break;
        ++pathIndex;
        segSpline = &other->m_path[pathIndex].lane->GetSpline();
        segStartT = 0.0f;
    }
    return pred;
}

// pred를 따라 distance만큼 전진했을 때의 예상 (뒷축 위치, 진행방향). 세그먼트가 바닥나면 마지막
// 지점에서 직진 외삽.
void Car::PredictOtherPose(const OtherPrediction &pred, float distance, Vec3 &outPos, Vec3 &outFwd)
{
    if (pred.segments.empty())
    {
        outPos = pred.basePos + pred.baseFwd * distance;
        outFwd = pred.baseFwd;
        return;
    }

    float remaining = distance;
    for (size_t i = 0; i < pred.segments.size(); ++i)
    {
        const OtherPrediction::Segment &seg = pred.segments[i];
        bool isLast = (i + 1 == pred.segments.size());
        if (remaining > seg.arcLength && !isLast)
        {
            remaining -= seg.arcLength;
            continue;
        }
        if (remaining > seg.arcLength) // 마지막 세그먼트도 넘어감 -> 레인 끝에서 직진 외삽
        {
            outFwd = seg.spline->GetDirectionAt(1.0f);
            Vec3 left(-outFwd.GetZ(), 0.0f, outFwd.GetX());
            outPos = seg.spline->GetPositionAt(1.0f) + left * pred.lateralOffset + outFwd * (remaining - seg.arcLength);
            return;
        }
        float length = seg.spline->GetLength();
        float t = (length > 0.0f) ? seg.startT + remaining / length : seg.startT;
        outFwd = seg.spline->GetDirectionAt(t);
        Vec3 left(-outFwd.GetZ(), 0.0f, outFwd.GetX());
        outPos = seg.spline->GetPositionAt(t) + left * pred.lateralOffset;
        return;
    }
}

// others는 각자 자기 레인 스플라인을 따라 등속 전진한다고 예측해(BuildOtherPrediction), 매 스텝
// ego 궤적과 겹치는지 OBB로 검사한다. 레인을 벗어난 차(주차 매뉴버 등)는 직진 외삽으로 폴백.
Car::TrajectorySafety Car::EvaluateTrajectorySafety(const std::vector<TrajectorySample> &trajectory,
                                                     const std::vector<NearbyCar> &others) const
{
    TrajectorySafety result;
    const std::vector<VehicleCollision::Obstacle> &staticObstacles = m_RoadDataManager->GetObstacles();
    if (others.empty() && staticObstacles.empty())
        return result;

    VehicleCollision::VehicleShape egoShape = BuildVehicleShape();

    std::vector<OtherPrediction> predictions;
    predictions.reserve(others.size());
    for (const NearbyCar &nearbyCar : others)
        predictions.push_back(BuildOtherPrediction(nearbyCar.car));

    for (size_t i = 0; i < trajectory.size(); ++i)
    {
        float t = (i + 1) * BEHAVIOR_SIM_STEP;
        const TrajectorySample &ego = trajectory[i];
        float egoHeadingRad = atan2f(ego.direction.GetZ(), ego.direction.GetX());
        Vec3 egoCenter = ego.position + ego.direction * egoShape.pivotToCenter;

        std::vector<VehicleCollision::Obstacle> predicted;
        predicted.reserve(others.size());
        for (size_t k = 0; k < others.size(); ++k)
        {
            Car *other = others[k].car;
            float travel = other->GetSpeed() * t;
            if (others[k].yieldsToMe)
            {
                // 나에게 양보할 차: 지금부터 maxBrake로 세운다고 보고 이동량을 제동거리로 캡.
                // (완전히 무시하면 이미 길을 막고 선 차를 뚫으므로, 멈춘 차체는 그대로 장애물로 남긴다)
                float stopDistance = (other->GetSpeed() * other->GetSpeed()) / (2.0f * other->m_maxBrake);
                travel = std::min(travel, stopDistance);
            }
            // 예측 피벗은 뒷축(rigidbody) 기준 -- pivotToCenter(colliderOffset.z)의 기준점과 맞춘다.
            Vec3 otherPivot;
            Vec3 otherFwd;
            PredictOtherPose(predictions[k], travel, otherPivot, otherFwd);
            VehicleCollision::VehicleShape otherShape = other->BuildVehicleShape();
            float otherHeadingRad = atan2f(otherFwd.GetZ(), otherFwd.GetX());
            Vec3 otherCenter = otherPivot + otherFwd * otherShape.pivotToCenter;

            predicted.push_back({otherCenter, otherShape.halfLength, otherShape.halfWidth, otherHeadingRad, 0.0f});
            result.minGap = std::min(result.minGap, (egoCenter - otherCenter).Length());
        }

        // 정적 장애물은 minGap 집계에서는 뺀다 -- 작은 장애물 옆을 지나가는 정상 궤적이
        // 중심거리 기준(MIN_SAFE_GAP)으로 unsafe 처리되는 걸 막기 위해 겹침 여부만 본다.
        if (VehicleCollision::IsColliding(ego.position, egoHeadingRad, predicted, egoShape) ||
            VehicleCollision::IsColliding(ego.position, egoHeadingRad, staticObstacles, egoShape))
        {
            result.collisionFree = false;
            break;
        }
    }
    return result;
}

// lane에 신호가 있고 지금 서야 하는 상황(ShouldStopForSignal)인데, trajectory 상 어느 시점엔가 정지선
// (신호 노드 위치)의 스플라인 t를 넘어버렸는데도 그 시점 속도가 거의 0이 아니면 "못 멈추고 통과" =
// 신호위반으로 본다.
bool Car::ViolatesSignal(const shared_ptr<Lane> &lane, const std::vector<TrajectorySample> &trajectory) const
{
    shared_ptr<RoadNode> signalNode = lane->GetSignalNode();
    if (signalNode == nullptr || !ShouldStopForSignal(lane))
        return false;

    constexpr float MOVING_EPSILON = 0.3f; // m/s -- 이 이상 속도로 정지선을 넘으면 "통과"로 본다
    const Spline &spline = lane->GetSpline();
    float signalT = spline.GetSplinePosition(signalNode->position);

    for (const TrajectorySample &sample : trajectory)
    {
        float sampleT = spline.GetSplinePosition(sample.position);
        if (sampleT >= signalT && sample.speed > MOVING_EPSILON)
            return true;
    }
    return false;
}

// laneChoice/speedAction 조합 하나를 궤적 시뮬레이션까지 돌려 완전히 채운 후보로 만든다. 차선변경
// 후보는 목적지까지의 새 경로도 같이 탐색해두고(newPath), 못 찾으면 targetLane을 비워 무효 후보로
// 표시한다. roadSamples는 UpdateBehaviorPlan이 한 번만 스캔해 넘긴 도로제약 샘플(제한속도/커브/신호) --
// 후보마다 다시 스캔할 필요 없이, 궤적의 각 지점에서 ComputeSpeedCapFromSamples로 국소 상한과 비교한다.
Car::BehaviorCandidate Car::BuildCandidate(LaneChoice laneChoice, SpeedAction speedAction,
                                           const shared_ptr<Lane> &lane,
                                           const std::vector<RoadSpeedSample> &roadSamples,
                                           const std::vector<NearbyCar> &nearbyCars) const
{
    BehaviorCandidate candidate;
    candidate.laneChoice = laneChoice;
    candidate.speedAction = speedAction;
    candidate.targetLane = lane;
    if (lane == nullptr)
        return candidate;

    if (laneChoice != LaneChoice::Keep)
    {
        candidate.newPath = m_RoadDataManager->FindPath(lane, m_destLane);
        if (candidate.newPath.empty())
        {
            candidate.targetLane = nullptr; // 이 레인으로는 목적지에 못 감 -- 무효 후보
            return candidate;
        }
    }

    float simAccel = (speedAction == SpeedAction::Accelerate)   ? m_maxAccel
                      : (speedAction == SpeedAction::Decelerate) ? -m_maxBrake
                                                                  : 0.0f;
    std::vector<TrajectorySample> trajectory = SimulateEgoTrajectory(lane, simAccel, roadSamples);
    if (trajectory.empty())
    {
        candidate.targetSpeed = m_speed;
        candidate.horizonEndSpeed = m_speed;
        return candidate;
    }

    // 차선유지든 변경이든 같은 반경 기반 주변 차 목록으로 궤적과 겹치는지 확인한다 (차선유지도 예외 없음).
    TrajectorySafety safety = EvaluateTrajectorySafety(trajectory, nearbyCars);
    candidate.collisionFree = safety.collisionFree;
    candidate.minApproachGap = safety.minGap;
    candidate.signalViolation = ViolatesSignal(lane, trajectory);

    // 궤적의 각 지점마다 "그 지점 기준" 국소 안전속도 상한과 비교해, 가장 심하게 넘어선 값을 찾는다.
    // 지금 시점의 desiredSpeed 하나로만 3초 뒤 속도를 비교하면, desiredSpeed 자체가 접근하면서 계속
    // 낮아지는 걸 못 따라가서 감속 판단이 늦어진다 (커브/정지선에 실제로 다다르는 지점 기준으로 봐야 함).
    float maxOvershoot = 0.0f;
    for (const TrajectorySample &sample : trajectory)
    {
        float localCap = ComputeSpeedCapFromSamples(roadSamples, sample.distanceTraveled);
        maxOvershoot = std::max(maxOvershoot, sample.speed - localCap);
    }
    candidate.maxSpeedOvershoot = maxOvershoot;

    size_t planStepIndex = std::min(trajectory.size(),
                                     static_cast<size_t>(BEHAVIOR_PLAN_INTERVAL / BEHAVIOR_SIM_STEP)) -
                            1;
    candidate.targetSpeed = trajectory[planStepIndex].speed;
    candidate.horizonEndSpeed = trajectory.back().speed;
    return candidate;
}

// 충돌하는(또는 목적지 도달이 불가능해진) 후보는 평가 이전에 걸러낸다. BuildCandidate에서 시뮬레이션한
// 결과(collisionFree/minApproachGap)를 그대로 검사 -- 차선유지도 예외 없이 같은 기준을 적용한다.
bool Car::IsCandidateSafe(const BehaviorCandidate &candidate) const
{
    if (candidate.targetLane == nullptr)
        return false; // BuildCandidate에서 이미 무효 처리된 후보(레인 없음/경로 없음)
    return candidate.collisionFree && candidate.minApproachGap >= MIN_SAFE_GAP;
}

// cost = w1*(속도 오차) + w2*(차선변경했으면 고정값)
//      + w4*(직전에 고른 후보와 차선/속도결정이 달라졌으면 고정값) + w5*(신호위반이면 고정값)
// TODO: 횡가속도 최대값(w3) 항은 추가 예정.
float Car::EvaluateCandidateCost(const BehaviorCandidate &candidate, float desiredSpeed) const
{
    // 속도 오차 = 목표속도에 못 미친 만큼(1배, 그냥 아쉬운 정도) + BuildCandidate가 궤적 전체를 훑어
    // 계산해둔 maxSpeedOvershoot(4배) -- 궤적 중 어느 지점에서든 그 지점 기준 안전속도를 넘어섰다는
    // 뜻이라, 신호/커브/목적지를 못 멈추고 지나칠 뻔했다는 것이므로 훨씬 나쁘게 취급한다.
    constexpr float OVERSHOOT_PENALTY = 4.0f;
    float undershoot = std::max(0.0f, desiredSpeed - candidate.horizonEndSpeed);
    float speedCost = undershoot + candidate.maxSpeedOvershoot * OVERSHOOT_PENALTY;
    float laneChangeCost = (candidate.laneChoice != LaneChoice::Keep) ? 1.0f : 0.0f;

    // 관성: 직전 틱에 고른 후보(m_currentBehaviorPlan)와 차선 결정 또는 속도 결정이 달라졌으면 물어서,
    // 매 0.2초 후보 간 근소한 비용 차이로 결정이 왔다갔다(플립플롭)하는 걸 억제한다.
    float inertiaCost = (candidate.laneChoice != m_currentBehaviorPlan.laneChoice ? 1.0f : 0.0f) +
                        (candidate.speedAction != m_currentBehaviorPlan.speedAction ? 1.0f : 0.0f);

    float signalViolationCost = candidate.signalViolation ? 1.0f : 0.0f;

    return m_behaviorWeights.speed * speedCost + m_behaviorWeights.laneChange * laneChangeCost +
           m_behaviorWeights.inertia * inertiaCost + m_behaviorWeights.signalViolation * signalViolationCost;
}

// BEHAVIOR_PLAN_INTERVAL(0.2초)마다 (차선유지/좌변경/우변경) x (가속/유지/감속) 후보를 만들어 평가하고,
// 가장 비용이 낮은 유효 후보를 채택한다. 차선변경 후보를 고르면 그 자리에서 바로 레인/경로를 전환하고,
// 이후 BEHAVIOR_PLAN_INTERVAL 동안은 DriveControl이 그 결과(m_currentBehaviorPlan)를 그대로 따라간다.
void Car::UpdateBehaviorPlan()
{
    if (m_currentLane == nullptr)
        return;
    if (m_currentTime - m_lastBehaviorPlanTime < BEHAVIOR_PLAN_INTERVAL)
        return;
    m_lastBehaviorPlanTime = m_currentTime;

    // 도로제약(제한속도/커브/신호)은 후보마다 다시 스캔할 필요 없이 여기서 한 번만 스캔해서, desiredSpeed
    // 계산과 각 후보의 궤적 상 지점별 국소 상한 계산(BuildCandidate) 양쪽에 그대로 재사용한다.
    constexpr float MIN_LOOK_DISTANCE = 20.0f; // 정지 상태에서도 바로 앞 신호/제한속도는 보이게 하는 최소치
    float lookDistance = std::max(MIN_LOOK_DISTANCE, BEHAVIOR_LOOKAHEAD_TIME * m_speed);
    std::vector<RoadSpeedSample> roadSamples = ScanRoadSpeedConstraints(lookDistance);

    // 주변 차도 후보마다 다시 찾을 필요 없이 여기서 한 번만 모은다 (레인 무관, 위치 기반이라 모든
    // 후보가 같은 목록을 공유한다). 우선순위(직진>회전) 판정도 이때 함께 확정된다.
    std::vector<NearbyCar> nearbyCars = CollectNearbyCars();
    // 코리도 안의 차를 가상 리더 샘플로 추가 -- desiredSpeed와 각 후보의 overshoot 계산 양쪽에
    // 반영되어, 신호/커브처럼 제동거리 기반으로 앞차에 선제 감속이 걸린다.
    AppendCarConstraintSamples(roadSamples, nearbyCars, lookDistance);

    float desiredSpeed = ComputeSpeedCapFromSamples(roadSamples, 0.0f);

    constexpr LaneChoice laneChoices[] = {LaneChoice::Keep, LaneChoice::ChangeLeft, LaneChoice::ChangeRight};
    constexpr SpeedAction speedActions[] = {SpeedAction::Accelerate, SpeedAction::Maintain, SpeedAction::Decelerate};

    std::vector<BehaviorCandidate> candidates;
    for (LaneChoice laneChoice : laneChoices)
    {
        shared_ptr<Lane> lane;
        if (laneChoice == LaneChoice::Keep)
            lane = m_currentLane;
        else if (laneChoice == LaneChoice::ChangeLeft)
            lane = m_currentLane->GetLeft().lock();
        else
            lane = m_currentLane->GetRight().lock();
        if (lane == nullptr)
            continue;

        for (SpeedAction speedAction : speedActions)
            candidates.push_back(BuildCandidate(laneChoice, speedAction, lane, roadSamples, nearbyCars));
    }

    const BehaviorCandidate *best = nullptr;
    float bestCost = std::numeric_limits<float>::max();
    for (const BehaviorCandidate &candidate : candidates)
    {
        if (!IsCandidateSafe(candidate))
            continue;
        float cost = EvaluateCandidateCost(candidate, desiredSpeed);
        if (cost < bestCost)
        {
            bestCost = cost;
            best = &candidate;
        }
    }

    // 안전한 후보가 하나도 없으면(주변이 꽉 막힌 극단적 상황) 차선유지 + 감속으로 되돌아가되,
    // 일반 제동으론 못 피한다는 뜻이므로 다음 플랜까지 DriveControl이 비상 제동을 밟게 한다.
    m_emergencyBrake = (best == nullptr);
    if (m_emergencyBrake)
        DebugConsole::Log(GetName() + ": [plan] no safe candidate -> EMERGENCY BRAKE");
    BehaviorCandidate chosen = (best != nullptr)
                                   ? *best
                                   : BuildCandidate(LaneChoice::Keep, SpeedAction::Decelerate, m_currentLane,
                                                    roadSamples, nearbyCars);

    if (chosen.laneChoice != LaneChoice::Keep && chosen.targetLane != nullptr)
    {
        m_path = std::move(chosen.newPath);
        m_pathIndex = 0;
        SetCurrentLane(chosen.targetLane); // 내부에서 RebuildSplineRender()까지 처리한다.
        DebugConsole::Log(GetName() + ": behavior plan -> lane change to lane " +
                          std::to_string(chosen.targetLane->GetId()) + " (target " +
                          std::to_string(chosen.targetSpeed * 3.6f) + " km/h)");
    }

    // [진단용] 후보별 평가 결과 전체를 찍어서 커브 감속이 왜 안 걸리는지(혹은 잘 걸리는지) 확인한다.
    // 확인 끝나면 지워도 된다.
    auto laneChoiceName = [](LaneChoice c)
    {
        switch (c)
        {
        case LaneChoice::Keep:
            return "Keep";
        case LaneChoice::ChangeLeft:
            return "Left";
        default:
            return "Right";
        }
    };
    auto speedActionName = [](SpeedAction a)
    {
        switch (a)
        {
        case SpeedAction::Accelerate:
            return "Accel";
        case SpeedAction::Maintain:
            return "Maint";
        default:
            return "Decel";
        }
    };
    DebugConsole::Log(GetName() + ": [plan] speed=" + std::to_string(m_speed * 3.6f) +
                      "km/h desired=" + std::to_string(desiredSpeed * 3.6f) + "km/h");
    // [진단용] 목적지 근처를 빙글빙글 도는 문제 확인용: 지금 레인이 목적지 레인인지, path 진행이
    // 멈춰있는지(pathIndex 정체), 목적지까지 직선거리 vs 스플라인 투영거리를 같이 찍는다.
    if (m_destLane != nullptr)
    {
        Vec3 projectedPosition = m_destLane->GetSpline().GetLookaheadPoint(GetPosition(), 0.0f);
        float straightDist = (m_destLane->GetEndPoint() - GetPosition()).Length();
        float splineDist = (m_destLane->GetEndPoint() - projectedPosition).Length();
        DebugConsole::Log(GetName() + ": [plan-nav] lane=" + std::to_string(m_currentLane->GetId()) +
                          " destLane=" + std::to_string(m_destLane->GetId()) +
                          " onDestLane=" + (m_currentLane == m_destLane ? "Y" : "N") +
                          " pathIdx=" + std::to_string(m_pathIndex) + "/" + std::to_string(m_path.size()) +
                          " straightDist=" + std::to_string(straightDist) +
                          "m splineDist=" + std::to_string(splineDist) + "m");
    }
    for (const BehaviorCandidate &candidate : candidates)
    {
        bool safe = IsCandidateSafe(candidate);
        float cost = safe ? EvaluateCandidateCost(candidate, desiredSpeed) : -1.0f;
        DebugConsole::Log(GetName() + ":   " + laneChoiceName(candidate.laneChoice) + "/" +
                          speedActionName(candidate.speedAction) +
                          " target=" + std::to_string(candidate.targetSpeed * 3.6f) +
                          "km/h end=" + std::to_string(candidate.horizonEndSpeed * 3.6f) +
                          "km/h overshoot=" + std::to_string(candidate.maxSpeedOvershoot * 3.6f) +
                          "km/h safe=" + (safe ? "Y" : "N") + " cost=" + std::to_string(cost));
    }
    DebugConsole::Log(GetName() + ":   -> chosen " + laneChoiceName(chosen.laneChoice) + "/" +
                      speedActionName(chosen.speedAction));

    m_currentBehaviorPlan = std::move(chosen);
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

        ImGui::Separator();
        ImGui::Text("Behavior Plan Weights");
        ImGui::SliderFloat("Speed Weight w1", &m_behaviorWeights.speed, 0.0f, 10.0f);
        ImGui::SliderFloat("Lane Change Weight w2", &m_behaviorWeights.laneChange, 0.0f, 20.0f);
        ImGui::SliderFloat("Inertia Weight w4", &m_behaviorWeights.inertia, 0.0f, 10.0f);
        ImGui::SliderFloat("Signal Violation Weight w5", &m_behaviorWeights.signalViolation, 0.0f, 50.0f);
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
    if (m_currentLane == nullptr)
    {
        m_splineRender.SetModel(nullptr);
        return;
    }

    const std::vector<Vec3> &splinePoints = m_currentLane->GetSpline().GetSplinePoints();
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


void Car::SetDestination(const shared_ptr<RoadNode> &destNode)
{
    m_destLane = m_RoadDataManager->GetClosestLaneEnd(destNode->position);
    DebugConsole::Log(GetName() + ": SetDestination -> node " + std::to_string(destNode->id) +
                      " (lane " + std::to_string(m_destLane ? m_destLane->GetId() : -1) + ")");
    if (destNode->nodeType == RoadNodeType::Park)
    {
        m_pendingParkNode = destNode;
    }

    // m_currentLane을 이미 아는 상태(도착 직후 다음 목적지를 새로 받는 보통의 경우)라면
    // UpdateFindPath는 "m_currentLane != nullptr"라서 절대 다시 경로를 안 찾는다 -- 그러면 새
    // 목적지로 바뀐 뒤에도 이전 목적지로 가던 낡은 m_path를 끝까지 따라가다 경로가 바닥나서야
    // "no destination lane"으로 실패한다. 여기서 바로 새 목적지 기준으로 경로를 다시 찾는다.
    // m_currentLane이 아직 없으면(막 스폰/출차 등) UpdateFindPath가 다음 프레임에 알아서 처리한다.
    if (m_currentLane != nullptr)
        TryFindPathAndSetLane();
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