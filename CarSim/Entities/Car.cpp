#include "Car.h"
#include "Core/PhysicsSystem.h"
#include "Rendering/Effects.h"
#include <ModelManager.h>
#include <algorithm>
#include <cmath>
#include <imgui.h>
#include <Core/DebugConsole.h>
#include <Core/Assert.h>

namespace
{
    // 저크 완화용 S자(smoothstep) 보간. 가속/제동 램프와 속도 프로파일 계산에서 공유.
    float SmoothStep(float t)
    {
        t = std::clamp(t, 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }
} // namespace

void Car::Init(const CarSpec &spec, const RoadDataManager *roadDataManager, JPH::Vec3 position)
{
    SetName(spec.name);
    m_render.SetModel(ModelManager::Get().CreateFromFile(spec.modelPath));
    SetRenderOffset(ToXMFLOAT3(spec.renderOffset));
    m_wheelbase = spec.wheelbase;

    // `position` is the front axle; GameObject::Init() below seeds the rigidbody from
    // m_transform, which must hold the rear axle, so convert before creating the body.
    DirectX::XMFLOAT3 fwd = m_transform.GetForwardAxis();
    m_transform.SetPosition(position.GetX() - fwd.x * m_wheelbase,
                            position.GetY() - fwd.y * m_wheelbase,
                            position.GetZ() - fwd.z * m_wheelbase);

    GameObject::Init(spec.halfExtents, Rigidbody::Type::Dynamic, spec.colliderOffset, spec.mass);

    m_spawnPosition = m_transform.GetPosition();
    m_spawnRotation = m_transform.GetRotationQuat();
    m_mass = spec.mass;

    // Line sticking out the front of the car, showing the current steering direction
    Model *pLine = ModelManager::Get().CreateFromGeometry("__steer_line__:" + GetName(),
                                                          Geometry::CreateLine(DirectX::XMFLOAT3(0.0f, 0.0f, 0.0f), DirectX::XMFLOAT3(0.0f, 0.0f, 6.0f)));
    pLine->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(0.0f, 0.0f, 1.0f, 1.0f));
    pLine->materials[0].Set<float>("$Opacity", 1.0f);
    m_steerLine.SetModel(pLine);

    // Small red marker showing the BT's current lookahead target on the road
    constexpr float TARGET_MARKER_SIZE = 0.5f;
    Model *pTargetMarker = ModelManager::Get().CreateFromGeometry("__target_marker__:" + GetName(),
                                                                  Geometry::CreatePlane(TARGET_MARKER_SIZE, TARGET_MARKER_SIZE));
    pTargetMarker->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f));
    pTargetMarker->materials[0].Set<float>("$Opacity", 1.0f);
    m_targetMarker.SetModel(pTargetMarker);

    // Red sphere marking the BT's current destination node (m_destNode)
    constexpr float DEST_MARKER_RADIUS = 0.6f;
    Model *pDestMarker = ModelManager::Get().CreateFromGeometry("__dest_marker__:" + GetName(),
                                                                Geometry::CreateSphere(DEST_MARKER_RADIUS));
    pDestMarker->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f));
    pDestMarker->materials[0].Set<float>("$Opacity", 1.0f);
    m_destMarker.SetModel(pDestMarker);

    m_RoadDataManager = roadDataManager;
    m_BehaviourTree = BehaviourTree();
    m_BehaviourTree.root = BuildBehaviourTree();

    for (auto &item : m_speedProfile)
    {
        item.first = Vec3::sZero();
        item.second = 0.0f;
    }
}

void Car::Update(float dt)
{
    m_deltaTime = dt;
    m_currentTime += dt;
    m_BehaviourTree.Tick();
    UpdateCar();
    UpdateDebugWindow();
    ApplyMotion();
    UpdateTrail();
}

void Car::Draw(ID3D11DeviceContext *context, IEffect &effect)
{
    GameObject::Draw(context, effect);

    if (m_drawCollider && (m_rearTrailRender.GetModel() || m_frontTrailRender.GetModel() || m_splineRender.GetModel()))
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

    if (m_destEdge && m_destMarker.GetModel())
    {
        m_destMarker.GetTransform().SetPosition(ToXMFLOAT3(m_destEdge->position));
        m_destMarker.Draw(context, effect);
    }
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
    AccelMode targetMode = AccelMode::None;
    if (desiredVelocity > m_speed)
        targetMode = AccelMode::Accelerating;
    else if (desiredVelocity < m_speed)
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

void Car::Steer(float radian)
{
    constexpr float STEER_RAMP_RATE = 0.4f; // todo: vary 0.3 (calm) ~ 1.0 (urgent) by input intensity

    if (m_steerAngle > radian)
        m_steerAngle -= STEER_RAMP_RATE * m_deltaTime;
    else if (m_steerAngle < radian)
        m_steerAngle += STEER_RAMP_RATE * m_deltaTime;
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
    constexpr float OFF_COURSE_DISTANCE = 3.0f; // 스플라인과의 최대 허용 거리 (m)

    Vec3 position = m_rigidbody.GetPosition();
    Vec3 closestPoint = m_currentSpline.GetLookaheadPoint(position, 0.0f);
    float distance = (closestPoint - position).Length();
    if (distance > OFF_COURSE_DISTANCE)
        return true;

    float t = m_currentSpline.GetSplinePosition(position);
    if (t < 0.0f)
        return false;

    Vec3 splineDirection = m_currentSpline.GetDirectionAt(t);
    float dot = std::clamp(GetForwardAxis().Dot(splineDirection), -1.0f, 1.0f);
    float angleDiff = std::acos(dot);
    return angleDiff > m_maxSteerAngle;
}

void Car::MoveSpeedProfile()
{
    size_t prevIndex = m_profileIndex;
    m_profileIndex = (m_profileIndex + 1) % SPEED_PROFILE_COUNT;

    // prevIndex 슬롯은 링버퍼 회전으로 가장 먼 미래(마지막 오프셋) 지점이 됨.
    // CalculateSpeedProfile 전체 재계산 대신 그 지점 하나만 도로를 훑어서 새로 채우고,
    // 그 결과를 더 가까운 기존 슬롯들에도 역전파해서 필요하면 미리 감속하게 한다.
    constexpr float CURVE_SPEED_COEFF = 1.22f; // 최대 코너링 속도 = CURVE_SPEED_COEFF * sqrt(R)
    constexpr float LOCAL_WINDOW = 0.01f;      // 로컬 곡률 추정용 t-window

    float deltaTime = LOOK_PROFILE_TIME / SPEED_PROFILE_COUNT;
    float targetDistance = (SPEED_PROFILE_COUNT - 1) * deltaTime * m_speed;

    const Spline *spline = &m_currentSpline;
    shared_ptr<RoadEdge> segmentNode = m_currentEdge;
    size_t pathIndex = m_pathIndex;
    Vec3 segmentStart = m_rigidbody.GetPosition();
    float remainingDistance = targetDistance;
    float maxSpeed = m_maxSpeed;

    Vec3 tailPosition = segmentStart;
    float tailSpeed = maxSpeed;

    while (spline)
    {
        float startT = spline->GetSplinePosition(segmentStart);
        float splineLength = spline->GetLength();
        float segmentDistance = splineLength > 0.0f ? (1.0f - startT) * splineLength : 0.0f;

        if (splineLength > 0.0f && remainingDistance <= segmentDistance)
        {
            float t = std::clamp(startT + remainingDistance / splineLength, 0.0f, 1.0f);
            const std::vector<Vec3> &points = spline->GetSplinePoints();
            size_t index = static_cast<size_t>(t * (points.size() - 1));

            float radius = spline->GetMinRadiusAhead(std::max(0.0f, t - LOCAL_WINDOW), std::min(1.0f, t + LOCAL_WINDOW));
            float curveSpeed = radius < std::numeric_limits<float>::max() ? CURVE_SPEED_COEFF * std::sqrt(radius) : m_maxSpeed;

            tailPosition = points[index];
            tailSpeed = std::min({maxSpeed, curveSpeed, m_maxSpeed});
            break;
        }

        remainingDistance -= segmentDistance;
        shared_ptr<RoadEdge> nextNode = (pathIndex + 1 < m_path.size()) ? m_path[pathIndex + 1] : nullptr;
        if (!nextNode)
        {
            tailPosition = segmentNode->position;
            tailSpeed = (segmentNode == m_destEdge) ? 0.0f : maxSpeed;
            break;
        }

        maxSpeed = std::min(maxSpeed, nextNode->GetLimitSpeed());
        shared_ptr<RoadEdge> edge = segmentNode->GetEdgeTo(nextNode->id);
        segmentNode = nextNode;
        segmentStart = nextNode->position;
        ++pathIndex;

        if (!edge)
        {
            tailPosition = segmentNode->position;
            tailSpeed = maxSpeed;
            break;
        }
        if (edge->spline.GetControlPointCount() == 0)
        {
            // 곡선이 미리 만들어지지 않은 구간(교차로 회전 등): 곡률 없이 edge 길이만큼만 소모하고 다음으로 넘어간다.
            remainingDistance -= edge->length;
            if (remainingDistance <= 0.0f)
            {
                tailPosition = segmentNode->position;
                tailSpeed = maxSpeed;
                break;
            }
        }
        spline = &edge->spline;
    }

    m_speedProfile[prevIndex] = {tailPosition, tailSpeed};

    // 뒤(먼 미래)에서부터 감속 역전파: 더 가까운 기존 슬롯들도 필요하면 낮춘다.
    Vec3 nextPosition = tailPosition;
    float nextSpeed = tailSpeed;
    for (size_t offset = SPEED_PROFILE_COUNT - 1; offset-- > 0;)
    {
        size_t index = (m_profileIndex + offset) % SPEED_PROFILE_COUNT;
        float distance = (nextPosition - m_speedProfile[index].first).Length();
        float entrySpeed = std::sqrt(nextSpeed * nextSpeed + 2.0f * m_maxBrake * distance);
        m_speedProfile[index].second = std::min(m_speedProfile[index].second, entrySpeed);

        nextPosition = m_speedProfile[index].first;
        nextSpeed = m_speedProfile[index].second;
    }

    float prevSpeed = m_speedProfile[m_profileIndex].second;
    for (size_t offset = 1; offset < SPEED_PROFILE_COUNT; ++offset)
    {
        size_t index = (m_profileIndex + offset) % SPEED_PROFILE_COUNT;
        float maxReachable = prevSpeed + m_maxAccel * deltaTime;
        float minReachable = std::max(0.0f, prevSpeed - m_maxBrake * deltaTime);
        float speed = std::clamp(m_speedProfile[index].second, minReachable, maxReachable);

        m_speedProfile[index].second = speed;
        prevSpeed = speed;
    }
}
void Car::CalculateSpeedProfile()
{
    m_profileIndex = 0;
    Vec3 calPosition = m_rigidbody.GetPosition();
    float calSpeed = m_speed;
    m_speedProfile[m_profileIndex].first = calPosition;
    m_speedProfile[m_profileIndex].second = calSpeed;

    float deltaTime = LOOK_PROFILE_TIME / SPEED_PROFILE_COUNT;

    // 가속 램프 도중이면 이미 경과한 시간(rampStart)부터 이어서 S자(SmoothStep) 가속 곡선을 그대로 적분한다.
    // (지금 감속/정지 중이면 미래에 새로 램프가 시작된다고 보수적으로 가정)
    float rampStart = (m_accelMode == AccelMode::Accelerating) ? std::min(m_accelRampTime, ACCEL_RAMP_DURATION) : 0.0f;
    float rampStartProgress = rampStart / ACCEL_RAMP_DURATION;

    // SmoothStep(τ)=3τ²-2τ³ 의 부정적분(F=∫SmoothStep, G=∫F). 속도/거리를 정확히 적분하는 데 사용.
    auto GetAccelSpeedRatio = [](float nomalizedTime)
    { return pow(nomalizedTime, 3) - 0.5f * pow(nomalizedTime, 4); };
    auto GetAccelDistanceRatio = [](float nomalizedTime)
    { return 0.25f * pow(nomalizedTime, 4) - 0.1f * pow(nomalizedTime, 5); };

    float startAccelSpeedRatio = GetAccelSpeedRatio(rampStartProgress);
    float startAccelDistanceRatio = GetAccelDistanceRatio(rampStartProgress);

    auto accelDistanceAtTime = [&](float t)
    {
        // 램프 구간
        float rampTime = std::clamp(t, 0.0f, ACCEL_RAMP_DURATION - rampStart);
        float rampEndProgress = rampStartProgress + rampTime / ACCEL_RAMP_DURATION;

        float distance = (calSpeed - m_maxAccel * ACCEL_RAMP_DURATION * startAccelSpeedRatio) * rampTime +
                         m_maxAccel * ACCEL_RAMP_DURATION * ACCEL_RAMP_DURATION * (GetAccelDistanceRatio(rampEndProgress) - startAccelDistanceRatio);
        float rampEndSpeed = calSpeed + m_maxAccel * ACCEL_RAMP_DURATION * (GetAccelSpeedRatio(rampEndProgress) - startAccelSpeedRatio);

        // 풀악셀로 달리기
        float remainTime = t - rampTime;
        float fullAccelSpeedTime = std::max(0.0f, (m_maxSpeed - rampEndSpeed) / m_maxAccel);
        ;
        float fullAccelTime = std::min(remainTime, fullAccelSpeedTime);
        float maxSpeed = rampEndSpeed + m_maxAccel * fullAccelTime;
        distance += (rampEndSpeed + maxSpeed) / 2.0f * fullAccelTime;

        // m_maxSpeed로 달리기
        float maxSpeedTime = std::max(0.0f, remainTime - fullAccelSpeedTime);
        distance += m_maxSpeed * maxSpeedTime;

        return distance;
    };

    float lookDistance = accelDistanceAtTime(LOOK_PROFILE_TIME);

    constexpr float ROAD_SAMPLE_SPACING = 5.0f; // 도로 스캔 샘플 간격 (m)
    constexpr float LOCAL_WINDOW = 0.01f;       // 로컬 곡률 추정용 t-window

    struct RoadSample
    {
        float distance;
        float maxSpeed; // 그 지점 자체의 순간 최대 속도 (커브 or 노드 제한속도)
        Vec3 position;
    };

    // m_currentSpline -> (필요시) path상의 다음 노드들의 lane spline 순으로 lookDistance까지 훑으며
    // 커브 지점(로컬 곡률 기반 최대속도)과 노드 지점(제한속도)의 샘플을 모은다.
    std::vector<RoadSample> samples;
    samples.reserve(static_cast<size_t>(lookDistance / ROAD_SAMPLE_SPACING) + 4);
    {
        float currentNodeT = m_currentSpline.GetSplinePosition(calPosition);
        Assert(currentNodeT >= 0.0f); // CalculateSpeedProfile 호출 전엔 항상 m_currentSpline이 세팅되어 있어야 함
        float currentNodeDistance = m_currentSpline.GetLength() * (1.0f - currentNodeT);
        float currentNodeSpeed = (m_currentEdge == m_destEdge) ? 0.0f : std::min(m_currentEdge->GetLimitSpeed(), m_maxSpeed);
        samples.push_back({currentNodeDistance, currentNodeSpeed, m_currentEdge->position});
    }
    {
        const Spline *spline = &m_currentSpline;
        shared_ptr<RoadEdge> segmentNode = m_currentEdge;
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
                    float maxSpeed = radius < std::numeric_limits<float>::max() ? CURVE_SPEED_COEFF * std::sqrt(radius) : m_maxSpeed;
                    samples.push_back({traveledDistance + localDistance, maxSpeed, points[index]});
                }
            }

            traveledDistance += walkDistance;
            remainingDistance -= walkDistance;
            if (remainingDistance <= 0.0f)
                break;

            shared_ptr<RoadEdge> nextNode = (pathIndex + 1 < m_path.size()) ? m_path[pathIndex + 1] : nullptr;
            if (!nextNode)
                break;

            float nextNodeSpeed = (nextNode == m_destEdge) ? 0.0f : std::min(nextNode->GetLimitSpeed(), m_maxSpeed);
            samples.push_back({traveledDistance, nextNodeSpeed, nextNode->position});

            shared_ptr<RoadEdge> edge = segmentNode->GetEdgeTo(nextNode->id);
            segmentNode = nextNode;
            segmentStart = nextNode->position;
            ++pathIndex;

            if (!edge)
                break;
            if (edge->spline.GetControlPointCount() == 0)
            {
                // 곡선이 없는 구간(교차로 회전 등): 곡률 샘플 없이 edge 길이만큼만 소모한다.
                traveledDistance += edge->length;
                remainingDistance -= edge->length;
            }
            spline = &edge->spline;
        }
    }
    std::sort(samples.begin(), samples.end(), [](const RoadSample &a, const RoadSample &b)
              { return a.distance < b.distance; });

    // 감속 램프구간 때문에 최대감속 가속도의 0.3만 사용(매우 안전운전), 나중에 도로 빌드시 고정 룩업 테이블로 변경
    constexpr float BRAKE_RAMP_AVG_FACTOR = 0.3f;

    // 뒤에서부터 감속 계획 전파
    std::vector<float> speedCap(samples.size());
    if (!samples.empty())
    {
        speedCap.back() = samples.back().maxSpeed;
        for (size_t i = samples.size() - 1; i-- > 0;)
        {
            float distance = samples[i + 1].distance - samples[i].distance;
            float entrySpeed = std::sqrt(speedCap[i + 1] * speedCap[i + 1] + 2.0f * m_maxBrake * BRAKE_RAMP_AVG_FACTOR * distance);
            speedCap[i] = std::min(samples[i].maxSpeed, entrySpeed);
        }
    }

    // distance 지점의 감속계획 속도/위치를 samples에서 보간해서 조회
    auto sampleAt = [&](float distance) -> std::pair<Vec3, float>
    {
        if (samples.empty())
            return {calPosition, m_maxSpeed};
        if (distance <= samples.front().distance)
            return {samples.front().position, speedCap.front()};
        if (distance >= samples.back().distance)
            return {samples.back().position, speedCap.back()};

        size_t hi = 1;
        while (hi < samples.size() && samples[hi].distance < distance)
            ++hi;
        size_t lo = hi - 1;
        float span = samples[hi].distance - samples[lo].distance;
        float ratio = span > 1e-4f ? (distance - samples[lo].distance) / span : 0.0f;
        Vec3 position = samples[lo].position + (samples[hi].position - samples[lo].position) * ratio;
        float speed = speedCap[lo] + (speedCap[hi] - speedCap[lo]) * ratio;
        return {position, speed};
    };

    // 이전 슬롯(deltaTime 전) 대비 가속/감속 한계를 S자 램프로 반영해서 스무딩한다.
    // 방향이 바뀌면 램프를 처음부터 다시 시작하고, 램프 구간 안에서는
    // (구간 시작 시점의 낮은 가속도) x deltaTime 만큼만 허용해 과대평가를 피한다.
    AccelMode phase = AccelMode::None;
    float rampElapsed = 0.0f;
    float prevSpeed = calSpeed;
    for (size_t i = 1; i < SPEED_PROFILE_COUNT; ++i)
    {
        size_t index = (i + m_profileIndex) % SPEED_PROFILE_COUNT;
        float distance = accelDistanceAtTime(i * deltaTime);
        auto [position, speed] = sampleAt(distance);

        speed = std::min(speed, m_maxSpeed);

        AccelMode direction = speed > prevSpeed   ? AccelMode::Accelerating
                              : speed < prevSpeed ? AccelMode::Braking
                                                  : AccelMode::None;
        if (direction != phase)
        {
            phase = direction;
            rampElapsed = 0.0f;
        }

        float maxReachable = prevSpeed + SmoothStep(rampElapsed / ACCEL_RAMP_DURATION) * m_maxAccel * deltaTime;
        float minReachable = std::max(0.0f, prevSpeed - SmoothStep(rampElapsed / BRAKE_RAMP_DURATION) * m_maxBrake * deltaTime);
        rampElapsed += deltaTime;

        speed = std::clamp(speed, minReachable, maxReachable);

        m_speedProfile[index].first = position;
        m_speedProfile[index].second = speed;
        prevSpeed = speed;
    }
    m_profileIndex = (m_profileIndex + 1) % SPEED_PROFILE_COUNT;
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
    }
    ImGui::End();

    m_BehaviourTree.Draw(("BT: " + GetName()).c_str());
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
    std::vector<DirectX::XMFLOAT3> points(trail.begin(), trail.end());
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

    std::vector<DirectX::XMFLOAT3> points;
    points.reserve(splinePoints.size());
    for (const Vec3 &point : splinePoints)
        points.push_back(ToXMFLOAT3(point));

    Model *pModel = ModelManager::Get().CreateFromGeometry("__current_spline__:" + GetName(), Geometry::CreatePolyline(points));
    pModel->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f));
    pModel->materials[0].Set<float>("$Opacity", 1.0f);
    m_splineRender.SetModel(pModel);
}
