#include "Car.h"
#include "VehicleSegment.h"
#include "Utill/DebugConsole.h"
#include "Nav/VehicleCollision.h"
#include "Nav/SimulationState.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace
{
    constexpr float VERTICAL_SEPARATION = 3.0f;

    float NearestBandOffset(const RoadRef &road, float d)
    {
        const LaneBand *band = RoadDataManager::Get().FindNearestBand(road.road, d);
        return band != nullptr ? band->centerOffset : d;
    }

    // OBB를 axis에 투영한 반폭
    float ObstacleHalfExtentAlong(const VehicleCollision::Obstacle &obstacle, const Vec3 &axis)
    {
        Vec3 forward(cosf(obstacle.headingRad), 0.0f, sinf(obstacle.headingRad));
        Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());
        return std::fabs(axis.Dot(forward)) * obstacle.halfLength +
               std::fabs(axis.Dot(right)) * obstacle.halfWidth;
    }

    bool ProjectObstacle(const Spline &referenceLine, const VehicleCollision::Obstacle &obstacle,
                         float &outOffset, float &outHalfExtent)
    {
        float t = referenceLine.GetSplinePosition(obstacle.center);
        Vec3 onRef = referenceLine.GetPositionAt(t);
        Vec3 dir = referenceLine.GetDirectionAt(t);
        Vec3 rightN(dir.GetZ(), 0.0f, -dir.GetX());
        outOffset = (obstacle.center - onRef).Dot(rightN);

        outHalfExtent = ObstacleHalfExtentAlong(obstacle, rightN);

        const std::vector<Vec3> &samples = referenceLine.GetSplinePoints();
        float sampleSpacing = samples.size() > 1 ? referenceLine.GetLength() / (samples.size() - 1.0f) : 0.0f;
        constexpr float PROJECTION_RESIDUAL_SLACK = 2.0f;
        constexpr float PROJECTION_RESIDUAL_MIN = 0.5f;
        float tolerance = sampleSpacing * PROJECTION_RESIDUAL_SLACK + PROJECTION_RESIDUAL_MIN;
        float alongResidual = (obstacle.center - onRef).Dot(dir);
        return std::fabs(alongResidual) <= tolerance;
    }

    float ComputeReferenceOffset(const Spline &referenceLine, const Vec3 &position)
    {
        float t = referenceLine.GetSplinePosition(position);
        Vec3 onRef = referenceLine.GetPositionAt(t);
        Vec3 dir = referenceLine.GetDirectionAt(t);
        Vec3 rightN(dir.GetZ(), 0.0f, -dir.GetX());
        return (position - onRef).Dot(rightN);
    }

    float DirectionToAngleRad(const Vec3 &direction)
    {
        return atan2f(direction.GetZ(), direction.GetX());
    }

    // [-pi, pi]
    float WrapAngleRad(float radian)
    {
        constexpr float TWO_PI = 6.28318530718f;
        radian = fmodf(radian + 3.14159265359f, TWO_PI);
        if (radian < 0.0f)
            radian += TWO_PI;
        return radian - 3.14159265359f;
    }

    float PurePursuitSteerAt(const Vec3 &rearAxle, float headingRad, const Vec3 &target, float wheelbase)
    {
        Vec3 forward(cosf(headingRad), 0.0f, sinf(headingRad));
        Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());
        float dx = target.GetX() - rearAxle.GetX();
        float dz = target.GetZ() - rearAxle.GetZ();
        float distance = std::sqrt(dx * dx + dz * dz);
        if (distance < 0.001f)
            return 0.0f;

        float cosAngle = std::clamp((forward.GetX() * dx + forward.GetZ() * dz) / distance, -1.0f, 1.0f);
        float headingError = acosf(cosAngle);
        float directionSign = (right.GetX() * dx + right.GetZ() * dz > 0.0f) ? 1.0f : -1.0f;
        return atanf((2.0f * wheelbase * sinf(headingError)) / distance) * directionSign;
    }

    // 오프셋 규약 동일
    Vec3 OffsetSampleAt(const std::vector<Vec3> &samples, size_t i, float d)
    {
        const size_t n = samples.size();
        const Vec3 &next = samples[i + 1 < n ? i + 1 : i];
        const Vec3 &prev = samples[i > 0 ? i - 1 : i];
        float tx = next.GetX() - prev.GetX();
        float tz = next.GetZ() - prev.GetZ();
        float len = std::sqrt(tx * tx + tz * tz);
        float rx = len > 1e-5f ? tz / len : 0.0f;
        float rz = len > 1e-5f ? -tx / len : 0.0f;
        return Vec3(samples[i].GetX() + rx * d, samples[i].GetY(), samples[i].GetZ() + rz * d);
    }

    // 스플라인 생성 없이 계산
    Vec3 OffsetLookaheadPoint(const Spline &referenceLine, const Vec3 &from, float lookahead, float d, bool reversed)
    {
        const std::vector<Vec3> &samples = referenceLine.GetSplinePoints();
        const size_t n = samples.size();
        if (n == 0)
            return from;
        if (n == 1)
            return OffsetSampleAt(samples, 0, d);

        size_t closest = 0;
        float bestDistance = std::numeric_limits<float>::max();
        Vec3 current;
        for (size_t i = 0; i < n; ++i)
        {
            Vec3 p = OffsetSampleAt(samples, i, d);
            float distance = (p - from).Length();
            if (distance < bestDistance)
            {
                bestDistance = distance;
                closest = i;
                current = p;
            }
        }

        float accumulated = 0.0f;
        size_t index = closest;
        while (accumulated < lookahead)
        {
            if (reversed ? index == 0 : index + 1 >= n)
                break;
            size_t nextIndex = reversed ? index - 1 : index + 1;
            Vec3 nextPoint = OffsetSampleAt(samples, nextIndex, d);
            accumulated += (nextPoint - current).Length();
            current = nextPoint;
            index = nextIndex;
        }
        return current;
    }

    Vec3 LookaheadOnPolyline(const std::vector<Vec3> &points, const Vec3 &from, float lookahead,
                             bool reversed, size_t &cursor)
    {
        const size_t n = points.size();
        if (n == 0)
            return from;

        constexpr size_t SEARCH_WINDOW = 8;
        size_t begin = 0;
        size_t end = n;
        if (cursor < n)
        {
            begin = cursor > SEARCH_WINDOW ? cursor - SEARCH_WINDOW : 0;
            end = std::min(n, cursor + SEARCH_WINDOW + 1);
        }

        float bestDistance = std::numeric_limits<float>::max();
        size_t closest = begin;
        for (size_t i = begin; i < end; ++i)
        {
            float distance = (points[i] - from).LengthSq(); // 최소값 위치만 쓰므로 sqrt 불필요
            if (distance < bestDistance)
            {
                bestDistance = distance;
                closest = i;
            }
        }
        cursor = closest;

        float accumulated = 0.0f;
        size_t index = closest;
        while (accumulated < lookahead)
        {
            if (reversed ? index == 0 : index + 1 >= n)
                break;
            size_t nextIndex = reversed ? index - 1 : index + 1;
            accumulated += (points[nextIndex] - points[index]).Length();
            index = nextIndex;
        }
        return points[index];
    }
}

#pragma region Common
void Car::UpdateMode()
{
    const char *reason = "";
    EnsureRoamingPath();
    Mode next = DecideNextMode(&reason);
    if (next != m_mode)
    {
        DebugConsole::Log(GetName() + ": " + StateToString(m_mode) + " -> " + StateToString(next) + " (" + reason + ")");
        OnModeExit(next);
        Mode prev = m_mode;
        m_mode = next;
        OnModeEnter(prev);
    }
}

Car::Mode Car::DecideNextMode(const char **reason) const
{
    if (m_mode == Mode::Stop)
    {
        if (m_currentRoad == nullptr)
            return Mode::Stop;
        *reason = "road found";
        return Mode::Drive;
    }

    return Mode::Drive;
}

void Car::BeginSegment(std::unique_ptr<VehicleSegment> segment)
{
    std::vector<std::unique_ptr<VehicleSegment>> segments;
    segments.push_back(std::move(segment));
    m_vehicleController.BeginPlan(std::move(segments));
}

void Car::OnModeEnter(Mode prev)
{
    if (m_mode == Mode::Drive)
    {
        SetSubMode(SubMode::D_Pursuit);
        m_planAccelDebug = 0.0f;
        BeginSegment(std::make_unique<SplineFollowSegment>());
    }
    else if (m_mode == Mode::Stop)
    {
        SetSubMode(SubMode::None);
    }
}

void Car::OnModeExit(Mode next)
{
    if (!m_vehicleController.IsFinished())
        m_vehicleController.Abort();
}

void Car::SetSubMode(SubMode next)
{
    if (next == m_subMode)
        return;
    DebugConsole::Log(GetName() + ": " + SubStateToString(m_subMode) + " -> " + SubStateToString(next));
    m_subMode = next;
}

VehicleCollision::VehicleShape Car::BuildVehicleShape() const
{
    VehicleCollision::VehicleShape shape;
    shape.pivotToCenter = m_colliderOffset.z;
    shape.halfWidth = m_halfExtents.GetX();
    shape.halfLength = m_halfExtents.GetZ();
    return shape;
}

RoadRef Car::PickRandomSuccessor(const RoadRef &road) const
{
    if (road.road == nullptr)
        return RoadRef{};

    const std::vector<RoadRef> &successors = RoadDataManager::Get().GetRoadSuccessors(road.road->GetId());
    if (successors.empty())
        return RoadRef{};
    return successors[rand() % successors.size()];
}

vector<RoadRef> Car::BuildRoamingPath(const RoadRef &startRoad) const
{
    vector<RoadRef> path;
    if (startRoad.road == nullptr)
        return path;

    path.push_back(startRoad);
    RoadRef current = startRoad;
    for (size_t i = 0; i < ROAMING_MIN_AHEAD; ++i)
    {
        RoadRef next = PickRandomSuccessor(current);
        if (next.road == nullptr)
            break;
        path.push_back(next);
        current = next;
    }
    return path;
}

void Car::EnsureRoamingPath()
{
    if (m_currentRoad == nullptr)
    {
        RoadPose pose = RoadDataManager::Get().GetClosestRoad(GetPosition());
        RoadRef entry{pose.road};
        float targetOffset = NearestBandOffset(entry, pose.d);
        SetCurrentRoad(entry.road, targetOffset);
        m_path = BuildRoamingPath(CurrentRoadRef());
        m_pathIndex = 0;
        return;
    }
    MaintainRoamingPath();
}

void Car::MaintainRoamingPath()
{
    constexpr size_t KEEP_BEHIND = 1;

    while (m_pathIndex > KEEP_BEHIND)
    {
        m_path.erase(m_path.begin());
        --m_pathIndex;
    }

    while (!m_path.empty() && m_path.size() - m_pathIndex <= ROAMING_MIN_AHEAD)
    {
        RoadRef next = PickRandomSuccessor(m_path.back());
        if (next.road == nullptr)
            break;
        m_path.push_back(next);
    }
}
#pragma endregion

#pragma region Stop
void Car::UpdateStop()
{
    Steer(0.0f);
    AccelerateVel(0.0f);
}
#pragma endregion

#pragma region Drive
void Car::UpdateDrive()
{
    if (!CheckPath())
        return;

    if (m_currentTime - m_lastBehaviorPlanTime >= BEHAVIOR_PLAN_INTERVAL)
    {
        UpdateSensors();
        UpdateDrivePlan();
        m_lastBehaviorPlanTime = m_currentTime;
    }

    m_wantSegmentTick = true;
}

bool Car::CanEnterFromCurrentBand(const RoadRef &next) const
{
    std::vector<const LaneBand *> entry = RoadDataManager::Get().GetEntryBands(CurrentRoadRef(), next);
    if (entry.empty())
        return true;

    for (const LaneBand *band : entry)
        if (band == m_currentBand)
            return true;
    return false;
}

bool Car::RepathFromCurrentBand()
{
    std::vector<RoadRef> candidates;
    for (const RoadRef &succ : RoadDataManager::Get().GetRoadSuccessors(m_currentRoad->GetId()))
        if (CanEnterFromCurrentBand(succ))
            candidates.push_back(succ);
    if (candidates.empty())
        return false;

    m_path.resize(m_pathIndex + 1);
    m_path.push_back(candidates[rand() % candidates.size()]);
    MaintainRoamingPath();
    return true;
}

bool Car::CheckPath()
{
    if (m_currentRoad == nullptr)
        return false;

    constexpr float LANE_TRANSITION_THRESHOLD = 2.0f;

    Vec3 position = GetPosition();
    auto roadEnd = [&]() -> Vec3
    {
        const std::vector<Vec3> &pts = m_currentSpline.GetSplinePoints();
        return pts.empty() ? position : pts.back();
    };

    Vec3 projectedPosition = m_currentSpline.GetLookaheadPoint(position, 0.0f);
    float roadEndDistance = (roadEnd() - projectedPosition).Length();
    while (roadEndDistance < LANE_TRANSITION_THRESHOLD)
    {
        if (m_pathIndex + 1 >= m_path.size())
        {
            MaintainRoamingPath();
            if (m_pathIndex + 1 >= m_path.size())
                break;
        }

        if (!CanEnterFromCurrentBand(m_path[m_pathIndex + 1]) && !RepathFromCurrentBand())
            break;

        ++m_pathIndex;
        const RoadRef &next = m_path[m_pathIndex];
        bool hasLaneMapping = false;
        float nextOffset = RoadDataManager::Get().ResolveConnectingOffset(CurrentRoadRef(), next, m_currentOffset, &hasLaneMapping);

        if (!hasLaneMapping)
            nextOffset = ComputeReferenceOffset(next.road->GetReferenceLine(), position);

        SetCurrentRoad(next.road, nextOffset);
        projectedPosition = m_currentSpline.GetLookaheadPoint(position, 0.0f);
        roadEndDistance = (roadEnd() - projectedPosition).Length();
    }
    return true;
}

void Car::DriveControl()
{
    const Spline &spline = m_currentSpline;

    float lookaheadDistance = 5.0f;
    Vec3 target = spline.GetLookaheadPoint(GetRigidbodyPosition(), lookaheadDistance);
    float targetSteer = ComputeContextSteer(target, PurePursuit(target));
    RebuildCtxSteerRender();

    Steer(targetSteer);

    bool emergBlocked = IsEmergencyRayBlocked();
    RebuildEmergRayRender();
    if (emergBlocked)
        EmergBrake();
    else
        AccelerateVel(RoadTargetSpeed(m_currentRoad));

    DirectX::XMFLOAT3 targetMarkerPos = ToXMFLOAT3(target);
    targetMarkerPos.y = GetPosition().GetY() + 0.2f;
    m_targetMarker.GetTransform().SetPosition(targetMarkerPos);
}

#pragma endregion

#pragma region BehaviorPlan

void Car::ComputeDrivableRange(const RoadRef &road, float &outMin, float &outMax) const
{

    float halfW = GetHalfWidth() * 0.5f;

    outMin = -(RoadDataManager::ROAD_WIDTH - halfW);
    outMax = RoadDataManager::ROAD_WIDTH - halfW;

    std::vector<const LaneBand *> bands = RoadDataManager::Get().GetDrivingBands(road.road);
    if (bands.empty())
        return;

    outMin = std::numeric_limits<float>::max();
    outMax = -std::numeric_limits<float>::max();
    for (const LaneBand *b : bands)
    {
        outMin = std::min(outMin, b->centerOffset - b->width * 0.5f);
        outMax = std::max(outMax, b->centerOffset + b->width * 0.5f);
    }
    outMin += halfW;
    outMax -= halfW;
    if (outMin > outMax)
        outMin = outMax = (outMin + outMax) * 0.5f;
}

float Car::RoadTargetSpeed(const shared_ptr<Road> &road) const
{
    if (road == nullptr)
        return m_maxSpeed;
    return std::min(road->GetSpeedLimit(), m_maxSpeed);
}

void Car::UpdateDrivePlan()
{
    if (m_currentRoad == nullptr)
        return;

    // 조향이 실제 위치를 만든다. 계획 오프셋은 따라가기만 하되 도로 밖으론 안 나가게 클램프
    float minOffset = 0.0f;
    float maxOffset = 0.0f;
    ComputeDrivableRange(CurrentRoadRef(), minOffset, maxOffset);
    float realOffset = ComputeReferenceOffset(m_currentRoad->GetReferenceLine(), GetPosition());
    SetCurrentOffset(std::clamp(realOffset, minOffset, maxOffset));
    m_currentSpline = RoadDataManager::Get().BuildOffsetSpline(m_currentRoad, m_currentOffset);
    RebuildSplineRender();
}

#pragma endregion

#pragma region Avoid

VehicleCollision::Obstacle Car::MakeVehicleObstacle() const
{
    VehicleCollision::Obstacle obstacle;
    obstacle.center = GetBodyCenter();
    obstacle.halfLength = m_halfExtents.GetZ();
    obstacle.halfWidth = m_halfExtents.GetX();
    obstacle.headingRad = DirectionToAngleRad(GetForwardAxis());
    obstacle.speed = GetSpeed();
    obstacle.type = VehicleCollision::ObstacleType::Dynamic;
    obstacle.isVehicle = true;
    obstacle.sourceCar = const_cast<Car *>(this);
    return obstacle;
}

Vec3 Car::GetBodyCenter() const
{

    Vec3 forward = GetForwardAxis();
    Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());
    return GetRigidbodyPosition() + forward * m_colliderOffset.z + right * m_colliderOffset.x;
}

// 헤딩 기준 부채꼴을 슬롯으로 쪼개, 가고 싶은 방향(interest)에서 위험한 슬롯(danger)을 빼고 고른다.
// 매 프레임 돌며 연속적인 조향을 만든다
float Car::ComputeContextSteer(const Vec3 &pursuitTarget, float pursuitSteer) const
{
    constexpr int SLOT_COUNT = 11;
    constexpr float FAN_HALF = ToRadians(60.0f);
    constexpr float TTC_HORIZON = 3.0f;     // 이 시간 밖 위협은 무시
    constexpr float PROXIMITY_RANGE = 6.0f; // 접근속도 0이어도 위험한 거리
    constexpr float DANGER_CUTOFF = 0.35f;  // 이 위 슬롯은 봉쇄로 본다
    constexpr float RIGHT_BIAS = 0.06f;     // 마주보기 교착 깨기
    constexpr float EDGE_FALLOFF = ToRadians(30.0f);
    constexpr float STEER_LOOKAHEAD = 6.0f;
    constexpr float MIN_CLOSING = 0.1f;

    Vec3 position = GetRigidbodyPosition();
    Vec3 forward = GetForwardAxis();
    float headingRad = DirectionToAngleRad(forward);
    Vec3 myCenter = GetBodyCenter();
    Vec3 myVelocity = forward * m_speed;
    float slotStep = (2.0f * FAN_HALF) / (SLOT_COUNT - 1);

    float danger[SLOT_COUNT] = {};
    float totalDanger = 0.0f;

    auto accumulate = [&](const VehicleCollision::Obstacle &threat)
    {
        Vec3 rel = threat.center - myCenter;
        float distance = rel.Length();
        if (distance < 0.01f)
            return;
        Vec3 relDir = rel * (1.0f / distance);

        Vec3 threatDir(cosf(threat.headingRad), 0.0f, sinf(threat.headingRad));
        float closing = (myVelocity - threatDir * threat.speed).Dot(relDir);

        Vec3 relRight(relDir.GetZ(), 0.0f, -relDir.GetX());
        float halfExtent = ObstacleHalfExtentAlong(threat, relRight) + GetHalfWidth();
        float gap = std::max(0.0f, distance - ObstacleHalfExtentAlong(threat, relDir) - GetHalfWidth());

        // 접근속도 기준(정면충돌은 이게 커서 일찍 반응) + 근접 기준 중 큰 쪽
        float weight = 0.0f;
        if (closing > MIN_CLOSING)
            weight = std::max(0.0f, 1.0f - (gap / closing) / TTC_HORIZON);
        weight = std::max(weight, 1.0f - std::min(1.0f, gap / PROXIMITY_RANGE));
        if (weight <= 0.0f)
            return;

        float threatRad = WrapAngleRad(DirectionToAngleRad(rel) - headingRad);
        float angularHalf = atan2f(halfExtent, std::max(1.0f, distance));

        for (int i = 0; i < SLOT_COUNT; ++i)
        {
            float delta = std::fabs(WrapAngleRad(-FAN_HALF + i * slotStep - threatRad));
            float falloff = delta <= angularHalf
                                ? 1.0f
                                : 1.0f - std::min(1.0f, (delta - angularHalf) / EDGE_FALLOFF);
            if (falloff <= 0.0f)
                continue;
            danger[i] += weight * falloff;
            totalDanger += weight * falloff;
        }
    };

    for (const VehicleCollision::Obstacle &stored : m_obstacles)
    {
        if (stored.sourceCar == nullptr)
        {
            accumulate(stored);
            continue;
        }
        if (!m_SimState->IsCarAlive(stored.sourceCar))
            continue;
        accumulate(stored.sourceCar->MakeVehicleObstacle()); // 0.2s 캐시 대신 현재 위치
    }

    m_ctxSteerOpenLines.clear();
    m_ctxSteerBlockedLines.clear();
    for (int i = 0; i < SLOT_COUNT; ++i)
    {
        float slotRad = headingRad + (-FAN_HALF + i * slotStep);
        Vec3 end = position + Vec3(cosf(slotRad), 0.0f, sinf(slotRad)) * STEER_LOOKAHEAD;
        std::vector<Vec3> &bucket = danger[i] >= DANGER_CUTOFF ? m_ctxSteerBlockedLines : m_ctxSteerOpenLines;
        bucket.push_back(position);
        bucket.push_back(end);
    }

    if (totalDanger <= 0.0f)
    {
        m_ctxSteerDebugRad = 0.0f;
        m_ctxSteerDebugDanger = 0.0f;
        return pursuitSteer; // 위협 없으면 슬롯 양자화로 흔들 필요 없다
    }

    float desiredRad = WrapAngleRad(DirectionToAngleRad(pursuitTarget - position) - headingRad);

    int best = -1;
    float bestInterest = -1.0f;
    for (int i = 0; i < SLOT_COUNT; ++i)
    {
        if (danger[i] >= DANGER_CUTOFF)
            continue;
        float slotRad = -FAN_HALF + i * slotStep;
        float interest = std::max(0.0f, cosf(slotRad - desiredRad)) + (slotRad < 0.0f ? RIGHT_BIAS : 0.0f);
        if (interest > bestInterest)
        {
            bestInterest = interest;
            best = i;
        }
    }

    if (best < 0) // 전부 봉쇄면 가장 덜 위험한 쪽으로라도
    {
        float least = std::numeric_limits<float>::max();
        for (int i = 0; i < SLOT_COUNT; ++i)
            if (danger[i] < least)
            {
                least = danger[i];
                best = i;
            }
    }

    m_ctxSteerDebugRad = -FAN_HALF + best * slotStep;
    m_ctxSteerDebugDanger = danger[best];

    float chosenRad = headingRad + m_ctxSteerDebugRad;
    Vec3 aim = position + Vec3(cosf(chosenRad), 0.0f, sinf(chosenRad)) * STEER_LOOKAHEAD;
    return std::clamp(PurePursuitSteerAt(position, headingRad, aim, m_wheelbase),
                      -m_maxSteerAngle, m_maxSteerAngle);
}

bool Car::IsEmergencyRayBlocked() const
{
    constexpr float MAX_RAY_TURN = 1.5708f; // 직선레이가 무의미해지는 각

    float rayDistance = m_speed * m_speed / (2.0f * m_maxBrake) + SAFE_GAP;

    Vec3 forward = GetForwardAxis();
    Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());
    Vec3 frontCenter = GetBodyCenter() + forward * m_halfExtents.GetZ();
    float halfWidth = m_halfExtents.GetX();

    const Vec3 origins[3] = {
        frontCenter - right * halfWidth,
        frontCenter,
        frontCenter + right * halfWidth,
    };

    float curvature = tanf(m_steerAngle) / m_wheelbase; // = 각속도 / 속도
    float dTheta = std::clamp(rayDistance * curvature, -MAX_RAY_TURN, MAX_RAY_TURN);
    bool turning = std::fabs(dTheta) > 1e-4f;
    Vec3 icr = turning ? GetRigidbodyPosition() + right * (1.0f / curvature) : Vec3::sZero();
    float cosT = cosf(dTheta);
    float sinT = sinf(dTheta);

    m_emergRayDebugLines.clear();
    m_emergRayDebugBlocked = false;
    bool blocked = false;

    auto castRay = [&](const Vec3 &origin, const Vec3 &direction, float length)
    {
        if (length <= 0.001f)
            return;

        const VehicleCollision::Obstacle *hit = VehicleCollision::RaycastObstaclesHit(
            origin, DirectionToAngleRad(direction), length, m_obstacles, nullptr);

        m_emergRayDebugLines.push_back(origin);
        m_emergRayDebugLines.push_back(origin + direction * length);

        if (hit == nullptr)
            return;
        blocked = true;
    };

    for (int i = 0; i < 3; ++i)
    {
        const Vec3 &origin = origins[i];
        Vec3 direction = forward;
        if (turning)
        {
            Vec3 rel = origin - icr;
            // +Y 각속도가 헤딩을 줄이는 방향(=시계) 회전
            Vec3 turned(rel.GetX() * cosT + rel.GetZ() * sinT, 0.0f,
                        rel.GetZ() * cosT - rel.GetX() * sinT);
            Vec3 chord = (icr + turned) - origin;
            if (chord.LengthSq() > 1e-6f)
                direction = chord.Normalized();
        }

        castRay(origin, direction, rayDistance);

        // 조향쪽 꼭짓점만 옆으로
        bool isTurnCorner = (i == 0 && curvature < 0.0f) || (i == 2 && curvature > 0.0f);
        if (isTurnCorner)
        {
            Vec3 lateralDir = (i == 0) ? -right : right;
            float lateralLen = std::fabs(direction.Dot(right)) * rayDistance;
            castRay(origin, lateralDir, lateralLen);
        }
    }

    m_emergRayDebugBlocked = blocked;
    if (blocked)
        return true;
    return false;
}

void Car::UpdateSensors()
{
    Vec3 myPosition = GetPosition();
    static constexpr float DETECT_RANGE = 100.0f;

    m_obstacles.clear();
    m_nearbyCars.clear();

    for (const VehicleCollision::Obstacle &obstacle : RoadDataManager::Get().GetObstacles())
    {
        if (std::fabs(obstacle.center.GetY() - myPosition.GetY()) > VERTICAL_SEPARATION)
            continue;
        if ((obstacle.center - myPosition).Length() > DETECT_RANGE)
            continue;
        m_obstacles.push_back(obstacle);
    }
    for (Car *other : m_SimState->GetCars())
    {
        if (other == this)
            continue;
        if (std::fabs(other->GetPosition().GetY() - myPosition.GetY()) > VERTICAL_SEPARATION)
            continue;
        if ((other->GetPosition() - myPosition).Length() > DETECT_RANGE)
            continue;

        m_nearbyCars.push_back(other);
    }

    m_obstacles.reserve(m_obstacles.size() + m_nearbyCars.size());
    for (Car *nearbyCar : m_nearbyCars)
    {
        if (!m_SimState->IsCarAlive(nearbyCar))
            continue;
        m_obstacles.push_back(nearbyCar->MakeVehicleObstacle());
    }
}

#pragma endregion
