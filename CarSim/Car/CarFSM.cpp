#include "Car.h"
#include "VehicleSegment.h"
#include "Utill/DebugConsole.h"
#include "Nav/ReedsShepp.h"
#include "Nav/VehicleCollision.h"
#include "Nav/SimulationState.h"
#include "Utill/Assert.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace
{
    constexpr float SAME_DIRECTION_COS = 0.7071f;

    constexpr float VERTICAL_SEPARATION = 3.0f;

    constexpr float SIREN_DETECT_RADIUS = 40.0f; // 사이렌 감지반경

    constexpr float CHASE_DETECT_RADIUS = 80.0f;  // 도망차 탐색반경
    constexpr float CHASE_REPATH_INTERVAL = 1.0f; // 추격경로 갱신주기
    constexpr float CHASE_BLOCK_RANGE = 20.0f;    // 이 안에서 앞서면 차단

    float NearestBandOffset(const RoadRef &road, float d)
    {
        const LaneBand *band = RoadDataManager::Get().FindNearestBand(road.road, d, road.direction);
        return band != nullptr ? band->centerOffset : d;
    }

    float TravelS(const Spline &referenceLine, const Vec3 &position, float dirSign)
    {
        return referenceLine.GetSplinePosition(position) * referenceLine.GetLength() * dirSign;
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

    float RoadDriveOffset(const RoadRef &road, const Vec3 &position)
    {
        if (RoadDataManager::Get().GetDrivingBands(road.road, road.direction).empty())
            return 0.0f;
        return NearestBandOffset(road, ComputeReferenceOffset(road.road->GetReferenceLine(), position));
    }

    LaneDirection ResolveOneWayDirection(const shared_ptr<Road> &road, LaneDirection fallback)
    {
        bool hasForward = !RoadDataManager::Get().GetDrivingBands(road, LaneDirection::Forward).empty();
        bool hasBackward = !RoadDataManager::Get().GetDrivingBands(road, LaneDirection::Backward).empty();
        if (hasForward == hasBackward)
            return fallback;
        return hasForward ? LaneDirection::Forward : LaneDirection::Backward;
    }

    LaneDirection NearSideDirection(const shared_ptr<Road> &road, float d)
    {
        LaneDirection best = LaneDirection::Forward;
        float bestGap = std::numeric_limits<float>::max();
        for (LaneDirection direction : {LaneDirection::Forward, LaneDirection::Backward})
        {
            const LaneBand *band = RoadDataManager::Get().FindNearestBand(road, d, direction);
            if (band == nullptr)
                continue;
            float gap = std::fabs(d - band->centerOffset);
            if (gap < bestGap)
            {
                bestGap = gap;
                best = direction;
            }
        }
        return best;
    }

    std::vector<std::unique_ptr<VehicleSegment>> BuildReedSheppSegments(const ReedsShepp::Path &path, const Vec3 &startPos,
                                                                        float startAngleRad, float turningRadius)
    {
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        std::vector<ReedsShepp::Leg> legs = ReedsShepp::GetLegs(path, startPos, startAngleRad, turningRadius);
        segments.reserve(legs.size());
        for (size_t i = 0; i < legs.size(); ++i)
        {
            ReedsShepp::Leg &leg = legs[i];
            bool isFinalLeg = (i + 1 == legs.size());
            segments.push_back(
                std::make_unique<RSFollowSegment>(std::move(leg.points), leg.gear, leg.endIndex, isFinalLeg));
        }
        return segments;
    }

    std::vector<std::unique_ptr<VehicleSegment>> BuildExactSegments(const ReedsShepp::Path &path, float maxSteerAngle)
    {
        std::vector<std::unique_ptr<VehicleSegment>> segments;
        segments.reserve(path.size());
        for (const ReedsShepp::PathElement &element : path)
        {
            float steerAngle = 0.0f;
            if (element.steering == ReedsShepp::Steering::Left)
                steerAngle = -maxSteerAngle;
            else if (element.steering == ReedsShepp::Steering::Right)
                steerAngle = maxSteerAngle;
            segments.push_back(std::make_unique<RSExactSegment>(element, steerAngle));
        }
        return segments;
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

    bool IsSameObstacle(const VehicleCollision::Obstacle &a, const VehicleCollision::Obstacle &b)
    {
        if (a.sourceCar != nullptr || b.sourceCar != nullptr)
            return a.sourceCar == b.sourceCar;
        return (a.center - b.center).LengthSq() < 0.01f; // 정적은 위치 고정
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
    UpdateFindPath();
    TryBeginParkCruise();
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

void Car::UpdateFindPath()
{
    if (m_roaming)
    {
        EnsureRoamingPath();
        return;
    }

    if (m_destRoad == nullptr || m_currentRoad != nullptr)
        return;

    Vec3 position = GetPosition();
    RoadPose pose = RoadDataManager::Get().GetClosestRoad(position, GetForwardAxis());
    RoadRef entry{pose.road, pose.direction};
    float targetOffset = NearestBandOffset(entry, pose.d);
    if (pose.road != nullptr && !IsSafeLaneEntry(entry, targetOffset))
        return;
    SetCurrentRoad(entry.road, targetOffset, entry.direction);
    TryFindPathAndSetRoad();
}

Car::Mode Car::DecideNextMode(const char **reason) const
{

    if (m_mode == Mode::Stop)
    {
        if (m_roaming)
        {
            if (m_currentRoad == nullptr)
                return Mode::Stop;
            *reason = "roaming";
            return Mode::Drive;
        }
        if (m_destRoad == nullptr || m_currentRoad == nullptr)
        {
            return Mode::Stop;
        }
        *reason = "go to Dest";

        return Mode::Park;
    }
    else if (m_mode == Mode::Park)
    {
        if (m_parkPlanPending || m_parkSequenceActive || !m_vehicleController.IsFinished())
        {
            *reason = "parking in progress";
            return Mode::Park;
        }
        if (m_subMode == SubMode::P_EXIT)
        {

            *reason = "normal driving";
            return Mode::Drive;
        }

        *reason = "done parking";
        return Mode::Stop;
    }
    else if (m_mode == Mode::Drive)
    {
        if (m_roaming)
            return Mode::Drive;

        if (m_lotEntryHold)
            return Mode::Drive;

        bool arrived = IsArrivedAtDest();
        if (arrived && GetParkTargetNode() != nullptr)
        {
            *reason = "arrived at destination";
            return Mode::Park;
        }
        if (m_destRoad == nullptr || arrived)
        {
            *reason = m_destRoad == nullptr ? "no destination road" : "arrived at destination";
            return Mode::Stop;
        }
        return Mode::Drive;
    }

    *reason = "unreachable";
    return m_mode;
}

void Car::BeginSegment(std::unique_ptr<VehicleSegment> segment)
{
    std::vector<std::unique_ptr<VehicleSegment>> segments;
    segments.push_back(std::move(segment));
    m_vehicleController.BeginPlan(std::move(segments));
}

void Car::OnModeEnter(Mode prev)
{
    m_currentLeader = nullptr;
    if (m_mode != Mode::Drive)
        m_lotCruise = false;

    if (m_mode == Mode::Drive)
    {
        SetSubMode(SubMode::D_Normal);
        m_planAccelDebug = 0.0f;

        m_staticBlockTimer = 0.0f;
        m_stuck = false;
        m_maneuver = ManeuverState{};
        m_speedCap = -1.0f;
        m_sirenPulledOver = false;
        BeginSegment(std::make_unique<SplineFollowSegment>());
    }
    else if (m_mode == Mode::Park)
    {
        SetSubMode(prev == Mode::Stop ? SubMode::P_EXIT : SubMode::P_ENTER_LEG1);
        m_parkPlanPending = true;
        m_parkSequenceActive = true;
    }
    else if (m_mode == Mode::Stop)
    {

        if (prev == Mode::Park && m_subMode == SubMode::P_ENTER_ALIGN)
            BeginSegment(std::make_unique<CenterSteerSegment>());
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

bool Car::TryFindPathAndSetRoad()
{
    m_path = RoadDataManager::Get().FindPath(CurrentRoadRef(), m_destRoad);
    m_pathIndex = 0;
    if (m_path.empty())
    {
        m_destRoad = nullptr;
        SetCurrentRoad(nullptr, 0.0f, LaneDirection::Forward);
        return false;
    }

    m_destDir = m_path.back().direction;
    return true;
}

RoadRef Car::PickRandomSuccessor(const RoadRef &road) const
{
    if (road.road == nullptr)
        return RoadRef{};

    const std::vector<RoadRef> &successors = RoadDataManager::Get().GetRoadSuccessors(road.road->GetId(), road.direction);
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
        RoadPose pose = RoadDataManager::Get().GetClosestRoad(GetPosition(), GetForwardAxis());
        RoadRef entry{pose.road, pose.direction};
        float targetOffset = NearestBandOffset(entry, pose.d);
        if (pose.road != nullptr && !IsSafeLaneEntry(entry, targetOffset))
            return;
        SetCurrentRoad(entry.road, targetOffset, entry.direction);
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

#pragma region Park

bool Car::IsArrivedAtDest() const
{
    if (m_destRoad == nullptr)
        return false;

    if (m_lotCruise)
        return (m_lotStopPos - GetPosition()).Length() < ARRIVE_DISTANCE;

    Vec3 destEnd = RoadDataManager::Get().GetTravelEnd(m_destRoad, m_destDir);
    Vec3 projectedPosition = m_destRoad->GetReferenceLine().GetLookaheadPoint(GetPosition(), 0.0f);
    bool arrived = (destEnd - projectedPosition).Length() < ARRIVE_DISTANCE;

    if (m_pendingParkNode != nullptr)
        arrived |= (m_pendingParkNode->position - GetPosition()).Length() < PARK_ARRIVE_DISTANCE;
    return arrived;
}

void Car::TryBeginParkCruise()
{
    m_lotEntryHold = false;
    if (m_mode != Mode::Drive || m_roaming || m_lotCruise || m_currentRoad == nullptr)
        return;
    if (m_pendingParkNode == nullptr || m_pendingParkNode->parkingRoadIds.empty())
        return;
    if (!IsArrivedAtDest())
        return;

    if (m_speed > STOPPED_SPEED)
    {
        m_lotEntryHold = true;
        return;
    }

    BeginParkCruise();
}

bool Car::BeginParkCruise()
{
    RoadDataManager &roadData = RoadDataManager::Get();
    shared_ptr<RoadNode> lotNode = m_pendingParkNode;
    const vector<int> &lotRoads = lotNode->parkingRoadIds;

    shared_ptr<RoadNode> spot = m_SimState->TryReserveParkSpot(lotNode->id);
    if (spot == nullptr)
        return false;

    RoadPose target = roadData.GetClosestParkingRoad(spot->position, spot->direction, lotRoads);
    RoadPose entry = roadData.GetClosestParkingRoad(GetPosition(), GetForwardAxis(), lotRoads);
    vector<RoadRef> path;
    if (target.road != nullptr && entry.road != nullptr)
    {

        if (entry.road == target.road)
        {
            const Spline &ref = entry.road->GetReferenceLine();
            float toSpot = ref.GetSplinePosition(spot->position) - ref.GetSplinePosition(GetPosition());
            entry.direction = toSpot < 0.0f ? LaneDirection::Backward : LaneDirection::Forward;
        }
        entry.direction = ResolveOneWayDirection(entry.road, entry.direction);

        for (LaneDirection dir : {entry.direction, GetOppositeDirection(entry.direction)})
        {
            path = roadData.FindPath(RoadRef{entry.road, dir}, target.road);
            if (!path.empty())
                break;
        }
    }
    if (path.empty())
    {
        m_SimState->ReleaseParkSpot(spot->id);
        return false;
    }

    m_parkSpot = spot;
    m_parkNodeId = lotNode->id;
    m_triedParkSpotIds.clear();
    m_pendingParkNode = nullptr;

    const Spline &targetLine = target.road->GetReferenceLine();
    m_lotStopPos = targetLine.GetLookaheadPoint(spot->position, PARK_PRE_LEAD_DISTANCE * GetTravelSign(path.back().direction));
    m_lotCruise = true;

    m_path = std::move(path);
    m_pathIndex = 0;
    m_destRoad = target.road;
    m_destDir = m_path.back().direction;
    SetCurrentRoad(m_path[0].road, RoadDriveOffset(m_path[0], GetPosition()), m_path[0].direction);

    return true;
}

bool Car::BeginParkExitCruise(int lotNodeId)
{
    if (m_currentRoad == nullptr || !m_currentRoad->IsParkingRoad() || m_destRoad == nullptr)
        return false;

    RoadDataManager &roadData = RoadDataManager::Get();
    shared_ptr<RoadNode> lotNode = roadData.GetNode(lotNodeId);
    if (lotNode == nullptr || lotNode->parkingRoadIds.empty())
        return false;

    RoadPose exitAisle = roadData.GetClosestParkingRoad(lotNode->position, GetForwardAxis(), lotNode->parkingRoadIds);
    RoadPose exitRoad = roadData.GetClosestRoad(lotNode->position);
    if (exitAisle.road == nullptr || exitRoad.road == nullptr)
        return false;

    RoadRef start = CurrentRoadRef();

    if (start.road == exitAisle.road)
    {
        const Spline &ref = start.road->GetReferenceLine();
        float toExit = ref.GetSplinePosition(lotNode->position) - ref.GetSplinePosition(GetPosition());
        start.direction = toExit < 0.0f ? LaneDirection::Backward : LaneDirection::Forward;
    }
    start.direction = ResolveOneWayDirection(start.road, start.direction);
    vector<RoadRef> lotPath = roadData.FindPath(start, exitAisle.road);
    if (lotPath.empty())
        return false;

    float exitRoadOffset = ComputeReferenceOffset(exitRoad.road->GetReferenceLine(), lotNode->position);
    LaneDirection nearSide = NearSideDirection(exitRoad.road, exitRoadOffset);
    vector<RoadRef> mainPath;
    for (LaneDirection direction : {nearSide, GetOppositeDirection(nearSide)})
    {
        mainPath = roadData.FindPath(RoadRef{exitRoad.road, direction}, m_destRoad);
        if (!mainPath.empty())
            break;
    }
    if (mainPath.empty())
        return false;

    m_path = std::move(lotPath);
    m_path.insert(m_path.end(), mainPath.begin(), mainPath.end());
    m_pathIndex = 0;
    m_destDir = m_path.back().direction;
    SetCurrentRoad(m_path[0].road, RoadDriveOffset(m_path[0], GetPosition()), m_path[0].direction);

    return true;
}

void Car::UpdatePark()
{
    if (m_parkPlanPending)
    {

        if (m_speed > STOPPED_SPEED)
        {
            AccelerateVel(0.0f);
            return;
        }
        m_parkPlanPending = false;
        BeginParkPlan();
    }

    if (!m_vehicleController.IsFinished())
    {
        m_wantSegmentTick = true;
        return;
    }

    if (m_subMode == SubMode::P_EXIT)
    {

        m_parkSequenceActive = false;
        if (m_parkSpot != nullptr)
        {
            m_SimState->ReleaseParkSpot(m_parkSpot->id);
            m_parkSpot = nullptr;
        }
        int lotNodeId = m_parkNodeId;

        m_parkNodeId = -1;

        if (BeginParkExitCruise(lotNodeId))
            return;

        shared_ptr<Road> savedDestRoad = m_destRoad;
        LaneDirection savedDestDir = m_destDir;
        shared_ptr<Road> exitRoad = m_currentRoad;
        float exitOffset = m_currentOffset;
        LaneDirection exitDir = m_travelDir;
        if (!TryFindPathAndSetRoad())
        {
            m_destRoad = savedDestRoad;
            m_destDir = savedDestDir;
            SetCurrentRoad(exitRoad, exitOffset, exitDir);
            m_path.clear();
            m_pathIndex = 0;
        }
        return;
    }

    if (m_parkSpot != nullptr)
    {

        if (m_subMode == SubMode::P_ENTER_LEG1)
        {

            if (m_speed > STOPPED_SPEED)
            {
                AccelerateVel(0.0f);
                return;
            }

            constexpr int PARK_MAX_LEG_TRIES = 4;
            constexpr float P_ARRIVE_DISTANCE = 2.0f;
            Vec3 pPos;
            float pAngleRad;
            if (ComputeParkPrePose(pPos, pAngleRad) &&
                (m_rigidbody.GetPosition() - pPos).Length() > P_ARRIVE_DISTANCE &&
                m_parkLegTries < PARK_MAX_LEG_TRIES && PlanEnterForCurrentSpot())
                return;

            SetSubMode(SubMode::P_ENTER_LEG2);
            BeginParkSpotLeg();
            return;
        }

        if (m_subMode == SubMode::P_ENTER_LEG2)
        {
            if (m_speed > STOPPED_SPEED)
            {
                AccelerateVel(0.0f);
                return;
            }
            SetSubMode(SubMode::P_ENTER_ALIGN);
            Vec3 spotTarget = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
            float spotAngleRad = DirectionToAngleRad(m_parkSpot->direction);
            if (PlanParkLegTo(spotTarget, spotAngleRad, true))
                return;
        }
    }

    if (m_subMode == SubMode::None || m_subMode == SubMode::P_ENTER_ALIGN)
    {
        m_parkSequenceActive = false;
        m_destRoad = nullptr;
        SetCurrentRoad(nullptr, 0.0f, LaneDirection::Forward);
        return;
    }
}

void Car::BeginParkPlan()
{
    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);
    Vec3 rigidPosition = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());

    if (m_subMode == SubMode::P_EXIT)
    {

        Vec3 frontPos = GetPosition();
        RoadPose exitPose;
        if (m_parkSpot != nullptr && m_parkSpot->id >= 0)
        {
            shared_ptr<RoadNode> lotNode = RoadDataManager::Get().GetNode(m_parkNodeId);
            exitPose = RoadDataManager::Get().GetClosestParkingRoad(
                frontPos, GetForwardAxis(), lotNode != nullptr ? lotNode->parkingRoadIds : vector<int>{});
        }
        if (exitPose.road == nullptr)
            exitPose = RoadDataManager::Get().GetClosestRoad(frontPos, GetForwardAxis());
        if (exitPose.road == nullptr)
        {
            m_destRoad = nullptr;
            SetSubMode(SubMode::None);
            return;
        }

        RoadRef exitRef{exitPose.road, ResolveOneWayDirection(exitPose.road, exitPose.direction)};
        SetCurrentRoad(exitRef.road, RoadDriveOffset(exitRef, frontPos), exitRef.direction);

        const Spline &exitSpline = m_currentSpline;
        Vec3 closestDir = exitSpline.GetDirectionAt(exitSpline.GetSplinePosition(frontPos));

        constexpr float EXIT_HEADING_ALIGN_ANGLE = ToRadians(60.0f);
        float headingDot = std::clamp(GetForwardAxis().Dot(closestDir), -1.0f, 1.0f);
        if (std::acos(headingDot) <= EXIT_HEADING_ALIGN_ANGLE)
        {
            m_vehicleController.BeginPlan({});
            return;
        }

        VehicleCollision::VehicleShape shape = BuildVehicleShape();
        const std::vector<VehicleCollision::Obstacle> &obstacles = RoadDataManager::Get().GetObstacles();
        auto isCollisionFree = [&](const ReedsShepp::Path &candidate)
        {
            for (const ReedsShepp::PoseSample &pose : ReedsShepp::GetPoses(candidate, rigidPosition, startAngleRad, turningRadius))
            {
                if (VehicleCollision::IsColliding(pose.position, pose.headingRad, obstacles, shape))
                    return false;
            }
            return true;
        };

        constexpr float EXIT_LEAD_DISTANCES[] = {6.0f, 12.0f, 18.0f};
        for (float leadDistance : EXIT_LEAD_DISTANCES)
        {
            Vec3 targetPos = exitSpline.GetLookaheadPoint(frontPos, leadDistance);
            float targetAngleRad = DirectionToAngleRad(exitSpline.GetDirectionAt(exitSpline.GetSplinePosition(targetPos)));
            ReedsShepp::Path path = ReedsShepp::GetOptimalPath(rigidPosition, startAngleRad, targetPos, targetAngleRad, turningRadius, isCollisionFree);
            if (path.empty())
                continue;
            m_vehicleController.BeginPlan(BuildReedSheppSegments(path, rigidPosition, startAngleRad, turningRadius));
            RebuildRSDebugRender(path, rigidPosition, startAngleRad, turningRadius, targetPos, targetAngleRad);
            return;
        }

        m_vehicleController.BeginPlan({});
        return;
    }

    int parkNodeId = m_parkNodeId;
    if (m_parkSpot == nullptr && m_pendingParkNode != nullptr)
    {
        parkNodeId = m_pendingParkNode->id;
        shared_ptr<RoadNode> pendingNode = m_pendingParkNode;
        m_parkSpot = m_SimState->TryReserveParkSpot(parkNodeId);
        m_pendingParkNode = nullptr;
        if (m_parkSpot == nullptr)
        {
            bool hasAnyParkSpot = std::any_of(pendingNode->children.begin(), pendingNode->children.end(),
                                              [](const weak_ptr<RoadNode> &weakChild)
                                              {
                                                  shared_ptr<RoadNode> child = weakChild.lock();
                                                  return child != nullptr && child->nodeType == RoadNodeType::ParkSpot;
                                              });
            if (hasAnyParkSpot)
            {
                m_parkSequenceActive = false;
                m_destRoad = nullptr;
                return;
            }

            m_parkSpot = pendingNode;
        }
        m_parkNodeId = parkNodeId;
        m_triedParkSpotIds.clear();
    }

    if (m_parkSpot == nullptr)
    {
        m_parkSequenceActive = false;
        return;
    }

    if (m_subMode == SubMode::P_ENTER_LEG1)
    {
        if (!BeginParkEnterOrRetry())
        {
            m_destRoad = nullptr;
            m_parkSequenceActive = false;
        }
    }
}

const Spline *Car::FindBestParkingSpline(LaneDirection *outDirection) const
{
    if (m_parkSpot == nullptr)
        return nullptr;

    if (m_parkSpot->nodeType != RoadNodeType::ParkSpot)
        return nullptr;

    shared_ptr<RoadNode> lotNode = RoadDataManager::Get().GetNode(m_parkNodeId);
    RoadPose pose = RoadDataManager::Get().GetClosestParkingRoad(
        m_parkSpot->position, GetForwardAxis(), lotNode != nullptr ? lotNode->parkingRoadIds : vector<int>{});
    if (pose.road == nullptr)
        return nullptr;

    if (outDirection != nullptr)
    {

        *outDirection = ResolveOneWayDirection(pose.road, (pose.road == m_currentRoad) ? m_travelDir : pose.direction);
    }
    return &pose.road->GetReferenceLine();
}

bool Car::ComputeParkPrePose(Vec3 &outPos, float &outAngleRad) const
{
    LaneDirection aisleDirection = LaneDirection::Forward;
    const Spline *bestSpline = FindBestParkingSpline(&aisleDirection);
    if (bestSpline == nullptr)
        return false;

    float dirSign = GetTravelSign(aisleDirection);
    outPos = bestSpline->GetLookaheadPoint(m_parkSpot->position, PARK_PRE_LEAD_DISTANCE * dirSign);
    float pParam = bestSpline->GetSplinePosition(outPos);
    outAngleRad = DirectionToAngleRad(bestSpline->GetDirectionAt(pParam) * dirSign);
    return true;
}

bool Car::PlanEnterForCurrentSpot()
{
    LaneDirection aisleDirection = LaneDirection::Forward;
    Vec3 pPos;
    float pAngleRad;
    if (ComputeParkPrePose(pPos, pAngleRad))
    {
        ++m_parkLegTries;
        if (PlanParkLegTo(pPos, pAngleRad))
        {
            SetSubMode(SubMode::P_ENTER_LEG1);
            return true;
        }

        constexpr float ENTRY_MIN_DISTANCE = 2.0f;
        const Spline *bestSpline = FindBestParkingSpline(&aisleDirection);
        if (bestSpline == nullptr)
            return false;
        float dirSign = GetTravelSign(aisleDirection);
        Vec3 rigidPosition = m_rigidbody.GetPosition();
        float entryT = bestSpline->GetSplinePosition(rigidPosition);
        Vec3 entryPos = bestSpline->GetPositionAt(entryT);
        if ((entryPos - rigidPosition).Length() > ENTRY_MIN_DISTANCE)
        {
            float entryAngleRad = DirectionToAngleRad(bestSpline->GetDirectionAt(entryT) * dirSign);
            if (PlanParkLegTo(entryPos, entryAngleRad))
            {
                SetSubMode(SubMode::P_ENTER_LEG1);
                return true;
            }
        }
        return false;
    }

    constexpr float MAX_DIRECT_LEG_DISTANCE = 15.0f;
    Vec3 spotTarget = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
    float spotAngleRad = DirectionToAngleRad(m_parkSpot->direction);
    if ((spotTarget - m_rigidbody.GetPosition()).Length() > MAX_DIRECT_LEG_DISTANCE)
    {
        return false;
    }
    if (PlanParkLegTo(spotTarget, spotAngleRad))
    {
        SetSubMode(SubMode::P_ENTER_LEG2);
        return true;
    }
    return false;
}

bool Car::ReserveNextParkSpot()
{
    if (m_parkSpot != nullptr)
    {
        m_triedParkSpotIds.insert(m_parkSpot->id);
        m_SimState->ReleaseParkSpot(m_parkSpot->id);
    }
    m_parkSpot = m_SimState->TryReserveParkSpot(m_parkNodeId, m_triedParkSpotIds);
    return m_parkSpot != nullptr;
}

bool Car::BeginParkEnterOrRetry()
{
    while (m_parkSpot != nullptr)
    {
        m_parkLegTries = 0;
        if (PlanEnterForCurrentSpot())
            return true;
        if (!ReserveNextParkSpot())
            return false;
    }
    return false;
}

bool Car::PlanParkLegTo(const Vec3 &targetPos, float targetAngleRad, bool exact)
{
    Vec3 rigidPosition = m_rigidbody.GetPosition();
    float startAngleRad = DirectionToAngleRad(GetForwardAxis());
    float turningRadius = m_wheelbase / tanf(m_maxSteerAngle);

    VehicleCollision::VehicleShape shape = BuildVehicleShape();
    const std::vector<VehicleCollision::Obstacle> &obstacles = RoadDataManager::Get().GetObstacles();
    auto isCollisionFree = [&](const ReedsShepp::Path &candidate)
    {
        for (const ReedsShepp::PoseSample &pose : ReedsShepp::GetPoses(candidate, rigidPosition, startAngleRad, turningRadius))
        {
            if (VehicleCollision::IsColliding(pose.position, pose.headingRad, obstacles, shape))
                return false;
        }
        return true;
    };

    ReedsShepp::Path path = ReedsShepp::GetOptimalPath(rigidPosition, startAngleRad, targetPos, targetAngleRad, turningRadius, isCollisionFree);
    if (path.empty())
        return false;

    if (exact)
        m_vehicleController.BeginPlan(BuildExactSegments(path, m_maxSteerAngle));
    else
        m_vehicleController.BeginPlan(BuildReedSheppSegments(path, rigidPosition, startAngleRad, turningRadius));
    RebuildRSDebugRender(path, rigidPosition, startAngleRad, turningRadius, targetPos, targetAngleRad);
    return true;
}

void Car::BeginParkSpotLeg()
{
    Vec3 spotTarget = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
    float spotAngleRad = DirectionToAngleRad(m_parkSpot->direction);
    if (PlanParkLegTo(spotTarget, spotAngleRad))
        return;

    if (ReserveNextParkSpot() && BeginParkEnterOrRetry())
        return;

    m_destRoad = nullptr;
    m_vehicleController.BeginPlan({});
}

#pragma endregion

#pragma region Stop
void Car::UpdateStop()
{
    if (m_destRoad != nullptr)
    {
        Vec3 destEnd = RoadDataManager::Get().GetTravelEnd(m_destRoad, m_destDir);
        Vec3 projectedPosition = m_destRoad->GetReferenceLine().GetLookaheadPoint(GetPosition(), 0.0f);
        if ((destEnd - projectedPosition).Length() < ARRIVE_DISTANCE)
            m_destRoad = nullptr;
    }

    if (!m_vehicleController.IsFinished())
    {
        m_wantSegmentTick = true;
        return;
    }

    if (m_speed > STOPPED_SPEED)
    {
        Steer(0.0f);
        AccelerateVel(0.0f);
        return;
    }

    std::shared_ptr<RoadNode> dest = RoadDataManager::Get().GetRandomParkNode();
    if (!dest)
        return;

    if ((dest->position - GetPosition()).Length() < ARRIVE_DISTANCE)
        return;

    SetDestination(dest);
}
#pragma endregion

#pragma region Drive
void Car::UpdateDrive()
{
    if (!CheckPath())
        return;

    if (m_lotEntryHold)
    {
        AccelerateVel(0.0f);
        return;
    }

    m_speedCap = -1.0f; // 매 프레임 해제, 핸들러가 다시 주장
    if (!HandleContactPending() && !UpdateSirenWait() && !UpdateChase())
    {
        if (IgnoresTrafficRules()) // 횡방향은 반응형 조향이 맡는다
        {
            if (m_subMode != SubMode::D_Normal)
            {
                m_maneuver = ManeuverState{};
                m_staticBlockTimer = 0.0f;
                m_stuck = false;
                SetSubMode(SubMode::D_Normal);
            }
            TryStartStaticReverseEscape();
        }
        else
        {
            switch (m_subMode)
            {
            case SubMode::D_Avoid:
                UpdateAvoid();
                break;
            case SubMode::D_LaneChange:
                UpdateLaneChange();
                break;
            default:
                DecideAvoidance();
                break;
            }
        }
    }

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
    if (IgnoresTrafficRules()) // 역주행/도로밖 밴드는 진입차로 판정 불가
        return true;

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
    for (const RoadRef &succ : RoadDataManager::Get().GetRoadSuccessors(m_currentRoad->GetId(), m_travelDir))
        if (CanEnterFromCurrentBand(succ))
            candidates.push_back(succ);
    if (candidates.empty())
        return false;

    if (m_roaming)
    {

        m_path.resize(m_pathIndex + 1);
        m_path.push_back(candidates[rand() % candidates.size()]);
        MaintainRoamingPath();
        return true;
    }

    std::vector<RoadRef> best;
    for (const RoadRef &candidate : candidates)
    {
        std::vector<RoadRef> tail = RoadDataManager::Get().FindPath(candidate, m_destRoad);
        if (!tail.empty() && (best.empty() || tail.size() < best.size()))
            best = std::move(tail);
    }
    if (best.empty())
        return false;

    m_path.resize(m_pathIndex + 1);
    m_path.insert(m_path.end(), best.begin(), best.end());
    m_destDir = m_path.back().direction;
    return true;
}

bool Car::CheckPath()
{
    if (m_currentRoad == nullptr)
    {
        m_destRoad = nullptr;
        return false;
    }

    constexpr float LANE_TRANSITION_THRESHOLD = 2.0f;
    constexpr float LOOSE_TRANSITION_TIME = 0.3f; // 추격/도망 여유시간

    // 역주행/도로밖은 투영이 어긋나 좁은 창을 넘겨버린다
    float transitionThreshold = LANE_TRANSITION_THRESHOLD;
    if (IgnoresTrafficRules())
        transitionThreshold += m_speed * LOOSE_TRANSITION_TIME;

    Vec3 position = GetPosition();
    auto roadEnd = [&]() -> Vec3
    {
        const std::vector<Vec3> &pts = m_currentSpline.GetSplinePoints();
        return pts.empty() ? position : pts.back();
    };

    Vec3 projectedPosition = m_currentSpline.GetLookaheadPoint(position, 0.0f);
    float roadEndDistance = (roadEnd() - projectedPosition).Length();
    while (roadEndDistance < transitionThreshold)
    {

        int nextRoadId = (m_pathIndex + 1 < m_path.size()) ? m_path[m_pathIndex + 1].road->GetId() : -1;
        if (ShouldStopForSignal(m_currentRoad, m_travelDir, nextRoadId))
            break;

        if (m_pathIndex + 1 >= m_path.size())
        {
            if (m_roaming)
            {
                MaintainRoamingPath();
                if (m_pathIndex + 1 >= m_path.size())
                    break;
            }
            else
            {
                m_destRoad = nullptr;
                SetCurrentRoad(nullptr, 0.0f, LaneDirection::Forward);
                return false;
            }
        }

        if (!CanEnterFromCurrentBand(m_path[m_pathIndex + 1]) && !RepathFromCurrentBand())
            break;

        if (ShouldHoldForMerge(m_path[m_pathIndex + 1]))
            break;

        if (!TryReserveJunction(m_path[m_pathIndex + 1]))
            break;

        ++m_pathIndex;
        const RoadRef &next = m_path[m_pathIndex];
        bool hasLaneMapping = false;
        float nextOffset = RoadDataManager::Get().ResolveConnectingOffset(CurrentRoadRef(), next, m_currentOffset, &hasLaneMapping);

        if (!hasLaneMapping)
            nextOffset = ComputeReferenceOffset(next.road->GetReferenceLine(), position);

        if (next.road->IsParkingRoad() || m_currentRoad->IsParkingRoad())
            nextOffset = RoadDriveOffset(next, position);
        SetCurrentRoad(next.road, nextOffset, next.direction);
        projectedPosition = m_currentSpline.GetLookaheadPoint(position, 0.0f);
        roadEndDistance = (roadEnd() - projectedPosition).Length();
    }
    return true;
}

void Car::DriveControl()
{
    if (m_reverseEscapeTimer > 0.0f) // 정면충돌 후진탈출
    {
        Steer(m_reverseEscapeSteer, REVERSE_STEER_RAMP);
        AccelerateVel(REVERSE_ESCAPE_SPEED);
        return;
    }

    const Spline &spline = m_currentSpline;

    constexpr float LOOKAHEAD_TIME = 1.5f;
    float lookaheadDistance = 5.0f;
    Vec3 target = spline.GetLookaheadPoint(GetRigidbodyPosition(), lookaheadDistance);
    float targetSteer = PurePursuit(target);
    if (UsesReactiveSteer())
    {
        targetSteer = ComputeContextSteer(target, targetSteer);
    }
    else if (!m_ctxSteerOpenLines.empty() || !m_ctxSteerBlockedLines.empty())
    {
        m_ctxSteerOpenLines.clear();
        m_ctxSteerBlockedLines.clear();
    }
    RebuildCtxSteerRender();

    Steer(targetSteer);

    float distanceOffset = (GetPosition() - m_planScanPosition).Length();
    float elapsedTime = m_currentTime - m_lastBehaviorPlanTime;
    float accelIDM = ComputeIdmAcceleration(m_lastRoadSamples, m_lastIdmParams, distanceOffset, elapsedTime, &m_currentLeader, &m_limitDebug);

    if (m_speedCap >= 0.0f && m_speed > m_speedCap)
    {
        float capGap = std::max(1.0f, (m_speed * m_speed - m_speedCap * m_speedCap) / (2.0f * m_maxBrake) + SAFE_GAP);
        float capAccel = IDM::CalculateAcceleration(m_speed, m_acceleration, m_speedCap, 0.0f, capGap, m_lastIdmParams);
        if (capAccel < accelIDM)
        {
            accelIDM = capAccel;
            m_limitDebug = SpeedLimitDebug{"speedCap", m_speedCap, capGap};
        }
    }

    float steerSpeedCap = CalcMaxSpeed(targetSteer);
    if (!UsesReactiveSteer() && m_speed > steerSpeedCap && -m_maxBrake < accelIDM)
    {
        accelIDM = -m_maxBrake;
        m_limitDebug = SpeedLimitDebug{"steerCap", steerSpeedCap, 0.0f};
    }

    bool emergBlocked = IsEmergencyRayBlocked();
    RebuildEmergRayRender();
    if (emergBlocked)
    {
        EmergBrake();
        m_limitDebug = SpeedLimitDebug{"emergRay", 0.0f, 0.0f};
    }
    else
    {
        Accelerate(accelIDM);
    }

    DirectX::XMFLOAT3 targetMarkerPos = ToXMFLOAT3(target);
    targetMarkerPos.y = GetPosition().GetY() + 0.2f;
    m_targetMarker.GetTransform().SetPosition(targetMarkerPos);
}

#pragma endregion

#pragma region BehaviorPlan

std::vector<Car::RoadSpeedSample> Car::ScanRoadSpeedConstraints(float lookDistance) const
{
    constexpr float ROAD_SAMPLE_SPACING = 5.0f;

    Vec3 calPosition = GetPosition();

    std::vector<VehicleCollision::Obstacle> obstacles;
    {
        float reach = lookDistance + GetLength();
        for (const VehicleCollision::Obstacle &obstacle : RoadDataManager::Get().GetObstacles())
        {
            if ((obstacle.center - calPosition).Length() > reach + obstacle.halfLength + obstacle.halfWidth)
                continue;
            obstacles.push_back(obstacle);
        }
    }

    std::vector<RoadSpeedSample> samples;
    samples.reserve(static_cast<size_t>(lookDistance / ROAD_SAMPLE_SPACING) + 4);
    auto splineEnd = [](const Spline *sp) -> Vec3
    {
        const std::vector<Vec3> &p = sp->GetSplinePoints();
        return p.empty() ? Vec3::sZero() : p.back();
    };
    {
        float currentNodeT = m_currentSpline.GetSplinePosition(calPosition);
        Assert(currentNodeT >= 0.0f);
        float currentNodeDistance = m_currentSpline.GetLength() * (1.0f - currentNodeT);
        float currentNodeSpeed = (m_currentRoad == m_destRoad) ? 0.0f : RoadTargetSpeed(m_currentRoad);
        samples.push_back({splineEnd(&m_currentSpline), currentNodeDistance, currentNodeSpeed, nullptr, "curRoadEnd"});

        if (m_lotCruise && m_currentRoad == m_destRoad)
        {
            float stopT = m_currentSpline.GetSplinePosition(m_lotStopPos);
            float stopDistance = (stopT - currentNodeT) * m_currentSpline.GetLength();
            samples.push_back({m_lotStopPos, stopDistance - SAFE_GAP, 0.0f, nullptr, "parkPre"});
        }

        else if (m_currentRoad == m_destRoad && m_pendingParkNode != nullptr && !m_pendingParkNode->parkingRoadIds.empty())
        {
            float entryT = m_currentSpline.GetSplinePosition(m_pendingParkNode->position);
            float entryDistance = (entryT - currentNodeT) * m_currentSpline.GetLength();
            samples.push_back({m_pendingParkNode->position, entryDistance - SAFE_GAP, 0.0f, nullptr, "lotEntry"});
        }

        if (m_subMode == SubMode::D_SirenWait && m_sirenPulledOver)
            samples.push_back({calPosition, -SAFE_GAP, 0.0f, nullptr, "sirenWait"});

        if (m_subMode == SubMode::D_ChaseBlock)
            samples.push_back({calPosition, -SAFE_GAP, 0.0f, nullptr, "chaseBlock"});
    }
    {

        Spline segmentLine;
        const Spline *spline = &m_currentSpline;
        RoadRef segment = CurrentRoadRef();
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

            RoadRef nextRoad = (pathIndex + 1 < m_path.size()) ? m_path[pathIndex + 1] : RoadRef{};
            int nextRoadId = nextRoad.road != nullptr ? nextRoad.road->GetId() : -1;

            shared_ptr<RoadNode> signalNode = RoadDataManager::Get().GetSignalNodeForRoad(segment.road->GetId(), nextRoadId);
            if (signalNode != nullptr && splineLength > 0.0f && ShouldStopForSignal(segment.road, segment.direction, nextRoadId))
            {
                float nodeT = spline->GetSplinePosition(signalNode->position);
                float nodeDistance = (segment.road == m_currentRoad)
                                         ? (signalNode->position - calPosition).Length()
                                         : traveledDistance + (nodeT - startT) * splineLength;
                float stopDistance = std::min(nodeDistance, traveledDistance + segmentDistance);
                samples.push_back({signalNode->position, stopDistance - SAFE_GAP, 0.0f, nullptr, "signal", RoadSampleKind::Signal});
                break;
            }

            traveledDistance += walkDistance;
            remainingDistance -= walkDistance;
            if (remainingDistance <= 0.0f)
                break;

            if (!nextRoad.road)
            {

                if (segment.road == m_destRoad)
                    samples.push_back({splineEnd(spline), traveledDistance, 0.0f, nullptr, "destEnd"});
                break;
            }

            bool holdForMerge = ShouldHoldForMerge(nextRoad);
            bool holdForJunction = segment.road == m_currentRoad && ShouldHoldForJunction(nextRoad);
            if (holdForMerge || holdForJunction)
            {
                const char *label = holdForJunction ? "junctionWait" : "mergeWait";
                RoadSampleKind kind = holdForJunction ? RoadSampleKind::JunctionWait : RoadSampleKind::Other;
                samples.push_back({splineEnd(spline), traveledDistance - SAFE_GAP, 0.0f, nullptr, label, kind});
                break;
            }

            float nextNodeSpeed = RoadTargetSpeed(nextRoad.road);
            segmentLine = RoadDataManager::Get().BuildOffsetSpline(nextRoad.road, 0.0f, nextRoad.direction);
            Vec3 nextStart = segmentLine.GetSplinePoints().empty() ? segmentStart : segmentLine.GetSplinePoints().front();
            samples.push_back({nextStart, traveledDistance, nextNodeSpeed, nullptr, "roadLimit"});

            segment = nextRoad;
            segmentStart = nextStart;
            spline = &segmentLine;
            ++pathIndex;
        }
    }
    std::sort(samples.begin(), samples.end(), [](const RoadSpeedSample &a, const RoadSpeedSample &b)
              { return a.distance < b.distance; });
    return samples;
}

void Car::ComputeDrivableRange(const RoadRef &road, float &outMin, float &outMax) const
{

    float halfW = GetHalfWidth() * 0.5f;

    outMin = -(RoadDataManager::ROAD_WIDTH - halfW);
    outMax = RoadDataManager::ROAD_WIDTH - halfW;

    std::vector<const LaneBand *> bands = GatherCandidateBands(road);
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

IDM::Params Car::BuildIdmParams(const shared_ptr<Road> &road) const
{
    constexpr float IDM_TIME_HEADWAY = 1.5f;

    IDM::Params idm;
    idm.v0 = IgnoresSpeedLimit(road) ? m_maxSpeed
                                     : std::min(road->GetSpeedLimit() * m_personality.speedFactor, m_maxSpeed);
    idm.T = IDM_TIME_HEADWAY * m_personality.headwayFactor;
    idm.s0 = SAFE_GAP * m_personality.headwayFactor;
    idm.a = m_maxAccel;
    idm.b = m_maxBrake * m_personality.brakeFactor;
    idm.delta = 4.0f;
    idm.coolness = 1.0f;
    return idm;
}

bool Car::IgnoresSpeedLimit(const shared_ptr<Road> &road) const
{
    return IgnoresTrafficRules() && road != nullptr && road->GetReferenceLine().IsStraight();
}

float Car::RoadTargetSpeed(const shared_ptr<Road> &road) const
{
    if (road == nullptr)
        return m_maxSpeed;
    return IgnoresSpeedLimit(road) ? m_maxSpeed : std::min(road->GetSpeedLimit(), m_maxSpeed);
}

float Car::ComputeIdmAcceleration(const std::vector<RoadSpeedSample> &samples, const IDM::Params &params,
                                  float distanceOffset, float elapsedTime,
                                  const VehicleCollision::Obstacle **outLeader,
                                  SpeedLimitDebug *outDebug) const
{

    float speedRatio = std::min(1.0f, m_speed / std::max(0.1f, params.v0));
    float bestAccel = params.a * (1.0f - std::pow(speedRatio, params.delta));
    const bool fastOvertake = m_fleeOn || IsChasing();
    const VehicleCollision::Obstacle *overtakeTarget = nullptr;
    float overtakeTargetDistance = std::numeric_limits<float>::max();
    if (outLeader != nullptr)
        *outLeader = nullptr;
    if (outDebug != nullptr)
        *outDebug = SpeedLimitDebug{"free", params.v0, 0.0f};

    for (const RoadSpeedSample &sample : samples)
    {
        if (sample.leader != nullptr && !m_SimState->IsCarAlive(sample.leader))
            continue;

        // Flee/chase cars use vehicle hits as lateral avoidance targets instead of IDM leaders.
        // Keep the nearest hit for DecideAvoidance, but do not slow down behind it.
        if (fastOvertake && sample.hasObstacle && sample.obstacle.isVehicle)
        {
            float targetDistance = sample.distance - distanceOffset;
            if (sample.leader != nullptr)
            {
                targetDistance += (sample.leader->GetPosition() - sample.leaderScanPosition)
                                      .Dot(GetForwardAxis());
            }
            if (targetDistance > 0.0f && targetDistance < overtakeTargetDistance)
            {
                overtakeTargetDistance = targetDistance;
                overtakeTarget = &sample.obstacle;
            }
            continue;
        }

        // 서로를 리더로 잡고 둘 다 서는 교착
        if (sample.leader != nullptr && sample.leader->GetIdmLeader() == this)
            continue;

        float leaderSpeed;
        float leaderAccel;
        float gap;
        if (sample.leader != nullptr)
        {

            // 리더 자기 헤딩 기준 스칼라를 내 진행축으로 투영 (비스듬한 리더도 일관되게)
            float leaderAlign = GetForwardAxis().Dot(sample.leader->GetForwardAxis());
            leaderSpeed = std::max(0.0f, leaderAlign * sample.leader->GetSpeed());
            leaderAccel = leaderAlign * sample.leader->GetAcceleration();

            float leaderTravel = (sample.leader->GetPosition() - sample.leaderScanPosition)
                                     .Dot(GetForwardAxis());
            gap = sample.distance - distanceOffset + leaderTravel;
        }
        else
        {

            float remaining = sample.distance - distanceOffset + sample.speed * elapsedTime;
            bool hardStop = sample.speed <= 0.0f;
            if (!hardStop && (remaining <= 0.0f || m_speed <= sample.speed))
                continue;

            leaderSpeed = sample.speed;
            leaderAccel = 0.0f;
            gap = remaining + SAFE_GAP;
        }
        float a = IDM::CalculateAcceleration(m_speed, m_acceleration, leaderSpeed, leaderAccel, gap, params);
        if (a < bestAccel)
        {
            bestAccel = a;
            if (outLeader != nullptr)
                *outLeader = sample.hasObstacle ? &sample.obstacle : nullptr;
            if (outDebug != nullptr)
                *outDebug = SpeedLimitDebug{sample.leader != nullptr ? "car:" + sample.leader->GetName() : sample.debugStr, leaderSpeed, gap};
        }
    }
    if (overtakeTarget != nullptr && outLeader != nullptr)
        *outLeader = overtakeTarget;
    return bestAccel;
}

Car::LaneNeighbors Car::GatherLaneNeighbors(const shared_ptr<Road> &road, const Spline &refLine, float bandCenter,
                                            float bandHalfWidth, float myS, float dirSign) const
{
    LaneNeighbors out;
    float bestLeaderGap = std::numeric_limits<float>::max();
    float bestFollowerGap = std::numeric_limits<float>::max();

    struct ScanRoad
    {
        const Spline *refLine = nullptr;
        float dirSign = 1.0f;
        float sBias = 0.0f; // 로컬 s + sBias = 기준로드 s
        float bandCenter = 0.0f;
        bool allowLeader = true;   // 다음로드 차는 무조건 앞
        bool allowFollower = true; // 이전로드 차는 무조건 뒤
    };

    ScanRoad scans[3];
    int scanCount = 0;
    scans[scanCount++] = ScanRoad{&refLine, dirSign, 0.0f, bandCenter, true, true};

    // 로드가 짧아 앞/뒤차가 인접 로드에 있는 일이 잦다
    if (road == m_currentRoad && m_pathIndex < m_path.size() && m_path[m_pathIndex].road == m_currentRoad)
    {
        RoadDataManager &rdm = RoadDataManager::Get();
        // TravelS는 정방향 [0,L], 역방향 [-L,0] -- 양쪽 다 진행할수록 증가
        auto travelStartS = [](const Spline &line, float sign)
        { return sign > 0.0f ? 0.0f : -line.GetLength(); };
        auto travelEndS = [](const Spline &line, float sign)
        { return sign > 0.0f ? line.GetLength() : 0.0f; };

        if (m_pathIndex + 1 < m_path.size() && m_path[m_pathIndex + 1].road != nullptr)
        {
            const RoadRef &next = m_path[m_pathIndex + 1];
            const Spline &nextRef = next.road->GetReferenceLine();
            float nextSign = GetTravelSign(next.direction);
            scans[scanCount++] = ScanRoad{&nextRef, nextSign,
                                          travelEndS(refLine, dirSign) - travelStartS(nextRef, nextSign),
                                          rdm.ResolveConnectingOffset(CurrentRoadRef(), next, bandCenter, nullptr),
                                          true, false};
        }
        if (m_pathIndex > 0 && m_path[m_pathIndex - 1].road != nullptr)
        {
            const RoadRef &prev = m_path[m_pathIndex - 1];
            // 역매핑이 없어서 prev 밴드를 정방향으로 훑어 내 밴드로 오는 걸 찾는다
            for (const LaneBand *band : rdm.GetDrivingBands(prev.road, prev.direction))
            {
                float mapped = rdm.ResolveConnectingOffset(prev, CurrentRoadRef(), band->centerOffset, nullptr);
                if (std::fabs(mapped - bandCenter) > bandHalfWidth)
                    continue;

                const Spline &prevRef = prev.road->GetReferenceLine();
                float prevSign = GetTravelSign(prev.direction);
                scans[scanCount++] = ScanRoad{&prevRef, prevSign,
                                              travelStartS(refLine, dirSign) - travelEndS(prevRef, prevSign),
                                              band->centerOffset, false, true};
                break;
            }
        }
    }

    for (int si = 0; si < scanCount; ++si)
    {
        const ScanRoad &scan = scans[si];
        for (const VehicleCollision::Obstacle &obstacle : m_obstacles)
        {
            float d = 0.0f;
            float halfExtent = 0.0f;
            if (!ProjectObstacle(*scan.refLine, obstacle, d, halfExtent))
                continue;
            if (std::fabs(d - scan.bandCenter) > bandHalfWidth + halfExtent)
                continue;

            Mobil::VehicleState st;
            Car *other = obstacle.sourceCar;
            if (other != nullptr)
            {
                if (!m_SimState->IsCarAlive(other))
                    continue;

                // m_currentRoad로 거르면 경계 걸친 차가 양쪽 스캔에서 다 빠진다.
                // 투영 잔차 + 밴드 검사가 이미 "이 로드 이 차선인가"를 판정한다.
                float otherT = scan.refLine->GetSplinePosition(other->GetPosition());
                float dirDot = scan.refLine->GetDirectionAt(otherT).Dot(other->GetForwardAxis()) * scan.dirSign;
                if (dirDot <= 0.0f && !IgnoresTrafficRules()) // 마주오는 차는 MOBIL 대상 아님
                    continue;

                // 역주행 후보에선 마주오는 차를 음수속도 리더로(접근속도 반영)
                st.speed = dirDot > 0.0f ? std::max(0.0f, other->GetSpeed() * dirDot) : other->GetSpeed() * dirDot;
                st.accel = other->GetAcceleration() * dirDot;
                st.position = TravelS(*scan.refLine, other->GetPosition(), scan.dirSign) + scan.sBias;
                st.length = other->GetLength();
            }
            else
            {
                if (si != 0) // 정적은 기준로드만 (로드체크가 없어 중복됨)
                    continue;
                st.speed = obstacle.speed;
                st.accel = 0.0f;
                st.position = TravelS(*scan.refLine, obstacle.center, scan.dirSign) + scan.sBias;
                st.length = obstacle.halfLength * 2.0f;
            }

            // 인접로드는 앞뒤가 기하학적으로 정해져 있다. s비교로 뒤집히면 버린다.
            bool ahead = st.position >= myS;
            if (ahead ? !scan.allowLeader : !scan.allowFollower)
                continue;

            if (ahead)
            {
                float gap = st.position - myS;
                if (gap < bestLeaderGap)
                {
                    bestLeaderGap = gap;
                    out.leader = st;
                    out.leaderCar = other;
                    out.hasLeader = true;
                }
            }
            else if (other != nullptr) // 정적 장애물은 뒤차로 안 본다
            {
                float gap = myS - st.position;
                if (gap < bestFollowerGap)
                {
                    bestFollowerGap = gap;
                    out.follower = st;
                    out.followerCar = other;
                    out.hasFollower = true;
                }
            }
        }
    }

    return out;
}

Car *Car::GetIdmLeader() const
{
    Car *leader = m_currentLeader != nullptr ? m_currentLeader->sourceCar : nullptr;
    return (leader != nullptr && m_SimState->IsCarAlive(leader)) ? leader : nullptr;
}

void Car::GetLaneChangeNeighbors(Car *&outLeader, Car *&outFollower) const
{
    outLeader = nullptr;
    outFollower = nullptr;
    if (m_subMode != SubMode::D_LaneChange || m_currentRoad == nullptr)
        return;

    const LaneBand *target = RoadDataManager::Get().FindNearestBand(m_currentRoad, m_maneuver.laneChangeTarget, m_travelDir);
    if (target == nullptr)
        return;

    const Spline &ref = m_currentRoad->GetReferenceLine();
    float dirSign = TravelSign();
    float myS = TravelS(ref, GetPosition(), dirSign);
    LaneNeighbors nbr = GatherLaneNeighbors(m_currentRoad, ref, target->centerOffset, target->width * 0.5f, myS, dirSign);
    outLeader = nbr.leaderCar;
    outFollower = nbr.followerCar;
}

bool Car::IsSafeLaneEntry(const RoadRef &road, float targetOffset, bool checkLeader) const
{
    const LaneBand *target = RoadDataManager::Get().FindNearestBand(road.road, targetOffset, road.direction);
    if (target == nullptr)
        return true;

    const Spline &ref = road.road->GetReferenceLine();
    float dirSign = GetTravelSign(road.direction);
    float myS = TravelS(ref, GetPosition(), dirSign);
    LaneNeighbors nbr = GatherLaneNeighbors(road.road, ref, target->centerOffset, target->width * 0.5f, myS, dirSign);

    Mobil::VehicleState myState;
    myState.speed = m_speed;
    myState.accel = m_acceleration;
    myState.position = myS;
    myState.length = GetLength();
    IDM::Params idm = BuildIdmParams(road.road);

    // 앞차 감속강요 과함
    if (checkLeader && nbr.hasLeader)
    {
        float accel = IDM::CalculateAcceleration(myState.speed, myState.accel, nbr.leader.speed, nbr.leader.accel,
                                                 Mobil::GetGap(myState, &nbr.leader), idm);
        if (accel < -MOBIL_B_SAFE)
            return false;
    }

    // 뒤차 강요감속 안전한지
    if (nbr.hasFollower)
    {
        Mobil::Params mobil{MOBIL_B_SAFE, m_personality.politeness, MOBIL_A_THR};
        if (!Mobil::IsSafeLaneChange(myState, &nbr.follower, mobil, idm))
            return false;
    }
    return true;
}

bool Car::ShouldHoldForMerge(const RoadRef &nextRoad) const
{
    if (m_fleeOn || IsChasing() || nextRoad.road == nullptr)
        return false;
    bool hasLaneMapping = false;
    float targetOffset = RoadDataManager::Get().ResolveConnectingOffset(CurrentRoadRef(), nextRoad, m_currentOffset, &hasLaneMapping);
    if (!hasLaneMapping)
        targetOffset = ComputeReferenceOffset(nextRoad.road->GetReferenceLine(), GetPosition());
    return !IsSafeLaneEntry(nextRoad, targetOffset, false); // 앞차는 IDM이 맡는다
}

bool Car::ShouldHoldForJunction(const RoadRef &nextRoad) const
{
    if (nextRoad.road == nullptr || m_currentRoad == nullptr || nextRoad.road->GetJunctionId() < 0 || m_SimState == nullptr)
        return false;
    if (IgnoresTrafficRules())
        return false;
    return !m_SimState->IsJunctionAvailable(nextRoad.road->GetJunctionId(), m_currentRoad->GetId(), this);
}

bool Car::TryReserveJunction(const RoadRef &nextRoad)
{
    if (nextRoad.road == nullptr || m_currentRoad == nullptr || nextRoad.road->GetJunctionId() < 0 || m_SimState == nullptr)
        return true;

    int junctionId = nextRoad.road->GetJunctionId();
    if (m_reservedJunctionId >= 0 && m_reservedJunctionId != junctionId)
        ReleaseJunctionReservation();
    if (!m_SimState->TryReserveJunction(junctionId, m_currentRoad->GetId(), this))
        return IgnoresTrafficRules(); // 점유 실패해도 그냥 진입

    m_reservedJunctionId = junctionId;
    return true;
}

void Car::ReleaseJunctionReservation()
{
    if (m_reservedJunctionId >= 0 && m_SimState != nullptr)
        m_SimState->ReleaseJunction(m_reservedJunctionId, this);
    m_reservedJunctionId = -1;
}

void Car::UpdateDrivePlan()
{
    if (m_currentRoad == nullptr)
        return;

    constexpr float MIN_LOOK_DISTANCE = 20.0f;
    m_lastIdmParams = BuildIdmParams(m_currentRoad);

    float interactGap = m_lastIdmParams.s0 + m_speed * m_lastIdmParams.T +
                        m_speed * m_speed / (2.0f * std::sqrt(std::max(0.0001f, m_lastIdmParams.a * m_lastIdmParams.b)));
    float lookDistance = std::max(MIN_LOOK_DISTANCE, interactGap + m_speed * BEHAVIOR_PLAN_INTERVAL);

    m_lastRoadSamples = ScanRoadSpeedConstraints(lookDistance);
    AppendSensorConstraintSample(m_lastRoadSamples);
    RebuildSensorRender();
    m_planScanPosition = GetPosition();

    // 조향이 실제 위치를 만든다. 계획 오프셋은 따라가기만 하되 도로 밖으론 안 나가게 클램프
    if (UsesReactiveSteer())
    {
        float minOffset = 0.0f;
        float maxOffset = 0.0f;
        ComputeDrivableRange(CurrentRoadRef(), minOffset, maxOffset);
        float realOffset = ComputeReferenceOffset(m_currentRoad->GetReferenceLine(), GetPosition());
        SetCurrentOffset(std::clamp(realOffset, minOffset, maxOffset));
        m_currentSpline = RoadDataManager::Get().BuildOffsetSpline(m_currentRoad, m_currentOffset, m_travelDir);
        RebuildSplineRender();
        return;
    }

    float laneCenter = m_currentOffset;
    float targetOffset;
    if (m_subMode == SubMode::D_Avoid || m_subMode == SubMode::D_LaneChange ||
        m_subMode == SubMode::D_SirenWait || m_subMode == SubMode::D_ChaseBlock)
    {
        targetOffset = AvoidTargetOffset();
    }
    else
    {
        bool coolingDown = (m_currentTime - m_lastLaneChangeTime) < MOBIL_LANE_CHANGE_COOLDOWN;
        const char *reason = "";
        targetOffset = coolingDown ? CurrentLaneCenter() : ComputeLateralTarget(m_lastIdmParams, &laneCenter, &reason);
        if (coolingDown)
            laneCenter = targetOffset;

        bool isLaneChange = std::fabs(targetOffset - laneCenter) > 0.01f;
        if (isLaneChange)
        {
            m_maneuver.laneOffset = laneCenter;
            m_maneuver.laneChangeTarget = targetOffset;
            m_lastLaneChangeTime = m_currentTime;
        }
        SetSubMode(isLaneChange ? SubMode::D_LaneChange : SubMode::D_Normal);
    }
    SetCurrentOffset(m_currentOffset + (targetOffset - m_currentOffset) * m_personality.laneChangeLerpAlpha);
    m_currentSpline = RoadDataManager::Get().BuildOffsetSpline(m_currentRoad, m_currentOffset, m_travelDir);
    RebuildSplineRender();
}

float Car::CurrentLaneCenter() const
{
    return m_currentBand != nullptr ? m_currentBand->centerOffset : m_currentOffset;
}

std::vector<const LaneBand *> Car::GatherCandidateBands(const RoadRef &road) const
{
    RoadDataManager &roadData = RoadDataManager::Get();
    std::vector<const LaneBand *> bands = roadData.GetDrivingBands(road.road, road.direction);
    if (!IgnoresTrafficRules())
        return bands;

    for (const LaneBand *band : roadData.GetDrivingBands(road.road, GetOppositeDirection(road.direction)))
        bands.push_back(band);

    // 인접 판정이 d 순서에 의존
    std::sort(bands.begin(), bands.end(), [](const LaneBand *a, const LaneBand *b)
              { return a->centerOffset < b->centerOffset; });

    // 양끝 밖 가상차로. 포인터 수명 때문에 현재도로만
    if (!bands.empty() && road.road == m_currentRoad)
    {
        const LaneBand *leftmost = bands.front();
        m_virtualBands[0] = *leftmost;
        m_virtualBands[0].centerOffset = leftmost->centerOffset - leftmost->width;

        const LaneBand *rightmost = bands.back();
        m_virtualBands[1] = *rightmost;
        m_virtualBands[1].centerOffset = rightmost->centerOffset + rightmost->width;

        bands.insert(bands.begin(), &m_virtualBands[0]);
        bands.push_back(&m_virtualBands[1]);
    }
    return bands;
}

Car::RouteLaneGoal Car::ComputeRouteLaneGoal() const
{
    constexpr float ROUTE_BIAS_MAX = 3.0f;
    constexpr float ROUTE_LANE_CHANGE_TIME = 4.0f;
    constexpr float ROUTE_MIN_PLAN_SPEED = 2.0f;

    RouteLaneGoal goal;
    if (m_currentRoad == nullptr || m_pathIndex + 1 >= m_path.size())
        return goal;

    std::vector<const LaneBand *> entry = RoadDataManager::Get().GetEntryBands(CurrentRoadRef(), m_path[m_pathIndex + 1]);
    if (entry.empty())
        return goal;

    const LaneBand *target = entry.front();
    for (const LaneBand *band : entry)
        if (std::fabs(m_currentOffset - band->centerOffset) < std::fabs(m_currentOffset - target->centerOffset))
            target = band;
    goal.active = true;
    goal.targetOffset = target->centerOffset;

    std::vector<const LaneBand *> bands = RoadDataManager::Get().GetDrivingBands(m_currentRoad, m_travelDir);
    int curIdx = -1;
    int targetIdx = -1;
    for (size_t i = 0; i < bands.size(); ++i)
    {
        if (curIdx < 0 || std::fabs(m_currentOffset - bands[i]->centerOffset) < std::fabs(m_currentOffset - bands[curIdx]->centerOffset))
            curIdx = static_cast<int>(i);
        if (bands[i] == target)
            targetIdx = static_cast<int>(i);
    }
    int lanesToCross = (curIdx >= 0 && targetIdx >= 0) ? ((targetIdx > curIdx) ? targetIdx - curIdx : curIdx - targetIdx) : 1;
    lanesToCross = std::max(lanesToCross, 1);

    float t = m_currentSpline.GetSplinePosition(GetPosition());
    float remainDistance = m_currentSpline.GetLength() * (1.0f - t);
    float timeLeft = remainDistance / std::max(m_speed, ROUTE_MIN_PLAN_SPEED);
    float timeNeeded = lanesToCross * ROUTE_LANE_CHANGE_TIME;
    goal.urgency = std::clamp(timeNeeded / std::max(timeLeft, 0.0001f), 0.0f, 1.0f);
    goal.bias = ROUTE_BIAS_MAX * goal.urgency;
    return goal;
}

float Car::ComputeLateralTarget(const IDM::Params &idm,
                                float *outLaneCenter, const char **outReason) const
{
    if (m_currentBand == nullptr)
        return m_currentOffset;

    if (outLaneCenter != nullptr)
        *outLaneCenter = m_currentBand->centerOffset;

    float realOffset = ComputeReferenceOffset(m_currentRoad->GetReferenceLine(), GetPosition());
    if (!IsOnLane(realOffset))
    {
        bool safe = IsSafeLaneEntry(CurrentRoadRef(), m_currentBand->centerOffset);
        return safe ? m_currentBand->centerOffset : realOffset;
    }
    constexpr float ROUTE_BSAFE_RELAX = 1.0f;

    const Spline &refLine = m_currentRoad->GetReferenceLine();
    float dirSign = TravelSign();
    float myS = TravelS(refLine, GetPosition(), dirSign);

    Mobil::VehicleState myState;
    myState.speed = m_speed;
    myState.accel = m_acceleration;
    myState.position = myS;
    myState.length = GetLength();

    Mobil::VehicleState farLeader;
    farLeader.speed = idm.v0;
    farLeader.accel = 0.0f;
    farLeader.position = myS + 1000.0f;
    farLeader.length = 0.0f;

    // 앞차 차선이탈시 보류
    LaneNeighbors cur = GatherLaneNeighbors(m_currentRoad, refLine, m_currentBand->centerOffset, m_currentBand->width * 0.5f, myS, dirSign);
    if (cur.leaderCar != nullptr && !cur.leaderCar->IsOnLane(cur.leaderCar->m_currentOffset))
        return m_currentOffset;
    // 내 차선의 앞/뒤차가 차선변경 중이면 그 차의 목표차로를 예측 못하니 보류
    if ((cur.leaderCar != nullptr && cur.leaderCar->m_subMode == SubMode::D_LaneChange) ||
        (cur.followerCar != nullptr && cur.followerCar->m_subMode == SubMode::D_LaneChange))
        return m_currentBand->centerOffset;
    const Mobil::VehicleState *curLeader = cur.hasLeader ? &cur.leader : &farLeader;
    const Mobil::VehicleState *oldFollower = cur.hasFollower ? &cur.follower : nullptr;
    const Mobil::VehicleState &myLeader = *curLeader;

    // 신호도 막는지점 포함
    float myBlockS = (curLeader != &farLeader) ? curLeader->position : std::numeric_limits<float>::infinity();
    for (const RoadSpeedSample &sample : m_lastRoadSamples)
    {
        if (sample.leader == nullptr && sample.kind == RoadSampleKind::Signal)
        {
            myBlockS = std::min(myBlockS, myS + sample.distance + SAFE_GAP);
            break;
        }
    }

    // 급할수록 안전기준 완화
    RouteLaneGoal goal = ComputeRouteLaneGoal();

    std::vector<const LaneBand *> bands = GatherCandidateBands(CurrentRoadRef());
    size_t curIdx = 0;
    for (size_t i = 0; i < bands.size(); ++i)
        if (bands[i] == m_currentBand)
        {
            curIdx = i;
            break;
        }

    int preferDir;
    if (goal.active && std::fabs(goal.targetOffset - m_currentBand->centerOffset) > 0.01f)
        preferDir = (goal.targetOffset > m_currentBand->centerOffset) ? 1 : -1;
    else
        preferDir = (m_currentOffset > m_currentBand->centerOffset) ? 1 : -1;

    for (int di : {preferDir, -preferDir})
    {
        long adjIdx = static_cast<long>(curIdx) + di;
        if (adjIdx < 0 || adjIdx >= static_cast<long>(bands.size()))
            continue;

        const LaneBand &adjBand = *bands[adjIdx];
        Mobil::Params mobil{MOBIL_B_SAFE, m_personality.politeness, MOBIL_A_THR};
        float bias = 0.0f;
        if (goal.active)
        {
            bool towardGoal = std::fabs(adjBand.centerOffset - goal.targetOffset) <
                              std::fabs(m_currentBand->centerOffset - goal.targetOffset);
            bias = towardGoal ? goal.bias : -goal.bias;
            if (towardGoal)
                mobil.b_safe *= 1.0f + ROUTE_BSAFE_RELAX * goal.urgency;
        }

        // 신호대기 중복 스킵
        LaneNeighbors nbr = GatherLaneNeighbors(m_currentRoad, refLine, adjBand.centerOffset, adjBand.width * 0.5f, myS, dirSign);
        if (nbr.leaderCar != nullptr && !nbr.leaderCar->IsOnLane(nbr.leaderCar->m_currentOffset))
            continue;
        if (nbr.hasLeader && std::fabs(nbr.leader.position - myBlockS) < MOBIL_LEADER_ALIGN_GAP)
            continue;
        // 옆차선의 앞/뒤차가 차선변경 중이면 거기 끼어들지 않는다
        if ((nbr.leaderCar != nullptr && nbr.leaderCar->m_subMode == SubMode::D_LaneChange) ||
            (nbr.followerCar != nullptr && nbr.followerCar->m_subMode == SubMode::D_LaneChange))
            continue;
        const Mobil::VehicleState &newLeader = nbr.hasLeader ? nbr.leader : farLeader;
        const Mobil::VehicleState *newFollower = nbr.hasFollower ? &nbr.follower : nullptr;
        if (Mobil::EvaluateLaneChange(myState, oldFollower, myLeader, newLeader, newFollower, mobil, idm, bias))
        {
            if (outReason != nullptr)
                *outReason = (goal.active && bias > 0.0f) ? "route" : "overtake";
            auto describe = [](Car *car) -> std::string
            {
                if (car == nullptr)
                    return "static";
                return car->GetName() + "(road " +
                       (car->m_currentRoad != nullptr ? ToString(car->m_currentRoad->GetId()) : "?") + ")";
            };
            std::string leaderName = nbr.hasLeader ? describe(nbr.leaderCar) : "none";
            std::string followerName = nbr.hasFollower ? describe(nbr.followerCar) : "none";
            float leaderGap = nbr.hasLeader ? nbr.leader.position - myS : -1.0f;
            float followerGap = nbr.hasFollower ? myS - nbr.follower.position : -1.0f;
            DebugConsole::Log(GetName() + ": lane change decide on road " + ToString(m_currentRoad->GetId()) +
                              " myS " + ToString(myS) + ", leader " + leaderName + " gap " + ToString(leaderGap) +
                              ", follower " + followerName + " gap " + ToString(followerGap));
            return adjBand.centerOffset;
        }
    }

    return m_currentBand->centerOffset;
}

void Car::AppendSensorConstraintSample(std::vector<RoadSpeedSample> &samples) const
{
    m_sweepDebugCorners.clear();

    if (m_currentRoad == nullptr)
        return;

    constexpr float SWEEP_MIN = 30.0f;
    constexpr float SWEEP_MAX = 100.0f;

    float maxDistance = std::clamp(m_speed * m_speed / (2.0f * m_maxBrake) + SAFE_GAP, SWEEP_MIN, SWEEP_MAX);
    float stepDistance = (GetLength() - MIN_SAFE_GAP) * 0.5f;
    int sweepSteps = std::max(static_cast<int>(maxDistance / stepDistance), 8);

    VehicleCollision::VehicleShape shape = BuildVehicleShape();
    Vec3 forward = GetForwardAxis();
    Vec3 position = GetRigidbodyPosition();

    float currentRemain = m_currentSpline.GetLength() * (1.0f - m_currentSpline.GetSplinePosition(position));

    Spline nextSplineStorage;
    const Spline *nextSpline = nullptr;
    if (currentRemain < maxDistance && m_pathIndex + 1 < m_path.size())
    {
        const RoadRef &next = m_path[m_pathIndex + 1];
        if (next.road != nullptr)
        {
            float nextOffset = RoadDataManager::Get().ResolveConnectingOffset(CurrentRoadRef(), next, m_currentOffset, nullptr);
            nextSplineStorage = RoadDataManager::Get().BuildOffsetSpline(next.road, nextOffset, next.direction);
            if (!nextSplineStorage.GetSplinePoints().empty())
                nextSpline = &nextSplineStorage;
        }
    }

    constexpr float STEER_LOOKAHEAD = 5.0f; // DriveControl과 동일값
    constexpr float BIKE_SUBSTEP = 0.25f;   // 궤적 적분 정밀도

    bool reversed = m_travelDir == LaneDirection::Backward;
    float targetOffset = AvoidTargetOffset();
    float maxSteerAngle = CalcMaxSteerAngle(m_speed);

    // 오프셋 점은 substep마다 같은 값이라 한 번만 만든다
    std::vector<Vec3> aimPoints;
    {
        const std::vector<Vec3> &refSamples = m_currentRoad->GetReferenceLine().GetSplinePoints();
        aimPoints.reserve(refSamples.size());
        for (size_t i = 0; i < refSamples.size(); ++i)
            aimPoints.push_back(OffsetSampleAt(refSamples, i, targetOffset));
    }
    size_t aimCursor = std::numeric_limits<size_t>::max();
    size_t nextCursor = std::numeric_limits<size_t>::max();

    int substeps = std::max(1, static_cast<int>(std::ceil(stepDistance / BIKE_SUBSTEP)));
    float substepDistance = stepDistance / static_cast<float>(substeps);

    // 현재 pose부터 자전거모델로만 적분, 스플라인 스냅 없음
    Vec3 bikePos = position;
    float bikeHeadingRad = DirectionToAngleRad(forward);
    float traveled = 0.0f;

    auto advance = [&](float distance)
    {
        bool onNextRoad = nextSpline != nullptr && traveled > currentRemain;
        Vec3 aim = onNextRoad
                       ? LookaheadOnPolyline(nextSpline->GetSplinePoints(), bikePos, STEER_LOOKAHEAD, false, nextCursor)
                       : LookaheadOnPolyline(aimPoints, bikePos, STEER_LOOKAHEAD, reversed, aimCursor);
        float steerAngle = std::clamp(PurePursuitSteerAt(bikePos, bikeHeadingRad, aim, m_wheelbase),
                                      -maxSteerAngle, maxSteerAngle);
        bikeHeadingRad -= distance * tanf(steerAngle) / m_wheelbase;
        bikePos += Vec3(cosf(bikeHeadingRad), 0.0f, sinf(bikeHeadingRad)) * distance;
        traveled += distance;
    };

    // 첫 박스가 내 차체와 안 겹치게 length(2*halfLength) 먼저 진행
    float leadLength = shape.halfLength * 2.0f;
    int leadSubsteps = std::max(1, static_cast<int>(std::ceil(leadLength / BIKE_SUBSTEP)));
    float leadDistance = leadLength / static_cast<float>(leadSubsteps);
    for (int s = 0; s < leadSubsteps; ++s)
        advance(leadDistance);

    m_sweepDebugCorners.reserve((sweepSteps + 1) * 4);

    for (int i = 0; i <= sweepSteps; ++i)
    {
        Vec3 point = bikePos;
        float headingRad = bikeHeadingRad;

        Vec3 boxFwd(cosf(headingRad), 0.0f, sinf(headingRad));
        Vec3 boxRight(boxFwd.GetZ(), 0.0f, -boxFwd.GetX());
        Vec3 boxCenter = point + boxFwd * shape.pivotToCenter;
        m_sweepDebugCorners.push_back(boxCenter + boxFwd * shape.halfLength - boxRight * shape.halfWidth);
        m_sweepDebugCorners.push_back(boxCenter + boxFwd * shape.halfLength + boxRight * shape.halfWidth);
        m_sweepDebugCorners.push_back(boxCenter - boxFwd * shape.halfLength + boxRight * shape.halfWidth);
        m_sweepDebugCorners.push_back(boxCenter - boxFwd * shape.halfLength - boxRight * shape.halfWidth);

        const VehicleCollision::Obstacle *hit = VehicleCollision::FindColliding(point, headingRad, m_obstacles, shape);
        if (hit != nullptr)
        {
            Car *leader = hit->sourceCar;
            if (leader != nullptr && !m_SimState->IsCarAlive(leader))
                leader = nullptr;

            Vec3 hitDir(cosf(hit->headingRad), 0.0f, sinf(hit->headingRad));
            float hitSpeed = std::max(0.0f, forward.Dot(hitDir) * hit->speed);

            // 뒷범퍼 중심 -> 리더 박스 위 가장 가까운 점 -> 그 점을 내 진행축에 투영해 보정
            Vec3 rearCenter = boxCenter - boxFwd * shape.halfLength;
            Vec3 closestOnLeader = VehicleCollision::ClosestPointOnObstacle(rearCenter, *hit);
            float t = (closestOnLeader - rearCenter).Dot(boxFwd);

            float distance = (traveled - 2.0f * shape.halfLength + t) - (leader == nullptr ? SAFE_GAP : 0.0f);

            RoadSpeedSample sample{point, distance, hitSpeed, leader, "sensorFront"};
            if (leader != nullptr)
                sample.leaderScanPosition = leader->GetPosition();
            sample.obstacle = *hit;
            sample.obstacle.sourceCar = leader; // 죽은 차 nullptr 처리
            sample.hasObstacle = true;
            samples.push_back(sample);
            return;
        }

        // 다음 박스 pose까지 잘게 적분
        for (int s = 0; s < substeps; ++s)
            advance(substepDistance);
    }
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

float Car::AvoidTargetOffset() const
{
    if (m_subMode == SubMode::D_Avoid || m_subMode == SubMode::D_SirenWait || m_subMode == SubMode::D_ChaseBlock)
        return m_maneuver.avoidOffset;
    if (m_subMode == SubMode::D_LaneChange)
        return m_maneuver.laneChangeTarget;
    return m_currentOffset;
}

float Car::SweepBodyPath(float targetOffset, const std::vector<VehicleCollision::Obstacle> &obstacles,
                         float speed, float maxDistance) const
{
    constexpr int AVOID_SWEEP_STEPS = 16;

    const Spline &referenceLine = m_currentRoad->GetReferenceLine();
    VehicleCollision::VehicleShape shape = BuildVehicleShape();

    bool reversed = m_travelDir == LaneDirection::Backward;
    Vec3 position = GetRigidbodyPosition();
    float headingRad = DirectionToAngleRad(GetForwardAxis());
    float maxSteerAngle = CalcMaxSteerAngle(speed);
    float lookahead = 5.0f;
    float offset = m_currentOffset;

    float stepDistance = maxDistance / AVOID_SWEEP_STEPS;
    float stepTime = (speed > 0.01f) ? stepDistance / speed : 0.0f;
    float lerpPerStep = m_personality.laneChangeLerpAlpha * stepTime / BEHAVIOR_PLAN_INTERVAL;

    for (int i = 0; i < AVOID_SWEEP_STEPS; ++i)
    {
        offset += (targetOffset - offset) * lerpPerStep;
        Vec3 aim = OffsetLookaheadPoint(referenceLine, position, lookahead, offset, reversed);
        float steerAngle = std::clamp(PurePursuitSteerAt(position, headingRad, aim, m_wheelbase),
                                      -maxSteerAngle, maxSteerAngle);
        headingRad -= stepDistance * tanf(steerAngle) / m_wheelbase;
        position += Vec3(cosf(headingRad), 0.0f, sinf(headingRad)) * stepDistance;

        if (VehicleCollision::IsColliding(position, headingRad, obstacles, shape))
            return stepDistance * static_cast<float>(i + 1);
    }
    return -1.0f;
}

float Car::DistanceToClearObstacle(const VehicleCollision::Obstacle &obstacle) const
{
    Vec3 forward = GetForwardAxis();
    Vec3 rearBumper = GetPosition() - forward * RearOverhang();
    float along = (obstacle.center - rearBumper).Dot(forward) + ObstacleHalfExtentAlong(obstacle, forward);
    return std::max(along + SAFE_GAP, 0.0f);
}

bool Car::SimulateAvoidPath(float targetOffset, const VehicleCollision::Obstacle &target,
                            const std::vector<VehicleCollision::Obstacle> &obstacles) const
{
    float distance = DistanceToClearObstacle(target);
    if (distance <= 0.0f)
        return true; // 이미 지나침

    float speed = std::max(m_speed, AVOID_SIM_MIN_SPEED);
    return SweepBodyPath(targetOffset, obstacles, speed, distance) < 0.0f;
}

bool Car::FindAvoidOffset(float laneCenter, const VehicleCollision::Obstacle &target, float &outOffset) const
{
    float minOffset = 0.0f;
    float maxOffset = 0.0f;
    ComputeDrivableRange(CurrentRoadRef(), minOffset, maxOffset);

    // Fast flee/chase vehicles clear only the obstacle width instead of shifting by lane-width steps.
    // ProjectObstacle accounts for obstacles rotated relative to the road.
    if (m_fleeOn || IsChasing())
    {
        float obstacleOffset = 0.0f;
        float obstacleHalfExtent = 0.0f;
        if (ProjectObstacle(m_currentRoad->GetReferenceLine(), target, obstacleOffset, obstacleHalfExtent))
        {
            float clearance = obstacleHalfExtent + GetHalfWidth() + MIN_SAFE_GAP;
            float candidates[] = {
                obstacleOffset - clearance,
                obstacleOffset + clearance,
            };

            // Try the side requiring the smaller lateral shift first.
            if (std::fabs(candidates[1] - m_currentOffset) < std::fabs(candidates[0] - m_currentOffset))
                std::swap(candidates[0], candidates[1]);

            for (float candidate : candidates)
            {
                candidate = std::clamp(candidate, minOffset, maxOffset);
                if (std::fabs(candidate - laneCenter) < AVOID_MIN_SHIFT)
                    continue;
                if (!SimulateAvoidPath(candidate, target, m_obstacles))
                    continue;
                outOffset = candidate;
                return true;
            }
        }
        return false;
    }

    float laneWidth = RoadDataManager::ROAD_WIDTH;
    if (const LaneBand *band = RoadDataManager::Get().FindNearestBand(m_currentRoad, laneCenter, m_travelDir))
        laneWidth = band->width;

    const float magnitudes[] = {laneWidth * 0.25f, laneWidth * 0.5f, laneWidth * 1.0f, laneWidth * 1.5f};

    bool leanPositive = m_currentOffset > 0.0f;
    for (float magnitude : magnitudes)
    {
        for (int side : {leanPositive ? 1 : -1, leanPositive ? -1 : 1})
        {
            float candidate = std::clamp(laneCenter + side * magnitude, minOffset, maxOffset);
            if (std::fabs(candidate - laneCenter) < AVOID_MIN_SHIFT)
                continue;
            if (!IsSafeLaneEntry(CurrentRoadRef(), candidate))
                continue;
            if (!SimulateAvoidPath(candidate, target, m_obstacles))
                continue;
            outOffset = candidate;
            return true;
        }
    }
    return false;
}

// 헤딩 기준 부채꼴을 슬롯으로 쪼개, 가고 싶은 방향(interest)에서 위험한 슬롯(danger)을 빼고 고른다.
// 매 프레임 돌며 연속적인 조향을 만든다 -- 이산 offset 후보를 고르는 D_Avoid와 달리 "막힘"이 없다.
float Car::ComputeContextSteer(const Vec3 &pursuitTarget, float pursuitSteer) const
{
    constexpr int SLOT_COUNT = 11;
    constexpr float FAN_HALF = ToRadians(60.0f);
    constexpr float TTC_HORIZON = 3.0f;     // 이 시간 밖 위협은 무시
    constexpr float PROXIMITY_RANGE = 6.0f; // 접근속도 0이어도 위험한 거리
    constexpr float DANGER_CUTOFF = 0.35f;  // 이 위 슬롯은 봉쇄로 본다
    constexpr float RIGHT_BIAS = 0.06f;     // 마주보기 교착 깨기
    constexpr float EDGE_FALLOFF = ToRadians(25.0f);
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

// 정적장애물은 Static 바디라 접촉 콜백이 없고, IDM이 닿기 전에 세운다.
// 그래서 "앞이 정적물이고 사실상 멈춰 있는 시간"으로 막힘을 판정한다.
void Car::TryStartStaticReverseEscape()
{
    if (m_currentLeader == nullptr || m_currentLeader->isVehicle || m_speed > STATIC_BLOCK_SPEED)
    {
        m_staticBlockTimer = 0.0f;
        return;
    }

    Vec3 forward = GetForwardAxis();
    Vec3 frontBumper = GetBodyCenter() + forward * m_halfExtents.GetZ();
    Vec3 toBlocker = VehicleCollision::ClosestPointOnObstacle(frontBumper, *m_currentLeader) - frontBumper;
    float gap = toBlocker.Length();

    bool blocked = gap <= STATIC_BLOCK_RANGE &&
                   (gap < 0.01f || forward.Dot(toBlocker * (1.0f / gap)) >= HEADON_FRONT_COS);
    if (!blocked)
    {
        m_staticBlockTimer = 0.0f;
        return;
    }

    m_staticBlockTimer += m_deltaTime;
    if (m_staticBlockTimer < STATIC_BLOCK_TRIGGER)
        return;

    m_staticBlockTimer = 0.0f;
    m_acceleration = 0.0f;
    m_speed = 0.0f;
    m_isReverse = true;
    m_reverseEscapeSteer = ComputeReverseEscapeSteer(m_currentLeader->center);
    m_reverseEscapeTimer = REVERSE_ESCAPE_TIME;
    DebugConsole::Log(GetName() + ": static block, reversing");
}

Car::ThreatKind Car::ClassifyFrontThreat() const
{
    if (m_currentLeader == nullptr)
        return ThreatKind::None;

    return m_currentLeader->isVehicle ? ThreatKind::Vehicle : ThreatKind::Static;
}

bool Car::IsChaseCarObstacle(const VehicleCollision::Obstacle &obstacle) const
{
    Car *car = obstacle.sourceCar;
    return car != nullptr && m_SimState->IsCarAlive(car) && car->IsChasing();
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
        if (m_currentLeader != nullptr && IsSameObstacle(*hit, *m_currentLeader))
            return;
        if (m_fleeOn && !IsChaseCarObstacle(*hit)) // 도망중엔 추격차만 급정지
            return;
        if (IsChasing()) // 추격중엔 급정지 안함
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

bool Car::IsOnLane(float offset) const
{
    if (m_currentRoad == nullptr)
        return false;
    if (m_subMode == SubMode::D_Avoid || m_subMode == SubMode::D_LaneChange)
        return false;
    return m_currentBand != nullptr && std::fabs(offset - m_currentBand->centerOffset) <= m_currentBand->width * 0.5f;
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

bool Car::HandleContactPending()
{
    if (m_reverseEscapeTimer > 0.0f) // 후진중엔 회피판단 중단, 조향은 DriveControl이
    {
        m_contactPending = false;
        return true;
    }

    if (!m_contactPending)
        return false;

    m_contactPending = false;
    if (m_subMode == SubMode::D_Avoid)
    {
        HandleAvoidStuck();
        return true;
    }
    return false;
}

void Car::UpdateAvoid()
{
    constexpr float AVOID_REPLAN_INTERVAL = 0.5f;
    bool fastAvoid = m_fleeOn || IsChasing();

    if (!fastAvoid)
        m_speedCap = LOW_SPEED; // Only ordinary cars slow down while shifting sideways.

    // Chain overtakes without returning to a lane between consecutive vehicles.
    if (fastAvoid && m_currentLeader != nullptr && m_currentLeader->isVehicle &&
        !IsSameObstacle(*m_currentLeader, m_maneuver.target))
    {
        float nextOffset = 0.0f;
        if (FindAvoidOffset(m_currentOffset, *m_currentLeader, nextOffset))
        {
            m_maneuver.avoidOffset = nextOffset;
            m_maneuver.target = *m_currentLeader;
            m_maneuver.hasTarget = true;
            m_maneuver.clearTimer = 0.0f;
            m_maneuver.lastPlanTime = m_currentTime;
        }
    }

    // A vehicle target moves, so keep the copied OBB current while overtaking it.
    Car *movingTarget = m_maneuver.target.sourceCar;
    if (movingTarget != nullptr && m_SimState->IsCarAlive(movingTarget))
        m_maneuver.target = movingTarget->MakeVehicleObstacle();

    if (m_currentTime - m_maneuver.lastPlanTime >= AVOID_REPLAN_INTERVAL)
    {
        m_maneuver.lastPlanTime = m_currentTime;
        bool laneEntrySafe = fastAvoid || IsSafeLaneEntry(CurrentRoadRef(), m_maneuver.avoidOffset);
        bool stillSafe = laneEntrySafe &&
                         SimulateAvoidPath(m_maneuver.avoidOffset, m_maneuver.target, m_obstacles);
        if (stillSafe)
        {
            m_stuck = false;
        }
        else
        {
            float replanOffset = 0.0f;
            if (FindAvoidOffset(m_maneuver.laneOffset, m_maneuver.target, replanOffset))
            {
                m_maneuver.avoidOffset = replanOffset;
                m_stuck = false;
            }
            else
            {
                HandleAvoidStuck();
            }
        }
    }

    bool returnSafe = fastAvoid || IsSafeLaneEntry(CurrentRoadRef(), m_maneuver.laneOffset);
    bool clear = HasPassedAvoidTarget() && returnSafe;
    m_maneuver.clearTimer = clear ? m_maneuver.clearTimer + m_deltaTime : 0.0f;

    if (m_maneuver.clearTimer >= AVOID_CLEAR_DELAY)
    {
        m_staticBlockTimer = 0.0f;
        m_stuck = false;
        m_maneuver = ManeuverState{};
        SetSubMode(SubMode::D_Normal);
    }
}

void Car::UpdateLaneChange()
{
    float total = m_maneuver.laneChangeTarget - m_maneuver.laneOffset;
    float progress = std::fabs(total) > 0.0001f ? (m_currentOffset - m_maneuver.laneOffset) / total : 1.0f;

    const Spline &referenceLine = m_currentRoad->GetReferenceLine();
    bool reversed = m_travelDir == LaneDirection::Backward;
    Vec3 rigidPosition = GetRigidbodyPosition();
    float headingRad = DirectionToAngleRad(GetForwardAxis());
    VehicleCollision::VehicleShape shape = BuildVehicleShape();

    // 옆+옆앞 두 지점 검사
    Vec3 sidePivot = OffsetLookaheadPoint(referenceLine, rigidPosition, 0.0f, m_maneuver.laneChangeTarget, reversed);
    Vec3 sideFrontPivot = OffsetLookaheadPoint(referenceLine, rigidPosition, GetLength(), m_maneuver.laneChangeTarget, reversed);
    bool sideBlocked = VehicleCollision::IsColliding(sidePivot, headingRad, m_obstacles, shape) ||
                       VehicleCollision::IsColliding(sideFrontPivot, headingRad, m_obstacles, shape);
    bool unsafe = !IsSafeLaneEntry(CurrentRoadRef(), m_maneuver.laneChangeTarget);

    if (progress < 0.3f && (unsafe || sideBlocked))
    {
        SetCurrentOffset(m_maneuver.laneOffset);
        m_maneuver.laneChangeTarget = m_maneuver.laneOffset;
        m_currentSpline = RoadDataManager::Get().BuildOffsetSpline(m_currentRoad, m_currentOffset, m_travelDir);
        RebuildSplineRender();
    }

    bool arrived = std::fabs(m_currentOffset - m_maneuver.laneChangeTarget) < AVOID_RETURN_TOLERANCE;
    if (arrived)
    {
        m_lastLaneChangeTime = m_currentTime; // 쿨다운은 완료 시점부터
        m_staticBlockTimer = 0.0f;
        m_stuck = false;
        m_maneuver = ManeuverState{};
        m_speedCap = -1.0f;
        SetSubMode(SubMode::D_Normal);
    }
}

bool Car::HasSirenCarBehind() const
{
    if (m_currentRoad == nullptr)
        return false;
    if (m_fleeOn) // 도망차는 양보 안함
        return false;

    const Spline &ref = m_currentRoad->GetReferenceLine();
    float dirSign = TravelSign();
    float myS = TravelS(ref, GetPosition(), dirSign);

    for (Car *other : m_nearbyCars)
    {
        if (!other->m_sirenOn || !m_SimState->IsCarAlive(other))
            continue;
        if ((other->GetPosition() - GetPosition()).Length() > SIREN_DETECT_RADIUS)
            continue;

        if (other->m_currentRoad == m_currentRoad)
        {
            if (TravelS(ref, other->GetPosition(), dirSign) < myS)
                return true;
            continue;
        }

        // 직전 도로 위의 사이렌차는 항상 내 뒤
        if (m_pathIndex > 0 && m_path[m_pathIndex - 1].road == other->m_currentRoad)
            return true;
    }
    return false;
}

float Car::ComputeSirenStopOffset() const
{
    std::vector<const LaneBand *> bands = RoadDataManager::Get().GetDrivingBands(m_currentRoad, m_travelDir);
    if (bands.empty())
        return m_currentOffset;

    // centerOffset 오름차순 정렬됨: Forward는 마지막이, Backward는 첫번째가 참조선기준 최우측
    bool forward = m_travelDir == LaneDirection::Forward;
    const LaneBand *rightmost = forward ? bands.back() : bands.front();
    float edge = forward ? (rightmost->centerOffset + rightmost->width * 0.5f)
                         : (rightmost->centerOffset - rightmost->width * 0.5f);

    float minOffset = 0.0f;
    float maxOffset = 0.0f;
    ComputeDrivableRange(CurrentRoadRef(), minOffset, maxOffset);
    edge = std::clamp(edge, minOffset, maxOffset);

    // 오른쪽 끝 vs 참조선(0) 중 지금 위치에서 가까운 쪽
    return std::fabs(m_currentOffset - edge) < std::fabs(m_currentOffset) ? edge : 0.0f;
}

bool Car::UpdateSirenWait()
{
    constexpr float SIREN_PULLOVER_MAX_TIME = 5.0f; // 못 붙어도 정차

    bool exempt = m_reservedJunctionId >= 0; // 교차로 통과중은 예외
    if (!exempt && HasSirenCarBehind())
    {
        if (m_subMode != SubMode::D_SirenWait)
        {
            m_maneuver = ManeuverState{};
            m_maneuver.avoidOffset = ComputeSirenStopOffset();
            m_maneuver.lastPlanTime = m_currentTime;
            m_sirenPulledOver = false;
            SetSubMode(SubMode::D_SirenWait);
        }

        // 계획 d는 정차중에도 수렴해 실측으로 판정
        float realOffset = ComputeReferenceOffset(m_currentRoad->GetReferenceLine(), GetPosition());
        if (std::fabs(realOffset - m_maneuver.avoidOffset) < AVOID_RETURN_TOLERANCE ||
            m_currentTime - m_maneuver.lastPlanTime > SIREN_PULLOVER_MAX_TIME)
            m_sirenPulledOver = true;

        if (!m_sirenPulledOver)
            m_speedCap = LOW_SPEED; // 붙는 동안 서행
        return true;
    }

    if (m_subMode == SubMode::D_SirenWait)
    {
        m_maneuver = ManeuverState{};
        m_sirenPulledOver = false;
        SetSubMode(SubMode::D_Normal);
    }
    return false;
}

Car *Car::FindFleeTarget() const
{
    Car *best = nullptr;
    float bestDistance = CHASE_DETECT_RADIUS;

    for (Car *other : m_nearbyCars)
    {
        if (!other->m_fleeOn || !m_SimState->IsCarAlive(other))
            continue;
        if (other->m_currentRoad == nullptr)
            continue;

        float distance = (other->GetPosition() - GetPosition()).Length();
        if (distance < bestDistance)
        {
            bestDistance = distance;
            best = other;
        }
    }
    return best;
}

bool Car::BuildChasePath(Car *target)
{
    // 목적지 주행중이면 경로를 뺏지 않는다
    if (!m_roaming || m_currentRoad == nullptr)
        return false;

    std::vector<RoadRef> path = RoadDataManager::Get().FindPath(CurrentRoadRef(), target->m_currentRoad);
    if (path.empty())
        return false;

    m_path = std::move(path);
    m_pathIndex = 0;
    return true;
}

bool Car::IsAheadOfFleeTarget(const Car *target) const
{
    if (m_currentRoad == nullptr || target->m_currentRoad != m_currentRoad || target->m_travelDir != m_travelDir)
        return false;
    if ((target->GetPosition() - GetPosition()).Length() > CHASE_BLOCK_RANGE)
        return false;

    const Spline &ref = m_currentRoad->GetReferenceLine();
    float dirSign = TravelSign();
    float lead = TravelS(ref, GetPosition(), dirSign) - TravelS(ref, target->GetPosition(), dirSign);
    return lead > RearOverhang() + target->FrontOverhang() + MIN_SAFE_GAP;
}

float Car::ComputeChaseBlockOffset(const Car *target) const
{
    float minOffset = 0.0f;
    float maxOffset = 0.0f;
    ComputeDrivableRange(CurrentRoadRef(), minOffset, maxOffset);
    return std::clamp(target->CurrentLaneCenter(), minOffset, maxOffset);
}

void Car::ClearChase()
{
    m_chaseTarget = nullptr;
    m_chaseTargetRoadId = -1;
    if (m_chaseSiren) // 내가 켠 것만 끈다
    {
        m_chaseSiren = false;
        SetSirenOn(false);
    }
    if (m_subMode == SubMode::D_ChaseBlock)
    {
        m_maneuver = ManeuverState{};
        SetSubMode(SubMode::D_Normal);
    }
}

bool Car::UpdateChase()
{
    if (!m_chaseOn)
    {
        ClearChase();
        return false;
    }

    Car *target = FindFleeTarget();
    if (target == nullptr)
    {
        ClearChase();
        return false;
    }

    if (!m_sirenOn) // 쫓는 차가 있으면 사이렌
    {
        m_chaseSiren = true;
        SetSirenOn(true);
    }

    int targetRoadId = target->m_currentRoad->GetId();
    if (target != m_chaseTarget || targetRoadId != m_chaseTargetRoadId ||
        m_currentTime - m_lastChasePlanTime > CHASE_REPATH_INTERVAL)
    {
        BuildChasePath(target); // 실패해도 다음 주기까지 대기
        m_chaseTarget = target;
        m_chaseTargetRoadId = targetRoadId;
        m_lastChasePlanTime = m_currentTime;
    }

    // 아직 못 앞질렀으면 평소 주행으로 따라붙는다
    if (!IsAheadOfFleeTarget(target))
    {
        if (m_subMode == SubMode::D_ChaseBlock)
        {
            m_maneuver = ManeuverState{};
            SetSubMode(SubMode::D_Normal);
        }
        return false;
    }

    if (m_subMode != SubMode::D_ChaseBlock)
    {
        m_maneuver = ManeuverState{};
        SetSubMode(SubMode::D_ChaseBlock);
    }
    m_maneuver.avoidOffset = ComputeChaseBlockOffset(target);
    return true;
}

void Car::DecideAvoidance()
{
    constexpr float AVOID_TRIGGER_DELAY = 0.6f;

    ThreatKind threat = ClassifyFrontThreat();
    bool fastOvertake = m_fleeOn || IsChasing();
    bool canOffsetAvoid = threat == ThreatKind::Static ||
                          (fastOvertake && threat == ThreatKind::Vehicle);

    if (!canOffsetAvoid)
    {
        m_staticBlockTimer = 0.0f;
        m_stuck = false;
        return;
    }

    m_staticBlockTimer += m_deltaTime;
    float triggerDelay = (fastOvertake && threat == ThreatKind::Vehicle) ? 0.0f : AVOID_TRIGGER_DELAY;
    if (m_staticBlockTimer < triggerDelay)
    {
        m_stuck = false;
        return;
    }

    float laneCenter = CurrentLaneCenter();
    float avoidOffset = 0.0f;
    if (FindAvoidOffset(laneCenter, *m_currentLeader, avoidOffset))
    {
        m_maneuver.laneOffset = laneCenter;
        m_maneuver.avoidOffset = avoidOffset;
        m_maneuver.lastPlanTime = m_currentTime;
        m_maneuver.target = *m_currentLeader; // 샘플 버퍼는 매틱 재생성, 값복사
        m_maneuver.hasTarget = true;
        m_stuck = false;
        if (!m_fleeOn && !IsChasing())
            m_speedCap = LOW_SPEED;
        SetSubMode(SubMode::D_Avoid);
        return;
    }

    HandleAvoidStuck();
}

void Car::HandleAvoidStuck()
{
    m_stuck = true;
}

bool Car::HasPassedAvoidTarget() const
{
    if (!m_maneuver.hasTarget)
        return true;

    // 치워졌으면 지나친 것으로
    bool stillThere = std::any_of(m_obstacles.begin(), m_obstacles.end(),
                                  [this](const VehicleCollision::Obstacle &obstacle)
                                  { return IsSameObstacle(obstacle, m_maneuver.target); });
    if (!stillThere)
        return true;

    return DistanceToClearObstacle(m_maneuver.target) <= 0.0f;
}

#pragma endregion
