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
#include <cstring>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace
{
    constexpr float SAME_DIRECTION_COS = 0.7071f;

    constexpr float VERTICAL_SEPARATION = 3.0f;

    float NearestBandOffset(const RoadRef &road, float d)
    {
        const LaneBand *band = RoadDataManager::Get().FindNearestBand(road.road, d, road.direction);
        return band != nullptr ? band->centerOffset : d;
    }

    float TravelS(const Spline &referenceLine, const Vec3 &position, float dirSign)
    {
        return referenceLine.GetSplinePosition(position) * referenceLine.GetLength() * dirSign;
    }

    bool ProjectObstacle(const Spline &referenceLine, const VehicleCollision::Obstacle &obstacle,
                         float &outOffset, float &outHalfExtent)
    {
        float t = referenceLine.GetSplinePosition(obstacle.center);
        Vec3 onRef = referenceLine.GetPositionAt(t);
        Vec3 dir = referenceLine.GetDirectionAt(t);
        Vec3 rightN(dir.GetZ(), 0.0f, -dir.GetX());
        outOffset = (obstacle.center - onRef).Dot(rightN);

        Vec3 forward(cosf(obstacle.headingRad), 0.0f, sinf(obstacle.headingRad));
        Vec3 right(forward.GetZ(), 0.0f, -forward.GetX());
        outHalfExtent = std::fabs(rightN.Dot(right)) * obstacle.halfWidth +
                        std::fabs(rightN.Dot(forward)) * obstacle.halfLength;

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
        BeginSegment(std::make_unique<SplineFollowSegment>());
    }
    else if (m_mode == Mode::Park)
    {
        SetSubMode(prev == Mode::Stop ? SubMode::P_EXIT : SubMode::P_ENTER_LEG1);
        m_parkPlanPending = true;
        m_parkSequenceActive = true;
        DebugConsole::Log(GetName() + ": Park plan pending: waiting for full stop before planning");
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

    DebugConsole::Log(GetName() + ": lot cruise -> spot " + std::to_string(spot->id) + " via road " +
                      std::to_string(m_path[0].road->GetId()) + " .. " + std::to_string(m_destRoad->GetId()));
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

    DebugConsole::Log(GetName() + ": lot exit -> road " + std::to_string(exitAisle.road->GetId()) + " -> " +
                      std::to_string(exitRoad.road->GetId()) + " .. dest " + std::to_string(m_destRoad->GetId()));
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
        DebugConsole::Log(GetName() + ": Park plan pending resolved: fully stopped, beginning RS plan");
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
            DebugConsole::Log(GetName() + ": BeginParkPlan: no road found to exit onto, abandoning this park attempt");
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

        DebugConsole::Log(GetName() + ": BeginParkPlan: RS exit failed for all lead distances, driving as-is");
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

                DebugConsole::Log(GetName() + ": Park spot reservation failed for node " + std::to_string(parkNodeId) +
                                  ": all ParkSpot children reserved, abandoning destination");
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
            DebugConsole::Log(GetName() + ": BeginParkPlan: no reachable ParkSpot for node " + std::to_string(parkNodeId) +
                              ", abandoning destination");
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
                DebugConsole::Log(GetName() + ": PlanEnterForCurrentSpot: P unreachable, detouring via aisle entry point");
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
        DebugConsole::Log(GetName() + ": PlanEnterForCurrentSpot: direct park target too far (" +
                          std::to_string((spotTarget - m_rigidbody.GetPosition()).Length()) + "m), giving up");
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

    DebugConsole::Log(GetName() + ": BeginParkSpotLeg: can't tuck into ParkSpot " + std::to_string(m_parkSpot->id) +
                      " from P, trying next spot");

    if (ReserveNextParkSpot() && BeginParkEnterOrRetry())
        return;

    DebugConsole::Log(GetName() + ": BeginParkSpotLeg: no reachable ParkSpot left, stopping");
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

    if (!HandleContactPending())
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
        DebugConsole::Log(GetName() + ": lane missed -> reroute (roaming) via road " + ToString(m_path[m_pathIndex + 1].road->GetId()));
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

    DebugConsole::Log(GetName() + ": lane missed -> reroute via road " + ToString(best.front().road->GetId()));
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
    const Spline &spline = m_currentSpline;

    constexpr float LOOKAHEAD_TIME = 1.5f;
    float lookaheadDistance = 5.0f;
    Vec3 target = spline.GetLookaheadPoint(GetRigidbodyPosition(), lookaheadDistance);
    float targetSteer = PurePursuit(target);

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
    if (m_speed > steerSpeedCap && -m_maxBrake < accelIDM)
    {
        accelIDM = -m_maxBrake;
        m_limitDebug = SpeedLimitDebug{"steerCap", steerSpeedCap, 0.0f};
    }

    Accelerate(accelIDM);

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
        float currentNodeSpeed = (m_currentRoad == m_destRoad) ? 0.0f : std::min(m_currentRoad->GetSpeedLimit(), m_maxSpeed);
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

            float nextNodeSpeed = std::min(nextRoad.road->GetSpeedLimit(), m_maxSpeed);
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

    std::vector<const LaneBand *> bands = RoadDataManager::Get().GetDrivingBands(road.road, road.direction);
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
    idm.v0 = std::min(road->GetSpeedLimit() * m_personality.speedFactor, m_maxSpeed);
    idm.T = IDM_TIME_HEADWAY * m_personality.headwayFactor;
    idm.s0 = SAFE_GAP * m_personality.headwayFactor;
    idm.a = m_maxAccel;
    idm.b = m_maxBrake * m_personality.brakeFactor;
    idm.delta = 4.0f;
    idm.coolness = 1.0f;
    return idm;
}

float Car::ComputeIdmAcceleration(const std::vector<RoadSpeedSample> &samples, const IDM::Params &params,
                                  float distanceOffset, float elapsedTime,
                                  const VehicleCollision::Obstacle **outLeader,
                                  SpeedLimitDebug *outDebug) const
{

    float speedRatio = std::min(1.0f, m_speed / std::max(0.1f, params.v0));
    float bestAccel = params.a * (1.0f - std::pow(speedRatio, params.delta));
    if (outLeader != nullptr)
        *outLeader = nullptr;
    if (outDebug != nullptr)
        *outDebug = SpeedLimitDebug{"free", params.v0, 0.0f};

    for (const RoadSpeedSample &sample : samples)
    {
        bool isSensorFront = std::strcmp(sample.debugStr, "sensorFront") == 0;

        if (sample.leader != nullptr && !m_SimState->IsCarAlive(sample.leader))
        {
            if (isSensorFront)
                DebugConsole::Log(GetName() + ": [IDM] sensorFront leader dead, skipped");
            continue;
        }

        // 서로를 리더로 잡고 둘 다 서는 교착
        if (sample.leader != nullptr && sample.leader->GetIdmLeader() == this)
        {
            if (isSensorFront)
                DebugConsole::Log(GetName() + ": [IDM] sensorFront mutual-leader deadlock with " + sample.leader->GetName() + ", skipped");
            continue;
        }

        float leaderSpeed;
        float leaderAccel;
        float gap;
        if (sample.leader != nullptr)
        {

            leaderSpeed = sample.leader->GetSpeed();
            leaderAccel = sample.leader->GetAcceleration();

            float leaderTravel = (sample.leader->GetPosition() - sample.leaderScanPosition)
                                     .Dot(sample.leader->GetForwardAxis());
            gap = sample.distance - distanceOffset + leaderTravel;
        }
        else
        {

            float remaining = sample.distance - distanceOffset + sample.speed * elapsedTime;
            bool hardStop = sample.speed <= 0.0f;
            if (!hardStop && (remaining <= 0.0f || m_speed <= sample.speed))
            {
                if (isSensorFront)
                    DebugConsole::Log(GetName() + ": [IDM] sensorFront static hit outrun/cleared, skipped (remaining " +
                                      ToString(remaining) + ", mySpeed " + ToString(m_speed) + ", hitSpeed " + ToString(sample.speed) + ")");
                continue;
            }

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
        else if (isSensorFront)
        {
            DebugConsole::Log(GetName() + ": [IDM] sensorFront a=" + ToString(a) + " gap=" + ToString(gap) +
                              " not binding (best " + ToString(bestAccel) + ")");
        }
    }
    return bestAccel;
}

Car::LaneNeighbors Car::GatherLaneNeighbors(const shared_ptr<Road> &road, const Spline &refLine, float bandCenter,
                                            float bandHalfWidth, float myS, float dirSign) const
{
    LaneNeighbors out;
    float bestLeaderGap = std::numeric_limits<float>::max();
    float bestFollowerGap = std::numeric_limits<float>::max();

    for (const VehicleCollision::Obstacle &obstacle : m_obstacles)
    {
        float d = 0.0f;
        float halfExtent = 0.0f;
        if (!ProjectObstacle(refLine, obstacle, d, halfExtent))
            continue;
        if (std::fabs(d - bandCenter) > bandHalfWidth + halfExtent)
            continue;

        Mobil::VehicleState st;
        Car *other = obstacle.sourceCar;
        if (other != nullptr)
        {
            if (!m_SimState->IsCarAlive(other) || other->m_currentRoad != road)
                continue;

            float otherT = refLine.GetSplinePosition(other->GetPosition());
            float dirDot = refLine.GetDirectionAt(otherT).Dot(other->GetForwardAxis()) * dirSign;

            float sign = dirDot < 0.0f ? -1.0f : 1.0f;
            st.speed = other->GetSpeed() * sign;
            st.accel = other->GetAcceleration() * sign;
            st.position = TravelS(refLine, other->GetPosition(), dirSign);
            st.length = other->GetLength();
        }
        else
        {
            st.speed = obstacle.speed;
            st.accel = 0.0f;
            st.position = TravelS(refLine, obstacle.center, dirSign);
            st.length = obstacle.halfLength * 2.0f;
        }

        if (st.position >= myS)
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
    if (nextRoad.road == nullptr)
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
        return false;

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

    float laneCenter = m_currentOffset;
    float targetOffset;
    if (m_subMode == SubMode::D_Avoid || m_subMode == SubMode::D_LaneChange)
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
            DebugConsole::Log(GetName() + ": lane change d " + ToString(laneCenter) + " -> " + ToString(targetOffset) +
                              " (" + reason + ")");
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

    std::vector<const LaneBand *> bands = RoadDataManager::Get().GetDrivingBands(m_currentRoad, m_travelDir);
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
        const Mobil::VehicleState &newLeader = nbr.hasLeader ? nbr.leader : farLeader;
        const Mobil::VehicleState *newFollower = nbr.hasFollower ? &nbr.follower : nullptr;
        if (Mobil::EvaluateLaneChange(myState, oldFollower, myLeader, newLeader, newFollower, mobil, idm, bias))
        {
            if (outReason != nullptr)
                *outReason = (goal.active && bias > 0.0f) ? "route" : "overtake";
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
    float stepDistance = GetLength() - MIN_SAFE_GAP;
    int sweepSteps = std::max(static_cast<int>(maxDistance / stepDistance), 8);

    VehicleCollision::VehicleShape shape = BuildVehicleShape();
    Vec3 forward = GetForwardAxis();
    Vec3 position = GetRigidbodyPosition();

    float currentRemain = m_currentSpline.GetLength() * (1.0f - m_currentSpline.GetSplinePosition(position));

    Spline nextSplineStorage;
    const Spline *nextSpline = nullptr;
    Vec3 nextSplineStart = Vec3::sZero();
    if (currentRemain < maxDistance && m_pathIndex + 1 < m_path.size())
    {
        const RoadRef &next = m_path[m_pathIndex + 1];
        if (next.road != nullptr)
        {
            float nextOffset = RoadDataManager::Get().ResolveConnectingOffset(CurrentRoadRef(), next, m_currentOffset, nullptr);
            nextSplineStorage = RoadDataManager::Get().BuildOffsetSpline(next.road, nextOffset, next.direction);
            if (!nextSplineStorage.GetSplinePoints().empty())
            {
                nextSpline = &nextSplineStorage;
                nextSplineStart = nextSplineStorage.GetSplinePoints().front();
            }
        }
    }

    constexpr float STEER_LOOKAHEAD = 2.0f;

    float realOffset = ComputeReferenceOffset(m_currentRoad->GetReferenceLine(), GetPosition());
    bool onLane = IsOnLane(realOffset);

    bool reversed = m_travelDir == LaneDirection::Backward;
    float targetOffset = AvoidTargetOffset();
    float maxSteerAngle = CalcMaxSteerAngle(m_speed);
    Vec3 bikePos = position;
    float bikeHeadingRad = DirectionToAngleRad(forward);
    bool useBike = !onLane;
    Vec3 splineAnchor = position;
    float anchorDistance = 0.0f;

    m_sweepDebugCorners.reserve(sweepSteps * 4);

    for (int i = 1; i <= sweepSteps; ++i)
    {
        float d = stepDistance * static_cast<float>(i);
        Vec3 point;
        float headingRad;
        if (useBike)
        {
            Vec3 aim = OffsetLookaheadPoint(m_currentRoad->GetReferenceLine(), bikePos, STEER_LOOKAHEAD, targetOffset, reversed);
            float steerAngle = std::clamp(PurePursuitSteerAt(bikePos, bikeHeadingRad, aim, m_wheelbase),
                                          -maxSteerAngle, maxSteerAngle);
            bikeHeadingRad -= stepDistance * tanf(steerAngle) / m_wheelbase;
            bikePos += Vec3(cosf(bikeHeadingRad), 0.0f, sinf(bikeHeadingRad)) * stepDistance;
            point = bikePos;
            headingRad = bikeHeadingRad;
            if ((aim - bikePos).Length() <= STEER_LOOKAHEAD)
            {
                useBike = false;
                splineAnchor = bikePos;
                anchorDistance = d;
            }
        }
        else
        {
            float remain = d - anchorDistance;
            bool onNextRoad = nextSpline != nullptr && d > currentRemain;
            const Spline &activeSpline = onNextRoad ? *nextSpline : m_currentSpline;
            point = onNextRoad ? nextSpline->GetLookaheadPoint(nextSplineStart, d - currentRemain)
                               : m_currentSpline.GetLookaheadPoint(splineAnchor, remain);
            headingRad = DirectionToAngleRad(activeSpline.GetDirectionAt(activeSpline.GetSplinePosition(point)));
        }

        Vec3 boxFwd(cosf(headingRad), 0.0f, sinf(headingRad));
        Vec3 boxRight(boxFwd.GetZ(), 0.0f, -boxFwd.GetX());
        Vec3 boxCenter = point + boxFwd * shape.pivotToCenter;
        m_sweepDebugCorners.push_back(boxCenter + boxFwd * shape.halfLength - boxRight * shape.halfWidth);
        m_sweepDebugCorners.push_back(boxCenter + boxFwd * shape.halfLength + boxRight * shape.halfWidth);
        m_sweepDebugCorners.push_back(boxCenter - boxFwd * shape.halfLength + boxRight * shape.halfWidth);
        m_sweepDebugCorners.push_back(boxCenter - boxFwd * shape.halfLength - boxRight * shape.halfWidth);

        const VehicleCollision::Obstacle *hit = VehicleCollision::FindColliding(point, headingRad, m_obstacles, shape);
        if (hit == nullptr)
            continue;

        Car *leader = hit->sourceCar;
        if (leader != nullptr && !m_SimState->IsCarAlive(leader))
            leader = nullptr;

        Vec3 hitDir(cosf(hit->headingRad), 0.0f, sinf(hit->headingRad));
        float hitSpeed = std::max(0.0f, forward.Dot(hitDir) * hit->speed);
        float distance = d - (leader == nullptr ? SAFE_GAP : 0.0f);

        RoadSpeedSample sample{point, distance, hitSpeed, leader, "sensorFront"};
        if (leader != nullptr)
            sample.leaderScanPosition = leader->GetPosition();
        sample.obstacle = *hit;
        sample.obstacle.sourceCar = leader; // 죽은 차 nullptr 처리
        sample.hasObstacle = true;
        samples.push_back(sample);
        return;
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
    if (m_subMode == SubMode::D_Avoid)
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

bool Car::SimulateAvoidPath(float targetOffset, const std::vector<VehicleCollision::Obstacle> &obstacles) const
{
    float speed = std::max(m_speed, AVOID_SIM_MIN_SPEED);
    return SweepBodyPath(targetOffset, obstacles, speed, speed * AVOID_SIM_TIME) < 0.0f;
}

bool Car::FindAvoidOffset(float laneCenter, float &outOffset) const
{
    float laneWidth = RoadDataManager::ROAD_WIDTH;
    if (const LaneBand *band = RoadDataManager::Get().FindNearestBand(m_currentRoad, laneCenter, m_travelDir))
        laneWidth = band->width;

    float minOffset = 0.0f;
    float maxOffset = 0.0f;
    ComputeDrivableRange(CurrentRoadRef(), minOffset, maxOffset);

    const float magnitudes[] = {laneWidth * 0.25f, laneWidth * 0.5f, laneWidth * 1.0f};

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
            if (!SimulateAvoidPath(candidate, m_obstacles))
                continue;
            outOffset = candidate;
            return true;
        }
    }
    return false;
}

Car::ThreatKind Car::ClassifyFrontThreat() const
{
    if (m_currentLeader == nullptr)
        return ThreatKind::None;

    return m_currentLeader->isVehicle ? ThreatKind::Vehicle : ThreatKind::Static;
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

    m_speedCap = -1.0f;
}

bool Car::HandleContactPending()
{
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

    m_speedCap = AVOID_LOW_SPEED;

    if (m_currentTime - m_maneuver.lastPlanTime >= AVOID_REPLAN_INTERVAL)
    {
        bool stillSafe = IsSafeLaneEntry(CurrentRoadRef(), m_maneuver.avoidOffset) &&
                         SimulateAvoidPath(m_maneuver.avoidOffset, m_obstacles);
        if (!stillSafe)
        {
            m_maneuver.lastPlanTime = m_currentTime;
            float replanOffset = 0.0f;
            if (FindAvoidOffset(m_maneuver.laneOffset, replanOffset))
            {
                m_maneuver.avoidOffset = replanOffset;
            }
            else
            {
                HandleAvoidStuck();
            }
        }
    }

    bool clear = IsSafeLaneEntry(CurrentRoadRef(), m_maneuver.laneOffset) &&
                 SimulateAvoidPath(m_maneuver.laneOffset, m_obstacles);
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

    if (sideBlocked && progress < 0.3f)
    {
        DebugConsole::Log(GetName() + ": lane change canceled, side blocked at d " + ToString(m_currentOffset));
        SetCurrentOffset(m_maneuver.laneOffset);
        m_maneuver.laneChangeTarget = m_maneuver.laneOffset;
        m_currentSpline = RoadDataManager::Get().BuildOffsetSpline(m_currentRoad, m_currentOffset, m_travelDir);
        RebuildSplineRender();
    }

    bool arrived = std::fabs(m_currentOffset - m_maneuver.laneChangeTarget) < AVOID_RETURN_TOLERANCE;
    if (arrived)
    {
        DebugConsole::Log(GetName() + ": lane change done at d " + ToString(m_currentOffset));
        m_lastLaneChangeTime = m_currentTime; // 쿨다운은 완료 시점부터
        m_staticBlockTimer = 0.0f;
        m_stuck = false;
        m_maneuver = ManeuverState{};
        m_speedCap = -1.0f;
        SetSubMode(SubMode::D_Normal);
    }
}

void Car::DecideAvoidance()
{
    constexpr float AVOID_TRIGGER_DELAY = 0.6f;

    ThreatKind threat = ClassifyFrontThreat();

    if (threat != ThreatKind::Static)
    {
        m_staticBlockTimer = 0.0f;
        m_stuck = false;
        return;
    }

    m_staticBlockTimer += m_deltaTime;
    if (m_staticBlockTimer < AVOID_TRIGGER_DELAY)
    {
        m_stuck = false;
        return;
    }

    float laneCenter = CurrentLaneCenter();
    float avoidOffset = 0.0f;
    if (FindAvoidOffset(laneCenter, avoidOffset))
    {
        m_maneuver.laneOffset = laneCenter;
        m_maneuver.avoidOffset = avoidOffset;
        m_maneuver.lastPlanTime = m_currentTime;
        m_speedCap = AVOID_LOW_SPEED;
        SetSubMode(SubMode::D_Avoid);
        return;
    }

    HandleAvoidStuck();
}

void Car::HandleAvoidStuck()
{
    m_stuck = true;
    m_speedCap = 0.0f;
}

#pragma endregion
