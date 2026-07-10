#include "Entities/Car.h"
#include "Core/DebugConsole.h"
#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

namespace
{
    // 동적으로 스플라인을 이어붙일 때 양끝 기울기 점을 이 거리 미만으로 붙이면 기존 스플라인의
    // 기울기(진행 방향)를 제대로 반영하지 못하고 너무 꺾여 보인다.
    constexpr float MIN_TANGENT_DISTANCE = 5.0f;
} // namespace

std::unique_ptr<BTNode> Car::BuildBehaviourTree()
{
    return MakeSequence(
        FindPathNode(),
        MakeSelector(
            StopNode(),
            DriveNode()));
}

std::unique_ptr<BTNode> Car::FindPathNode()
{
    // 경로 (재)탐색 조건
    auto notSearch = std::make_unique<BTCondition>(
        [this]()
        {
            if (m_destEdge == nullptr)
                return true;
            if (m_currentEdge == nullptr)
                return false;
            if (IsOffCourse())
                return false;
            return true;
        });
    notSearch->name = "NotSearch?";

    // 경로를 탐색
    auto searchPath = std::make_unique<BTAction>(
        [this]()
        {
            if (m_currentEdge != nullptr)
                return BTStatus::Success;

            Vec3 position = GetPosition();
            m_currentEdge = m_RoadDataManager->GetClosestNode(position);
            m_path = m_RoadDataManager->FindPath(m_currentEdge->endNode.lock(), m_destEdge->endNode.lock());
            m_pathIndex = -1;
            if (m_path.empty())
            {
                m_destEdge = nullptr;
                m_currentEdge = nullptr;
                return BTStatus::Success;
            }

            shared_ptr<RoadEdge> firstEdge = m_path[0];
            float splinePosition = firstEdge->spline.GetSplinePosition(position);
            Vec3 edgePosition = firstEdge->spline.GetPositionAt(splinePosition);
            Vec3 splineDirection = firstEdge->spline.GetDirectionAt(splinePosition);

            m_currentSpline = Spline({position - GetForwardAxis() * MIN_TANGENT_DISTANCE,
                                      position,
                                      edgePosition,
                                      edgePosition + splineDirection * MIN_TANGENT_DISTANCE});
            RebuildSplineRender();
            CalculateSpeedProfile();
            return BTStatus::Success;
        });
    searchPath->name = "SearchPath";

    return MakeSelector(std::move(notSearch), std::move(searchPath));
}

std::unique_ptr<BTNode> Car::StopNode()
{
    // 정지 조건 : 목적지가 없거나 도착
    auto shouldStop = std::make_unique<BTCondition>(
        [this]()
        {
            constexpr float ARRIVE_DISTANCE = 5.0f;
            return m_destEdge == nullptr || (m_destEdge->endNode.lock()->position - GetPosition()).Length() < ARRIVE_DISTANCE;
        });
    shouldStop->name = "ShouldStop?";

    auto brake = std::make_unique<BTAction>(
        [this]()
        {
            if (m_speed > 0.0f)
            {
                Accelerate(0.0f);
                return BTStatus::Running;
            }
            m_destEdge = nullptr;
            m_currentEdge = nullptr;
            return BTStatus::Success;
        });
    brake->name = "Stop";

    return MakeSequence(std::move(shouldStop), std::move(brake));
}

std::unique_ptr<BTNode> Car::DriveNode()
{
    auto checkPath = std::make_unique<BTAction>(
        [this]()
        {
            // path find
            Vec3 position = GetPosition();

            float currentNodeDistance = (m_currentEdge->position - position).Length();
            auto prevNode = m_currentEdge;
            shared_ptr<RoadEdge> takenEdge;
            while (currentNodeDistance < 3.0f)
            {
                if (m_pathIndex + 1 >= m_path.size())
                {
                    m_destEdge = nullptr;
                    m_currentEdge = nullptr;
                    return BTStatus::Failure;
                }
                shared_ptr<RoadEdge> fromNode = m_currentEdge;
                m_currentEdge = m_path[++m_pathIndex];
                takenEdge = fromNode->GetEdgeTo(m_currentEdge->id);
                if (takenEdge && takenEdge->spline.GetControlPointCount() > 0)
                    m_currentSpline = takenEdge->spline;
                currentNodeDistance = (m_currentEdge->position - position).Length();
                DebugConsole::Get().Log("next node" + to_string(m_currentEdge->id));
                RebuildSplineRender();
            }
            // 방금 지나온 엣지에 미리 만들어둔 곡선이 없으면(교차로 회전 등) 다음 구간 스플라인으로 부드럽게 붙여준다.
            bool needsDynamicConnector = prevNode != m_currentEdge &&
                                         (!takenEdge || takenEdge->spline.GetControlPointCount() == 0);
            if (needsDynamicConnector)
            {
                // 차선변경 엣지(스플라인 없음)는 무조건 옆 차로의 스플라인에 붙는다.
                // -> 지금 도착한 노드로 들어오는 다른 엣지 중 스플라인이 있는 걸 찾는다(옆 차로의 진짜 경로).
                Spline targetSpline = m_currentSpline;
                for (const shared_ptr<RoadEdge> &edge : m_RoadDataManager->GetEdges())
                {
                    shared_ptr<RoadEdge> end = edge->endNode.lock();
                    if (end == m_currentEdge && edge->spline.GetControlPointCount() > 0)
                    {
                        targetSpline = edge->spline;
                        break;
                    }
                }

                float minRadius = powf(m_speed / CURVE_SPEED_COEFF, 2);
                float width = m_RoadDataManager->ROAD_WIDTH;
                float insideRoot = (4 * minRadius * width) - (width * width);
                float L = insideRoot > 0 ? sqrt(insideRoot) : 5.0f;

                Vec3 changePosition = targetSpline.GetLookaheadPoint(position, L);
                float changeSplinePosition = targetSpline.GetSplinePosition(changePosition);

                auto changeLineSpline = Spline({position - GetForwardAxis() * MIN_TANGENT_DISTANCE,
                                                position,
                                                changePosition,
                                                changePosition + targetSpline.GetDirectionAt(changeSplinePosition) * MIN_TANGENT_DISTANCE});
                targetSpline.AddSplinePointsFront(changeLineSpline.GetSplinePoints(), changeSplinePosition);
                m_currentSpline = targetSpline;
                RebuildSplineRender();
                CalculateSpeedProfile();
            }
            return BTStatus::Success;
        });
    checkPath->name = "CheckNextNode";
    auto drive = std::make_unique<BTAction>(
        [this]()
        {
            // steering
            constexpr float MIN_LOOKAHEAD_DISTANCE = 5.0f; // 저속/정지 시 최소 lookahead (m)
            constexpr float LOOKAHEAD_TIME = 0.5f;         // 몇 초 앞을 볼지
            float lookaheadDistance = std::max(MIN_LOOKAHEAD_DISTANCE, m_speed * LOOKAHEAD_TIME);
            float shortfall = 0.0f;
            // 회전만 리지드바디 포지션 사용
            auto targetPosition = m_currentSpline.GetLookaheadPoint(m_rigidbody.GetPosition(), lookaheadDistance, &shortfall);
            // 지금 스플라인 끝까지 봐도 lookaheadDistance가 안 채워지면, 다음 엣지 스플라인으로 아예 넘어간다.
            if (shortfall > 0.0f && m_pathIndex + 1 < m_path.size())
            {
                shared_ptr<RoadEdge> nextEdge = m_currentEdge->GetEdgeTo(m_path[m_pathIndex + 1]->id);
                if (nextEdge && nextEdge->spline.GetControlPointCount() > 0)
                {
                    m_currentSpline = nextEdge->spline;
                    targetPosition = m_currentSpline.GetLookaheadPoint(position, lookaheadDistance);
                    RebuildSplineRender();
                }
            }
            m_targetMarker.GetTransform().SetPosition(ToXMFLOAT3(targetPosition));
            float targetSteer = PurePursuit(targetPosition);
            Steer(targetSteer);

            // speed control
            float currentTime = m_currentTime;
            if (currentTime - m_lastProfileTime >= LOOK_PROFILE_TIME / SPEED_PROFILE_COUNT)
            {
                float profileSpeed = m_speedProfile[m_profileIndex].second;
                if (IsOffCourse() || abs(profileSpeed - m_speed) > (5.0f / 3.6f))
                {
                    CalculateSpeedProfile();
                }
                else
                {
                    MoveSpeedProfile();
                }
                m_lastProfileTime = m_currentTime;
            }
            float maxSpeed = CalcMaxSpeed(targetSteer) * 0.8f;
            float targetSpeed = min(m_speedProfile[m_profileIndex].second, maxSpeed);

            Accelerate(targetSpeed);

            return BTStatus::Success;
        });
    drive->name = "DriveControl";
    return MakeSequence(std::move(checkPath), std::move(drive));
}