#include "HybridAStar.h"
#include "Utill/DebugConsole.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <queue>
#include <string>
#include <unordered_set>
#include <Utill/PerfLog.h>

namespace HybridAStar
{
    namespace
    {
        constexpr double PI = 3.14159265358979323846;

        // 차량/장애물 사각형 판정에 쓰는 2D pose (y는 무시하되 결과 표시용으로 들고 다닌다).
        struct Pose
        {
            Vec3 position;
            float headingRad;
        };

        // 탐색 트리 노드. parent==-1이면 시작 노드. steering/gear는 parent -> 이 노드로 온 스텝.
        struct PlanNode
        {
            Pose pose;
            float gCost;
            int parent;
            ReedsShepp::Steering steering;
            ReedsShepp::Gear gear;
        };

        float NormalizeRad(float rad)
        {
            float r = std::fmod(rad, static_cast<float>(2.0 * PI));
            if (r < 0.0f)
                r += static_cast<float>(2.0 * PI);
            return r;
        }

        // closed-set 키: (x격자, z격자, heading격자) 3칸.
        using StateKey = std::array<int, 3>;
        struct StateKeyHash
        {
            size_t operator()(const StateKey &k) const
            {
                size_t h = std::hash<int>()(k[0]);
                h ^= std::hash<int>()(k[1]) + 0x9e3779b9 + (h << 6) + (h >> 2);
                h ^= std::hash<int>()(k[2]) + 0x9e3779b9 + (h << 6) + (h >> 2);
                return h;
            }
        };

        StateKey Discretize(const Pose &pose, const Params &params)
        {
            int ix = static_cast<int>(std::floor(pose.position.GetX() / params.gridResolution));
            int iz = static_cast<int>(std::floor(pose.position.GetZ() / params.gridResolution));
            int headingBins = std::max(1, static_cast<int>((2.0 * PI) / params.headingResolutionRad));
            int ith = static_cast<int>(std::floor(NormalizeRad(pose.headingRad) / params.headingResolutionRad)) % headingBins;
            return {ix, iz, ith};
        }

        // ReedsShepp::SamplePath와 같은 원호 전진 공식(부호 규약 동일)으로 pose를 한 스텝 이동.
        Pose StepPose(const Pose &pose, ReedsShepp::Steering steering, ReedsShepp::Gear gear,
                      float distance, float turningRadius)
        {
            double theta = static_cast<double>(pose.headingRad);
            double g = (gear == ReedsShepp::Gear::Backward) ? -1.0 : 1.0;
            double kappa = 0.0;
            if (steering == ReedsShepp::Steering::Left)
                kappa = 1.0 / turningRadius;
            else if (steering == ReedsShepp::Steering::Right)
                kappa = -1.0 / turningRadius;

            double x = pose.position.GetX();
            double z = pose.position.GetZ();

            if (std::fabs(kappa) < 1e-9)
            {
                x += g * distance * std::cos(theta);
                z += g * distance * std::sin(theta);
            }
            else
            {
                double invKappa = 1.0 / kappa;
                double cx = x - invKappa * std::sin(theta);
                double cz = z + invKappa * std::cos(theta);
                double kappaActual = g * kappa;
                theta += kappaActual * distance;
                x = cx + invKappa * std::sin(theta);
                z = cz - invKappa * std::cos(theta);
            }

            Pose result;
            result.position = Vec3(static_cast<float>(x), pose.position.GetY(), static_cast<float>(z));
            result.headingRad = static_cast<float>(theta);
            return result;
        }

        // 2D SAT: 두 회전된 사각형(중심+heading+반길이/반폭)이 겹치는지.
        bool ObbOverlap(const Vec3 &centerA, float halfLengthA, float halfWidthA, float headingRadA,
                        const Vec3 &centerB, float halfLengthB, float halfWidthB, float headingRadB)
        {
            Vec3 fwdA(std::cos(headingRadA), 0.0f, std::sin(headingRadA));
            Vec3 rightA(-fwdA.GetZ(), 0.0f, fwdA.GetX());
            Vec3 fwdB(std::cos(headingRadB), 0.0f, std::sin(headingRadB));
            Vec3 rightB(-fwdB.GetZ(), 0.0f, fwdB.GetX());

            Vec3 d = centerB - centerA;
            const Vec3 axes[4] = {fwdA, rightA, fwdB, rightB};
            for (const Vec3 &axis : axes)
            {
                float projA = halfLengthA * std::fabs(fwdA.Dot(axis)) + halfWidthA * std::fabs(rightA.Dot(axis));
                float projB = halfLengthB * std::fabs(fwdB.Dot(axis)) + halfWidthB * std::fabs(rightB.Dot(axis));
                float dist = std::fabs(d.Dot(axis));
                if (dist > projA + projB)
                    return false; // 분리축 발견 -> 안 겹침
            }
            return true;
        }

        bool IsPoseCollision(const Pose &pose, const std::vector<Obstacle> &obstacles, const VehicleShape &shape)
        {
            Vec3 forward(std::cos(pose.headingRad), 0.0f, std::sin(pose.headingRad));
            Vec3 bodyCenter = pose.position + forward * shape.pivotToCenter;

            for (const Obstacle &obstacle : obstacles)
            {
                if (ObbOverlap(bodyCenter, shape.halfLength, shape.halfWidth, pose.headingRad,
                               obstacle.center, obstacle.halfLength, obstacle.halfWidth, obstacle.headingRad))
                    return true;
            }
            return false;
        }

        // rsPath를 startPose에서부터 촘촘히 샘플링하며 매 지점 충돌을 검사. 빈 경로는 항상 충돌 없음(=현재
        // pose가 이미 목표라는 뜻).
        bool IsPathCollisionFree(const ReedsShepp::Path &rsPath, const Pose &startPose, float turningRadius,
                                 const std::vector<Obstacle> &obstacles, const VehicleShape &shape)
        {
            constexpr float SAMPLE_SPACING = 0.5f;
            Pose pose = startPose;
            for (const ReedsShepp::PathElement &element : rsPath)
            {
                int sampleCount = std::max(1, static_cast<int>(element.param / SAMPLE_SPACING));
                float ds = element.param / sampleCount;
                for (int i = 0; i < sampleCount; ++i)
                {
                    pose = StepPose(pose, element.steering, element.gear, ds, turningRadius);
                    if (IsPoseCollision(pose, obstacles, shape))
                        return false;
                }
            }
            return true;
        }

        // 장애물을 무시한 Reeds-Shepp 최단거리 휴리스틱 (non-holonomic-without-obstacles).
        float Heuristic(const Pose &pose, const Vec3 &goal, float goalHeadingRad, float turningRadius)
        {
            ReedsShepp::Path path = ReedsShepp::GetOptimalPath(pose.position, pose.headingRad, goal, goalHeadingRad, turningRadius);
            if (path.empty())
                return (goal - pose.position).Length();
            return ReedsShepp::GetPathLength(path);
        }

        // path 끝에 (steering, gear, param) 스텝 하나를 이어붙인다. 직전 세그먼트와 steering/gear가
        // 같으면 거리만 합쳐서 세그먼트 수를 줄인다.
        void AppendStep(ReedsShepp::Path &path, ReedsShepp::Steering steering, ReedsShepp::Gear gear, float param)
        {
            if (!path.empty() && path.back().steering == steering && path.back().gear == gear)
            {
                path.back().param += param;
                return;
            }
            path.push_back(ReedsShepp::PathElement{param, steering, gear});
        }

        ReedsShepp::Path ReconstructPath(const std::deque<PlanNode> &nodes, int nodeIndex, float stepSize)
        {
            ReedsShepp::Path reversed;
            for (int i = nodeIndex; nodes[i].parent != -1; i = nodes[i].parent)
                reversed.push_back(ReedsShepp::PathElement{stepSize, nodes[i].steering, nodes[i].gear});

            ReedsShepp::Path path;
            for (auto it = reversed.rbegin(); it != reversed.rend(); ++it)
                AppendStep(path, it->steering, it->gear, it->param);
            return path;
        }
    }

    ReedsShepp::Path FindPath(const Vec3 &start, float startHeadingRad,
                              const Vec3 &goal, float goalHeadingRad,
                              const std::vector<Obstacle> &obstacles,
                              const VehicleShape &shape,
                              bool &foundPath,
                              const Params &params)
    {
        PERF_LOG_SCOPE("Hybrid A* FindPath");
        PerfLog::LogMemory("Hybrid A* FindPath Start");
        foundPath = false;

        auto startTime = std::chrono::steady_clock::now();
        auto LogElapsed = [startTime](const char *label)
        {
            double elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - startTime).count();
            DebugConsole::Log("HybridAStar::FindPath " + std::string(label) + " in " + std::to_string(elapsedMs) + " ms");
        };

        float turningRadius = shape.wheelbase / std::tan(shape.maxSteerAngleRad);
        if (turningRadius <= 0.0f)
        {
            DebugConsole::Log("HybridAStar::FindPath failed: invalid turning radius (check maxSteerAngleRad)");
            return {};
        }

        // 시작/목표 pose 자체가 이미 장애물과 겹치면 탐색해봐야 못 찾는다 — 원인 파악용으로 미리 찍어둔다.
        if (IsPoseCollision(Pose{start, startHeadingRad}, obstacles, shape))
        {
            DebugConsole::Log("HybridAStar::FindPath: start pose overlaps an obstacle");
        }
        if (IsPoseCollision(Pose{goal, goalHeadingRad}, obstacles, shape))
        {
            DebugConsole::Log("HybridAStar::FindPath: goal pose overlaps an obstacle - path can never succeed");
            return {};
        }

        // 1. Open Set(우선순위 큐)에 시작 노드 삽입
        std::deque<PlanNode> nodes;
        nodes.push_back(PlanNode{Pose{start, startHeadingRad}, 0.0f, -1, ReedsShepp::Steering::Straight, ReedsShepp::Gear::Forward});

        using QueueItem = std::pair<float, int>; // (f_cost, node index)
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> openSet;
        openSet.push({Heuristic(nodes[0].pose, goal, goalHeadingRad, turningRadius), 0});

        // 3D 방문 여부를 체크할 테이블
        std::unordered_set<StateKey, StateKeyHash> closedSet;

        constexpr ReedsShepp::Steering STEERINGS[3] = {ReedsShepp::Steering::Left, ReedsShepp::Steering::Straight, ReedsShepp::Steering::Right};
        constexpr ReedsShepp::Gear GEARS[2] = {ReedsShepp::Gear::Forward, ReedsShepp::Gear::Backward};

        int expansions = 0;
        while (!openSet.empty())
        {
            if (++expansions > params.maxExpansions)
            {
                DebugConsole::Log("HybridAStar::FindPath failed: exceeded maxExpansions (" +
                                  std::to_string(params.maxExpansions) + ")");
                LogElapsed("failed (maxExpansions)");
                PerfLog::LogMemory("Hybrid A* FindPath maxExpansions " + std::to_string(params.maxExpansions));
                return {}; // Failure: 탐색 한도 초과
            }

            // 2. f_cost가 가장 낮은 노드를 꺼냄
            int currentIdx = openSet.top().second;
            openSet.pop();
            const PlanNode &current = nodes[currentIdx];

            // 3. [치트키] 리드-쉽 숏컷 시도 (Shot-to-Goal)
            ReedsShepp::Path rsPath = ReedsShepp::GetOptimalPath(current.pose.position, current.pose.headingRad, goal, goalHeadingRad, turningRadius);
            if (IsPathCollisionFree(rsPath, current.pose, turningRadius, obstacles, shape))
            {
                ReedsShepp::Path result = ReconstructPath(nodes, currentIdx, params.stepSize);
                for (const ReedsShepp::PathElement &element : rsPath)
                    AppendStep(result, element.steering, element.gear, element.param);
                foundPath = true;
                LogElapsed("succeeded");

                PerfLog::LogMemory("Hybrid A* FindPath Succeeded " + std::to_string(expansions));
                return result;
            }

            // 이미 방문한 격자(3D)라면 스킵, 아니면 방문 처리
            StateKey currentKey = Discretize(current.pose, params);
            if (closedSet.count(currentKey))
                continue;
            closedSet.insert(currentKey);

            // 4. 다음 한 걸음 뻗기 (Node Expansion)
            for (ReedsShepp::Steering steering : STEERINGS)
            {
                for (ReedsShepp::Gear gear : GEARS)
                {
                    Pose nextPose = StepPose(current.pose, steering, gear, params.stepSize, turningRadius);

                    if (IsPoseCollision(nextPose, obstacles, shape))
                        continue;

                    float moveCost = params.stepSize * (gear == ReedsShepp::Gear::Backward ? params.reverseCostMul : 1.0f);
                    float penalty = 0.0f;
                    // currentIdx==0(시작 노드)엔 들어온 스텝이 없으니 그때는 페널티를 매기지 않는다.
                    if (currentIdx != 0)
                    {
                        if (gear != current.gear)
                            penalty += params.gearChangeCost;
                        if (steering != current.steering)
                            penalty += params.steerChangeCost;
                    }

                    float gCost = current.gCost + moveCost + penalty;

                    nodes.push_back(PlanNode{nextPose, gCost, currentIdx, steering, gear});
                    int nextIdx = static_cast<int>(nodes.size()) - 1;

                    float fCost = gCost + Heuristic(nextPose, goal, goalHeadingRad, turningRadius);
                    openSet.push({fCost, nextIdx});
                }
            }
        }

        DebugConsole::Log("HybridAStar::FindPath failed: open set exhausted after " +
                          std::to_string(expansions) + " expansions");
        LogElapsed("failed (open set exhausted)");

        PerfLog::LogMemory("Hybrid A* FindPath exhausted");
        return {}; // Failure: 경로 없음
    }

    bool IsColliding(const Vec3 &position, float headingRad,
                     const std::vector<Obstacle> &obstacles, const VehicleShape &shape)
    {
        return IsPoseCollision(Pose{position, headingRad}, obstacles, shape);
    }
}
