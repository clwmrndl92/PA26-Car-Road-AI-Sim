#include "HybridAStar.h"
#include "Utill/DebugConsole.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

namespace HybridAStar
{
    namespace
    {
        constexpr double PI = 3.14159265358979323846;

        // 차량/장애물 사각형 판정에 쓰는 2D pose (y는 무시하되 결과 표시용으로 들고 다닌다).
        struct Pose
        {
            Vec3 position;
            float headingDeg;
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

        float NormalizeDeg(float deg)
        {
            float d = std::fmod(deg, 360.0f);
            if (d < 0.0f)
                d += 360.0f;
            return d;
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
            int headingBins = std::max(1, static_cast<int>(360.0f / params.headingResolution));
            int ith = static_cast<int>(std::floor(NormalizeDeg(pose.headingDeg) / params.headingResolution)) % headingBins;
            return {ix, iz, ith};
        }

        // ReedsShepp::SamplePath와 같은 원호 전진 공식(부호 규약 동일)으로 pose를 한 스텝 이동.
        Pose StepPose(const Pose &pose, ReedsShepp::Steering steering, ReedsShepp::Gear gear,
                     float distance, float turningRadius)
        {
            double theta = static_cast<double>(pose.headingDeg) * PI / 180.0;
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
            result.headingDeg = static_cast<float>(theta * 180.0 / PI);
            return result;
        }

        Vec3 HeadingForward(float headingDeg)
        {
            float rad = ToRadians(headingDeg);
            return Vec3(std::cos(rad), 0.0f, std::sin(rad));
        }

        // OBB 중심에서 모서리까지의 거리 = bounding circle 반경.
        float BoundingRadius(float halfLength, float halfWidth)
        {
            return std::sqrt(halfLength * halfLength + halfWidth * halfWidth);
        }

        // heading OBB의 축(fwd/right)과 bounding 반경을 미리 계산해 둔 장애물. 장애물은 FindPath 한 번
        // 도는 동안 안 움직이므로, 매 충돌검사마다 삼각함수를 다시 돌리지 않도록 진입 시 한 번만 만든다.
        struct PreparedObstacle
        {
            Vec3 center;
            float halfLength;
            float halfWidth;
            Vec3 fwd;
            Vec3 right;
            float boundingRadius;
        };

        std::vector<PreparedObstacle> PrepareObstacles(const std::vector<Obstacle> &obstacles)
        {
            std::vector<PreparedObstacle> prepared;
            prepared.reserve(obstacles.size());
            for (const Obstacle &o : obstacles)
            {
                Vec3 fwd = HeadingForward(o.headingDeg);
                Vec3 right(-fwd.GetZ(), 0.0f, fwd.GetX());
                prepared.push_back(PreparedObstacle{o.center, o.halfLength, o.halfWidth, fwd, right,
                                                    BoundingRadius(o.halfLength, o.halfWidth)});
            }
            return prepared;
        }

        // 2D SAT: 두 OBB의 축(fwd/right)이 모두 주어진 상태 — 삼각함수 없이 분리축만 검사한다.
        bool ObbOverlapAxes(const Vec3 &centerA, float halfLengthA, float halfWidthA, const Vec3 &fwdA, const Vec3 &rightA,
                            const Vec3 &centerB, float halfLengthB, float halfWidthB, const Vec3 &fwdB, const Vec3 &rightB)
        {
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

        // 미리 준비된 장애물에 대한 pose 충돌검사. 차량 축은 pose당 한 번만 계산하고, 각 장애물마다
        // bounding circle로 조기 기각한 뒤에만 SAT를 돌린다.
        bool IsPoseCollisionPrepared(const Pose &pose, const std::vector<PreparedObstacle> &obstacles,
                                     const VehicleShape &shape, float vehicleBoundingRadius)
        {
            Vec3 fwd = HeadingForward(pose.headingDeg);
            Vec3 right(-fwd.GetZ(), 0.0f, fwd.GetX());
            Vec3 bodyCenter = pose.position + fwd * shape.pivotToCenter;

            for (const PreparedObstacle &o : obstacles)
            {
                // 브로드페이즈: 두 bounding circle가 안 닿으면 SAT 생략(제곱거리 비교라 sqrt 불필요).
                float dx = o.center.GetX() - bodyCenter.GetX();
                float dz = o.center.GetZ() - bodyCenter.GetZ();
                float sumR = vehicleBoundingRadius + o.boundingRadius;
                if (dx * dx + dz * dz > sumR * sumR)
                    continue;

                if (ObbOverlapAxes(bodyCenter, shape.halfLength, shape.halfWidth, fwd, right,
                                   o.center, o.halfLength, o.halfWidth, o.fwd, o.right))
                    return true;
            }
            return false;
        }

        // 단발/사전준비 없는 호출용 래퍼 — 장애물을 즉석에서 준비해 위 함수로 넘긴다.
        bool IsPoseCollision(const Pose &pose, const std::vector<Obstacle> &obstacles, const VehicleShape &shape)
        {
            std::vector<PreparedObstacle> prepared = PrepareObstacles(obstacles);
            return IsPoseCollisionPrepared(pose, prepared, shape, BoundingRadius(shape.halfLength, shape.halfWidth));
        }

        // rsPath를 startPose에서부터 촘촘히 샘플링하며 매 지점 충돌을 검사. 빈 경로는 항상 충돌 없음(=현재
        // pose가 이미 목표라는 뜻).
        bool IsPathCollisionFree(const ReedsShepp::Path &rsPath, const Pose &startPose, float turningRadius,
                                 const std::vector<PreparedObstacle> &obstacles, const VehicleShape &shape,
                                 float vehicleBoundingRadius)
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
                    if (IsPoseCollisionPrepared(pose, obstacles, shape, vehicleBoundingRadius))
                        return false;
                }
            }
            return true;
        }

        // 점 p에서 OBB(o) 표면까지의 최단 거리(XZ, 내부면 0). 2D 격자 셀의 장애물 팽창 판정에 쓴다.
        float PointObbDistance(const Vec3 &p, const PreparedObstacle &o)
        {
            Vec3 d = p - o.center;
            float along = std::fabs(d.Dot(o.fwd)) - o.halfLength;   // heading축 초과분
            float across = std::fabs(d.Dot(o.right)) - o.halfWidth; // 수직축 초과분
            float ox = std::max(along, 0.0f);
            float oz = std::max(across, 0.0f);
            return std::sqrt(ox * ox + oz * oz);
        }

        // holonomic-with-obstacles 휴리스틱: 회전은 무시하되 벽은 우회하는 2D 격자 거리 테이블.
        // goal에서 8-연결 Dijkstra를 한 번 돌려 각 셀의 goal까지 거리를 미리 굽는다. 벽에 막혀 도달
        // 불가한 셀은 INF(=그 방향 노드 가지치기). 차량은 반경 halfWidth 원으로 근사해 장애물을 팽창.
        struct Grid2DHeuristic
        {
            static constexpr float INF = std::numeric_limits<float>::infinity();
            float originX = 0.0f;
            float originZ = 0.0f;
            float resolution = 0.5f;
            int width = 0;
            int height = 0;
            bool usable = false;
            std::vector<float> dist; // width*height, INF = 차단/미도달

            int Index(int cx, int cz) const { return cz * width + cx; }

            // pos가 속한 셀의 goal까지 거리. 비활성/범위 밖이면 0(정보 없음 → RS로 폴백), 차단/미도달이면 INF.
            float Lookup(const Vec3 &pos) const
            {
                if (!usable)
                    return 0.0f;
                int cx = static_cast<int>(std::floor((pos.GetX() - originX) / resolution));
                int cz = static_cast<int>(std::floor((pos.GetZ() - originZ) / resolution));
                if (cx < 0 || cx >= width || cz < 0 || cz >= height)
                    return 0.0f;
                return dist[Index(cx, cz)];
            }
        };

        Grid2DHeuristic Build2DHeuristic(const Vec3 &start, const Vec3 &goal,
                                         const std::vector<PreparedObstacle> &obstacles,
                                         const VehicleShape &shape, const Params &params)
        {
            Grid2DHeuristic g;
            g.resolution = params.gridResolution;

            // AABB: start/goal/장애물(반경 포함)을 감싸고 여유 마진.
            float minX = std::min(start.GetX(), goal.GetX());
            float maxX = std::max(start.GetX(), goal.GetX());
            float minZ = std::min(start.GetZ(), goal.GetZ());
            float maxZ = std::max(start.GetZ(), goal.GetZ());
            for (const PreparedObstacle &o : obstacles)
            {
                minX = std::min(minX, o.center.GetX() - o.boundingRadius);
                maxX = std::max(maxX, o.center.GetX() + o.boundingRadius);
                minZ = std::min(minZ, o.center.GetZ() - o.boundingRadius);
                maxZ = std::max(maxZ, o.center.GetZ() + o.boundingRadius);
            }
            float margin = std::max(5.0f * g.resolution, shape.halfLength + shape.halfWidth);
            minX -= margin;
            maxX += margin;
            minZ -= margin;
            maxZ += margin;

            g.originX = minX;
            g.originZ = minZ;
            g.width = std::max(1, static_cast<int>(std::ceil((maxX - minX) / g.resolution)));
            g.height = std::max(1, static_cast<int>(std::ceil((maxZ - minZ) / g.resolution)));

            // 셀 수 과다 방지 — 지나치게 크면 2D 휴리스틱 비활성(순수 RS로).
            constexpr long MAX_CELLS = 1000000;
            if (static_cast<long>(g.width) * g.height > MAX_CELLS)
                return g; // usable=false

            const size_t cellCount = static_cast<size_t>(g.width) * g.height;
            g.dist.assign(cellCount, Grid2DHeuristic::INF);

            // 차단 마스크: 셀 중심이 (장애물 표면까지 거리 <= halfWidth)이면 차단.
            const float inflate = shape.halfWidth;
            std::vector<char> blocked(cellCount, 0);
            for (int cz = 0; cz < g.height; ++cz)
                for (int cx = 0; cx < g.width; ++cx)
                {
                    Vec3 c(g.originX + (cx + 0.5f) * g.resolution, 0.0f, g.originZ + (cz + 0.5f) * g.resolution);
                    for (const PreparedObstacle &o : obstacles)
                    {
                        float ddx = c.GetX() - o.center.GetX();
                        float ddz = c.GetZ() - o.center.GetZ();
                        float rr = o.boundingRadius + inflate;
                        if (ddx * ddx + ddz * ddz > rr * rr)
                            continue; // bounding circle 조기기각
                        if (PointObbDistance(c, o) <= inflate)
                        {
                            blocked[g.Index(cx, cz)] = 1;
                            break;
                        }
                    }
                }

            // goal 셀은 시작점 — 팽창으로 차단됐더라도 소스로 강제 사용.
            int gx = std::min(std::max(static_cast<int>(std::floor((goal.GetX() - g.originX) / g.resolution)), 0), g.width - 1);
            int gz = std::min(std::max(static_cast<int>(std::floor((goal.GetZ() - g.originZ) / g.resolution)), 0), g.height - 1);

            using QI = std::pair<float, int>; // (dist, cellIndex)
            std::priority_queue<QI, std::vector<QI>, std::greater<QI>> pq;
            int goalIdx = g.Index(gx, gz);
            g.dist[goalIdx] = 0.0f;
            pq.push({0.0f, goalIdx});

            const int dcx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
            const int dcz[8] = {0, 0, 1, -1, 1, -1, 1, -1};
            const float card = g.resolution;
            const float diag = std::sqrt(2.0f) * g.resolution;
            const float stepCost[8] = {card, card, card, card, diag, diag, diag, diag};

            while (!pq.empty())
            {
                QI top = pq.top();
                pq.pop();
                float d = top.first;
                int idx = top.second;
                if (d > g.dist[idx])
                    continue;
                int cx = idx % g.width;
                int cz = idx / g.width;
                for (int k = 0; k < 8; ++k)
                {
                    int nx = cx + dcx[k];
                    int nz = cz + dcz[k];
                    if (nx < 0 || nx >= g.width || nz < 0 || nz >= g.height)
                        continue;
                    int nidx = g.Index(nx, nz);
                    if (blocked[nidx])
                        continue;
                    float nd = d + stepCost[k];
                    if (nd < g.dist[nidx])
                    {
                        g.dist[nidx] = nd;
                        pq.push({nd, nidx});
                    }
                }
            }

            // start 셀이 도달 불가면 2D 휴리스틱을 신뢰할 수 없음 → 비활성(순수 RS로 폴백).
            int sx = static_cast<int>(std::floor((start.GetX() - g.originX) / g.resolution));
            int sz = static_cast<int>(std::floor((start.GetZ() - g.originZ) / g.resolution));
            if (sx < 0 || sx >= g.width || sz < 0 || sz >= g.height || std::isinf(g.dist[g.Index(sx, sz)]))
                return g; // usable=false

            g.usable = true;
            return g;
        }

        // 두 하한을 합친 휴리스틱: 회전제약(Reeds-Shepp, 장애물 무시)과 벽 우회(2D 격자, 회전 무시)
        // 중 더 강한 병목을 취한다(max). 둘 다 실제 잔여 비용의 하한이라 합쳐도 하한이 유지된다.
        float Heuristic(const Pose &pose, const Vec3 &goal, float goalHeadingDeg, float turningRadius,
                        const Grid2DHeuristic &grid2d)
        {
            ReedsShepp::Path path = ReedsShepp::GetOptimalPath(pose.position, pose.headingDeg, goal, goalHeadingDeg, turningRadius);
            float rsDist = path.empty() ? (goal - pose.position).Length() : ReedsShepp::GetPathLength(path);

            float obstacleDist = grid2d.Lookup(pose.position);
            return std::max(rsDist, obstacleDist);
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

    ReedsShepp::Path FindPath(const Vec3 &start, float startHeadingDeg,
                              const Vec3 &goal, float goalHeadingDeg,
                              const std::vector<Obstacle> &obstacles,
                              const VehicleShape &shape,
                              bool &foundPath,
                              const Params &params)
    {
        foundPath = false;

        float turningRadius = shape.wheelbase / std::tan(ToRadians(shape.maxSteerAngleDeg));
        if (turningRadius <= 0.0f)
        {
            DebugConsole::Log("HybridAStar::FindPath failed: invalid turning radius (check maxSteerAngleDeg)");
            return {};
        }
        
        // 장애물 축/bounding 반경은 탐색 내내 안 변하므로 한 번만 준비해 재사용한다.
        std::vector<PreparedObstacle> preparedObstacles = PrepareObstacles(obstacles);
        float vehicleBoundingRadius = BoundingRadius(shape.halfLength, shape.halfWidth);

        // 시작/목표 pose 자체가 이미 장애물과 겹치면 탐색해봐야 못 찾는다 — 원인 파악용으로 미리 찍어둔다.
        if (IsPoseCollisionPrepared(Pose{start, startHeadingDeg}, preparedObstacles, shape, vehicleBoundingRadius)){
            DebugConsole::Log("HybridAStar::FindPath: start pose overlaps an obstacle " + ToString(start));
            return {};
        }
        if (IsPoseCollisionPrepared(Pose{goal, goalHeadingDeg}, preparedObstacles, shape, vehicleBoundingRadius)) {
            DebugConsole::Log("HybridAStar::FindPath: goal pose overlaps an obstacle " + ToString(goal));
            return {};
        }

        // holonomic-with-obstacles 휴리스틱 테이블을 goal 기준으로 한 번 구워둔다(2D라 매우 저렴).
        Grid2DHeuristic grid2d = Build2DHeuristic(start, goal, preparedObstacles, shape, params);

        // 1. Open Set(우선순위 큐)에 시작 노드 삽입
        std::deque<PlanNode> nodes;
        nodes.push_back(PlanNode{Pose{start, startHeadingDeg}, 0.0f, -1, ReedsShepp::Steering::Straight, ReedsShepp::Gear::Forward});

        using QueueItem = std::pair<float, int>; // (f_cost, node index)
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> openSet;
        openSet.push({Heuristic(nodes[0].pose, goal, goalHeadingDeg, turningRadius, grid2d), 0});

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
                return {}; // Failure: 탐색 한도 초과
            }

            // 2. f_cost가 가장 낮은 노드를 꺼냄
            int currentIdx = openSet.top().second;
            openSet.pop();
            const PlanNode &current = nodes[currentIdx];

            // 3. [치트키] 리드-쉽 숏컷 시도 (Shot-to-Goal)
            ReedsShepp::Path rsPath = ReedsShepp::GetOptimalPath(current.pose.position, current.pose.headingDeg, goal, goalHeadingDeg, turningRadius);
            if (IsPathCollisionFree(rsPath, current.pose, turningRadius, preparedObstacles, shape, vehicleBoundingRadius))
            {
                ReedsShepp::Path result = ReconstructPath(nodes, currentIdx, params.stepSize);
                for (const ReedsShepp::PathElement &element : rsPath)
                    AppendStep(result, element.steering, element.gear, element.param);
                foundPath = true;
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

                    if (IsPoseCollisionPrepared(nextPose, preparedObstacles, shape, vehicleBoundingRadius))
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

                    float fCost = gCost + Heuristic(nextPose, goal, goalHeadingDeg, turningRadius, grid2d);
                    openSet.push({fCost, nextIdx});
                }
            }
        }

        DebugConsole::Log("HybridAStar::FindPath failed: open set exhausted after " +
                          std::to_string(expansions) + " expansions");
        return {}; // Failure: 경로 없음
    }

    bool IsColliding(const Vec3 &position, float headingDeg,
                     const std::vector<Obstacle> &obstacles, const VehicleShape &shape)
    {
        return IsPoseCollision(Pose{position, headingDeg}, obstacles, shape);
    }
}
