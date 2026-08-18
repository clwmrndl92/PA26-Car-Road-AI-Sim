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

    struct RacingPathData
    {
        std::vector<Vec3> points; // 직전 도로 꼬리 + 현재 도로
        std::vector<float> arcLength;
        std::vector<float> curvature;
        size_t closest = 0;
    };

    struct RacingPlan
    {
        Vec3 target;
        Vec3 pathPoint;
        Vec3 pathDirection;
        float targetSpeed = 0.0f;
        float brakingAccel = 0.0f;
        float maxAccelNow = 0.0f; // 지금 곡률에서 마찰원이 남겨준 가속 여유
        float remainingDistance = 0.0f;
        bool hasLocalCorner = false;
    };

    float HorizontalLength(const Vec3 &v)
    {
        return std::sqrt(v.GetX() * v.GetX() + v.GetZ() * v.GetZ());
    }

    // points가 채워진 뒤 호길이/곡률/최근접점을 메운다.
    void FillRacingMetrics(RacingPathData &data, const Vec3 &position)
    {
        const std::vector<Vec3> &points = data.points;
        data.arcLength.assign(points.size(), 0.0f);
        data.curvature.assign(points.size(), 0.0f);
        float closestDistance = std::numeric_limits<float>::max();
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (i > 0)
                data.arcLength[i] = data.arcLength[i - 1] + HorizontalLength(points[i] - points[i - 1]);
            float distance = (points[i] - position).LengthSq();
            if (distance < closestDistance)
            {
                closestDistance = distance;
                data.closest = i;
            }
        }

        constexpr float CURVATURE_WINDOW = 2.0f;
        for (size_t i = 1; i + 1 < points.size(); ++i)
        {
            size_t prev = i;
            while (prev > 0 && data.arcLength[i] - data.arcLength[prev] < CURVATURE_WINDOW)
                --prev;
            size_t next = i;
            while (next + 1 < points.size() && data.arcLength[next] - data.arcLength[i] < CURVATURE_WINDOW)
                ++next;
            if (prev == i || next == i)
                continue;

            Vec3 a = points[i] - points[prev];
            Vec3 b = points[next] - points[i];
            Vec3 chord = points[next] - points[prev];
            float denominator = HorizontalLength(a) * HorizontalLength(b) * HorizontalLength(chord);
            if (denominator < 0.001f)
                continue;
            float cross = a.GetX() * b.GetZ() - a.GetZ() * b.GetX();
            data.curvature[i] = 2.0f * cross / denominator;
        }
    }

    // QP 스텐실이 등간격 h를 가정하므로 먼저 균일 리샘플한다.
    std::vector<Vec3> ResampleUniform(const std::vector<Vec3> &in, float spacing)
    {
        std::vector<Vec3> out;
        if (in.size() < 2 || spacing <= 0.001f)
            return out;

        out.push_back(in.front());
        float carry = 0.0f;
        for (size_t i = 0; i + 1 < in.size(); ++i)
        {
            Vec3 seg = in[i + 1] - in[i];
            float segLen = HorizontalLength(seg);
            if (segLen < 1e-5f)
                continue;

            float placed = spacing - carry;
            while (placed <= segLen)
            {
                out.push_back(in[i] + seg * (placed / segLen));
                placed += spacing;
            }
            carry = segLen - (placed - spacing);
        }
        if (out.size() < 2)
            out.push_back(in.back());
        return out;
    }

    // 최소 곡률 QP.  min  sum_i k_i^2 + w * sum_i kRef_i * d_i   s.t.  dMin <= d_i <= dMax
    //
    // 경로 곡률을 선형화하면  k_i = kRef_i - (d_{i-1} - 2d_i + d_{i+1}) / h^2  이고,
    // 목적함수가 d에 대해 2차 + 제약이 박스뿐이라 볼록 QP다. 정상조건을 정리하면
    // d의 4차 스텐실만 남는다:
    //     d_{j-2} - 4d_{j-1} + 6d_j - 4d_{j+1} + d_{j+2} = R_j
    //     R_j = h^2 * (kRef_{j-1} - 2kRef_j + kRef_{j+1}) - w * kRef_j
    // 이걸 projected Gauss-Seidel(SOR)로 푼다. 매 갱신마다 클램프하는 게 곧 박스 제약이라
    // 도로 밖으로는 원리적으로 못 나간다.
    void SolveMinCurvature(const std::vector<float> &refCurvature, float h, float dMin, float dMax,
                           float lengthWeight, int iterations, std::vector<float> &d)
    {
        const size_t n = refCurvature.size();
        if (n < 5 || d.size() != n)
            return;

        std::vector<float> rhs(n, 0.0f);
        for (size_t i = 1; i + 1 < n; ++i)
        {
            float secondDiff = refCurvature[i - 1] - 2.0f * refCurvature[i] + refCurvature[i + 1];
            rhs[i] = h * h * secondDiff - lengthWeight * refCurvature[i];
        }

        constexpr float OMEGA = 1.7f; // 과이완(SOR)
        for (int iter = 0; iter < iterations; ++iter)
        {
            for (size_t j = 2; j + 2 < n; ++j)
            {
                float target = (rhs[j] - d[j - 2] + 4.0f * d[j - 1] + 4.0f * d[j + 1] - d[j + 2]) / 6.0f;
                d[j] = std::clamp(d[j] + OMEGA * (target - d[j]), dMin, dMax);
            }
        }
    }

    float SampleLinear(const std::vector<float> &v, float x)
    {
        if (v.empty())
            return 0.0f;
        float clamped = std::clamp(x, 0.0f, static_cast<float>(v.size() - 1));
        size_t i = static_cast<size_t>(clamped);
        size_t j = std::min(i + 1, v.size() - 1);
        return v[i] + (v[j] - v[i]) * (clamped - static_cast<float>(i));
    }

    // 4차 스텐실은 Gauss-Seidel 정보 전파가 sweep당 한 칸이라, 점이 수천 개면 수만 번
    // 돌려도 초기값에서 못 벗어난다. 거친 격자부터 풀어 초기값으로 내려주면(nested iteration)
    // 각 레벨은 고주파 오차만 남아 금방 수렴한다.
    std::vector<float> SolveMinCurvatureCascade(const std::vector<float> &refCurvature, float h,
                                                float dMin, float dMax, float lengthWeight)
    {
        const size_t n = refCurvature.size();
        if (n < 8)
            return std::vector<float>(n, 0.0f);

        size_t stride = 1;
        while (stride * 64 < n)
            stride *= 2;

        std::vector<float> coarse; // 직전(두 배 거친) 레벨의 해
        for (; stride >= 1; stride /= 2)
        {
            const size_t m = (n + stride - 1) / stride;
            std::vector<float> levelCurvature(m);
            for (size_t i = 0; i < m; ++i)
                levelCurvature[i] = refCurvature[std::min(i * stride, n - 1)];

            std::vector<float> d(m, 0.0f);
            for (size_t i = 0; i < m; ++i)
                d[i] = SampleLinear(coarse, static_cast<float>(i) * 0.5f);

            int iterations = static_cast<int>(std::clamp(200000.0 / static_cast<double>(m), 300.0, 20000.0));
            SolveMinCurvature(levelCurvature, h * static_cast<float>(stride), dMin, dMax,
                              lengthWeight, iterations, d);
            coarse.swap(d);
            if (stride == 1)
                break;
        }
        coarse.resize(n, coarse.empty() ? 0.0f : coarse.back());
        return coarse;
    }

    // 전략 = 곡률 최소 <-> 거리 최소 사이의 가중치.
    // 거리항을 키우면 안쪽을 파고들어(=이른 에이펙스) 짧게 가고, 0이면 순수 최소곡률(=늦은 에이펙스).
    float StrategyLengthWeight(Car::ApexStrategy strategy)
    {
        switch (strategy)
        {
        case Car::ApexStrategy::Early:
            return 0.20f;
        case Car::ApexStrategy::Apex:
            return 0.05f;
        default:
            return 0.0f;
        }
    }

    // 문맥 도로들을 이어 최소곡률 라인을 푼다. 도로 문맥이 바뀔 때만 호출할 것(무겁다).
    RaceLine BuildRaceLine(const std::vector<const Spline *> &lines, size_t drawBegin, size_t drawEnd,
                           float laneCenter, float lineRoom, Car::ApexStrategy strategy)
    {
        RaceLine line;
        std::vector<Vec3> merged;
        float beforeDraw = 0.0f;
        float throughDraw = 0.0f;
        for (size_t i = 0; i < lines.size(); ++i)
        {
            const std::vector<Vec3> &points = lines[i]->GetSplinePoints();
            float length = 0.0f;
            for (size_t k = 0; k + 1 < points.size(); ++k)
                length += HorizontalLength(points[k + 1] - points[k]);
            if (i < drawBegin)
                beforeDraw += length;
            if (i <= drawEnd)
                throughDraw += length;
            merged.insert(merged.end(), points.begin(), points.end());
        }
        if (merged.size() < 2)
            return line;

        constexpr float SPACING = 1.0f;
        RacingPathData ref;
        ref.points = ResampleUniform(merged, SPACING);
        if (ref.points.size() < 5)
            return line;
        FillRacingMetrics(ref, ref.points.front());

        std::vector<float> d = SolveMinCurvatureCascade(ref.curvature, SPACING, -lineRoom, lineRoom,
                                                        StrategyLengthWeight(strategy));

        RacingPathData solved;
        solved.points.reserve(ref.points.size());
        for (size_t i = 0; i < ref.points.size(); ++i)
            solved.points.push_back(OffsetSampleAt(ref.points, i, laneCenter + d[i]));
        FillRacingMetrics(solved, solved.points.front());

        line.points = std::move(solved.points);
        line.arcLength = std::move(solved.arcLength);
        line.curvature = std::move(solved.curvature);
        line.drawFromS = beforeDraw;
        line.drawToS = throughDraw;
        return line;
    }

    size_t ClosestOnLine(const RaceLine &line, const Vec3 &position)
    {
        size_t closest = 0;
        float best = std::numeric_limits<float>::max();
        for (size_t i = 0; i < line.points.size(); ++i)
        {
            float distance = (line.points[i] - position).LengthSq();
            if (distance < best)
            {
                best = distance;
                closest = i;
            }
        }
        return closest;
    }

    size_t IndexAtS(const RaceLine &line, float s)
    {
        size_t i = static_cast<size_t>(
            std::lower_bound(line.arcLength.begin(), line.arcLength.end(), s) - line.arcLength.begin());
        return std::min(i, line.points.size() - 1);
    }

    // 마찰원: 타이어가 낼 수 있는 가속도는 횡/종방향을 나눠 쓰는 게 아니라 하나의 원 예산을
    // 공유한다. 코너링에 횡가속도를 이미 많이 쓰고 있으면, 남은 종방향(가감속) 여유는
    // sqrt(1 - (횡/한계)^2)만큼만 남는다 -- 이게 트레일 브레이킹/파워다운을 자연스럽게 만든다.
    float FrictionLongitudinalBudget(float longitudinalCap, float lateralAccel, float frictionLimit)
    {
        if (frictionLimit <= 0.001f)
            return 0.0f;
        float ratio = std::clamp(lateralAccel / frictionLimit, 0.0f, 1.0f);
        return longitudinalCap * std::sqrt(std::max(0.0f, 1.0f - ratio * ratio));
    }

    RacingPlan BuildRacingPlan(const RaceLine &line, const Vec3 &position, float lookahead,
                               float cruiseSpeed, float currentSpeed, float maxAccel, float maxBrake)
    {
        RacingPlan plan;
        plan.target = position;
        plan.pathPoint = position;
        plan.targetSpeed = cruiseSpeed;
        if (line.points.size() < 2)
            return plan;

        const size_t closest = ClosestOnLine(line, position);
        const float currentS = line.arcLength[closest];
        plan.remainingDistance = line.arcLength.back() - currentS;

        // 코너를 서다/서다로 도는 게 아니라, 마찰원 안에서 브레이킹하며 진입하고
        // 여유가 생기는 만큼 가속하며 탈출하도록 구간 전체를 체이닝해서 푼다.
        //
        // 미리보는 거리가 고정 100m였는데, 최고속(약 88.9m/s)에서 완전정지까지 필요한
        // 거리가 v^2/(2*maxBrake) ~= 257m라 100m로는 코너를 발견했을 때 이미 너무 늦다.
        // 코너 근처에서는 마찰원이 제동 여유를 더 깎기까지 하니 1.5배 여유를 둔다.
        const float currentSpeedSq = currentSpeed * currentSpeed;
        const float stopDistance = currentSpeedSq / (2.0f * std::max(1.0f, maxBrake));
        const float SPEED_PREVIEW = std::max(50.0f, stopDistance * 1.5f);
        constexpr float CURVATURE_MIN = 1.0f / 500.0f;
        constexpr float FRICTION_ACCEL = 8.0f; // 타이어 마찰원 반경(횡/종 공유)

        size_t last = closest;
        while (last + 1 < line.points.size() && line.arcLength[last] - currentS <= SPEED_PREVIEW)
            ++last;

        std::vector<float> vLat(last - closest + 1, cruiseSpeed);
        for (size_t i = closest; i <= last; ++i)
        {
            float absCurvature = std::fabs(line.curvature[i]);
            if (absCurvature >= CURVATURE_MIN)
            {
                vLat[i - closest] = std::min(cruiseSpeed, std::sqrt(FRICTION_ACCEL / absCurvature));
                plan.hasLocalCorner = true;
            }
        }

        // 뒤에서부터: 코너 진입 전에 마찰원이 허락하는 만큼만 감속(트레일 브레이킹).
        // idx-1==0 전이에서 쓰인 avail이 "지금 이 순간" 낼 수 있는 감속 여유다.
        std::vector<float> vBrake = vLat;
        float brakeAvailNow = maxBrake;
        for (size_t i = last; i > closest; --i)
        {
            size_t idx = i - closest;
            float ds = std::max(0.001f, line.arcLength[i] - line.arcLength[i - 1]);
            float lateralAtNext = vBrake[idx] * vBrake[idx] * std::fabs(line.curvature[i]);
            float avail = FrictionLongitudinalBudget(maxBrake, lateralAtNext, FRICTION_ACCEL);
            float candidate = std::sqrt(vBrake[idx] * vBrake[idx] + 2.0f * avail * ds);
            vBrake[idx - 1] = std::min(vLat[idx - 1], candidate);
            if (idx == 1)
                brakeAvailNow = avail;
        }

        // 앞에서부터: 지금 속도/곡률에서 마찰원이 허락하는 만큼만 가속(파워다운 이후 점진 가속).
        // 이 값을 plan.maxAccelNow로 내보내 DriveControl의 가속 명령 자체를 캡한다 --
        // 목표속도 배열만으론 지금 순간의 가속 여유가 P제어기에 전달되지 않는다.
        std::vector<float> vAccel = vLat;
        vAccel[0] = currentSpeed;
        float accelAvailNow = maxAccel;
        for (size_t i = closest + 1; i <= last; ++i)
        {
            size_t idx = i - closest;
            float ds = std::max(0.001f, line.arcLength[i] - line.arcLength[i - 1]);
            float lateralAtPrev = vAccel[idx - 1] * vAccel[idx - 1] * std::fabs(line.curvature[i - 1]);
            float avail = FrictionLongitudinalBudget(maxAccel, lateralAtPrev, FRICTION_ACCEL);
            float candidate = std::sqrt(vAccel[idx - 1] * vAccel[idx - 1] + 2.0f * avail * ds);
            vAccel[idx] = std::min(vLat[idx], candidate);
            if (idx == 1)
                accelAvailNow = avail;
        }

        plan.targetSpeed = vBrake[0];
        plan.maxAccelNow = accelAvailNow;
        if (currentSpeed > plan.targetSpeed + 0.2f)
            plan.brakingAccel = -brakeAvailNow;

        plan.pathPoint = line.points[closest];
        plan.target = line.points[IndexAtS(line, currentS + lookahead)];

        const size_t aheadIndex = IndexAtS(line, currentS + 2.0f);
        Vec3 plannedDirection = line.points[aheadIndex] - plan.pathPoint;
        if (HorizontalLength(plannedDirection) > 0.001f)
            plan.pathDirection = plannedDirection.Normalized();
        else if (closest + 1 < line.points.size())
            plan.pathDirection = (line.points[closest + 1] - plan.pathPoint).Normalized();
        return plan;
    }

    // 레이싱 라인이 이미 앞 도로들을 포함하므로 여기선 도로별 제한속도만 본다.
    void PreviewUpcomingRoad(RacingPlan &plan, const std::vector<RoadRef> &path, size_t pathIndex,
                             float maxSpeed, float currentSpeed, float maxBrake)
    {
        const float racingBrake = std::max(1.0f, maxBrake * 0.75f);
        // 고정 100m는 고속에서 정지거리보다 짧아 제한속도 급변 구간을 늦게 발견했다.
        const float SPEED_PREVIEW = std::max(50.0f, (currentSpeed * currentSpeed) / (2.0f * racingBrake) * 1.5f);
        float distanceToRoad = plan.remainingDistance;

        for (size_t pathI = pathIndex + 1;
             pathI < path.size() && distanceToRoad <= SPEED_PREVIEW; ++pathI)
        {
            const shared_ptr<Road> &road = path[pathI].road;
            if (road == nullptr)
                continue;

            float roadSpeed = std::min(road->GetSpeedLimit(), maxSpeed);
            float entrySpeed = std::sqrt(roadSpeed * roadSpeed + 2.0f * racingBrake * distanceToRoad);
            plan.targetSpeed = std::min(plan.targetSpeed, entrySpeed);
            distanceToRoad += road->GetReferenceLine().GetLength();
        }

        // 더 강하게(음수로 더 큰 쪽) 요구하는 제동만 반영 -- 코너용 마찰원 제동값을 덮어쓰지 않는다.
        if (currentSpeed > plan.targetSpeed + 0.2f)
            plan.brakingAccel = std::min(plan.brakingAccel, -racingBrake);
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
    // 참조선(중심선) 기준 arc-length로 도로 끝을 판단한다 -- m_currentSpline은
    // m_currentOffset만큼 민 평행곡선이라, 레이싱 라인이 차를 그 오프셋과 다른 쪽으로
    // 멀리 태워가면(코너에서 흔함) 이 평행곡선의 끝점이 실제 차 위치와 동떨어져서
    // roadEndDistance가 계속 커 보인다 -- 도로를 다 지나도 전환이 안 걸리는 원인이었다.
    auto remainingOnRoad = [](const Spline &referenceLine, const Vec3 &pos) -> float
    {
        float t = std::clamp(referenceLine.GetSplinePosition(pos), 0.0f, 1.0f);
        return referenceLine.GetLength() * (1.0f - t);
    };

    float remaining = remainingOnRoad(m_currentRoad->GetReferenceLine(), position);
    while (remaining < LANE_TRANSITION_THRESHOLD)
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
        // 레이싱 라인이 차를 어디로 태워왔든, 항상 실측 위치로 다음 도로의 오프셋/밴드를
        // 잡는다. 정적 차선 연결 매핑(ResolveConnectingOffset)은 차선 중앙을 따라가는
        // 일반 주행용이라 레이싱 라인과 무관하게 옆 차선으로 튀어 버린다.
        float nextOffset = ComputeReferenceOffset(next.road->GetReferenceLine(), position);

        SetCurrentRoad(next.road, nextOffset);
        remaining = remainingOnRoad(next.road->GetReferenceLine(), position);
    }
    return true;
}

bool Car::IsOutsideDrivingBands() const
{
    if (m_currentRoad == nullptr)
        return false;

    const Spline &referenceLine = m_currentRoad->GetReferenceLine();
    if (referenceLine.GetSplinePoints().size() < 2)
        return false;

    Vec3 center = GetBodyCenter();
    float t = std::clamp(referenceLine.GetSplinePosition(center), 0.0f, 1.0f);
    float s = t * referenceLine.GetLength();
    const LaneSection *section = RoadDataManager::Get().GetLateralProfile(m_currentRoad, s);
    if (section == nullptr)
        return false;

    float minOffset = std::numeric_limits<float>::max();
    float maxOffset = -std::numeric_limits<float>::max();
    bool hasDrivingBand = false;
    for (const LaneBand &band : section->bands)
    {
        if (band.type != LaneType::Driving)
            continue;
        minOffset = std::min(minOffset, band.centerOffset - band.width * 0.5f);
        maxOffset = std::max(maxOffset, band.centerOffset + band.width * 0.5f);
        hasDrivingBand = true;
    }
    if (!hasDrivingBand)
        return false;

    float centerOffset = ComputeReferenceOffset(referenceLine, center);
    return centerOffset < minOffset || centerOffset > maxOffset;
}

void Car::DriveControl()
{
    // if (IsOutsideDrivingBands())
    // {
    //     SetSpeed(0.0f);
    //     m_acceleration = 0.0f;
    //     m_planAccelDebug = 0.0f;
    //     Steer(0.0f);
    //     return;
    // }

    const Vec3 position = GetRigidbodyPosition();

    // High speed needs a longer sight line so steering stays progressive instead of twitchy.
    float lookaheadDistance = std::clamp(4.5f + m_speed * 0.35f, 5.0f, 13.0f);
    float laneCenter = m_currentBand != nullptr ? m_currentBand->centerOffset : m_currentOffset;
    float laneHalfWidth = m_currentBand != nullptr ? m_currentBand->width * 0.5f : 0.0f;
    // 차체가 도로 밖으로 나가면 안 된다
    constexpr float EDGE_MARGIN = 0.2f;
    constexpr float MAX_RACING_OFFSET = 6.0f;
    float lineRoom = std::clamp(laneHalfWidth - GetHalfWidth() - EDGE_MARGIN, 0.0f, MAX_RACING_OFFSET);
    // 앞뒤 한 칸은 QP 경계 효과를 흡수시키는 문맥용. 그리진 않는다.
    std::vector<const Road *> contextRoads;
    size_t drawBegin = 0;
    size_t drawEnd = 0;
    for (size_t i = m_pathIndex > 0 ? m_pathIndex - 1 : 0;
         i < m_path.size() && i <= m_pathIndex + 3; ++i)
    {
        if (m_path[i].road == nullptr)
            continue;
        if (i == m_pathIndex)
            drawBegin = contextRoads.size();
        if (i <= m_pathIndex + 2)
            drawEnd = contextRoads.size();
        contextRoads.push_back(m_path[i].road.get());
    }
    // 최소곡률 QP는 무거우니 도로 문맥이나 전략이 바뀔 때만 푼다
    if (contextRoads != m_racingLineRoads || m_racingLineStrategy != m_apexStrategy)
    {
        std::vector<const Spline *> lines;
        lines.reserve(contextRoads.size());
        for (const Road *road : contextRoads)
            lines.push_back(&road->GetReferenceLine());
        m_raceLine = BuildRaceLine(lines, drawBegin, drawEnd, laneCenter, lineRoom, m_apexStrategy);
        m_racingLineRoads = std::move(contextRoads);
        m_racingLineStrategy = m_apexStrategy;
        RebuildRacingLineRender();
    }

    RacingPlan racing = BuildRacingPlan(m_raceLine, position, lookaheadDistance,
                                        RoadTargetSpeed(m_currentRoad), m_speed, m_maxAccel, m_maxBrake);
    PreviewUpcomingRoad(racing, m_path, m_pathIndex, m_maxSpeed, m_speed, m_maxBrake);

    Vec3 target = racing.target;
    float targetSteer = ComputeContextSteer(target, Stanley(racing.pathPoint, racing.pathDirection));
    RebuildCtxSteerRender();

    Steer(targetSteer);

    // 여전히 돌고 있는 동안엔 마찰원이 남긴 만큼만 가속(파워다운). 감속이 필요하면
    // (다가오는 코너의 마찰원 제동 요구, 또는 도로 제한속도) brakingAccel이 우선한다.
    float desiredAccel = m_speedGain * (racing.targetSpeed - m_speed);
    if (racing.brakingAccel < 0.0f)
        desiredAccel = std::min(desiredAccel, racing.brakingAccel);
    else
        desiredAccel = std::min(desiredAccel, racing.maxAccelNow);
    Accelerate(desiredAccel);

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
