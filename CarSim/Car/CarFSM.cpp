#include "Car.h"
#include "RacingLineQP.h"
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

#pragma region RaceHelpers

    struct RacingPathData
    {
        std::vector<Vec3> points;
        std::vector<float> arcLength;
        std::vector<float> curvature;
        size_t closest = 0;
    };

    struct RacingPlan
    {
        Vec3 target;
        Vec3 pathPoint;
        Vec3 pathDirection;
        Vec3 targetDirection; // lookahead 지점의 접선. 오프셋을 얹을 때 여기 법선을 쓴다
        float targetSpeed = 0.0f;
        float brakingAccel = 0.0f;
        float maxAccelNow = 0.0f; // 지금 곡률에서 마찰원이 남겨준 가속 여유
        float remainingDistance = 0.0f;
        bool hasLocalCorner = false;
        size_t closestIndex = 0;    // 레이싱 라인 위 내 최근접 점
        float currentS = 0.0f;      // 그 점의 호길이
        float brakeAvailNow = 0.0f; // 지금 곡률에서 마찰원이 남겨준 제동 여유
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

    // 전략 = 에이펙스를 코너의 앞/뒤 어디에 찍을지. 최소곡률 해의 오프셋 프로파일을
    // 통째로 s축으로 밀어서 만든다. 거리항 가중치를 키우는 방식은 코너 전체를 안쪽으로
    // 눌러 라인이 짜부되기만 하고 에이펙스 위치는 그대로라 쓰지 않는다.
    // 앞으로 당기면(양수) 봉우리가 일찍 와서 이른 에이펙스가 된다. 1m 간격이라 단위는 m.
    int StrategyApexShift(Car::ApexStrategy strategy)
    {
        switch (strategy)
        {
        case Car::ApexStrategy::Early:
            return 12;
        case Car::ApexStrategy::Late:
            return -12;
        default:
            return 0; // Apex -- 최소곡률 해 그대로(곡률 정점)
        }
    }

    // 도로를 이어 붙인 참조선은 이음매에서 헤딩이 튄다. 그대로 두면 두 가지가 망가진다.
    //   1) 곡률이 스파이크라 QP가 감당하지 못하고 꺾임이 라인에 그대로 남는다.
    //   2) 법선이 1m 사이에 크게 돌아, 오프셋을 얹은 라인이 옆으로 홱 튄다.
    // 원본에서 CENTERLINE_MAX_SHIFT 이내로만 움직이도록 묶어 두고 고주파 꺾임만 깎는다.
    // 여기서 잡은 만큼 레이싱 라인이 쓸 수 있는 폭(SharedLineRoom)이 줄어드니
    // 필요 이상으로 크게 잡지 않는다.
    constexpr float CENTERLINE_MAX_SHIFT = 0.5f;
    constexpr int CENTERLINE_SMOOTH_ITERATIONS = 120;

    // 라플라시안만 쓰면 닫힌 곡선이 통째로 수축해서 모든 코너가 안쪽으로 밀린다.
    // Taubin(축소 -> 팽창 교대)은 저주파를 보존하면서 고주파만 깎는다.
    void SmoothClosedPolyline(std::vector<Vec3> &points, float maxShift, int iterations)
    {
        const size_t n = points.size();
        if (n < 8 || maxShift <= 0.0f)
            return;

        const std::vector<Vec3> original = points;
        std::vector<Vec3> next(n);

        auto step = [&](float factor)
        {
            for (size_t i = 0; i < n; ++i)
            {
                const Vec3 &prev = points[(i + n - 1) % n];
                const Vec3 &forward = points[(i + 1) % n];
                Vec3 laplacian = (prev + forward) * 0.5f - points[i];
                next[i] = points[i] + laplacian * factor;
            }
            points.swap(next);
        };

        // 통과대역 k = 1/shrink + 1/inflate. 위치 스무딩은 여기까지만 하고
        // 남는 곡률 스파이크는 중앙값 필터로 따로 잡는다.
        constexpr float TAUBIN_SHRINK = 0.5f;
        constexpr float TAUBIN_INFLATE = -0.53f;
        for (int iteration = 0; iteration < iterations; ++iteration)
        {
            step(TAUBIN_SHRINK);
            step(TAUBIN_INFLATE);

            // 원본에서 너무 멀어지면 도로를 벗어난다. 매 반복마다 되끌어 온다.
            for (size_t i = 0; i < n; ++i)
            {
                Vec3 delta = points[i] - original[i];
                float distance = HorizontalLength(delta);
                if (distance > maxShift)
                    points[i] = original[i] + delta * (maxShift / distance);
            }
        }
    }

    // 곡률 스파이크 제거용 중앙값 필터.
    //
    // 이음매에 남은 꺾임은 한두 샘플짜리 곡률 스파이크로 나타나는데, 속도 계획이 이걸
    // 진짜 코너로 읽으면 멀쩡한 구간에서 급제동한다. 평균이 아니라 중앙값을 쓰는 이유는,
    // 진짜 코너(여러 샘플에 걸쳐 곡률이 유지됨)는 그대로 두고 고립된 이상치만 걷어내기
    // 위해서다.
    void MedianFilterCurvature(std::vector<float> &curvature, size_t halfWindow)
    {
        const size_t n = curvature.size();
        if (n < halfWindow * 2 + 1 || halfWindow == 0)
            return;

        const std::vector<float> source = curvature;
        std::vector<float> window;
        window.reserve(halfWindow * 2 + 1);
        for (size_t i = 0; i < n; ++i)
        {
            window.clear();
            const size_t begin = i > halfWindow ? i - halfWindow : 0;
            const size_t end = std::min(n, i + halfWindow + 1);
            for (size_t k = begin; k < end; ++k)
                window.push_back(source[k]);
            std::nth_element(window.begin(), window.begin() + window.size() / 2, window.end());
            curvature[i] = window[window.size() / 2];
        }
    }

    // 이음매에서 법선이 급회전하는 걸 막으려면 접선을 넓은 구간에서 잡아야 한다.
    // 바로 옆 점(±1)으로 잡으면 꺾임 각도가 그대로 법선에 실려 라인이 튄다.
    Vec3 OffsetSampleSmooth(const std::vector<Vec3> &samples, size_t i, float d, size_t window)
    {
        const size_t n = samples.size();
        const size_t back = i > window ? i - window : 0;
        const size_t forward = std::min(i + window, n - 1);
        float tx = samples[forward].GetX() - samples[back].GetX();
        float tz = samples[forward].GetZ() - samples[back].GetZ();
        float length = std::sqrt(tx * tx + tz * tz);
        if (length < 1e-5f)
            return OffsetSampleAt(samples, i, d);
        float rx = tz / length;
        float rz = -tx / length;
        return Vec3(samples[i].GetX() + rx * d, samples[i].GetY(), samples[i].GetZ() + rz * d);
    }

    // 결승선을 넘는 순간에도 lookahead와 속도 미리보기가 끊기지 않도록 랩 뒤에 붙이는 여유.
    // 최고속에서의 제동 미리보기(약 390m)보다 커야 한다.
    constexpr float WRAP_OVERLAP = 450.0f;

    // 트랙 한 바퀴 전체를 한 번에 푼다. 시작할 때 전략마다 한 번씩만 호출할 것(수백 ms).
    //
    // 닫힌 루프라 QP 경계 조건이 문제가 된다. 양 끝을 그냥 고정하면 결승선 부근 라인이
    // 찌그러지므로, 앞뒤로 한 바퀴의 꼬리/머리를 덧대어 풀고 가운데만 취한다.
    // 덧댄 뒷부분의 d는 랩 앞부분 값으로 덮어써서 겹침 구간이 정확히 이어지게 만든다.
    RaceLine BuildTrackRaceLine(const std::vector<const Spline *> &loop, const std::vector<int> &roadIds,
                                float laneCenter, float lineRoom, Car::ApexStrategy strategy)
    {
        RaceLine line;
        std::vector<Vec3> merged;
        for (const Spline *spline : loop)
        {
            const std::vector<Vec3> &points = spline->GetSplinePoints();
            merged.insert(merged.end(), points.begin(), points.end());
        }
        if (merged.size() < 2)
            return line;

        constexpr float SPACING = 1.0f;
        std::vector<Vec3> lap = ResampleUniform(merged, SPACING);
        const size_t lapCount = lap.size();
        if (lapCount < 16)
            return line;

        // 이음매 꺾임 제거. 닫힌 루프이므로 양끝을 감아서 처리한다.
        SmoothClosedPolyline(lap, CENTERLINE_MAX_SHIFT, CENTERLINE_SMOOTH_ITERATIONS);

        const size_t pad = std::min(lapCount / 2, static_cast<size_t>(WRAP_OVERLAP / SPACING));

        // [꼬리 pad][랩 전체][머리 pad]
        RacingPathData ref;
        ref.points.reserve(lapCount + pad * 2);
        ref.points.insert(ref.points.end(), lap.end() - pad, lap.end());
        ref.points.insert(ref.points.end(), lap.begin(), lap.end());
        ref.points.insert(ref.points.end(), lap.begin(), lap.begin() + pad);
        FillRacingMetrics(ref, ref.points.front());
        // QP가 스파이크를 없애려고 라인을 통째로 휘게 만드는 걸 막는다.
        // 여긴 좁게 -- 진짜 코너 형상은 QP가 봐야 하므로.
        constexpr size_t QP_CURVATURE_MEDIAN_HALF = 2;
        MedianFilterCurvature(ref.curvature, QP_CURVATURE_MEDIAN_HALF);

        std::vector<float> dPadded = RacingLineQP::SolveMinCurvatureCascade(
            ref.curvature, SPACING, -lineRoom, lineRoom, 0.0f);

        // 우리가 쓸 구간은 [pad, pad + lapCount + pad) -- 한 바퀴 + 겹침.
        // 겹침 구간 d를 랩 앞부분 값으로 맞춰야 결승선에서 좌표가 정확히 이어진다.
        // 전략 시프트는 랩 인덱스를 구한 뒤에 걸어야(랩 길이로 감싸서) 겹침 구간이
        // 랩 앞부분과 계속 같은 값을 갖는다.
        const size_t keep = lapCount + pad;
        const int apexShift = StrategyApexShift(strategy);
        std::vector<float> d(keep);
        for (size_t i = 0; i < keep; ++i)
        {
            const size_t lapIndex = (i < lapCount ? i : i - lapCount);
            const size_t shifted = static_cast<size_t>(
                (static_cast<long long>(lapIndex) + apexShift + static_cast<long long>(lapCount)) %
                static_cast<long long>(lapCount));
            d[i] = dPadded[pad + shifted];
        }

        RacingPathData solved;
        solved.points.reserve(keep);
        for (size_t i = 0; i < keep; ++i)
        {
            // 겹침 구간은 랩 앞부분과 같은 기하를 보게 해서 좌표가 완전히 일치하도록 한다.
            size_t src = pad + (i < lapCount ? i : i - lapCount);
            constexpr size_t NORMAL_WINDOW = 4; // 1m 간격이므로 8m 구간에서 접선을 잡는다
            solved.points.push_back(OffsetSampleSmooth(ref.points, src, laneCenter + d[i], NORMAL_WINDOW));
        }
        FillRacingMetrics(solved, solved.points.front());

        // 속도 계획이 읽는 곡률에서는 더 넓게 본다. 이음매 스파이크가 좁은 창을 통과해
        // 남으면, 그걸 진짜 코너로 읽고 멀쩡한 직선에서 급제동한다.
        constexpr size_t SPEED_CURVATURE_MEDIAN_HALF = 5;
        MedianFilterCurvature(solved.curvature, SPEED_CURVATURE_MEDIAN_HALF);

        line.points = std::move(solved.points);
        line.arcLength = std::move(solved.arcLength);
        line.curvature = std::move(solved.curvature);
        line.laneOffset = std::move(d);
        line.lapPoints = lapCount;
        line.lapLength = line.arcLength[lapCount];

        // 도로 경계 s를 기록해 둔다. 앞 도로 제한속도 미리보기가 m_path 대신 이걸 쓴다.
        //
        // 참조선 길이를 누적해서 쓰면 안 된다 -- 레이싱 라인은 최대 lineRoom만큼 옆으로
        // 밀려 있어 길이가 참조선과 1~2% 다르고, 한 바퀴를 돌면 그 오차가 수십 m가 된다.
        // 각 도로의 시작점을 실제 라인 위에서 찾는다(시작할 때 한 번뿐이라 전체 탐색해도 된다).
        line.roadIds = roadIds;
        line.roadStartS.reserve(loop.size());
        for (const Spline *spline : loop)
        {
            const std::vector<Vec3> &roadPoints = spline->GetSplinePoints();
            if (roadPoints.empty())
            {
                line.roadStartS.push_back(0.0f);
                continue;
            }
            size_t best = 0;
            float bestDistance = std::numeric_limits<float>::max();
            for (size_t i = 0; i < lapCount; ++i)
            {
                float distance = HorizontalLength(line.points[i] - roadPoints.front());
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    best = i;
                }
            }
            line.roadStartS.push_back(line.arcLength[best]);
        }
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

    // 공유 라인은 트랙 한 바퀴(7000점 이상)라, 매 프레임 차마다 전체를 훑으면 QP를 없애
    // 아낀 비용을 그대로 되뱉는다. 직전 위치 주변만 보고, 벗어났을 때만 전체를 다시 찾는다.
    size_t ClosestOnLineCached(const RaceLine &line, const Vec3 &position, size_t &cursor)
    {
        const size_t n = line.points.size();
        if (n == 0)
            return 0;

        // 1m 간격이므로 한 프레임에 움직이는 거리(최고속에서도 2m 남짓)의 수십 배면 충분하다.
        constexpr size_t WINDOW = 64;
        constexpr float REACQUIRE_DISTANCE_SQ = 25.0f * 25.0f;

        size_t begin = cursor > WINDOW ? cursor - WINDOW : 0;
        size_t end = std::min(n, cursor + WINDOW + 1);

        size_t closest = begin;
        float best = std::numeric_limits<float>::max();
        for (size_t i = begin; i < end; ++i)
        {
            float distance = (line.points[i] - position).LengthSq();
            if (distance < best)
            {
                best = distance;
                closest = i;
            }
        }

        // 창 안에서 못 찾았으면(스폰 직후, 순간이동, 랩 되감기 직후) 한 번만 전체 탐색.
        if (best > REACQUIRE_DISTANCE_SQ)
            closest = ClosestOnLine(line, position);

        // 겹침 구간에 들어갔으면 한 바퀴 되감는다. 좌표가 정확히 같으므로 위치는 그대로다.
        if (line.lapPoints > 0 && closest >= line.lapPoints)
            closest -= line.lapPoints;

        cursor = closest;
        return closest;
    }

    size_t IndexAtS(const RaceLine &line, float s)
    {
        size_t i = static_cast<size_t>(
            std::lower_bound(line.arcLength.begin(), line.arcLength.end(), s) - line.arcLength.begin());
        return std::min(i, line.points.size() - 1);
    }

    // 차체가 라인과 나란하지 않으면 옆으로 차지하는 폭이 반폭보다 넓다.
    // 5.67m짜리 차가 5도만 틀어져도 0.25m를 더 먹고, 두 대면 0.5m다 -- 반폭만으로
    // 판정하면 "여유 0.3m 확보"라고 믿으면서 실제로는 스친다.
    float LateralHalfExtent(const Car &car, const Vec3 &tangent)
    {
        Vec3 forward = car.GetForwardAxis();
        // tangent의 오른쪽 법선에 차체 축을 투영한다.
        const float normalX = tangent.GetZ();
        const float normalZ = -tangent.GetX();
        const float alongForward = std::fabs(forward.GetX() * normalX + forward.GetZ() * normalZ);
        const float alongRight = std::fabs(forward.GetZ() * normalX - forward.GetX() * normalZ);
        return alongForward * car.GetLength() * 0.5f + alongRight * car.GetHalfWidth();
    }

    // 레이싱 라인 기준 프레네 좌표. d는 ComputeReferenceOffset과 같은 규약(오른쪽 법선 부호).
    struct FrenetPoint
    {
        float s = 0.0f;
        float d = 0.0f;
        Vec3 tangent = Vec3(1.0f, 0.0f, 0.0f);
    };

    // 월드 좌표를 레이싱 라인 위로 투영한다. 세그먼트 단위로 내려 찍으므로 1m 샘플 간격보다
    // 잘게 s가 나온다 -- 갭을 재는 데 쓰이니 양자화 오차가 그대로 거리 오차가 된다.
    // 전 구간을 훑으면 (차 수 x 점 수)가 되니 관심 구간은 호출자가 좁혀서 넘긴다.
    bool ProjectOnRaceLine(const RaceLine &line, const Vec3 &position, size_t beginIndex, size_t endIndex,
                           FrenetPoint &out)
    {
        endIndex = std::min(endIndex, line.points.size());
        if (beginIndex + 1 >= endIndex)
            return false;

        float bestDistance = std::numeric_limits<float>::max();
        bool found = false;
        for (size_t i = beginIndex; i + 1 < endIndex; ++i)
        {
            Vec3 seg = line.points[i + 1] - line.points[i];
            float segLen = HorizontalLength(seg);
            if (segLen < 1e-4f)
                continue;

            // 고도차는 무시한다. 입체교차는 UpdateSensors의 VERTICAL_SEPARATION이 이미 걸러낸다.
            Vec3 tangent(seg.GetX() / segLen, 0.0f, seg.GetZ() / segLen);
            Vec3 rel = position - line.points[i];
            float along = std::clamp(rel.GetX() * tangent.GetX() + rel.GetZ() * tangent.GetZ(), 0.0f, segLen);
            Vec3 toPoint = position - (line.points[i] + tangent * along);
            float distance = HorizontalLength(toPoint);
            if (distance >= bestDistance)
                continue;

            bestDistance = distance;
            out.s = line.arcLength[i] + along;
            out.d = toPoint.GetX() * tangent.GetZ() - toPoint.GetZ() * tangent.GetX();
            out.tangent = tangent;
            found = true;
        }
        return found;
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

    RacingPlan BuildRacingPlan(const RaceLine &line, const Vec3 &position, size_t &cursor, float lookahead,
                               float cruiseSpeed, float currentSpeed, float maxAccel, float maxBrake,
                               float frictionAccel)
    {
        RacingPlan plan;
        plan.target = position;
        plan.pathPoint = position;
        plan.targetSpeed = cruiseSpeed;
        plan.brakeAvailNow = maxBrake;
        if (line.points.size() < 2)
            return plan;

        const size_t closest = ClosestOnLineCached(line, position, cursor);
        const float currentS = line.arcLength[closest];
        plan.closestIndex = closest;
        plan.currentS = currentS;
        // 공유 라인은 닫힌 루프라 "남은 거리"는 이번 바퀴가 끝날 때까지의 거리다.
        plan.remainingDistance = std::max(0.0f, line.lapLength - currentS);

        // 코너를 서다/서다로 도는 게 아니라, 마찰원 안에서 브레이킹하며 진입하고
        // 여유가 생기는 만큼 가속하며 탈출하도록 구간 전체를 체이닝해서 푼다.
        //
        // 미리보는 거리를 고정으로 두면 안 된다. 최고속(약 88.9m/s)에서 완전정지까지
        // 필요한 거리가 v^2/(2*maxBrake) ~= 257m라, 100m로는 코너를 발견했을 때 이미
        // 너무 늦다. 코너 근처에서는 마찰원이 제동 여유를 더 깎으니 1.5배 여유를 둔다.
        const float currentSpeedSq = currentSpeed * currentSpeed;
        const float stopDistance = currentSpeedSq / (2.0f * std::max(1.0f, maxBrake));
        const float SPEED_PREVIEW = std::max(50.0f, stopDistance * 1.5f);
        constexpr float CURVATURE_MIN = 1.0f / 500.0f;
        const float FRICTION_ACCEL = std::max(0.1f, frictionAccel); // 타이어 마찰원 반경(횡/종 공유)

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
        // 이 값을 plan.maxAccelNow로 내보내 가속 명령 자체를 캡한다 -- 목표속도 배열만으론
        // 지금 순간의 가속 여유가 P제어기에 전달되지 않는다.
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
        plan.brakeAvailNow = brakeAvailNow;
        if (currentSpeed > plan.targetSpeed + 0.2f)
            plan.brakingAccel = -brakeAvailNow;

        plan.pathPoint = line.points[closest];
        const size_t targetIndex = IndexAtS(line, currentS + lookahead);
        plan.target = line.points[targetIndex];

        const size_t aheadIndex = IndexAtS(line, currentS + 2.0f);
        Vec3 plannedDirection = line.points[aheadIndex] - plan.pathPoint;
        if (HorizontalLength(plannedDirection) > 0.001f)
            plan.pathDirection = plannedDirection.Normalized();
        else if (closest + 1 < line.points.size())
            plan.pathDirection = (line.points[closest + 1] - plan.pathPoint).Normalized();

        // lookahead가 13m까지 가므로 코너에서는 근방 접선과 방향이 꽤 다르다.
        // 추월 오프셋을 근방 법선으로 얹으면 목표점이 엉뚱한 곳에 찍힌다.
        // (pathDirection이 정해진 뒤에 계산해야 한다 -- 폴백이 그 값을 쓴다)
        const size_t targetNext = std::min(targetIndex + 1, line.points.size() - 1);
        Vec3 targetTangent = line.points[targetNext] - line.points[targetIndex];
        plan.targetDirection = HorizontalLength(targetTangent) > 0.001f ? targetTangent.Normalized()
                                                                       : plan.pathDirection;
        return plan;
    }

    // 레이싱 라인이 트랙 전체를 덮으므로 도로 경계는 라인이 들고 있는 마커로 찾는다.
    // (차마다 다른 m_path를 훑을 이유가 없다 -- 공유 라인 자체가 순서를 안다)
    void PreviewUpcomingRoad(RacingPlan &plan, const RaceLine &line, float maxSpeed,
                             float currentSpeed, float maxBrake)
    {
        if (line.roadStartS.size() != line.roadIds.size() || line.roadStartS.empty())
            return;

        const float racingBrake = std::max(1.0f, maxBrake * 0.75f);
        // 고정 100m는 고속에서 정지거리보다 짧아 제한속도 급변 구간을 늦게 발견한다.
        const float SPEED_PREVIEW = std::max(50.0f, (currentSpeed * currentSpeed) / (2.0f * racingBrake) * 1.5f);

        const size_t roadCount = line.roadStartS.size();
        // 지금 있는 도로 구간을 찾고, 거기서부터 한 바퀴 안에서 앞으로 훑는다.
        size_t index = 0;
        for (size_t i = 0; i < roadCount; ++i)
            if (plan.currentS >= line.roadStartS[i])
                index = i;

        for (size_t step = 1; step <= roadCount; ++step)
        {
            const size_t next = (index + step) % roadCount;
            // 랩을 넘어가면 한 바퀴를 더해 거리로 환산한다.
            float startS = line.roadStartS[next];
            if (next <= index)
                startS += line.lapLength;
            const float distanceToRoad = startS - plan.currentS;
            if (distanceToRoad > SPEED_PREVIEW)
                break;

            const shared_ptr<Road> road = RoadDataManager::Get().GetRoad(line.roadIds[next]);
            if (road == nullptr)
                continue;

            float roadSpeed = std::min(road->GetSpeedLimit(), maxSpeed);
            float entrySpeed = std::sqrt(roadSpeed * roadSpeed +
                                         2.0f * racingBrake * std::max(0.0f, distanceToRoad));
            plan.targetSpeed = std::min(plan.targetSpeed, entrySpeed);
        }

        // 더 강하게(음수로 더 큰 쪽) 요구하는 제동만 반영 -- 코너용 마찰원 제동값을 덮어쓰지 않는다.
        if (currentSpeed > plan.targetSpeed + 0.2f)
            plan.brakingAccel = std::min(plan.brakingAccel, -racingBrake);
    }

#pragma endregion
}

#pragma region Common
void Car::UpdateMode()
{
    const char *reason = "";
    UpdateFindPath();
    if (!m_raceMode) // 레이스 중엔 주차 목적지가 끼어들면 안 된다
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
    // 레이스는 다른 모든 전이보다 우선한다. 켜져 있는 동안은 주차/목적지 로직이 끼어들면
    // 안 되고, 꺼지는 즉시 Stop으로 내려 일반 FSM이 처음부터 다시 판단하게 한다.
    if (m_raceMode)
    {
        if (m_currentRoad == nullptr) // 아직 도로를 못 잡았으면 Stop이 EnsureRoamingPath를 돌린다
            return Mode::Stop;
        if (m_mode != Mode::Race)
            *reason = "race mode on";
        return Mode::Race;
    }
    if (m_mode == Mode::Race)
    {
        *reason = "race mode off";
        return Mode::Stop;
    }

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
        if (m_subMode == SubMode::P_Exit)
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
        SetSubMode(BaseDriveSubMode());
        m_planAccelDebug = 0.0f;

        m_staticBlockTimer = 0.0f;
        m_stuck = false;
        m_maneuver = ManeuverState{};
        m_speedCap = -1.0f;
        m_sirenPulledOver = false;
        BeginSegment(std::make_unique<SplineFollowSegment>());
    }
    else if (m_mode == Mode::Race)
    {
        SetSubMode(SubMode::R_Racing);
        m_planAccelDebug = 0.0f;
        m_speedCap = -1.0f;
        m_maneuver = ManeuverState{};
        m_stuck = false;
        OnRaceModeEnter();
        // 레이스도 조향/가감속을 DriveControl에서 내므로 같은 세그먼트를 쓴다.
        BeginSegment(std::make_unique<SplineFollowSegment>());
    }
    else if (m_mode == Mode::Park)
    {
        SetSubMode(prev == Mode::Stop ? SubMode::P_Exit : SubMode::P_Enter_Prep);
        m_parkPlanPending = true;
        m_parkSequenceActive = true;
    }
    else if (m_mode == Mode::Stop)
    {

        if (prev == Mode::Park && m_subMode == SubMode::P_Enter_Align)
            BeginSegment(std::make_unique<CenterSteerSegment>());
        SetSubMode(SubMode::None);
    }
}

void Car::OnModeExit(Mode next)
{
    if (m_mode == Mode::Race && next != Mode::Race)
        OnRaceModeExit();
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
        // 레이싱 라인은 참조선 순서(=Forward successor 체인)로 만들어진다. 진행방향이
        // 그와 어긋나면 조향은 라인을 따라 앞으로 가는데 도로 추적(CheckPath)은 반대로
        // 돌아 경로가 통째로 어긋난다. GetClosestRoad는 Forward 차로가 없어 보이면
        // 방향을 뒤집는데(이 트랙은 차로가 전부 Backward로 추정된다) 레이스에선 그 보정이
        // 오히려 해가 되므로 Forward로 고정한다.
        RoadRef entry{pose.road, m_raceMode ? LaneDirection::Forward : pose.direction};
        float targetOffset = NearestBandOffset(entry, pose.d);
        // 레이스 그리드는 애초에 서로 붙여 세우는 게 정상이라 차선진입 안전검사를 걸면
        // 뒷줄이 영영 출발하지 못한다.
        if (!m_raceMode && pose.road != nullptr && !IsSafeLaneEntry(entry, targetOffset))
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

    if (m_subMode == SubMode::P_Exit)
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

        if (m_subMode == SubMode::P_Enter_Prep)
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

            SetSubMode(SubMode::P_Enter);
            BeginParkSpotLeg();
            return;
        }

        if (m_subMode == SubMode::P_Enter)
        {
            if (m_speed > STOPPED_SPEED)
            {
                AccelerateVel(0.0f);
                return;
            }
            SetSubMode(SubMode::P_Enter_Align);
            Vec3 spotTarget = m_parkSpot->position - m_parkSpot->direction.Normalized() * m_wheelbase;
            float spotAngleRad = DirectionToAngleRad(m_parkSpot->direction);
            if (PlanParkLegTo(spotTarget, spotAngleRad, true))
                return;
        }
    }

    if (m_subMode == SubMode::None || m_subMode == SubMode::P_Enter_Align)
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

    if (m_subMode == SubMode::P_Exit)
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

    if (m_subMode == SubMode::P_Enter_Prep)
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
    Vec3 pDir = bestSpline->GetDirectionAt(pParam) * dirSign;

    // 스팟 반대쪽으로 틀기
    Vec3 spotDir = m_parkSpot->direction.Normalized();
    float spotCross = pDir.GetX() * spotDir.GetZ() - pDir.GetZ() * spotDir.GetX();
    float yawSign = spotCross > 0.0f ? 1.0f : -1.0f;
    outAngleRad = DirectionToAngleRad(pDir) + yawSign * m_parkPreYaw;

    // 조향각만큼 옆으로도 밀기
    Vec3 pLeft(-pDir.GetZ(), 0.0f, pDir.GetX());
    float sideRatio = PARK_PRE_YAW_MAX > 0.0f ? m_parkPreYaw / PARK_PRE_YAW_MAX : 0.0f;
    outPos += pLeft * (yawSign * PARK_PRE_SIDE_MAX * sideRatio);
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
            SetSubMode(SubMode::P_Enter_Prep);
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
                SetSubMode(SubMode::P_Enter_Prep);
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
        SetSubMode(SubMode::P_Enter);
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
        m_parkPreYaw = PARK_PRE_YAW_MAX * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
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

    if (m_raceMode) // 레이스는 주차 목적지를 잡지 않는다. 도로만 잡히면 곧 Race로 넘어간다
        return;

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
            if (m_subMode != SubMode::D_Pursuit)
            {
                m_maneuver = ManeuverState{};
                m_staticBlockTimer = 0.0f;
                m_stuck = false;
                SetSubMode(SubMode::D_Pursuit);
            }
            TryStartStaticReverseEscape();
        }
        else
        {
            if (m_subMode == SubMode::D_Pursuit) // 역할 풀리면 일반 주행으로
            {
                m_maneuver = ManeuverState{};
                SetSubMode(SubMode::D_Normal);
            }
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
    if (m_mode == Mode::Race)
    {
        RaceControl();
        return;
    }

    if (m_reverseEscapeTimer > 0.0f) // 정면충돌 후진탈출
    {
        Steer(m_reverseEscapeSteer, REVERSE_STEER_RAMP);
        if (m_fleeOn)
            FullAccelerateVel(REVERSE_ESCAPE_SPEED);
        else
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
    else if (m_fleeOn && m_speed < FLEE_LAUNCH_SPEED && accelIDM > 0.0f)
    {
        FullAccelerate(accelIDM); // 출발은 즉시 최대가속
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
    // 레이스는 트랙 폭 그대로를 쓴다. GatherCandidateBands는 추격/도망용으로 도로 양끝
    // 밖에 가상차로를 하나씩 더 붙이는데(IgnoresTrafficRules가 레이스에서도 참이다),
    // 그걸 쓰면 코스 이탈이 영영 감지되지 않고 추월 오프셋도 도로 밖으로 나간다.
    // 반폭도 여기서는 온전히 뺀다 -- 절반만 빼면 차체 절반이 나가도 안쪽으로 판정돼
    // 복귀가 늦어진다.
    const bool race = m_mode == Mode::Race;
    float halfW = race ? GetHalfWidth() : GetHalfWidth() * 0.5f;

    outMin = -(RoadDataManager::ROAD_WIDTH - halfW);
    outMax = RoadDataManager::ROAD_WIDTH - halfW;

    std::vector<const LaneBand *> bands =
        race ? RoadDataManager::Get().GetAllDrivingBands(road.road)
             : GatherCandidateBands(road);
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
    // 레이스는 트랙 루프를 그대로 도는 거라 합류 대기가 성립하지 않는다. 여기서 멈추면
    // 도로 이음매마다 선두가 서고 뒤가 밀려 레이스가 성립하지 않는다.
    if (IgnoresTrafficRules() || nextRoad.road == nullptr)
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
    if (UsesReactiveSteer())
        m_sweepDebugCorners.clear(); // 스윕이 실제 조향과 어긋남
    else
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

// 스윕을 안 쓰니 앞범퍼 근접으로 직접 판정한다.
void Car::TryStartStaticReverseEscape()
{
    Vec3 position = GetRigidbodyPosition();
    Vec3 moved = position - m_staticBlockLastPos;
    m_staticBlockLastPos = position;

    // 지령속도는 벽에 밀려도 안 떨어진다
    float actualSpeed = m_deltaTime > 0.0001f ? moved.Length() / m_deltaTime : 0.0f;
    if (actualSpeed > STATIC_BLOCK_SPEED)
    {
        m_staticBlockTimer = 0.0f;
        return;
    }

    Vec3 forward = GetForwardAxis();
    Vec3 frontBumper = GetBodyCenter() + forward * m_halfExtents.GetZ();

    const VehicleCollision::Obstacle *blocker = nullptr;
    float nearestGap = STATIC_BLOCK_RANGE;
    for (const VehicleCollision::Obstacle &obstacle : m_obstacles)
    {
        if (obstacle.isVehicle)
            continue;

        // 장애물 y는 바닥값이라 차체 높이가 섞이면 안 된다
        Vec3 delta = VehicleCollision::ClosestPointOnObstacle(frontBumper, obstacle) - frontBumper;
        Vec3 toBlocker(delta.GetX(), 0.0f, delta.GetZ());
        float gap = toBlocker.Length();
        if (gap > nearestGap)
            continue;
        if (gap >= 0.01f && forward.Dot(toBlocker * (1.0f / gap)) < HEADON_FRONT_COS)
            continue;

        nearestGap = gap;
        blocker = &obstacle;
    }

    if (blocker == nullptr)
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
    m_reverseEscapeSteer = ComputeReverseEscapeSteer(blocker->center);
    m_reverseEscapeStartFwd = forward.Normalized();
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
        bool inRange = (other->GetPosition() - myPosition).Length() <= DETECT_RANGE;
        if (!inRange && !(m_chaseOn && other->m_fleeOn)) // 추격차는 도망차를 거리 무관 인지
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
    if (m_fleeOn || m_chaseOn) // 도망/추격차는 양보 안함
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
        SetSubMode(BaseDriveSubMode());
    }
    return false;
}

Car *Car::FindFleeTarget() const
{
    Car *best = nullptr;
    float bestDistance = std::numeric_limits<float>::max();

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
        SetSubMode(BaseDriveSubMode());
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
            SetSubMode(BaseDriveSubMode());
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

#pragma region Race

// 공유 라인은 모든 차종이 같이 쓰므로 가장 넓은 차를 기준으로 여유를 잡는다.
// 좁은 차 기준으로 잡으면 넓은 차가 라인을 따라갔을 때 도로 밖으로 나간다.
float Car::SharedLineRoom()
{
    constexpr float EDGE_MARGIN = 0.2f;
    constexpr float MAX_RACING_OFFSET = 6.0f;

    float widestHalfWidth = 0.0f;
    for (int i = 0; i < static_cast<int>(CarType::Count); ++i)
        widestHalfWidth = std::max(widestHalfWidth, GetCarSpec(static_cast<CarType>(i)).halfExtents.GetX());

    // 트랙 전체 폭을 쓴다. 방향으로 거른 밴드를 쓰면 안 된다 -- 참조선 왼쪽에 그려진
    // 차로는 방향 추정에서 Backward로 잡혀서, Forward만 물으면 트랙이 빈 것으로 나오고
    // lineRoom이 0으로 눌린다(= 라인이 참조선 그대로가 된다).
    float laneHalfWidth = 0.0f;
    const std::vector<shared_ptr<Road>> &roads = RoadDataManager::Get().GetRoads();
    if (!roads.empty())
    {
        float trackMin = 0.0f;
        float trackMax = 0.0f;
        if (RoadDataManager::Get().GetTrackExtent(roads.front(), trackMin, trackMax))
            laneHalfWidth = (trackMax - trackMin) * 0.5f;
    }
    // 이음매 꺾임을 깎느라 참조선 자체가 최대 CENTERLINE_MAX_SHIFT만큼 움직인다.
    // QP 경계는 그 움직인 참조선 기준이므로, 그만큼 빼야 실제 도로 밖으로 안 나간다.
    return std::clamp(laneHalfWidth - widestHalfWidth - EDGE_MARGIN - CENTERLINE_MAX_SHIFT,
                      0.0f, MAX_RACING_OFFSET);
}

// 트랙 루프를 따라가며 전략별로 한 번씩 QP를 푼다. 도로를 다 읽은 뒤 레이스를 시작하기
// 전에 한 번만 부를 것 -- 7000점 규모라 전략당 수백 ms 걸린다(Debug 기준).
void Car::BuildSharedRaceLines(SimulationState &simState)
{
    RoadDataManager &roadData = RoadDataManager::Get();
    const std::vector<shared_ptr<Road>> &roads = roadData.GetRoads();
    if (roads.empty())
        return;

    // successor를 따라 한 바퀴 돈다. 갈림길이 있으면 첫 번째를 택한다(서킷은 단일 루프).
    std::vector<const Spline *> loop;
    std::vector<int> roadIds;
    shared_ptr<Road> road = roads.front();
    const int startId = road->GetId();
    for (size_t guard = 0; guard < roads.size() && road != nullptr; ++guard)
    {
        loop.push_back(&road->GetReferenceLine());
        roadIds.push_back(road->GetId());

        const vector<RoadRef> &successors =
            roadData.GetRoadSuccessors(road->GetId(), LaneDirection::Forward);
        if (successors.empty() || successors.front().road == nullptr)
        {
            road = nullptr;
            break;
        }
        road = successors.front().road;
        if (road->GetId() == startId) // 한 바퀴 닫혔다
            break;
    }

    // 노면 중심. 참조선이 트랙 한쪽 가장자리에 그려져 있을 수 있으므로(이 트랙은 차로가
    // 참조선 왼쪽 -6에 통째로 있다) 밴드가 덮는 구간의 중앙을 쓴다.
    float laneCenter = 0.0f;
    float trackMin = 0.0f;
    float trackMax = 0.0f;
    if (roadData.GetTrackExtent(roads.front(), trackMin, trackMax))
        laneCenter = (trackMin + trackMax) * 0.5f;

    const float lineRoom = SharedLineRoom();

    std::vector<RaceLine> lines(static_cast<size_t>(ApexStrategy::Count));
    for (int i = 0; i < static_cast<int>(ApexStrategy::Count); ++i)
        lines[i] = BuildTrackRaceLine(loop, roadIds, laneCenter, lineRoom,
                                      static_cast<ApexStrategy>(i));

    // 라인이 이상할 때 제일 먼저 봐야 할 값들이다. room이 0이면 라인 = 참조선이고,
    // roads가 1이면 successor 루프가 안 닫힌 것이다.
    DebugConsole::Log("RaceLine: " + ToString(static_cast<int>(loop.size())) + " roads, " +
                      ToString(static_cast<int>(lines[0].lapLength)) + " m lap, center " +
                      ToString(laneCenter) + " m, room +-" + ToString(lineRoom) + " m");
    simState.SetRaceLines(std::move(lines));
}

void Car::SetRaceMode(bool on)
{
    if (m_raceMode == on)
        return;
    m_raceMode = on;
    // 실제 성능 envelope 교체는 모드 전이(OnModeEnter/Exit)에서 한 번만 한다.
    // 여기서 직접 바꾸면 UpdateMode가 아직 Race로 넘어가기 전에 값이 어긋난다.
    if (on)
        m_roaming = true; // 레이스는 목적지가 없다. 트랙 루프를 계속 돈다
}

// 레이싱 성능으로 갈아끼운다. 일반 주행 값은 되돌릴 수 있게 스냅샷을 남긴다.
void Car::OnRaceModeEnter()
{
    m_savedMaxAccel = m_maxAccel;
    m_savedJerkUp = m_jerkUp;
    m_savedJerkDown = m_jerkDown;

    m_maxSpeed = RACE_BASE_MAX_SPEED * m_personality.topSpeedFactor;
    m_maxAccel = RACE_BASE_MAX_ACCEL * m_personality.accelFactor;
    m_maxBrake = RACE_BASE_MAX_BRAKE * m_personality.brakeFactor;
    m_gripAccel = RACE_BASE_GRIP_ACCEL * m_personality.gripFactor;
    m_speedGain = RACE_SPEED_GAIN;
    // MakeRacerPersonality가 준 저크는 이미 레이싱 값이다. 일반 프리셋(4/15)으로
    // 들어온 차는 그대로 두면 스로틀·브레이크가 느려 계획을 못 따라간다.
    m_jerkUp = std::max(m_jerkUp, 22.0f);
    m_jerkDown = std::max(m_jerkDown, 45.0f);
    m_raceSpeedCap = m_maxSpeed;

    // 라인 바인딩과 계측은 다음 RaceControl이 잡는다.
    m_raceLine = nullptr;
    m_attackLine = nullptr;
    m_boundStrategy = ApexStrategy::Count;
    m_raceLineCursor = 0;
    m_raceLateralOffsetInitialized = false;
    m_raceS = -1.0f;
    m_lapStartTime = -1.0f;
    m_overtakeState = OvertakeState::None;
    m_overtakeTarget = nullptr;
    m_overtakeSide = 0.0f;
    m_overtakeHoldOffset = false;
    m_yieldTarget = nullptr;
    m_yieldSide = 0.0f;
    m_slipstream = 0.0f;
    m_leader = LeaderInfo{};
}

void Car::OnRaceModeExit()
{
    m_maxSpeed = ROAD_MAX_SPEED;
    m_maxBrake = ROAD_MAX_BRAKE;
    m_speedGain = ROAD_SPEED_GAIN;
    m_maxAccel = m_savedMaxAccel > 0.0f ? m_savedMaxAccel : m_personality.maxAccel;
    m_jerkUp = m_savedJerkUp > 0.0f ? m_savedJerkUp : m_personality.jerkUp;
    m_jerkDown = m_savedJerkDown > 0.0f ? m_savedJerkDown : m_personality.jerkDown;

    m_raceLine = nullptr;
    m_attackLine = nullptr;
    m_racingLineRender.SetModel(nullptr);
    m_overtakeState = OvertakeState::None;
    m_overtakeTarget = nullptr;
    m_yieldTarget = nullptr;
    m_raceLateralOffset = 0.0f;
    m_slipstream = 0.0f;
    m_leader = LeaderInfo{};
}

void Car::OnLapCompleted()
{
    const float now = m_SimState->GetSimTime();
    if (m_lapStartTime >= 0.0f) // 스폰 지점이 결승선이 아니므로 첫 통과는 계측하지 않는다
    {
        m_lastLapTime = now - m_lapStartTime;
        if (m_bestLapTime <= 0.0f || m_lastLapTime < m_bestLapTime)
            m_bestLapTime = m_lastLapTime;
    }
    ++m_lap;
    m_lapStartTime = now;
}

bool Car::IsOutsideDrivingBands() const
{
    if (m_currentRoad == nullptr)
        return false;

    const Spline &referenceLine = m_currentRoad->GetReferenceLine();
    if (referenceLine.GetSplinePoints().size() < 2)
        return false;

    Vec3 center = GetBodyCenter();
    float minOffset = 0.0f;
    float maxOffset = 0.0f;
    ComputeDrivableRange(CurrentRoadRef(), minOffset, maxOffset);
    float centerOffset = ComputeReferenceOffset(referenceLine, center);
    return centerOffset < minOffset || centerOffset > maxOffset;
}

// Mode::Race 틱. 경로(m_path/m_currentRoad)는 일반 주행과 같은 코드를 쓰고 -- 레이스는
// IgnoresTrafficRules라 신호/교차로 대기가 자동으로 빠진다 -- 실제 조향/가감속은
// SplineFollowSegment -> DriveControl -> RaceControl로 내려간다.
void Car::UpdateRace()
{
    if (!CheckPath())
        return;

    if (m_currentTime - m_lastBehaviorPlanTime >= BEHAVIOR_PLAN_INTERVAL)
    {
        UpdateSensors();
        UpdateRacePlan();
        m_lastBehaviorPlanTime = m_currentTime;
    }

    m_wantSegmentTick = true;
}

// 조향이 실제 위치를 만든다. 계획 오프셋은 따라가기만 하되 도로 밖으론 안 나가게 클램프.
void Car::UpdateRacePlan()
{
    if (m_currentRoad == nullptr)
        return;

    float minOffset = 0.0f;
    float maxOffset = 0.0f;
    ComputeDrivableRange(CurrentRoadRef(), minOffset, maxOffset);
    float realOffset = ComputeReferenceOffset(m_currentRoad->GetReferenceLine(), GetPosition());
    SetCurrentOffset(std::clamp(realOffset, minOffset, maxOffset));
    m_currentSpline = RoadDataManager::Get().BuildOffsetSpline(m_currentRoad, m_currentOffset, m_travelDir);
    RebuildSplineRender();
}

void Car::RaceControl()
{
    const Vec3 position = GetRigidbodyPosition();
    const bool outsideRoad = IsOutsideDrivingBands();

    // 레이싱 라인은 시작할 때 전략별로 한 번 푼 트랙 전체 라인을 공유한다.
    // 전략을 바꿨을 때만 다른 라인으로 갈아끼우면 되고, 여기서 QP를 돌리지 않는다.
    if (m_raceLine == nullptr || m_boundStrategy != m_apexStrategy)
    {
        const RaceLine *shared = m_SimState->GetRaceLine(static_cast<int>(m_apexStrategy));
        if (shared == nullptr)
            return; // 아직 안 만들어졌으면 이번 프레임은 아무것도 하지 않는다
        m_raceLine = shared;
        // 추월용 참고 라인. 지금 라인이 이미 Early면 델타가 0이 되어 자연히 비활성화된다.
        m_attackLine = m_SimState->GetRaceLine(static_cast<int>(ApexStrategy::Early));
        m_boundStrategy = m_apexStrategy;
        m_raceLineCursor = 0; // 새 라인에서 다시 찾게 한다
        m_raceLateralOffsetInitialized = false;
        RebuildRacingLineRender();
    }
    const RaceLine &raceLine = *m_raceLine;

    // 고속일수록 시야를 길게 잡아야 조향이 튀지 않고 완만해진다.
    float lookaheadDistance = std::clamp(4.5f + m_speed * 0.35f, 5.0f, 13.0f);
    // 공유 라인이라 폭도 공유값이다. 추월 오프셋 클램프에 쓰는 여유는 라인을 만들 때
    // 쓴 값과 같아야 하므로 SharedLineRoom()으로 한 곳에서 뽑는다.
    const float lineRoom = SharedLineRoom();

    // 와류는 직전 프레임 값을 쓴다. 계획을 세우려면 속도 상한이 먼저 필요한데, 그 상한을
    // 정하는 와류 세기는 계획이 내놓는 (s, d)로 찾기 때문이다. 아래에서 감쇠를 걸어 천천히
    // 변하게 만들었으므로 한 프레임 지연은 드러나지 않는다.
    m_raceSpeedCap = outsideRoad
                         ? m_maxSpeed * 0.5f
                         : m_maxSpeed * (1.0f + SLIPSTREAM_TOP_SPEED_GAIN * m_slipstream);
    const float slipMaxAccel = outsideRoad
                                   ? m_maxAccel * 0.5f
                                   : m_maxAccel * (1.0f + SLIPSTREAM_ACCEL_GAIN * m_slipstream);

    const float cruiseSpeed = m_currentRoad != nullptr
                                  ? std::min(m_currentRoad->GetSpeedLimit(), m_raceSpeedCap)
                                  : m_raceSpeedCap;
    RacingPlan racing = BuildRacingPlan(raceLine, position, m_raceLineCursor, lookaheadDistance,
                                        cruiseSpeed, m_speed, slipMaxAccel, m_maxBrake,
                                        m_gripAccel * PLAN_GRIP_RATIO);
    PreviewUpcomingRoad(racing, raceLine, m_raceSpeedCap, m_speed, m_maxBrake);

    // 앞차를 보는 게 회피 조향뿐이면, 피할 공간이 없을 때 감속 없이 그대로 밀고 들어간다.
    // 뒤에 안정적으로 붙어 기회를 기다리는 그림이 나오려면 종방향에서도 앞차를 봐야 한다.
    RaceNeighbors neighbors = FindRaceNeighbors(raceLine, racing.closestIndex, racing.currentS);
    m_leader = neighbors.leader;

    // 레이스 계측 스냅샷. 순위/랩타임이 전부 이 값들을 읽는다.
    m_lapLength = raceLine.lapLength;
    m_raceD = neighbors.leader.myLateralOffset;
    // s가 한 바퀴 가까이 되돌아갔으면 결승선을 넘은 것이다.
    if (m_raceS >= 0.0f && racing.currentS < m_raceS - raceLine.lapLength * 0.5f)
        OnLapCompleted();
    m_raceS = racing.currentS;
    m_slipstreamGapDebug = neighbors.slipstreamGap;
    // 와류는 붙고 떨어지는 데 시간이 걸린다. 프레임마다 튀게 두면 속도가 덜컹거린다.
    m_slipstream += (neighbors.slipstream - m_slipstream) *
                    std::min(1.0f, m_deltaTime * SLIPSTREAM_RESPONSE);

    // 전술 레이어: 몇 초짜리 추월 의도를 상태로 유지하고 목표 횡오프셋만 내놓는다.
    float laneOffsetAtCar = racing.closestIndex < raceLine.laneOffset.size()
                                ? raceLine.laneOffset[racing.closestIndex]
                                : 0.0f;
    // 출발 그리드의 실제 횡위치를 첫 목표로 삼는다. 0에서 시작하면 모든 차가 출발과 동시에
    // 같은 레이싱 라인으로 수렴해, 가속하기도 전에 옆 차와 서로 교차한다.
    if (!m_raceLateralOffsetInitialized && neighbors.selfProjected)
    {
        m_raceLateralOffset = neighbors.leader.myLateralOffset;
        m_raceLateralOffsetInitialized = true;
    }
    // 곡률은 추월 시작 조건에만 쓴다 -- 코너 한복판에서 나란히 서지 않게.
    float curvatureAtCar = racing.closestIndex < raceLine.curvature.size()
                               ? raceLine.curvature[racing.closestIndex]
                               : 0.0f;
    m_raceCurvature = curvatureAtCar;
    if (outsideRoad)
    {
        // 도로 밖에서는 추월보다 복귀가 우선이다. 이전 추월 상태가 복귀 목표를 다시
        // 바깥쪽으로 밀지 않도록 전술 상태를 해제한다.
        m_overtakeState = OvertakeState::None;
        m_overtakeTarget = nullptr;
        m_overtakeSide = 0.0f;
        m_overtakeHoldOffset = false;
        m_yieldTarget = nullptr;
        m_yieldSide = 0.0f;
    }
    else
    {
        UpdateOvertake(raceLine, racing.closestIndex, neighbors, lineRoom, laneOffsetAtCar,
                       racing.targetSpeed, curvatureAtCar);
    }

    // 경로 레이어: QP를 다시 풀지 않고 추종 점만 옆으로 민다. 라인을 벗어난 만큼
    // 코너 속도를 손해 보는 건 실제로도 맞다(속도 계획은 원래 라인 곡률 기준이다).
    if (std::fabs(m_raceLateralOffset) > 0.01f)
    {
        Vec3 nearRight(racing.pathDirection.GetZ(), 0.0f, -racing.pathDirection.GetX());
        Vec3 farRight(racing.targetDirection.GetZ(), 0.0f, -racing.targetDirection.GetX());
        racing.pathPoint = racing.pathPoint + nearRight * m_raceLateralOffset;
        racing.target = racing.target + farRight * m_raceLateralOffset;
    }

    if (outsideRoad && m_currentRoad != nullptr)
    {
        // 현재 도로의 가장 가까운 안쪽 경계로 복귀한다. 경계에 정확히 붙이면 수치 오차로
        // on/off가 반복되므로 차체 중심 목표를 조금 더 안쪽에 둔다.
        const Spline &referenceLine = m_currentRoad->GetReferenceLine();
        float minOffset = 0.0f;
        float maxOffset = 0.0f;
        ComputeDrivableRange(CurrentRoadRef(), minOffset, maxOffset);

        const Vec3 frontAxle = GetPosition();
        const float actualOffset = ComputeReferenceOffset(referenceLine, GetBodyCenter());
        constexpr float RECOVERY_INSET = 0.5f;
        const float inset = std::min(RECOVERY_INSET, std::max(0.0f, (maxOffset - minOffset) * 0.25f));
        const float recoveryOffset = actualOffset < minOffset ? minOffset + inset : maxOffset - inset;

        const float t = std::clamp(referenceLine.GetSplinePosition(frontAxle), 0.0f, 1.0f);
        Vec3 recoveryDirection = referenceLine.GetDirectionAt(t);
        if (recoveryDirection.LengthSq() > 0.000001f)
            recoveryDirection = recoveryDirection.Normalized();
        else
            recoveryDirection = GetForwardAxis();
        Vec3 recoveryRight(recoveryDirection.GetZ(), 0.0f, -recoveryDirection.GetX());

        racing.pathPoint = referenceLine.GetPositionAt(t) + recoveryRight * recoveryOffset;
        racing.pathDirection = recoveryDirection;
        racing.target = OffsetLookaheadPoint(referenceLine, frontAxle,
                                             std::max(10.0f, lookaheadDistance), recoveryOffset, false);
    }

    Vec3 target = racing.target;
    float headingRad = DirectionToAngleRad(GetForwardAxis());
    float pursuitSteer = std::clamp(PurePursuitSteerAt(position, headingRad, target, m_wheelbase),
                                    -m_maxSteerAngle, m_maxSteerAngle);
    float targetSteer = ComputeRaceContextSteer(pursuitSteer);
    RebuildCtxSteerRender();

    Steer(targetSteer);

    // 여전히 돌고 있는 동안엔 마찰원이 남긴 만큼만 가속(파워다운). 감속이 필요하면
    // (다가오는 코너의 마찰원 제동 요구, 또는 도로 제한속도) brakingAccel이 우선한다.
    float desiredAccel = m_speedGain * (racing.targetSpeed - m_speed);
    if (racing.brakingAccel < 0.0f)
        desiredAccel = std::min(desiredAccel, racing.brakingAccel);
    else
        desiredAccel = std::min(desiredAccel, racing.maxAccelNow);
    if (m_leader.IsValid())
    {
        // 나가기로 했으면 더 바짝 붙어야 오버랩을 만들 수 있다.
        float headwayScale = m_overtakeState == OvertakeState::Attack ? OVERTAKE_HEADWAY_SCALE : 1.0f;
        desiredAccel = std::min(desiredAccel,
                                LeaderFollowAccel(m_leader, racing.brakeAvailNow, headwayScale));
    }
    if (outsideRoad && desiredAccel > 0.0f)
        desiredAccel = std::min(desiredAccel, m_maxAccel * 0.5f);
    Accelerate(desiredAccel);

    DirectX::XMFLOAT3 targetMarkerPos = ToXMFLOAT3(target);
    targetMarkerPos.y = GetPosition().GetY() + 0.2f;
    m_targetMarker.GetTransform().SetPosition(targetMarkerPos);
}

// 레이싱 라인에 투영해 앞차를 훑는다. 추종 대상(leader)과 슬립스트림을 한 패스에서
// 같이 뽑는다 -- 둘 다 같은 (s, d)를 필요로 하는데 투영이 이 함수에서 제일 비싼 일이다.
//
// leader: 뒤차/마주 오는 차/횡으로 빗겨난 차는 제외한다. 그런 차까지 종방향 제어에 넣으면
//         옆을 스쳐 지나가는 것만으로 브레이크를 밟게 된다.
// 슬립스트림: 횡 조건이 훨씬 느슨하다. 추월하려고 라인을 벗어나는 순간 와류가 뚝 끊기면
//         "토우 받다가 늦게 빠져나와 브레이킹에서 찌른다"는 기본기가 성립하지 않는다.
RaceNeighbors Car::FindRaceNeighbors(const RaceLine &line, size_t myIndex, float myS) const
{
    constexpr float LEADER_RANGE = 80.0f; // 이보다 먼 앞차는 종방향에 영향 없음
    // "옆으로 지나갈 수 있다"고 판단하는 기준. 이 값이 작으면 아직 안 벌어진 상태로
    // 옆에 서게 되고, 나란히 겹친 뒤에야 leader로 인식된다 -- 그 시점엔 제동으로
    // 횡방향 공간이 생기지 않는다. 확실히 비켜서지 못한 차는 그냥 뒤에 줄을 서는 게 맞다.
    constexpr float LATERAL_MARGIN = 1.0f;
    // 앞지른 뒤에도 대상이 이 구간 안에 남아 있어야 "통과했다"를 판정할 수 있다.
    constexpr size_t BACK_WINDOW = 25; // 뒤쪽으로 볼 샘플 수(약 1m 간격)
    // 회피 조향이 옆으로 밀어내는 순간 leader가 풀리고, 라인으로 되돌아오면 다시 걸린다.
    // 붙잡는 쪽 임계를 넓혀 그 경계에서 가감속이 떨리는 걸 막는다.
    constexpr float RELEASE_HYSTERESIS = 1.4f;

    RaceNeighbors neighbors;
    if (line.points.size() < 2)
        return neighbors;

    // 내 뒤~LEADER_RANGE 앞까지만 투영 구간으로 쓴다. 라인 전체를 훑을 이유가 없다.
    size_t windowBegin = myIndex > BACK_WINDOW ? myIndex - BACK_WINDOW : 0;
    size_t windowEnd = IndexAtS(line, myS + LEADER_RANGE) + 1;

    FrenetPoint mine;
    if (!ProjectOnRaceLine(line, GetBodyCenter(), windowBegin, windowEnd, mine))
        return neighbors;
    neighbors.selfProjected = true;
    neighbors.leader.myLateralOffset = mine.d;

    // 내 차체가 라인 기준으로 옆으로 차지하는 폭. 매 상대마다 다시 구할 필요는 없다.
    const float myLateralExtent = LateralHalfExtent(*this, mine.tangent);

    const Car *previousLeader = m_leader.car; // 호출자가 반환값을 대입하기 전이라 아직 직전 프레임 값
    float bestGap = std::numeric_limits<float>::max();
    for (Car *other : m_nearbyCars)
    {
        if (!m_SimState->IsCarAlive(other)) // 0.2초 캐시라 이미 사라진 차가 있을 수 있다
            continue;

        FrenetPoint theirs;
        if (!ProjectOnRaceLine(line, other->GetBodyCenter(), windowBegin, windowEnd, theirs))
            continue;

        float deltaS = theirs.s - mine.s;

        // 추월 대상만은 앞/뒤 무관하게 기록한다. 옆으로 나가면 leader 조건(횡 오버랩)에서
        // 빠지는데, 그때도 "앞질렀는가"를 판정하려면 상대 위치를 계속 알아야 한다.
        if (other == m_overtakeTarget)
        {
            neighbors.targetSeen = true;
            neighbors.targetDeltaS = deltaS;
            neighbors.targetLateral = theirs.d - mine.d;
            neighbors.targetSpeed = other->GetSpeed();
            neighbors.targetRequiredLateral = myLateralExtent +
                                              LateralHalfExtent(*other, theirs.tangent) +
                                              OVERTAKE_SETUP_CLEARANCE;
        }

        if (other == m_yieldTarget)
        {
            neighbors.yieldTargetSeen = true;
            neighbors.yieldTargetDeltaS = deltaS;
            neighbors.yieldTargetLateral = theirs.d - mine.d;
        }

        // 종방향으로 겹쳐 있으면 나란히 달리는 중이다. 앞/뒤 구분 없이 잡아야 한다 --
        // 추월당하는 쪽에게 상대는 "뒤차"라 leader 경로로는 영영 안 보인다.
        const float halfLengthSum = (GetLength() + other->GetLength()) * 0.5f;
        const float sideLateral = theirs.d - mine.d;

        // 현재 겹친 차뿐 아니라 횡이동이 끝나기 전에 종방향으로 만날 차까지 점유 구간에 넣는다.
        // 목적지만 검사하면 이동 중 옆 차를 가로지를 수 있으므로, 현재 상대가 있는 쪽의 경계를
        // 통째로 닫아 그 차 반대편으로 순간이동하는 목표도 만들지 않는다.
        Vec3 otherForward = other->GetForwardAxis();
        const bool sameDirection = otherForward.GetX() * theirs.tangent.GetX() +
                                       otherForward.GetZ() * theirs.tangent.GetZ() >
                                   0.0f;
        constexpr float OFFSET_SIDE_EPS = 0.2f;
        constexpr float OFFSET_SAFETY_TIME = 1.0f;
        constexpr float OFFSET_MAX_APPROACH = 12.0f;
        const float longitudinalClearance = std::fabs(deltaS) - halfLengthSum;
        const float approachPreview = ALONGSIDE_APPROACH +
                                      std::min(OFFSET_MAX_APPROACH,
                                               std::fabs(m_speed - other->GetSpeed()) *
                                                   OFFSET_SAFETY_TIME);
        if (sameDirection && longitudinalClearance < approachPreview)
        {
            const float required =
                myLateralExtent + LateralHalfExtent(*other, theirs.tangent) + SIDE_SAFE_CLEARANCE;

            // 거의 같은 선에 겹쳐 있으면 어느 쪽으로 비킬지 부호를 정할 수 없다. 제약을
            // 통째로 걸지 않으면 두 대가 같은 레이싱 라인으로 나란히 수렴하다 그대로 박는다
            // (출발 직후가 정확히 이 상황이다). id로 방향을 갈라 서로 반대쪽으로 비키게 한다.
            float side = sideLateral;
            if (std::fabs(side) <= OFFSET_SIDE_EPS)
                side = GetId() < other->GetId() ? 1.0f : -1.0f;

            // 두 차가 모두 상대의 현재 위치만 보고 중앙으로 움직이면 각자의 목표는 안전해도
            // 이동 경로가 중앙에서 겹친다. 상대 쪽으로는 현재 d를 한 발짝도 넘지 못하게 해서
            // 기존 횡순서를 보존한다. 이미 너무 가까우면 required 경계가 바깥으로 밀어낸다.
            if (side > 0.0f) // 상대가 오른쪽 -- 나는 그 왼쪽에 머문다
                neighbors.safeOffsetMax = std::min(
                    neighbors.safeOffsetMax, std::min(mine.d, theirs.d - required));
            else
                neighbors.safeOffsetMin = std::max(
                    neighbors.safeOffsetMin, std::max(mine.d, theirs.d + required));
        }

        // 상대가 실제로 나를 추월 중이고 아직 뒤에 있을 때만 권리 후보로 본다. 같은 라인에서
        // 단순 추종 중인 차 때문에 앞차가 임의의 방향으로 비키면 안 된다.
        const bool activePass = other->m_overtakeTarget == this &&
                                (other->m_overtakeState == OvertakeState::Setup ||
                                 other->m_overtakeState == OvertakeState::Attack);
        const float longitudinalOverlap = halfLengthSum - std::fabs(deltaS);
        if (activePass && deltaS < 0.0f && longitudinalOverlap > 0.0f)
        {
            // 실제 횡위치를 우선한다. 아직 거의 같은 선이면 추월자가 고정한 방향을 써서
            // 오버랩 임계에서 부호가 프레임마다 바뀌는 일을 막는다.
            float passSide = std::fabs(sideLateral) > 0.1f
                                 ? (sideLateral > 0.0f ? 1.0f : -1.0f)
                                 : other->m_overtakeSide;
            if (passSide != 0.0f && longitudinalOverlap > neighbors.yieldChallengerOverlap)
            {
                neighbors.yieldChallenger = other;
                neighbors.yieldChallengerSide = passSide;
                neighbors.yieldChallengerOverlap = longitudinalOverlap;
                neighbors.yieldChallengerLateral = sideLateral;
            }
        }

        if (deltaS <= 0.0f || deltaS > LEADER_RANGE)
            continue;

        // 마주 오는 차를 추종하면 정면충돌 코스로 감속만 하게 된다. 회피 레이어가 처리할 몫.
        Vec3 theirForward = other->GetForwardAxis();
        if (theirForward.GetX() * theirs.tangent.GetX() + theirForward.GetZ() * theirs.tangent.GetZ() <= 0.0f)
            continue;

        const float gap = deltaS - (GetLength() + other->GetLength()) * 0.5f;
        const float lateral = std::fabs(theirs.d - mine.d);

        // --- 슬립스트림: 앞차 와류 안에 얼마나 들어가 있는가 ---
        // 가까울수록, 그리고 바로 뒤에 가까울수록 강하다. 횡으로는 서서히 줄기만 하므로
        // 라인을 벗어나 나란히 가기 시작해도 토우가 한동안 남는다.
        if (gap < SLIPSTREAM_RANGE)
        {
            float lateralSpan = (GetHalfWidth() + other->GetHalfWidth()) * SLIPSTREAM_LATERAL_SPAN;
            float alongFactor = 1.0f - std::max(0.0f, gap) / SLIPSTREAM_RANGE;
            float lateralFactor = 1.0f - std::min(1.0f, lateral / std::max(0.1f, lateralSpan));
            float strength = alongFactor * lateralFactor;
            if (strength > neighbors.slipstream)
            {
                neighbors.slipstream = strength;
                neighbors.slipstreamGap = std::max(0.0f, gap);
            }
        }

        // --- leader: 내 라인을 실제로 막고 있는 차 ---
        // 횡으로 겹치지 않으면 막고 있는 게 아니다 -- 나란히 달리는 차는 leader가 아니다
        float overlapLimit = myLateralExtent + LateralHalfExtent(*other, theirs.tangent) + LATERAL_MARGIN;
        // 추월 대상에게는 히스테리시스를 걸지 않는다. 넓혀두면 옆에 나란히 선 뒤에도
        // 계속 leader로 남아 그 차 속도에 눌리고, 추월이 영영 완성되지 않는다.
        // 떨림 걱정은 추월 FSM이 방향을 붙잡아 주므로 여기선 필요 없다.
        if (other == previousLeader && other != m_overtakeTarget)
            overlapLimit *= RELEASE_HYSTERESIS;
        if (lateral > overlapLimit)
            continue;

        if (gap >= bestGap)
            continue;

        bestGap = gap;
        neighbors.leader.car = other;
        neighbors.leader.gap = std::max(0.0f, gap);
        neighbors.leader.speed = other->GetSpeed();
        neighbors.leader.closingSpeed = m_speed - other->GetSpeed();
        neighbors.leader.lateralOffset = theirs.d;
    }

    // 항력은 v^2에 비례한다. 저속에서는 줄일 항력 자체가 없으니 토우도 없다.
    float speedGate = std::clamp((m_speed - SLIPSTREAM_MIN_SPEED) / SLIPSTREAM_MIN_SPEED, 0.0f, 1.0f);
    neighbors.slipstream *= speedGate;
    if (neighbors.slipstream <= 0.0f)
        neighbors.slipstreamGap = 0.0f;
    return neighbors;
}

// IDM 상호작용 항. 자유주행 항(목표속도까지 가속)은 레이싱 플랜이 이미 담당하므로
// 여기선 갭이 목표보다 좁을 때 나오는 감속만 뽑아 쓴다.
// 제동 상한은 마찰원이 남겨준 여유(brakeAvailNow)로 자른다 -- 코너링 중엔 앞차에
// 맞추고 싶어도 타이어가 그만큼 못 준다.
float Car::LeaderFollowAccel(const LeaderInfo &leader, float brakeAvailNow, float headwayScale) const
{
    const float s0 = FOLLOW_STANDSTILL_GAP * m_personality.headwayFactor * headwayScale;
    const float headway = FOLLOW_TIME_HEADWAY * m_personality.headwayFactor * headwayScale;
    // m_maxBrake에 brakeFactor가 이미 반영돼 있으니 여기서 또 곱하면 이중 적용이 된다.
    const float comfortBrake = std::max(1.0f, m_maxBrake);

    // 접근 중일 때만 예측항이 붙는다. 멀어지는 앞차 때문에 목표갭이 s0 밑으로 내려가면 안 된다.
    float dynamicGap = m_speed * headway +
                       m_speed * leader.closingSpeed / (2.0f * std::sqrt(m_maxAccel * comfortBrake));
    float desiredGap = s0 + std::max(0.0f, dynamicGap);

    float gap = std::max(0.1f, leader.gap);
    float ratio = desiredGap / gap;
    return std::max(-brakeAvailNow, m_maxAccel * (1.0f - ratio * ratio));
}

// 상대와 나의 "방해받지 않았을 때 속도"를 같은 기준으로 비교한다.
//
// 계획 목표속도와 상대의 현재 속도를 비교하면 안 된다. 목표속도는 앞의 직선을 내다본
// 값이라, 코너를 빠져나오는 중에는 내 목표가 이미 직선 값으로 올라가 있어서 차이가
// 크게 잡힌다. 그러면 "끝낼 수 없는 추월"이 계속 시작되고, 두 대가 레이스 내내 나란히
// 맞물린 채 서로를 긁는다.
float Car::PaceAdvantageOver(const Car &other, float curvature) const
{
    const float absCurvature = std::fabs(curvature);
    if (absCurvature > 1e-4f)
    {
        const float mine = std::sqrt(m_gripAccel * PLAN_GRIP_RATIO / absCurvature);
        const float theirs = std::sqrt(other.m_gripAccel * PLAN_GRIP_RATIO / absCurvature);
        return std::min(mine, m_raceSpeedCap) - std::min(theirs, other.m_raceSpeedCap);
    }
    return m_raceSpeedCap - other.m_raceSpeedCap;
}

// 접근속도 0에서의 평형 갭. LeaderFollowAccel의 예측항을 뺀 값과 같다.
float Car::DesiredFollowGap(float headwayScale) const
{
    const float scale = m_personality.headwayFactor * headwayScale;
    return FOLLOW_STANDSTILL_GAP * scale + m_speed * FOLLOW_TIME_HEADWAY * scale;
}

// 얼리 에이펙스 라인이 현재 라인보다 얼마나 안쪽인가. 직선에서는 두 라인이 거의 겹쳐
// 0에 가깝고, 코너 진입에서 벌어진다.
float Car::EarlyApexDelta(size_t index, float laneOffsetAtCar) const
{
    if (m_attackLine == nullptr || index >= m_attackLine->laneOffset.size())
        return 0.0f;
    return m_attackLine->laneOffset[index] - laneOffsetAtCar;
}

// 지금 열린 쪽이 아니라 "추월이 끝날 때까지 열려 있는 쪽"을 고른다.
// 코너에서 레이싱 라인은 안쪽 에이펙스로 흐르므로, 그쪽을 골랐다가는 room이 0으로
// 줄면서 오프셋이 짜부라지고 상대 위로 올라탄다. 앞을 미리 보면 자연스럽게
// "코너 바깥쪽으로 추월"이 나온다.
float Car::PickOvertakeSide(const RaceLine &line, size_t myIndex, float lineRoom,
                            float laneOffsetAtCar) const
{
    float minRoomRight = lineRoom - laneOffsetAtCar;
    float minRoomLeft = lineRoom + laneOffsetAtCar;

    if (myIndex < line.arcLength.size())
    {
        const float startS = line.arcLength[myIndex];
        for (size_t i = myIndex; i < line.laneOffset.size() && i < line.arcLength.size(); ++i)
        {
            if (line.arcLength[i] - startS > OVERTAKE_SIDE_PREVIEW)
                break;
            minRoomRight = std::min(minRoomRight, lineRoom - line.laneOffset[i]);
            minRoomLeft = std::min(minRoomLeft, lineRoom + line.laneOffset[i]);
        }
    }

    const float bestRoom = std::max(minRoomRight, minRoomLeft);
    if (bestRoom < OVERTAKE_MIN_ROOM)
        return 0.0f;

    // 앞으로 나올 코너에서 얼리 에이펙스 라인이 파고드는 쪽이 있으면 그쪽을 먼저 본다.
    // 브레이킹 다이브가 실제 레이싱의 기본 추월이고, 공간만 보고 고르면 항상 코너
    // 바깥쪽(먼 길)으로 돌게 된다.
    if (m_attackLine != nullptr && myIndex < line.arcLength.size())
    {
        const float startS = line.arcLength[myIndex];
        float strongest = 0.0f;
        for (size_t i = myIndex; i < line.laneOffset.size() && i < m_attackLine->laneOffset.size() &&
                                 i < line.arcLength.size();
             ++i)
        {
            if (line.arcLength[i] - startS > OVERTAKE_SIDE_PREVIEW)
                break;
            float delta = m_attackLine->laneOffset[i] - line.laneOffset[i];
            if (std::fabs(delta) > std::fabs(strongest))
                strongest = delta;
        }
        if (std::fabs(strongest) >= OVERTAKE_EARLY_MIN_DELTA)
        {
            float earlySide = strongest > 0.0f ? 1.0f : -1.0f;
            float earlyRoom = earlySide > 0.0f ? minRoomRight : minRoomLeft;
            if (earlyRoom >= OVERTAKE_MIN_ROOM)
                return earlySide;
        }
    }

    return minRoomRight >= minRoomLeft ? 1.0f : -1.0f;
}

// 나란히 설 수 있을 만큼 벌리되 도로 밖으론 못 나간다.
float Car::OvertakeOffset(size_t myIndex, float lineRoom, float laneOffsetAtCar) const
{
    if (m_overtakeSide == 0.0f)
        return 0.0f;

    float targetHalfWidth = GetHalfWidth();
    if (m_overtakeTarget != nullptr && m_SimState->IsCarAlive(m_overtakeTarget))
        targetHalfWidth = m_overtakeTarget->GetHalfWidth();

    // 오버랩 판정 폭(반폭 합 + LATERAL_MARGIN)보다 넉넉히 커야 한다. 감쇠 이동이라
    // 목표에 정확히 도달하지 않으므로, 아슬아슬하게 잡으면 경계에서 맴돈다.
    constexpr float SIDE_BY_SIDE_MARGIN = 1.6f;
    float need = GetHalfWidth() + targetHalfWidth + SIDE_BY_SIDE_MARGIN;

    // 같은 쪽으로 얼리 에이펙스 라인이 더 깊이 파고든다면 거기까지 간다. 코너를 안쪽으로
    // 짧게 물고 들어가는 게 다이브의 본체다. 다만 이건 "더 깊이"일 뿐 대체가 아니다 --
    // 얼리 라인 단독으로는 오버랩이 풀리지 않는다.
    float earlyDelta = EarlyApexDelta(myIndex, laneOffsetAtCar);
    if (earlyDelta * m_overtakeSide > 0.0f)
        need = std::max(need, std::fabs(earlyDelta));

    float room = m_overtakeSide > 0.0f ? (lineRoom - laneOffsetAtCar) : (lineRoom + laneOffsetAtCar);
    return m_overtakeSide * std::clamp(need, 0.0f, std::max(0.0f, room));
}

void Car::ApplyYieldRule(const RaceNeighbors &neighbors, float &desiredOffset)
{
    auto release = [this]()
    {
        m_yieldTarget = nullptr;
        m_yieldSide = 0.0f;
    };

    if (m_yieldTarget != nullptr)
    {
        if (!m_SimState->IsCarAlive(m_yieldTarget) || !neighbors.yieldTargetSeen)
            release();
        else
        {
            const float halfLengthSum = (GetLength() + m_yieldTarget->GetLength()) * 0.5f;
            const float deltaS = neighbors.yieldTargetDeltaS;
            const float overlap = halfLengthSum - std::fabs(deltaS);
            const bool passedClear = deltaS - halfLengthSum > OVERTAKE_CLEAR_MARGIN;
            const bool fellBackClear = deltaS < 0.0f &&
                                       overlap < m_yieldTarget->GetLength() * YIELD_RELEASE_OVERLAP;
            if (passedClear || fellBackClear)
                release();
        }
    }

    bool targetSeen = neighbors.yieldTargetSeen;
    float targetLateral = neighbors.yieldTargetLateral;
    if (m_yieldTarget == nullptr && neighbors.yieldChallenger != nullptr &&
        neighbors.yieldChallengerOverlap >=
            neighbors.yieldChallenger->GetLength() * YIELD_ACQUIRE_OVERLAP)
    {
        m_yieldTarget = neighbors.yieldChallenger;
        m_yieldSide = neighbors.yieldChallengerSide;
        targetSeen = true;
        targetLateral = neighbors.yieldChallengerLateral;
    }

    if (m_yieldTarget == nullptr || !targetSeen)
        return;

    // 양쪽 차체와 측면 안전 여유가 들어갈 경계를 계산해 앞차의 목표 오프셋을 제한한다.
    // 속도를 양보하는 게 아니라 추월차 쪽 라인을 닫지 않는 규칙이다.
    const float requiredSeparation =
        GetHalfWidth() + m_yieldTarget->GetHalfWidth() + SIDE_SAFE_CLEARANCE;
    const float challengerOffset = neighbors.leader.myLateralOffset + targetLateral;
    if (m_yieldSide > 0.0f)
        desiredOffset = std::min(desiredOffset, challengerOffset - requiredSeparation);
    else
        desiredOffset = std::max(desiredOffset, challengerOffset + requiredSeparation);
}

// 추월 전술 레이어. 결과는 m_raceLateralOffset 하나로 나가고, 경로 레이어가 그걸 레이싱
// 라인 위에 얹는다. 조향을 직접 만들지 않는 게 핵심이다 -- 조향까지 여기서 건드리면
// 반응형 회피와 매 프레임 싸운다.
void Car::UpdateOvertake(const RaceLine &line, size_t myIndex, const RaceNeighbors &neighbors,
                         float lineRoom, float laneOffsetAtCar, float paceSpeed, float curvature)
{
    const float dt = m_deltaTime;
    m_overtakeTimer += dt;
    if (m_overtakeCooldown > 0.0f)
        m_overtakeCooldown -= dt;
    m_overtakeBlock = "";

    auto enter = [this](OvertakeState next)
    {
        if (m_overtakeState == next)
            return;
        m_overtakeState = next;
        m_overtakeTimer = 0.0f;
    };

    switch (m_overtakeState)
    {
    case OvertakeState::None:
    case OvertakeState::Follow:
    {
        m_overtakeTarget = nullptr;
        m_overtakeSide = 0.0f;
        m_overtakeHoldOffset = false;
        if (!neighbors.leader.IsValid())
        {
            enter(OvertakeState::None);
            break;
        }
        enter(OvertakeState::Follow);

        if (m_overtakeCooldown > 0.0f)
        {
            m_overtakeBlock = "cooldown";
            break;
        }
        if (neighbors.leader.gap > DesiredFollowGap(1.0f) * OVERTAKE_TRIGGER_SCALE)
        {
            m_overtakeBlock = "gap";
            break;
        }
        // 코너 한복판에서는 걸지 않는다. 여기를 안 막으면 헤어핀에서 나란히 서서
        // 몇 초를 버티다 접촉하고 끝난다.
        if (std::fabs(curvature) > OVERTAKE_MAX_CURVATURE)
        {
            m_overtakeBlock = "corner";
            break;
        }
        // 여기서 최고속을 쓰면 안 된다. 느리게 도는 코너에서도 차이가 최고속 차이로
        // 잡혀 게이트가 무조건 통과한다. 지금 이 지점에서 낼 수 있는 속도(곡률·마찰원이
        // 반영된 값)와 비교해야 진짜 페이스 우위가 드러난다.
        const float paceDelta = PaceAdvantageOver(*neighbors.leader.car, curvature);
        if (paceDelta < OVERTAKE_PACE_MARGIN)
        {
            m_overtakeBlock = "pace";
            break;
        }

        // 그리고 "빠르다"만으로는 부족하다. 주어진 시간 안에 실제로 앞지를 수 있어야 한다.
        // 끝낼 수 없는 추월을 시작하면 두 대가 레이스 내내 나란히 맞물린 채 계속 긁는다.
        const float needGain = neighbors.leader.gap +
                               (GetLength() + neighbors.leader.car->GetLength()) * 0.5f +
                               OVERTAKE_CLEAR_MARGIN;
        if (needGain / paceDelta > OVERTAKE_SETUP_TIMEOUT + OVERTAKE_TIMEOUT)
        {
            m_overtakeBlock = "tooSlow";
            break;
        }

        float side = PickOvertakeSide(line, myIndex, lineRoom, laneOffsetAtCar);
        if (side == 0.0f)
        {
            m_overtakeBlock = "room";
            break;
        }

        m_overtakeTarget = neighbors.leader.car;
        m_overtakeSide = side;
        ++m_overtakeAttempts;
        enter(OvertakeState::Setup);
        break;
    }
    case OvertakeState::Setup:
    {
        m_overtakeHoldOffset = false;
        if (m_overtakeTarget == nullptr || !m_SimState->IsCarAlive(m_overtakeTarget))
        {
            enter(OvertakeState::Return);
            break;
        }

        // 여기서는 오프셋만 벌리고 차간은 평소대로 둔다(headwayScale 1.0). 옆으로 나가는
        // 것과 앞차에 붙는 걸 동시에 하면, 횡이동이 조향 지연 때문에 훨씬 느려서
        // 아직 덜 벌어진 상태로 나란히 서게 된다 -- 측면 접촉의 주원인이다.
        const float required = neighbors.targetRequiredLateral;
        if (neighbors.targetSeen && required > 0.0f &&
            std::fabs(neighbors.targetLateral) >= required)
        {
            // 명령값이 아니라 실제로 벌어진 걸 확인하고 넘어간다.
            m_overtakeBestDeltaS = neighbors.targetDeltaS;
            m_overtakeStall = 0.0f;
            enter(OvertakeState::Attack);
            break;
        }

        // 벌리는 동안 앞차가 알아서 뒤로 빠졌으면 그대로 끝낸다.
        const float setupRearGap = -neighbors.targetDeltaS -
                                   (GetLength() + m_overtakeTarget->GetLength()) * 0.5f;
        if (neighbors.targetSeen && setupRearGap > OVERTAKE_CLEAR_MARGIN)
        {
            enter(OvertakeState::Return);
            break;
        }

        if (m_overtakeTimer > OVERTAKE_SETUP_TIMEOUT) // 공간이 없어 못 벌린 것이다
        {
            m_overtakeCooldown = OVERTAKE_COOLDOWN;
            ++m_overtakeAborts;
            enter(OvertakeState::Return);
        }
        break;
    }
    case OvertakeState::Attack:
    {
        m_overtakeHoldOffset = false;
        if (m_overtakeTarget == nullptr || !m_SimState->IsCarAlive(m_overtakeTarget))
        {
            enter(OvertakeState::Return);
            break;
        }
        // 통과 판정을 leader 유효성에 걸면 안 된다. 옆으로 나가는 순간 오버랩이 풀리면서
        // 곧바로 "성공"으로 읽혀 라인에 복귀해 버리고, 나란히 달리는 구간이 사라진다.
        // targetDeltaS는 중심간 거리다. 반길이 합을 빼야 내 뒤범퍼와 상대 앞범퍼 사이 간격이 된다.
        float rearGap = -neighbors.targetDeltaS -
                        (GetLength() + m_overtakeTarget->GetLength()) * 0.5f;
        if (neighbors.targetSeen && rearGap > OVERTAKE_CLEAR_MARGIN)
        {
            ++m_overtakeSuccess;
            enter(OvertakeState::Return);
            break;
        }
        // Setup에서 확보한 간격이 Attack 도중 다시 좁혀질 수 있다. 종방향으로 겹친 상태에서
        // 간격을 잃으면 밀어붙이지 말고 물러난다 -- Return이 오프셋을 붙잡은 채 뒤로 빼준다.
        const float halfLengthSum = (GetLength() + m_overtakeTarget->GetLength()) * 0.5f;
        const bool longitudinallyOverlapping =
            neighbors.targetSeen && std::fabs(neighbors.targetDeltaS) < halfLengthSum;
        constexpr float ATTACK_ABORT_HYSTERESIS = 0.8f;
        if (longitudinallyOverlapping && neighbors.targetRequiredLateral > 0.0f &&
            std::fabs(neighbors.targetLateral) <
                neighbors.targetRequiredLateral * ATTACK_ABORT_HYSTERESIS)
        {
            m_overtakeCooldown = OVERTAKE_COOLDOWN;
            ++m_overtakeAborts;
            enter(OvertakeState::Return);
            break;
        }

        // 진전이 없으면 타임아웃을 다 기다리지 않고 접는다. 나란히 붙어 있는 시간이 길수록
        // 접촉 확률만 올라간다.
        if (neighbors.targetSeen)
        {
            if (neighbors.targetDeltaS < m_overtakeBestDeltaS - OVERTAKE_PROGRESS_EPS)
            {
                m_overtakeBestDeltaS = neighbors.targetDeltaS;
                m_overtakeStall = 0.0f;
            }
            else
                m_overtakeStall += dt;
        }
        if (m_overtakeStall > OVERTAKE_STALL_TIME || m_overtakeTimer > OVERTAKE_TIMEOUT)
        {
            m_overtakeCooldown = OVERTAKE_COOLDOWN;
            ++m_overtakeAborts;
            enter(OvertakeState::Return);
        }
        break;
    }
    case OvertakeState::Return:
    {
        // 여기서 곧바로 오프셋을 0으로 되돌리면, 나란히 붙어 있는 상대를 향해 그대로
        // 스티어링해 들어간다. 실패한 추월이 반드시 접촉으로 끝나던 원인이 이것이다.
        // 종방향 겹침이 풀릴 때까지는 옆으로 벌린 상태를 유지한다.
        bool overlapping = false;
        if (m_overtakeTarget != nullptr && m_SimState->IsCarAlive(m_overtakeTarget) &&
            neighbors.targetSeen)
        {
            float halfLengthSum = (GetLength() + m_overtakeTarget->GetLength()) * 0.5f;
            overlapping = std::fabs(neighbors.targetDeltaS) - halfLengthSum < OVERTAKE_CLEAR_MARGIN;
        }
        m_overtakeHoldOffset = overlapping;

        if (!overlapping)
        {
            m_overtakeTarget = nullptr;
            m_overtakeSide = 0.0f;
            if (std::fabs(m_raceLateralOffset) < 0.15f)
                enter(OvertakeState::None);
        }
        break;
    }
    }

    float desired;
    if (m_overtakeState == OvertakeState::Setup || m_overtakeState == OvertakeState::Attack)
        desired = OvertakeOffset(myIndex, lineRoom, laneOffsetAtCar);
    else if (m_overtakeHoldOffset)
        desired = m_raceLateralOffset; // 겹침이 풀릴 때까지 벌린 채로 붙잡는다
    else
        desired = 0.0f;

    // 뒷차가 절반 길이 이상 들어왔으면 앞차도 수동적인 장애물이 아니다. 일반 근접 회피가
    // 만든 값에 최종 경계를 걸어, 같은 여유를 두 번 더해 과도하게 밀려나는 것도 막는다.
    ApplyYieldRule(neighbors, desired);
    // 도로 경계와 모든 주변 차량의 점유 경계를 동시에 만족하는 구간만 허용한다.
    const float roadMin = -(lineRoom + laneOffsetAtCar);
    const float roadMax = lineRoom - laneOffsetAtCar;
    const float safeMin = std::max(roadMin, neighbors.safeOffsetMin);
    const float safeMax = std::min(roadMax, neighbors.safeOffsetMax);
    if (safeMin <= safeMax)
    {
        // 이전 프레임 목표가 새로 점유된 구간 안에 남아 있으면 감쇠를 기다리지 않고 취소한다.
        // desired만 자르면 기존 m_raceLateralOffset이 몇 프레임 동안 계속 옆 차를 향한다.
        m_raceLateralOffset = std::clamp(m_raceLateralOffset, safeMin, safeMax);
        desired = std::clamp(desired, safeMin, safeMax);
    }
    else
    {
        // 양쪽이 모두 막히면 전술 목표를 버리고 실제 위치를 그대로 유지한다.
        const float hold = std::clamp(neighbors.leader.myLateralOffset, roadMin, roadMax);
        m_raceLateralOffset = hold;
        desired = hold;
    }

    // 목표 오프셋으로 감쇠 이동. 한 프레임에 꺾어버리면 Pure Pursuit이 과조향한다.
    m_raceLateralOffset += (desired - m_raceLateralOffset) * std::min(1.0f, dt * LATERAL_RESPONSE);
}

// 레이싱 라인 조향을 그대로 유지하되, 그 궤적이 곧 실제로 충돌할 때만 최소한으로 비튼다.
// 일반 주행의 부채꼴 슬롯 회피를 쓰면, 추월하려고 옆에 나란히 붙는 것 자체가 "위험"으로
// 읽혀 계속 밀려나고 전술 레이어가 무슨 오프셋을 내놓든 매 프레임 무효가 된다.
float Car::ComputeRaceContextSteer(float pursuitSteer) const
{
    constexpr float COLLISION_HORIZON = 0.45f;
    constexpr float MAX_SWEEP_STEP = 2.0f;
    constexpr float STEER_RATE = 1.0f;
    constexpr float CORRECTIONS[] = {
        ToRadians(4.0f),
        ToRadians(8.0f),
        ToRadians(12.0f),
    };

    m_ctxSteerDebugRad = 0.0f;
    m_ctxSteerDebugDanger = 0.0f;
    m_ctxSteerOpenLines.clear();
    m_ctxSteerBlockedLines.clear();

    if (m_speed < 0.1f || m_obstacles.empty())
        return pursuitSteer;

    // 차량 항목은 센서 멤버십용으로 캐시된 값이라, 충돌 예측은 현재 위치에서 시작해야 한다.
    std::vector<VehicleCollision::Obstacle> threats;
    threats.reserve(m_obstacles.size());
    for (const VehicleCollision::Obstacle &stored : m_obstacles)
    {
        if (stored.sourceCar == nullptr)
        {
            threats.push_back(stored);
            continue;
        }
        if (m_SimState->IsCarAlive(stored.sourceCar))
            threats.push_back(stored.sourceCar->MakeVehicleObstacle());
    }
    if (threats.empty())
        return pursuitSteer;

    const Vec3 startPosition = GetRigidbodyPosition();
    const float startHeading = DirectionToAngleRad(GetForwardAxis());
    const VehicleCollision::VehicleShape shape = BuildVehicleShape();
    const int sweepSteps = std::clamp(
        static_cast<int>(std::ceil(m_speed * COLLISION_HORIZON / MAX_SWEEP_STEP)), 3, 24);
    const float dt = COLLISION_HORIZON / static_cast<float>(sweepSteps);
    std::vector<VehicleCollision::Obstacle> predicted = threats;

    auto collisionTime = [&](float targetSteer)
    {
        Vec3 position = startPosition;
        float heading = startHeading;
        float steer = m_steerAngle;

        for (int step = 1; step <= sweepSteps; ++step)
        {
            const float maxSteerDelta = STEER_RATE * dt;
            if (steer < targetSteer)
                steer = std::min(steer + maxSteerDelta, targetSteer);
            else
                steer = std::max(steer - maxSteerDelta, targetSteer);

            const float distance = m_speed * dt;
            Vec3 forward(cosf(heading), 0.0f, sinf(heading));
            position += forward * distance;
            heading -= distance * tanf(steer) / std::max(0.1f, m_wheelbase);

            const float elapsed = step * dt;
            predicted = threats;
            for (VehicleCollision::Obstacle &obstacle : predicted)
            {
                if (obstacle.type != VehicleCollision::ObstacleType::Dynamic)
                    continue;
                Vec3 obstacleForward(cosf(obstacle.headingRad), 0.0f, sinf(obstacle.headingRad));
                obstacle.center += obstacleForward * (obstacle.speed * elapsed);
            }

            if (VehicleCollision::IsColliding(position, heading, predicted, shape))
                return elapsed;
        }
        return COLLISION_HORIZON + dt;
    };

    const float safeTime = COLLISION_HORIZON + dt;
    float bestSteer = pursuitSteer;
    float bestCollisionTime = collisionTime(pursuitSteer);
    if (bestCollisionTime >= safeTime)
        return pursuitSteer;

    // 충돌이 예측된 뒤에야, 가장 작은 보정부터 차례로 시도한다.
    for (float correction : CORRECTIONS)
    {
        const float candidates[2] = {
            std::clamp(pursuitSteer - correction, -m_maxSteerAngle, m_maxSteerAngle),
            std::clamp(pursuitSteer + correction, -m_maxSteerAngle, m_maxSteerAngle),
        };
        const float candidateTimes[2] = {
            collisionTime(candidates[0]),
            collisionTime(candidates[1]),
        };

        for (int i = 0; i < 2; ++i)
        {
            if (candidateTimes[i] > bestCollisionTime)
            {
                bestCollisionTime = candidateTimes[i];
                bestSteer = candidates[i];
            }
        }
        if (bestCollisionTime >= safeTime)
            break;
    }

    m_ctxSteerDebugRad = bestSteer - pursuitSteer;
    m_ctxSteerDebugDanger = 1.0f;
    return bestSteer;
}

#pragma endregion
