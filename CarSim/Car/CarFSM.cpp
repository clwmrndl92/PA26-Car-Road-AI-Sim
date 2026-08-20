#include "Car.h"
#include "RacingLineQP.h"
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

    // 도로를 이어 붙인 참조선은 이음매에서 헤딩이 튄다(이 트랙 측정값 최대 18.6도/1m,
    // 곡률 반경으로 3m). 그대로 두면 두 가지가 망가진다.
    //   1) 곡률이 스파이크라 QP가 감당하지 못하고 꺾임이 라인에 그대로 남는다.
    //   2) 법선이 1m 사이에 18도 돌아, 오프셋을 얹은 라인이 옆으로 홱 튄다.
    // 원본에서 CENTERLINE_MAX_SHIFT 이내로만 움직이도록 묶어 두고 고주파 꺾임만 깎는다.
    // 실측 이동량은 0.35m 수준이라 0.5m면 충분하다. 여기서 잡은 만큼 레이싱 라인이
    // 쓸 수 있는 폭(SharedLineRoom)이 줄어드니 필요 이상으로 크게 잡지 않는다.
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

        // 통과대역 k = 1/shrink + 1/inflate. 더 낮춰(0.6/-0.62) 봐도 꺾임이 4.30 -> 4.29도로
        // 사실상 그대로였다. 남은 건 단일 샘플 스파이크가 아니라 실제 형상이라는 뜻이라,
        // 위치 스무딩은 여기까지만 하고 곡률 쪽은 중앙값 필터로 따로 잡는다.
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
    // 진짜 코너로 읽으면 멀쩡한 구간에서 급제동한다(반경 15m -> 40km/h까지 감속).
    // 평균이 아니라 중앙값을 쓰는 이유는, 진짜 코너(여러 샘플에 걸쳐 곡률이 유지됨)는
    // 그대로 두고 고립된 이상치만 걷어내기 위해서다.
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

        // 속도 계획이 읽는 곡률에서는 더 넓게 본다. 이음매 스파이크가 ±2 창을 통과해
        // 반경 10m(32km/h)로 남았는데, 이걸 진짜 코너로 읽으면 멀쩡한 직선에서 급제동한다.
        // 11m 창이면 진짜 코너(p99가 반경 34m -- 11m 구간에서 18도를 도는 형상)는 그대로 남고
        // 몇 샘플짜리 이상치만 걷힌다.
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
        // 미리보는 거리가 고정 100m였는데, 최고속(약 88.9m/s)에서 완전정지까지 필요한
        // 거리가 v^2/(2*maxBrake) ~= 257m라 100m로는 코너를 발견했을 때 이미 너무 늦다.
        // 코너 근처에서는 마찰원이 제동 여유를 더 깎기까지 하니 1.5배 여유를 둔다.
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
    // (예전엔 차마다 다른 m_path를 훑었는데, 공유 라인에서는 라인 자체가 순서를 안다)
    void PreviewUpcomingRoad(RacingPlan &plan, const RaceLine &line, float maxSpeed,
                             float currentSpeed, float maxBrake)
    {
        if (line.roadStartS.size() != line.roadIds.size() || line.roadStartS.empty())
            return;

        const float racingBrake = std::max(1.0f, maxBrake * 0.75f);
        // 고정 100m는 고속에서 정지거리보다 짧아 제한속도 급변 구간을 늦게 발견했다.
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
    m_leader = LeaderInfo{}; // DriveControl이 안 도는 동안 직전 값을 들고 있지 않게
    m_slipstream = 0.0f;
    m_speedCap = m_maxSpeed;
    m_overtakeState = OvertakeState::None;
    m_overtakeTarget = nullptr;
    m_overtakeSide = 0.0f;
    m_raceLateralOffset = 0.0f;
    m_raceLateralOffsetInitialized = false;
    m_overtakeHoldOffset = false;
    m_overtakeBackOff = false;
    m_yieldTarget = nullptr;
    m_yieldSide = 0.0f;
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
    float minOffset = 0.0f;
    float maxOffset = 0.0f;
    ComputeDrivableRange(CurrentRoadRef(), minOffset, maxOffset);
    float centerOffset = ComputeReferenceOffset(referenceLine, center);
    return centerOffset < minOffset || centerOffset > maxOffset;
}

void Car::DriveControl()
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

    // High speed needs a longer sight line so steering stays progressive instead of twitchy.
    float lookaheadDistance = std::clamp(4.5f + m_speed * 0.35f, 5.0f, 13.0f);
    // 공유 라인이라 폭도 공유값이다. 추월 오프셋 클램프에 쓰는 여유는 라인을 만들 때
    // 쓴 값과 같아야 하므로 SharedLineRoom()으로 한 곳에서 뽑는다.
    const float lineRoom = SharedLineRoom();

    // 와류는 직전 프레임 값을 쓴다. 계획을 세우려면 속도 상한이 먼저 필요한데, 그 상한을
    // 정하는 와류 세기는 계획이 내놓는 (s, d)로 찾기 때문이다. 아래에서 감쇠를 걸어 천천히
    // 변하게 만들었으므로 한 프레임 지연은 드러나지 않는다.
    m_speedCap = outsideRoad
                     ? m_maxSpeed * 0.5f
                     : m_maxSpeed * (1.0f + SLIPSTREAM_TOP_SPEED_GAIN * m_slipstream);
    const float slipMaxAccel = outsideRoad
                                   ? m_maxAccel * 0.5f
                                   : m_maxAccel * (1.0f + SLIPSTREAM_ACCEL_GAIN * m_slipstream);

    RacingPlan racing = BuildRacingPlan(raceLine, position, m_raceLineCursor, lookaheadDistance,
                                        RoadTargetSpeed(m_currentRoad, m_speedCap), m_speed,
                                        slipMaxAccel, m_maxBrake, m_gripAccel * PLAN_GRIP_RATIO);
    PreviewUpcomingRoad(racing, raceLine, m_speedCap, m_speed, m_maxBrake);

    // 지금까지 앞차를 보는 건 회피 조향뿐이었다. 피할 공간이 없으면 감속 없이 그대로
    // 밀고 들어갔기 때문에, 뒤에 안정적으로 붙어 기회를 기다리는 그림 자체가 안 나왔다.
    RaceNeighbors neighbors = FindRaceNeighbors(raceLine, racing.closestIndex, racing.currentS);
    m_leader = neighbors.leader;

    // 레이스 계측 스냅샷. 순위/랩타임과 충돌 로그가 전부 이 값들을 읽는다.
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
    // 페이스 비교는 "지금 이 지점에서 낼 수 있는 속도"로 해야 한다(racing.targetSpeed).
    // 곡률은 시작 조건에만 쓴다 -- 코너 한복판에서 나란히 서지 않게.
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
        // on/off가 반복되므로 차체 중심 목표를 0.5m 더 안쪽에 둔다.
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
    float targetSteer = ComputeContextSteer(target, pursuitSteer);
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

#pragma endregion

#pragma region BehaviorPlan

void Car::ComputeDrivableRange(const RoadRef &road, float &outMin, float &outMax) const
{

    // GetHalfWidth()는 이미 반폭이다. 다시 절반으로 줄이면 차체 절반이 도로 밖에 나가도
    // 안쪽으로 판정되어 복귀가 늦어진다.
    float halfW = GetHalfWidth();

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

float Car::RoadTargetSpeed(const shared_ptr<Road> &road, float maxSpeed) const
{
    if (road == nullptr)
        return maxSpeed;
    return std::min(road->GetSpeedLimit(), maxSpeed);
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

    float laneHalfWidth = 0.0f;
    const std::vector<shared_ptr<Road>> &roads = RoadDataManager::Get().GetRoads();
    if (!roads.empty())
    {
        std::vector<const LaneBand *> bands = RoadDataManager::Get().GetDrivingBands(roads.front());
        if (!bands.empty())
            laneHalfWidth = bands.front()->width * 0.5f;
    }
    // 이음매 꺾임을 깎느라 참조선 자체가 최대 CENTERLINE_MAX_SHIFT만큼 움직인다.
    // QP 경계는 그 움직인 참조선 기준이므로, 그만큼 빼야 실제 도로 밖으로 안 나간다.
    return std::clamp(laneHalfWidth - widestHalfWidth - EDGE_MARGIN - CENTERLINE_MAX_SHIFT,
                      0.0f, MAX_RACING_OFFSET);
}

// 트랙 루프를 따라가며 전략별로 한 번씩 QP를 푼다. 도로를 다 읽은 뒤 차를 스폰하기 전에
// 한 번만 부를 것 -- 7000점 규모라 전략당 수백 ms 걸린다(Debug 기준).
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

        const vector<RoadRef> &successors = roadData.GetRoadSuccessors(road->GetId());
        if (successors.empty() || successors.front().road == nullptr)
        {
            road = nullptr;
            break;
        }
        road = successors.front().road;
        if (road->GetId() == startId) // 한 바퀴 닫혔다
            break;
    }

    float laneCenter = 0.0f;
    std::vector<const LaneBand *> bands = roadData.GetDrivingBands(roads.front());
    if (!bands.empty())
        laneCenter = bands.front()->centerOffset;

    const float lineRoom = SharedLineRoom();

    std::vector<RaceLine> lines(static_cast<size_t>(ApexStrategy::Count));
    for (int i = 0; i < static_cast<int>(ApexStrategy::Count); ++i)
        lines[i] = BuildTrackRaceLine(loop, roadIds, laneCenter, lineRoom,
                                      static_cast<ApexStrategy>(i));

    DebugConsole::Log("RaceLine: " + ToString(static_cast<int>(loop.size())) + " roads, " +
                      ToString(static_cast<int>(lines[0].lapLength)) + " m lap, " +
                      ToString(static_cast<int>(ApexStrategy::Count)) + " strategies");
    simState.SetRaceLines(std::move(lines));
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
    constexpr float LEADER_RANGE = 80.0f;  // 이보다 먼 앞차는 종방향에 영향 없음
    // "옆으로 지나갈 수 있다"고 판단하는 기준. 이 값이 작으면 아직 안 벌어진 상태로
    // 옆에 서게 되고, 나란히 겹친 뒤에야 leader로 인식된다 -- 그 시점엔 제동으로
    // 횡방향 공간이 생기지 않는다(로그의 접촉 전부가 gap=0에서 났다).
    // 확실히 비켜서지 못한 차는 그냥 뒤에 줄을 서는 게 맞다.
    constexpr float LATERAL_MARGIN = 1.0f;
    // 앞지른 뒤에도 대상이 이 구간 안에 남아 있어야 "통과했다"를 판정할 수 있다.
    constexpr size_t BACK_WINDOW = 25;     // 뒤쪽으로 볼 샘플 수(약 1m 간격)
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

            // 거의 같은 선에 겹쳐 있으면 어느 쪽으로 비킬지 부호를 정할 수 없다. 예전엔
            // 이 경우 제약을 통째로 걸지 않았는데, 그러면 두 대가 같은 레이싱 라인으로
            // 나란히 수렴하다 그대로 박는다(출발 직후가 정확히 이 상황이다).
            // id로 방향을 갈라 서로 반대쪽으로 비키게 한다.
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
// 계획 목표속도(racing.targetSpeed)와 상대의 현재 속도를 비교하면 안 된다. 목표속도는
// 앞의 직선을 내다본 값이라, 코너를 빠져나오는 중에는 내 목표가 이미 직선 값으로 올라가
// 있어서 차이가 50m/s로 잡힌다. 그 결과 "끝낼 수 없는 추월"이 계속 시작됐고, 두 대가
// 레이스 내내 나란히 맞물린 채 서로를 긁었다.
float Car::PaceAdvantageOver(const Car &other, float curvature) const
{
    const float absCurvature = std::fabs(curvature);
    if (absCurvature > 1e-4f)
    {
        const float mine = std::sqrt(m_gripAccel * PLAN_GRIP_RATIO / absCurvature);
        const float theirs = std::sqrt(other.m_gripAccel * PLAN_GRIP_RATIO / absCurvature);
        return std::min(mine, m_speedCap) - std::min(theirs, other.m_speedCap);
    }
    return m_speedCap - other.m_speedCap;
}

// 접근속도 0에서의 평형 갭. LeaderFollowAccel의 예측항을 뺀 값과 같다.
float Car::DesiredFollowGap(float headwayScale) const
{
    const float scale = m_personality.headwayFactor * headwayScale;
    return FOLLOW_STANDSTILL_GAP * scale + m_speed * FOLLOW_TIME_HEADWAY * scale;
}

// 어느 쪽으로 나갈지 한 번만 정한다. 앞차가 없는 쪽을 먼저 보고, 거기가 좁으면 반대쪽.
// 둘 다 좁으면 0을 돌려 시도 자체를 접는다.
// 얼리 에이펙스 라인이 현재 라인보다 얼마나 안쪽인가. 직선에서는 두 라인이 거의 겹쳐
// 0에 가깝고, 코너 진입에서 벌어진다(측정: 코너 평균 1.33m, 최대 3.61m).
float Car::EarlyApexDelta(size_t index, float laneOffsetAtCar) const
{
    if (m_attackLine == nullptr || index >= m_attackLine->laneOffset.size())
        return 0.0f;
    return m_attackLine->laneOffset[index] - laneOffsetAtCar;
}

// 지금 열린 쪽이 아니라 "추월이 끝날 때까지 열려 있는 쪽"을 고른다.
// 코너에서 레이싱 라인은 인쪽 에이펙스로 흐르므로, 그쪽을 골랐다가는 room이 0으로
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
    // 얼리 라인 단독으로는 코너 평균 1.33m라 오버랩(2.98m)이 풀리지 않는다.
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
    const float challengerOffset =
        neighbors.leader.myLateralOffset + targetLateral;
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
        m_overtakeBackOff = false;
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
        // 6초를 버티다 접촉하고 끝난다.
        if (std::fabs(curvature) > OVERTAKE_MAX_CURVATURE)
        {
            m_overtakeBlock = "corner";
            break;
        }
        // 여기서 최고속(m_speedCap)을 쓰면 안 된다. 139km/h로 도는 코너에서도 차이가
        // 190km/h로 잡혀 게이트가 무조건 통과한다. 지금 이 지점에서 낼 수 있는 속도
        // (곡률·마찰원이 반영된 계획 목표속도)와 비교해야 진짜 페이스 우위가 드러난다.
        const float paceDelta = PaceAdvantageOver(*neighbors.leader.car, curvature);
        if (paceDelta < OVERTAKE_PACE_MARGIN)
        {
            m_overtakeBlock = "pace";
            break;
        }

        // 그리고 "빠르다"만으로는 부족하다. 주어진 시간 안에 실제로 앞지를 수 있어야 한다.
        // 로그에서 1.5m/s 차이로 20m 뒤에서 시작한 추월이 19초를 필요로 했는데 예산은
        // 10초였고, 그 결과 두 대가 레이스 내내 나란히 맞물린 채 계속 긁었다.
        // 끝낼 수 없는 추월은 시작하지 않는다.
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
        m_overtakeBackOff = false;
        if (m_overtakeTarget == nullptr || !m_SimState->IsCarAlive(m_overtakeTarget))
        {
            enter(OvertakeState::Return);
            break;
        }

        // 여기서는 오프셋만 벌리고 차간은 평소대로 둔다(headwayScale 1.0). 옆으로 나가는
        // 것과 앞차에 붙는 걸 동시에 하면, 횡이동이 조향 지연 때문에 훨씬 느려서
        // 아직 2m밖에 안 벌어진 상태로 나란히 서게 된다 -- 측면 접촉의 주원인이었다.
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
            ++m_abortSetup;
            enter(OvertakeState::Return);
        }
        break;
    }
    case OvertakeState::Attack:
    {
        m_overtakeHoldOffset = false;
        m_overtakeBackOff = false;
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
        // Setup에서 확보한 간격이 Attack 도중 다시 좁혀질 수 있다(로그: 4.1m로 진입해
        // 1.5m까지 줄어든 채로 접촉). 종방향으로 겹친 상태에서 간격을 잃으면 밀어붙이지
        // 말고 물러난다 -- Return이 오프셋을 붙잡은 채 뒤로 빼준다.
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
            ++m_abortClearance;
            enter(OvertakeState::Return);
            break;
        }

        // 진전이 없으면 6초를 다 기다리지 않고 접는다. 나란히 붙어 있는 시간이 길수록
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
            ++m_abortStall;
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
        // 종방향 속도에는 손대지 않는다. 겹침이 풀릴 때까지 횡오프셋만 유지한다.
        m_overtakeBackOff = false;

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
    // Keep the racing-line steering unless its actual swept OBB will collide shortly.
    // The old fan/slot search below is intentionally disabled: nearby cars alone must
    // not pull this car away from the racing line.
    constexpr float COLLISION_HORIZON = 0.45f;
    constexpr float MAX_SWEEP_STEP = 2.0f;
    constexpr float STEER_RATE = 1.0f;
    constexpr float CORRECTIONS[] = {
        ToRadians(4.0f),
        ToRadians(8.0f),
        ToRadians(12.0f),
    };

    (void)pursuitTarget;
    m_ctxSteerDebugRad = 0.0f;
    m_ctxSteerDebugDanger = 0.0f;
    m_ctxSteerOpenLines.clear();
    m_ctxSteerBlockedLines.clear();

    if (m_speed < 0.1f || m_obstacles.empty())
        return pursuitSteer;

    // Vehicle entries are cached for sensor membership, but collision prediction
    // must start from their current transforms.
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

    // Only after a predicted collision, try the smallest correction first.
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

#if 0 // Legacy fan/slot avoidance steering.
    // 옆(90도)까지 본다. 나란히 붙은 차는 예전 60도 부채꼴에 아예 안 잡혀서,
    // 측면 접촉에 대한 최후 방어선이 없었다.
    constexpr int SLOT_COUNT = 15; // 180도/14 ~= 12.9도per슬롯 (기존 12도와 비슷한 해상도)
    constexpr float FAN_HALF = ToRadians(90.0f);
    // 측면 감지는 ±90도로 유지하되 실제 회피 방향은 앞쪽 ±40도 안에서만 고른다.
    // 넓게 열면 장애물을 피하려다 옆 차선으로 급격히 꺾는 명령이 나온다.
    constexpr float STEER_PICK_HALF = ToRadians(40.0f);
    // 이 각도 밖을 '옆'으로 보고, 막혔으면 그쪽으로 꺾는 것 자체를 막는다.
    constexpr float SIDE_ZONE_BEGIN = ToRadians(50.0f);
    constexpr float SIDE_BLOCK_CUTOFF = 0.5f;
    constexpr float TTC_HORIZON = 3.0f;     // 이 시간 밖 위협은 무시
    constexpr float PROXIMITY_RANGE = 6.0f; // 접근속도 0이어도 위험한 거리
    constexpr float DANGER_CUTOFF = 0.35f;  // 이 위 슬롯은 봉쇄로 본다
    // 예전엔 위협이 조금이라도 있으면 슬롯 조향이 추종 조향을 통째로 덮었다. 그러면
    // 추월하려고 옆에 나란히 붙는 것 자체가 "위험"으로 읽혀 계속 밀려나, 전술 레이어가
    // 무슨 오프셋을 내놓든 매 프레임 무효가 된다. 이제는 가려던 방향이 실제로 막혔을
    // 때만 개입하는 안전망이다.
    constexpr float INTERVENE_CUTOFF = 0.6f;
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

    // 좌우 옆구리가 각각 얼마나 막혀 있는지. 조향 방향 선택과 별개로 쓴다 --
    // 정면이 비어 있으면 아래 개입 조건에 안 걸리는데, 그때도 옆으로는 꺾으면 안 된다.
    float leftSideDanger = 0.0f;
    float rightSideDanger = 0.0f;
    for (int i = 0; i < SLOT_COUNT; ++i)
    {
        float slotRad = -FAN_HALF + i * slotStep;
        if (slotRad <= -SIDE_ZONE_BEGIN)
            leftSideDanger = std::max(leftSideDanger, danger[i]);
        else if (slotRad >= SIDE_ZONE_BEGIN)
            rightSideDanger = std::max(rightSideDanger, danger[i]);
    }

    // 옆이 막혔으면 그쪽으로 꺾는 것을 금지한다. 양수 조향 = 우회전.
    auto applySideVeto = [&](float steer)
    {
        if (rightSideDanger >= SIDE_BLOCK_CUTOFF)
            steer = std::min(steer, 0.0f);
        if (leftSideDanger >= SIDE_BLOCK_CUTOFF)
            steer = std::max(steer, 0.0f);
        return steer;
    };

    if (totalDanger <= 0.0f)
    {
        m_ctxSteerDebugRad = 0.0f;
        m_ctxSteerDebugDanger = 0.0f;
        return pursuitSteer; // 위협 없으면 슬롯 양자화로 흔들 필요 없다
    }

    float desiredRad = WrapAngleRad(DirectionToAngleRad(pursuitTarget - position) - headingRad);

    // 가려던 방향 자체가 막히지 않았으면 추종 조향을 그대로 둔다(옆 금지는 그래도 건다).
    int desiredSlot = std::clamp(static_cast<int>(std::lround((desiredRad + FAN_HALF) / slotStep)),
                                 0, SLOT_COUNT - 1);
    if (danger[desiredSlot] < INTERVENE_CUTOFF)
    {
        m_ctxSteerDebugRad = desiredRad;
        m_ctxSteerDebugDanger = danger[desiredSlot];
        return applySideVeto(pursuitSteer);
    }

    int best = -1;
    float bestInterest = -1.0f;
    for (int i = 0; i < SLOT_COUNT; ++i)
    {
        if (danger[i] >= DANGER_CUTOFF)
            continue;
        float slotRad = -FAN_HALF + i * slotStep;
        if (std::fabs(slotRad) > STEER_PICK_HALF) // 옆 슬롯은 감지용이지 주행 방향 후보가 아니다
            continue;
        float interest = std::max(0.0f, cosf(slotRad - desiredRad)) + (slotRad < 0.0f ? RIGHT_BIAS : 0.0f);
        if (interest > bestInterest)
        {
            bestInterest = interest;
            best = i;
        }
    }

    if (best < 0) // 전부 봉쇄면 앞쪽 중 가장 덜 위험한 쪽으로라도
    {
        float least = std::numeric_limits<float>::max();
        for (int i = 0; i < SLOT_COUNT; ++i)
        {
            if (std::fabs(-FAN_HALF + i * slotStep) > STEER_PICK_HALF)
                continue;
            if (danger[i] < least)
            {
                least = danger[i];
                best = i;
            }
        }
    }

    m_ctxSteerDebugRad = -FAN_HALF + best * slotStep;
    m_ctxSteerDebugDanger = danger[best];

    float chosenRad = headingRad + m_ctxSteerDebugRad;
    Vec3 aim = position + Vec3(cosf(chosenRad), 0.0f, sinf(chosenRad)) * STEER_LOOKAHEAD;
    float steer = std::clamp(PurePursuitSteerAt(position, headingRad, aim, m_wheelbase),
                             -m_maxSteerAngle, m_maxSteerAngle);
    return applySideVeto(steer);
#endif
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
