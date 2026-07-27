#include "Spline.h"
#include <limits>
#include <algorithm>
#include <cmath>
#include <Utill/DebugConsole.h>

Spline::Spline()
{
}

Spline::Spline(const std::vector<Vec3> &points)
    : m_controlPoints(points)
{
    m_splinePoints = ComputeSplinePoints();
    ComputeMinRadius();
}

Spline::~Spline()
{
}

size_t Spline::GetControlPointCount() const
{
    return m_controlPoints.size();
}

Vec3 Spline::GetCatmullRomPoint(float t, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) const
{
    float t2 = t * t;
    float t3 = t2 * t;

    float x = 0.5f * (2.0f * p1.GetX() +
                      (-p0.GetX() + p2.GetX()) * t +
                      (2.0f * p0.GetX() - 5.0f * p1.GetX() + 4.0f * p2.GetX() - p3.GetX()) * t2 +
                      (-p0.GetX() + 3.0f * p1.GetX() - 3.0f * p2.GetX() + p3.GetX()) * t3);

    float y = 0.5f * (2.0f * p1.GetY() +
                      (-p0.GetY() + p2.GetY()) * t +
                      (2.0f * p0.GetY() - 5.0f * p1.GetY() + 4.0f * p2.GetY() - p3.GetY()) * t2 +
                      (-p0.GetY() + 3.0f * p1.GetY() - 3.0f * p2.GetY() + p3.GetY()) * t3);

    float z = 0.5f * (2.0f * p1.GetZ() +
                      (-p0.GetZ() + p2.GetZ()) * t +
                      (2.0f * p0.GetZ() - 5.0f * p1.GetZ() + 4.0f * p2.GetZ() - p3.GetZ()) * t2 +
                      (-p0.GetZ() + 3.0f * p1.GetZ() - 3.0f * p2.GetZ() + p3.GetZ()) * t3);

    return Vec3(x, y, z);
}

Vec3 Spline::GetCatmullRomTangent(float t, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) const
{
    float t2 = t * t;

    float x = 0.5f * ((-p0.GetX() + p2.GetX()) +
                      2.0f * (2.0f * p0.GetX() - 5.0f * p1.GetX() + 4.0f * p2.GetX() - p3.GetX()) * t +
                      3.0f * (-p0.GetX() + 3.0f * p1.GetX() - 3.0f * p2.GetX() + p3.GetX()) * t2);

    float y = 0.5f * ((-p0.GetY() + p2.GetY()) +
                      2.0f * (2.0f * p0.GetY() - 5.0f * p1.GetY() + 4.0f * p2.GetY() - p3.GetY()) * t +
                      3.0f * (-p0.GetY() + 3.0f * p1.GetY() - 3.0f * p2.GetY() + p3.GetY()) * t2);

    float z = 0.5f * ((-p0.GetZ() + p2.GetZ()) +
                      2.0f * (2.0f * p0.GetZ() - 5.0f * p1.GetZ() + 4.0f * p2.GetZ() - p3.GetZ()) * t +
                      3.0f * (-p0.GetZ() + 3.0f * p1.GetZ() - 3.0f * p2.GetZ() + p3.GetZ()) * t2);

    return Vec3(x, y, z);
}

std::vector<Vec3> Spline::ComputeSplinePoints()
{
    std::vector<Vec3> splinePoints;
    m_length = 0.0f;
    int n = static_cast<int>(m_controlPoints.size());
    if (n < 4)
        return splinePoints; // Not enough control points for Catmull-Rom spline

    // 방향은 유지하고, 거리만 고정값으로 맞춘다.
    auto AdjustPhantom = [](Vec3 &phantom, const Vec3 &endpoint)
    {
        constexpr float PHANTOM_DISTANCE = 10.0f;

        Vec3 arm = phantom - endpoint; // endpoint -> phantom (접선 팔 방향)
        float armLen = arm.Length();
        if (armLen < 1e-4f)
            return;

        phantom = endpoint + (arm / armLen) * PHANTOM_DISTANCE;
    };

    // control points 치환됨
    AdjustPhantom(m_controlPoints.front(), m_controlPoints[1]);
    AdjustPhantom(m_controlPoints.back(), m_controlPoints[n - 2]);

    Vec3 prevSplinePoint = m_controlPoints[1];

    auto MakeSegment = [&](const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3)
    {
        for (int j = 0; j < CURVE_RESOLUTION; ++j)
        {
            float t = static_cast<float>(j) / (CURVE_RESOLUTION - 1);
            Vec3 point = GetCatmullRomPoint(t, p0, p1, p2, p3);
            splinePoints.push_back(point);
            m_length += (prevSplinePoint - point).Length();
            prevSplinePoint = point;
        }
    };

    for (int i = 1; i < n - 2; ++i)
    {
        MakeSegment(m_controlPoints[i - 1], m_controlPoints[i], m_controlPoints[i + 1], m_controlPoints[i + 2]);
    }

    return splinePoints;
}

Vec3 Spline::GetLookaheadPoint(const Vec3 &position, float lookaheadDistance) const
{
    if (m_splinePoints.empty())
        return position;

    float closestDistance = std::numeric_limits<float>::max();
    size_t closestIndex = 0;
    for (size_t i = 0; i < m_splinePoints.size(); ++i)
    {
        float distance = (m_splinePoints[i] - position).Length();
        if (distance < closestDistance)
        {
            closestDistance = distance;
            closestIndex = i;
        }
    }

    size_t lastIndex = m_splinePoints.size() - 1;
    size_t lookaheadIndex = closestIndex;
    float accumulatedDistance = 0.0f;
    if (lookaheadDistance >= 0.0f)
    {
        while (accumulatedDistance < lookaheadDistance && lookaheadIndex < lastIndex)
        {
            accumulatedDistance += (m_splinePoints[lookaheadIndex + 1] - m_splinePoints[lookaheadIndex]).Length();
            ++lookaheadIndex;
        }
    }
    else
    {
        while (accumulatedDistance < -lookaheadDistance && lookaheadIndex > 0)
        {
            accumulatedDistance += (m_splinePoints[lookaheadIndex - 1] - m_splinePoints[lookaheadIndex]).Length();
            --lookaheadIndex;
        }
    }

    return m_splinePoints[lookaheadIndex];
}

Vec3 Spline::GetPositionAt(float t) const
{
    if (m_splinePoints.empty())
        return Vec3(0.0f, 0.0f, 0.0f);

    // t=1.0이면 size()가 나와 배열 끝을 벗어나므로 마지막 인덱스로 클램프한다.
    int lastIndex = static_cast<int>(m_splinePoints.size()) - 1;
    int index = std::clamp(static_cast<int>(static_cast<float>(m_splinePoints.size()) * t), 0, lastIndex);
    return m_splinePoints[index];
}

Vec3 Spline::GetDirectionAt(float t) const
{
    int n = static_cast<int>(m_controlPoints.size());
    int segmentCount = n - 3;
    if (segmentCount < 1)
        return Vec3(0.0f, 0.0f, 0.0f);

    t = std::clamp(t, 0.0f, 1.0f);
    float scaledT = t * segmentCount;
    int segmentIndex = std::min(static_cast<int>(scaledT), segmentCount - 1);
    float localT = scaledT - segmentIndex;

    const Vec3 &p0 = m_controlPoints[segmentIndex];
    const Vec3 &p1 = m_controlPoints[segmentIndex + 1];
    const Vec3 &p2 = m_controlPoints[segmentIndex + 2];
    const Vec3 &p3 = m_controlPoints[segmentIndex + 3];

    return GetCatmullRomTangent(localT, p0, p1, p2, p3).Normalized();
}

bool Spline::IsStraight() const
{
    constexpr float STRAIGHT_EPSILON = 0.05f; // 양끝을 잇는 직선에서 이 거리(m) 안이면 일직선으로 본다

    if (m_controlPoints.size() < 2)
        return true;

    Vec3 dir = m_controlPoints.back() - m_controlPoints.front();
    float dirLength = dir.Length();
    if (dirLength < 1e-4f)
        return false; // 시작/끝이 사실상 같은 점 -- 직선으로 판단할 기준선이 없으므로 곡률 스캔으로 폴백

    dir = dir / dirLength;

    for (size_t i = 1; i + 1 < m_controlPoints.size(); ++i)
    {
        Vec3 toPoint = m_controlPoints[i] - m_controlPoints.front();
        Vec3 alongDir = dir * toPoint.Dot(dir);
        float perpDistance = (toPoint - alongDir).Length();
        if (perpDistance > STRAIGHT_EPSILON)
            return false;
    }
    return true;
}

void Spline::ComputeMinRadius()
{
    m_minRadius = std::numeric_limits<float>::max();
    m_apexT = 1.0f;

    if (IsStraight())
        return; // 직선, 곡률 없음

    if (m_splinePoints.size() < 2)
        return;

    size_t lastIndex = m_splinePoints.size() - 1;
    Vec3 prevTangent = GetDirectionAt(0.0f);

    for (size_t index = 0; index < lastIndex; ++index)
    {
        float segmentLength = (m_splinePoints[index + 1] - m_splinePoints[index]).Length();
        Vec3 tangent = GetDirectionAt(static_cast<float>(index + 1) / lastIndex);

        if (segmentLength > 1e-4f)
        {
            // 인접 샘플 사이의 각도가 매우 작아 acos(dot)는 정밀도가 무너짐 -> sin 근사(cross)로 계산
            float sinAngle = std::clamp(prevTangent.Cross(tangent).Length(), 0.0f, 1.0f);
            float deltaAngle = std::asin(sinAngle);
            if (deltaAngle > 1e-6f)
            {
                float radius = segmentLength / deltaAngle;
                if (radius < m_minRadius)
                {
                    m_minRadius = radius;
                    m_apexT = static_cast<float>(index) / lastIndex;
                }
            }
        }

        prevTangent = tangent;
    }
}

/// @param position
/// @return 0~1, 가까운 spline상의 위치, -1 : spline이 없음
float Spline::GetSplinePosition(const Vec3 &position) const
{
    if (m_splinePoints.empty() || m_splinePoints.size() < 2)
    {
        DebugConsole::Log("[Error] GetSplinePosition() : splinePoins.size() < 2 (" + ToString(m_splinePoints.size()) + ")");
        return -1.0f;
    }

    float closestDistance = std::numeric_limits<float>::max();
    size_t closestIndex = 0;
    for (size_t i = 0; i < m_splinePoints.size(); ++i)
    {
        float distance = (m_splinePoints[i] - position).Length();
        if (distance < closestDistance)
        {
            closestDistance = distance;
            closestIndex = i;
        }
    }

    return closestIndex / (m_splinePoints.size() - 1.0f);
}
