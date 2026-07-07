#include "Spline.h"

Spline::Spline()
{
}

Spline::~Spline()
{
}

void Spline::AddControlPoint(const Vec3 &point)
{
    m_controlPoints.push_back(point);
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

std::vector<Vec3> Spline::GenerateSplinePoints() const
{
    std::vector<Vec3> splinePoints;
    int n = static_cast<int>(m_controlPoints.size());
    std::vector<Vec3> controlPoints;
    if (n < 2)
        return splinePoints; // Not enough control points for Catmull-Rom spline

    controlPoints.push_back(m_controlPoints[0]);
    for (auto const &point : m_controlPoints)
    {
        controlPoints.push_back(point);
    }
    controlPoints.push_back(m_controlPoints.back());

    auto MakeSegment = [&](const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3)
    {
        for (int j = 0; j < CURVE_RESOLUTION; ++j)
        {
            float t = static_cast<float>(j) / (CURVE_RESOLUTION - 1);
            Vec3 point = GetCatmullRomPoint(t, p0, p1, p2, p3);
            splinePoints.push_back(point);
        }
    };

    for (int i = 1; i < n; ++i)
    {
        MakeSegment(controlPoints[i - 1], controlPoints[i], controlPoints[i + 1], controlPoints[i + 2]);
    }

    return splinePoints;
}
