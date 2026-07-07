#pragma once
#include <vector>
#include "Core/MathUtil.h"

class Spline
{
public:
    Spline();
    ~Spline();

    void AddControlPoint(const Vec3 &point);
    size_t GetControlPointCount() const;
    const std::vector<Vec3> &GetControlPoints() const { return m_controlPoints; }

    std::vector<Vec3> GenerateSplinePoints() const;

private:
    // Private members for spline implementation
    std::vector<Vec3> m_controlPoints; // Control points for the spline
    static constexpr int CURVE_RESOLUTION = 100;

    Vec3 GetCatmullRomPoint(float t, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) const;
};