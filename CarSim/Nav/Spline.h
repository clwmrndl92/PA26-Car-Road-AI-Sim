#pragma once
#include <vector>
#include "Core/MathUtil.h"

class Spline
{
public:
    Spline();
    Spline(const std::vector<Vec3> &points);
    ~Spline();

    size_t GetControlPointCount() const;
    const std::vector<Vec3> &GetControlPoints() const { return m_controlPoints; }

    const std::vector<Vec3> &GenerateSplinePoints() const { return m_splinePoints; }

    Vec3 GetLookaheadPoint(const Vec3 &position, float lookaheadDistance) const;
    Vec3 GetDirectionAt(float t) const;
    float GetMinRadiusAhead(const Vec3 &position, float lookaheadDistance) const;
    bool IsStraight() const;

private:
    // Private members for spline implementation
    std::vector<Vec3> m_controlPoints; // Control points for the spline
    std::vector<Vec3> m_splinePoints;  // Catmull-Rom 샘플 점들 (생성자에서 미리 계산해 캐싱)
    static constexpr int CURVE_RESOLUTION = 100;

    Vec3 GetCatmullRomPoint(float t, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) const;
    Vec3 GetCatmullRomTangent(float t, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) const;
    std::vector<Vec3> ComputeSplinePoints() const;
};