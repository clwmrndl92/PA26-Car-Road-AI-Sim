#pragma once
#include <vector>
#include "Utill/MathUtil.h"

class Spline
{
public:
    Spline();
    Spline(const std::vector<Vec3> &points);
    ~Spline();

    size_t GetControlPointCount() const;
    const std::vector<Vec3> &GetControlPoints() const { return controlPoints; }
    const std::vector<Vec3> &GetSplinePoints() const { return splinePoints; }
    float GetLength() const { return length; }

    Vec3 GetLookaheadPoint(const Vec3 &position, float lookaheadDistance) const;
    float GetSplinePosition(const Vec3 &position) const;
    Vec3 GetPositionAt(float t) const;
    Vec3 GetDirectionAt(float t) const;
    float GetMinRadiusAhead(float start, float end, float *outApexT = nullptr) const;
    void AddSplinePointsFront(const std::vector<Vec3> &points, float t = 0);
    void AddSplinePointsBack(const std::vector<Vec3> &points, float t = 1);

private:
    float length = 0.0f;
    std::vector<Vec3> controlPoints; // Control points for the spline
    std::vector<Vec3> splinePoints;  // Catmull-Rom 샘플 점들 (생성자에서 미리 계산해 캐싱)
    static constexpr int CURVE_RESOLUTION = 50;

    Vec3 GetCatmullRomPoint(float t, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) const;
    Vec3 GetCatmullRomTangent(float t, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) const;
    std::vector<Vec3> ComputeSplinePoints();
};