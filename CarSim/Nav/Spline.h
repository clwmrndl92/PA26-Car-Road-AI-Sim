#pragma once
#include <limits>
#include <vector>
#include "Utill/MathUtil.h"

class Spline
{
public:
    Spline();
    Spline(const std::vector<Vec3> &points);
    ~Spline();

    size_t GetControlPointCount() const;
    const std::vector<Vec3> &GetControlPoints() const { return m_controlPoints; }
    const std::vector<Vec3> &GetSplinePoints() const { return m_splinePoints; }
    float GetLength() const { return m_length; }

    Vec3 GetLookaheadPoint(const Vec3 &position, float lookaheadDistance) const;
    float GetSplinePosition(const Vec3 &position) const;
    Vec3 GetPositionAt(float t) const;
    Vec3 GetDirectionAt(float t) const;

    // 스플라인 전체에서 가장 급한 커브(정점)의 반경/위치. 생성자에서 미리 계산해 캐싱한다.
    float GetMinRadiusAhead() const { return m_minRadius; }
    float GetApexT() const { return m_apexT; }

private:
    float m_length = 0.0f;
    std::vector<Vec3> m_controlPoints; // Control points for the spline
    std::vector<Vec3> m_splinePoints;  // Catmull-Rom 샘플 점들
    float m_minRadius = std::numeric_limits<float>::max();
    float m_apexT = 1.0f;
    static constexpr int CURVE_RESOLUTION = 50;

    Vec3 GetCatmullRomPoint(float t, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) const;
    Vec3 GetCatmullRomTangent(float t, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) const;
    std::vector<Vec3> ComputeSplinePoints();
    void ComputeMinRadius();
    bool IsStraight() const; // 컨트롤 포인트가 전부 한 직선 위에 있는지 (ComputeMinRadius가 스캔 생략 여부 판단에 씀)
};