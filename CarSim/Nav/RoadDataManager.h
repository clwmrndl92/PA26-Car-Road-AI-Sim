#pragma once
#include <string>
#include "Spline.h"

class RoadDataManager
{
public:
    RoadDataManager() = default;
    ~RoadDataManager() = default;
    void Init(const std::string &filePath);
    Vec3 GetPositionOnRoad(Vec3 position, float lookaheadDistance) const;
    Spline GetSpline() const { return m_spline; }

private:
    Spline m_spline;
};