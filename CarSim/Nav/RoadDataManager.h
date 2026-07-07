#pragma once
#include <string>
#include "Spline.h"

class RoadDataManager
{
public:
    RoadDataManager() = default;
    ~RoadDataManager() = default;
    void Init(const std::string &filePath);
    bool HasDestination() const;
    Vec3 GetDestination() const;
    Vec3 GetPositionOnRoad(Vec3 position, float lookaheadDistance) const;
    Spline GetSpline() const { return m_spline; }

private:
    Spline m_spline;
};

// 도로데이터
// 도로가 몇차선일지
// 교차로인지
//