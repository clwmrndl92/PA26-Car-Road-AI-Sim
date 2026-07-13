#pragma once
#include <memory>
#include "Spline.h"

using namespace std;

class Road;

class Lane
{
public:
    enum class LaneType
    {
        Straight,
        Curve
    };

    Lane(int id, const Spline &spline, const shared_ptr<Road> &road);
    ~Lane();

    int GetId() const { return m_id; }
    const Spline &GetSpline() const { return m_spline; }
    const shared_ptr<Road> &GetRoad() const { return m_road; }
    LaneType GetLaneType() const { return m_laneType; }
    bool IsStraight() const { return m_laneType == LaneType::Straight; }

private:
    Spline m_spline; // Spline representing the lane's path
    int m_id;
    shared_ptr<Road> m_road;
    LaneType m_laneType;
};