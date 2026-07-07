#pragma once
#include "Spline.h"

using namespace std;

class Lane
{
public:
    // enum class LaneType
    // {
    //     Straight,
    //     Curve
    // };
    Lane(int id, const Spline &spline, int roadID);
    ~Lane();

    Vec3 GetLookaheadPoint(Vec3 position, float lookaheadDistance, Vec3 targetPosition);

    int GetId() const { return m_id; }
    const Spline &GetSpline() const { return m_spline; }
    int GetRoadID() const { return m_roadID; }

private:
    Spline m_spline; // Spline representing the lane's path
    int m_id;
    int m_roadID;
    // LaneType m_LaneType;
};