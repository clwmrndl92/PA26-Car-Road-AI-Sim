#include "Lane.h"
#include <vector>

Lane::Lane(int id, const Spline &spline, int roadID)
    : m_spline(spline), m_id(id), m_roadID(roadID)
{
}

Lane::~Lane()
{
}

Vec3 Lane::GetLookaheadPoint(Vec3 position, float lookaheadDistance, Vec3 targetPosition)
{
    vector<Vec3> splinePoints = m_spline.GenerateSplinePoints();
    if (splinePoints.empty())
        return position;

    float closestDistance = numeric_limits<float>::max();
    size_t closestIndex = 0;
    for (size_t i = 0; i < splinePoints.size(); ++i)
    {
        float distance = (splinePoints[i] - position).Length();
        if (distance < closestDistance)
        {
            closestDistance = distance;
            closestIndex = i;
        }
    }

    // Nodes can share a lane (e.g. a lane's whole spline spans several nodes), so the
    // lookahead must not run past the node we're currently heading to, or it overshoots
    // straight to the lane's endpoint instead of passing through the node.
    float closestTargetDistance = numeric_limits<float>::max();
    size_t targetIndex = splinePoints.size() - 1;
    for (size_t i = 0; i < splinePoints.size(); ++i)
    {
        float distance = (splinePoints[i] - targetPosition).Length();
        if (distance < closestTargetDistance)
        {
            closestTargetDistance = distance;
            targetIndex = i;
        }
    }

    size_t lookaheadIndex = closestIndex;
    float accumulatedDistance = 0.0f;
    while (accumulatedDistance < lookaheadDistance && lookaheadIndex < targetIndex)
    {
        accumulatedDistance += (splinePoints[lookaheadIndex + 1] - splinePoints[lookaheadIndex]).Length();
        ++lookaheadIndex;
    }

    return splinePoints[lookaheadIndex];
}
