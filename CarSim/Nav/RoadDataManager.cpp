#include "RoadDataManager.h"
#include "DataParser.h"

void RoadDataManager::Init(const std::string &filePath)
{

    m_spline = DataParser::ParseSplineData(filePath);
}

Vec3 RoadDataManager::GetPositionOnRoad(Vec3 position, float lookaheadDistance) const
{
    std::vector<Vec3> splinePoints = m_spline.GenerateSplinePoints();
    if (splinePoints.empty())
        return position;

    float closestDistance = std::numeric_limits<float>::max();
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

    size_t lookaheadIndex = closestIndex;
    float accumulatedDistance = 0.0f;
    while (accumulatedDistance < lookaheadDistance && lookaheadIndex + 1 < splinePoints.size())
    {
        accumulatedDistance += (splinePoints[lookaheadIndex + 1] - splinePoints[lookaheadIndex]).Length();
        ++lookaheadIndex;
    }

    return splinePoints[lookaheadIndex];
}
