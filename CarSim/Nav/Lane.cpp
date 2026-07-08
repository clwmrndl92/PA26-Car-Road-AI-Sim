#include "Lane.h"

Lane::Lane(int id, const Spline &spline, const shared_ptr<Road> &road)
    : m_spline(spline), m_id(id), m_road(road),
      m_laneType(spline.IsStraight() ? LaneType::Straight : LaneType::Curve)
{
}

Lane::~Lane()
{
}
