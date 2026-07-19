#include "Lane.h"
#include "Road.h"

Lane::Lane(int id, const Spline &spline, const shared_ptr<Road> &road)
    : m_spline(spline), m_id(id), m_road(road),
      length(m_spline.GetLength())
{
}

Lane::~Lane()
{
}

float Lane::GetLimitSpeed() const
{
    return m_road ? m_road->GetSpeedLimit() : 999.0f;
}
