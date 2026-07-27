#include "Lane.h"
#include "Road.h"

Lane::Lane(int id, const Spline &spline, const shared_ptr<Road> &road)
    : m_spline(spline), m_id(id), m_road(road),
      m_length(m_spline.GetLength())
{
}

Lane::~Lane()
{
}

float Lane::GetLimitSpeed() const
{
    constexpr float SPEED_LIMIT = 999.0f;
    return m_road ? m_road->GetSpeedLimit() : SPEED_LIMIT;
}
