#pragma once
#include <vector>
#include <memory>

class Road
{
public:
    Road(int id, float speedLimit);
    ~Road() = default;

    int GetId() const { return m_id; }
    float GetSpeedLimit() const { return m_speedLimit; }

private:
    int m_id;
    float m_speedLimit;
};