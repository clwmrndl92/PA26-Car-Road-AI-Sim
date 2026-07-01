#pragma once
#include "Core/GameObject.h"

class Car : public GameObject
{
public:
    void Update(float dt) override;

    void SetAcceleration(float accel) { m_acceleration = accel; }
    float GetSpeed() const { return m_speed; }

private:
    const float m_maxSpeed = 5.0f;
    float m_speed        = 0.0f;
    float m_acceleration = 0.1f;
};
