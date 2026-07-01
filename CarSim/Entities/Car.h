#pragma once
#include "Core/GameObject.h"
#include "CarSpec.h"

class Car : public GameObject
{
public:
    void Init(const CarSpec& spec);

    void Update(float dt) override;

    void SetAcceleration(float accel) { m_acceleration = accel; }
    float GetSpeed() const { return m_speed; }

private:
    float m_maxSpeed     = 0.0f;
    float m_speed        = 0.0f;
    float m_acceleration = 0.0f;
};
