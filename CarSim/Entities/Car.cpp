#include "Car.h"
#include <ModelManager.h>
#include <algorithm>

void Car::Init(const CarSpec& spec)
{
    GetRender().SetModel(ModelManager::Get().CreateFromFile(spec.modelPath));
    GameObject::Init(spec.halfExtents, Rigidbody::Type::Dynamic);

    m_maxSpeed     = spec.maxSpeed;
    m_acceleration = spec.acceleration;
}

void Car::Update(float dt)
{
    m_speed += m_acceleration * dt;
    m_speed = std::clamp(m_speed, 0.0f, m_maxSpeed);

    DirectX::XMFLOAT3 fwd = m_render.GetTransform().GetForwardAxis();
    float vy = m_rigidbody.GetLinearVelocity().GetY();
    m_rigidbody.SetLinearVelocity(JPH::Vec3(fwd.x * m_speed, vy, fwd.z * m_speed));
}
