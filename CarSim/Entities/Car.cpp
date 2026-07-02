#include "Car.h"
#include <ModelManager.h>
#include <algorithm>
#include <imgui.h>

void Car::Init(const CarSpec &spec)
{
    GetRender().SetModel(ModelManager::Get().CreateFromFile(spec.modelPath));
    SetRenderOffset(ToXMFLOAT3(spec.renderOffset));
    GameObject::Init(spec.halfExtents, Rigidbody::Type::Dynamic, spec.colliderOffset);
}

void Car::Update(float dt)
{
    constexpr float ACCEL_RAMP_RATE = 10.0f; // acceleration change per second held
    constexpr float BRAKE_RAMP_RATE = 50.0f; // braking ramps up faster than accelerating

    if (ImGui::IsKeyDown(ImGuiKey_X)) // Brake
        m_acceleration = std::min(m_acceleration, 0.0f) - BRAKE_RAMP_RATE * dt;
    else if (ImGui::IsKeyDown(ImGuiKey_Z)) // Accelerate
        m_acceleration = std::max(m_acceleration, 0.0f) + ACCEL_RAMP_RATE * dt;
    else
        m_acceleration = 0.0f;
    m_acceleration = std::clamp(m_acceleration, -m_maxAcceleration, m_maxAcceleration);

    m_speed += m_acceleration * dt;
    m_speed = std::clamp(m_speed, 0.0f, m_maxSpeed);

    DirectX::XMFLOAT3 fwd = GetTransform().GetForwardAxis();
    float vy = m_rigidbody.GetLinearVelocity().GetY();
    m_rigidbody.SetLinearVelocity(JPH::Vec3(fwd.x * m_speed, vy, fwd.z * m_speed));
}
