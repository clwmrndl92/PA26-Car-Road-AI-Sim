#pragma once
#include "Core/GameObject.h"
#include "CarSpec.h"

class Car : public GameObject
{
public:
    void Init(const CarSpec &spec);

    void Update(float dt) override;
    void Draw(ID3D11DeviceContext *context, IEffect &effect) override;

    void SetAcceleration(float accel) { m_acceleration = accel; }
    float GetSpeed() const { return m_speed; }
    float GetAcceleration() const { return m_acceleration; }
    void SetControlled(bool controlled) { m_isControlled = controlled; }

private:
    void UpdateGear();
    void UpdateReset();
    void UpdateAcceleration(float dt);
    float UpdateSteering(float dt); // returns current speed-scaled max steer angle
    void UpdateDebugWindow(float maxSteerAngle);
    void ApplyMotion();

    bool m_isControlled = false; // only respond to keyboard input while selected (see GameApp)

    const float m_maxSpeed = 55.56f;            // 200 km/h
    const float m_maxAcceleration = 2.78f;      // 0-100 km/h in 10s
    const float m_maxBrakeDeceleration = 9.26f; // 100-0 km/h in 3s
    float m_wheelbase;                          // distance between front and rear axles
    float m_mass = 1.0f;                        // kg, set from CarSpec in Init(); force = mass * m_acceleration
    float m_speed = 0.0f;                       // planar speed magnitude, read back from the rigidbody each frame
    float m_acceleration = 0.0f;
    float m_maxSteerAngle = 0.785f; // radians (45 deg), cap at zero speed (shrinks as speed increases)
    float m_steerAngle = 0.0f;
    bool m_isReverse = false;

    DirectX::XMFLOAT3 m_spawnPosition = {0.0f, 0.0f, 0.0f}; // set in Init(), restored on reset (Space)
    DirectX::XMFLOAT4 m_spawnRotation = {0.0f, 0.0f, 0.0f, 1.0f};

    RenderObject m_steerLine; // shown in front of the car when SetDrawCollider(true)
};
