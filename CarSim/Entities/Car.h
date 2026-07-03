#pragma once
#include "Core/GameObject.h"
#include "CarSpec.h"
#include <deque>
#include <string>

class Car : public GameObject
{
public:
    // `position` is the front axle (see GetPosition/SetPosition below); defaults to world origin.
    void Init(const CarSpec &spec, JPH::Vec3 position = JPH::Vec3::sZero());

    void Update(float dt) override;
    void Draw(ID3D11DeviceContext *context, IEffect &effect) override;

    // GameObject's origin is the rear axle (steering/physics reference point), but everything
    // outside this class should see/set the front axle instead -- add wheelbase forward of it.
    DirectX::XMFLOAT3 GetPosition() const override;
    void SetPosition(float x, float y, float z) override;

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
    void UpdateTrail(); // records rear/front axle trail points and rebuilds their render geometry
    void RebuildTrailRender(RenderObject &render, const std::deque<DirectX::XMFLOAT3> &trail,
                            const std::string &name, const DirectX::XMFLOAT4 &color);
    float GetSignedSpeed() const { return m_speed * (m_isReverse ? -1.0f : 1.0f); } // + forward gear, - reverse gear
    JPH::Vec3 ComputeDesiredVelocity() const;                                       // target linear velocity from current speed/gear/facing

    bool m_isControlled = false; // only respond to keyboard input while selected (see GameApp)

    const float m_maxSpeed = 55.56f;            // 200 km/h
    const float m_maxAcceleration = 2.78f;      // 0-100 km/h in 10s
    const float m_maxBrakeDeceleration = 9.26f; // 100-0 km/h in 3s
    float m_wheelbase = 0.0f;                   // distance between front and rear axles; set in Init(), so 0 until then
    float m_mass = 1.0f;                        // kg, set from CarSpec in Init(); force = mass * m_acceleration
    float m_speed = 0.0f;                       // planar speed magnitude, unsigned
    float m_acceleration = 0.0f;
    float m_maxSteerAngle = 0.785f; // radians (45 deg), cap at zero speed (shrinks as speed increases)
    float m_steerAngle = 0.0f;
    bool m_isReverse = false;

    DirectX::XMFLOAT3 m_spawnPosition = {0.0f, 0.0f, 0.0f}; // set in Init(), restored on reset (Space)
    DirectX::XMFLOAT4 m_spawnRotation = {0.0f, 0.0f, 0.0f, 1.0f};

    RenderObject m_steerLine; // shown in front of the car when SetDrawCollider(true)

    static constexpr float TRAIL_SAMPLE_DISTANCE = 0.5f; // meters travelled between recorded trail points
    static constexpr size_t TRAIL_MAX_POINTS = 2000;     // oldest points drop off once exceeded

    std::deque<DirectX::XMFLOAT3> m_rearTrail;  // world-space path of the rear axle (car's true origin)
    std::deque<DirectX::XMFLOAT3> m_frontTrail; // world-space path of the front axle (origin + wheelbase forward)
    RenderObject m_rearTrailRender;
    RenderObject m_frontTrailRender;
};
