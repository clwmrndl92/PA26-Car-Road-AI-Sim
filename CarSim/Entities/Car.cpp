#include "Car.h"
#include "Rendering/Effects.h"
#include <ModelManager.h>
#include <algorithm>
#include <cmath>
#include <imgui.h>

void Car::Init(const CarSpec &spec)
{
    GetRender().SetModel(ModelManager::Get().CreateFromFile(spec.modelPath));
    SetRenderOffset(ToXMFLOAT3(spec.renderOffset));
    GameObject::Init(spec.halfExtents, Rigidbody::Type::Dynamic, spec.colliderOffset, spec.mass);

    m_spawnPosition = GetTransform().GetPosition();
    m_spawnRotation = GetTransform().GetRotationQuat();
    m_wheelbase = spec.wheelbase;
    m_mass = spec.mass;

    // Line sticking out the front of the car, showing the current steering direction
    Model *pLine = ModelManager::Get().CreateFromGeometry("__steer_line__:" + GetName(),
                                                          Geometry::CreateLine(DirectX::XMFLOAT3(0.0f, 0.0f, 2.0f), DirectX::XMFLOAT3(0.0f, 0.0f, 8.0f)));
    pLine->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(0.0f, 0.0f, 1.0f, 1.0f));
    pLine->materials[0].Set<float>("$Opacity", 1.0f);
    m_steerLine.SetModel(pLine);
}

void Car::Update(float dt)
{
    UpdateReset();
    UpdateGear();
    UpdateAcceleration(dt);
    float maxSteerAngle = UpdateSteering(dt);
    UpdateDebugWindow(maxSteerAngle);
    ApplyMotion();
    UpdateTrail();
}

void Car::UpdateGear()
{
    constexpr float GEAR_SWITCH_SPEED_THRESHOLD = 2.0f / 3.6f; // 2 km/h

    if (m_isControlled && m_speed <= GEAR_SWITCH_SPEED_THRESHOLD && ImGui::IsKeyPressed(ImGuiKey_Z, false)) // Toggle Drive / Reverse gear
        m_isReverse = !m_isReverse;
}

void Car::UpdateReset()
{
    if (!m_isControlled || !ImGui::IsKeyPressed(ImGuiKey_Space, false))
        return;

    m_rigidbody.SetPositionAndRotation(
        JPH::Vec3(m_spawnPosition.x, m_spawnPosition.y, m_spawnPosition.z),
        JPH::Quat(m_spawnRotation.x, m_spawnRotation.y, m_spawnRotation.z, m_spawnRotation.w));
    m_rigidbody.SetLinearVelocity(JPH::Vec3::sZero());
    m_rigidbody.SetAngularVelocity(JPH::Vec3::sZero());
    m_speed = 0.0f;
    m_acceleration = 0.0f;

    m_rearTrail.clear();
    m_frontTrail.clear();
    m_rearTrailRender.SetModel(nullptr);
    m_frontTrailRender.SetModel(nullptr);
}

void Car::UpdateAcceleration(float dt)
{
    constexpr float ACCEL_RAMP_RATE = 11.1f; // reaches m_maxAcceleration in ~0.25s
    constexpr float BRAKE_RAMP_RATE = 55.6f; // reaches m_maxBrakeDeceleration in ~0.17s
    // constexpr float FRICT_DECEL_RATE = 0.1f;
    constexpr float FRICT_DECEL_RATE = 0.0f;

    if (m_isControlled && ImGui::IsKeyDown(ImGuiKey_DownArrow)) // Brake
        m_acceleration = std::max(std::min(m_acceleration, 0.0f) - BRAKE_RAMP_RATE * dt, -m_maxBrakeDeceleration);
    else if (m_isControlled && ImGui::IsKeyDown(ImGuiKey_UpArrow)) // Accelerate
        m_acceleration = std::min(std::max(m_acceleration, 0.0f) + ACCEL_RAMP_RATE * dt, m_maxAcceleration);
    else
        m_acceleration = 0.0f;

    if (m_acceleration == 0.0f)
    {
        // natural deceleration (drag) when coasting
        m_speed -= m_speed * FRICT_DECEL_RATE * dt;

        if (m_speed < 0.1f)
            m_speed = 0.0f;
    }
    else
        m_speed += m_acceleration * dt;

    m_speed = std::clamp(m_speed, 0.0f, m_maxSpeed);
}

float Car::UpdateSteering(float dt)
{
    // constexpr float STEER_RAMP_RATE = 0.4f;           // todo: vary 0.3 (calm) ~ 1.0 (urgent) by input intensity
    constexpr float STEER_RAMP_RATE = 0.4f;           // todo: vary 0.3 (calm) ~ 1.0 (urgent) by input intensity
    constexpr float LOW_SPEED_CUTOFF = 18.26f / 3.6f; // below this, use m_maxSteerAngle (formula below would exceed it)

    if (m_isControlled && ImGui::IsKeyDown(ImGuiKey_LeftArrow))
        m_steerAngle = std::min(m_steerAngle, 0.0f) - STEER_RAMP_RATE * dt;
    else if (m_isControlled && ImGui::IsKeyDown(ImGuiKey_RightArrow))
        m_steerAngle = std::max(m_steerAngle, 0.0f) + STEER_RAMP_RATE * dt;
    else if (m_steerAngle > 0.0f) // Return to center
        m_steerAngle = std::max(m_steerAngle - STEER_RAMP_RATE * dt, 0.0f);
    else
        m_steerAngle = std::min(m_steerAngle + STEER_RAMP_RATE * dt, 0.0f);

    float maxSteerAngle = (m_speed <= LOW_SPEED_CUTOFF) ? m_maxSteerAngle : 20.2f / (m_speed * m_speed); // tuned so 100 km/h -> ~1.5 deg
    m_steerAngle = std::clamp(m_steerAngle, -maxSteerAngle, maxSteerAngle);
    return maxSteerAngle;
}

void Car::UpdateDebugWindow(float maxSteerAngle)
{
    if (!m_drawCollider || !m_isControlled)
        return;

    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x, 0.0f), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
    if (ImGui::Begin(("Car: " + GetName()).c_str()))
    {
        DirectX::XMFLOAT3 pos = GetTransform().GetPosition();
        ImGui::Text("Pos: %.1f %.1f %.1f", pos.x, pos.y, pos.z);
        ImGui::Text("Speed: %.1f km/h", m_speed * 3.6f);
        ImGui::Text("Accel: %.1f km/h/s", m_acceleration * 3.6f);
        ImGui::Text("Steer: %.2f / %.2f", m_steerAngle, maxSteerAngle);
        ImGui::Text("ActualVel: %.2f", m_rigidbody.GetLinearVelocity().Length());
        ImGui::Text("DesiredVel: %.2f", ComputeDesiredVelocity().Length());
    }
    ImGui::End();
}

JPH::Vec3 Car::ComputeDesiredVelocity() const
{
    float signedSpeed = GetSignedSpeed();
    DirectX::XMFLOAT3 fwd = GetTransform().GetForwardAxis();
    float vy = m_rigidbody.GetLinearVelocity().GetY();
    return JPH::Vec3(fwd.x * signedSpeed, vy, fwd.z * signedSpeed);
}

void Car::ApplyMotion()
{
    // Steering stays kinematic -- the bicycle model already gives the correct yaw rate.
    float angularVelocity = GetSignedSpeed() * tan(m_steerAngle) / m_wheelbase;
    m_rigidbody.SetAngularVelocity(JPH::Vec3(0.0f, angularVelocity, 0.0f));

    JPH::Vec3 actualVel = m_rigidbody.GetLinearVelocity();
    JPH::Vec3 desiredVel = ComputeDesiredVelocity();
    m_rigidbody.SetLinearVelocity(desiredVel);

    // constexpr float SPEED_DIVERGENCE_THRESHOLD = 5.0f; // m/s; how far physics can drift before we yield to it

    // if ((actualVel - desiredVel).Length() > SPEED_DIVERGENCE_THRESHOLD)
    // {
    //     m_speed = JPH::Vec3(actualVel.GetX(), 0.0f, actualVel.GetZ()).Length();
    //     m_acceleration = 0.0f;
    // }
    // else
    // {
    // }
}

void Car::UpdateTrail()
{
    if (!m_drawCollider) // only track/rebuild the trail for cars actually shown in debug view
        return;

    using namespace DirectX;

    XMFLOAT3 rearPos = GetTransform().GetPosition();
    XMFLOAT3 fwd = GetTransform().GetForwardAxis();
    XMFLOAT3 frontPos(rearPos.x + fwd.x * m_wheelbase, rearPos.y + fwd.y * m_wheelbase, rearPos.z + fwd.z * m_wheelbase);

    auto recordPoint = [](std::deque<XMFLOAT3> &trail, const XMFLOAT3 &pos)
    {
        if (!trail.empty())
        {
            XMVECTOR diff = XMVectorSubtract(XMLoadFloat3(&pos), XMLoadFloat3(&trail.back()));
            if (XMVectorGetX(XMVector3LengthSq(diff)) < TRAIL_SAMPLE_DISTANCE * TRAIL_SAMPLE_DISTANCE)
                return false;
        }
        trail.push_back(pos);
        if (trail.size() > TRAIL_MAX_POINTS)
            trail.pop_front();
        return true;
    };

    if (recordPoint(m_rearTrail, rearPos) && m_rearTrail.size() >= 2)
        RebuildTrailRender(m_rearTrailRender, m_rearTrail, "__rear_trail__:" + GetName(), XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f));

    if (recordPoint(m_frontTrail, frontPos) && m_frontTrail.size() >= 2)
        RebuildTrailRender(m_frontTrailRender, m_frontTrail, "__front_trail__:" + GetName(), XMFLOAT4(0.0f, 1.0f, 1.0f, 1.0f));
}

void Car::RebuildTrailRender(RenderObject &render, const std::deque<DirectX::XMFLOAT3> &trail,
                             const std::string &name, const DirectX::XMFLOAT4 &color)
{
    std::vector<DirectX::XMFLOAT3> points(trail.begin(), trail.end());
    Model *pModel = ModelManager::Get().CreateFromGeometry(name, Geometry::CreatePolyline(points));
    pModel->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", color);
    pModel->materials[0].Set<float>("$Opacity", 1.0f);
    render.SetModel(pModel);
}

void Car::Draw(ID3D11DeviceContext *context, IEffect &effect)
{
    GameObject::Draw(context, effect);

    if (m_drawCollider && (m_rearTrailRender.GetModel() || m_frontTrailRender.GetModel()))
    {
        if (auto *pBasic = dynamic_cast<BasicEffect *>(&effect))
        {
            pBasic->SetRenderLines();
            if (m_rearTrailRender.GetModel())
                m_rearTrailRender.Draw(context, effect);
            if (m_frontTrailRender.GetModel())
                m_frontTrailRender.Draw(context, effect);
            pBasic->SetRenderDefault();
        }
    }

    if (!m_drawCollider || !m_steerLine.GetModel())
        return;

    using namespace DirectX;

    // Car only ever yaws around world Y, so the steer-angle offset and the car's own
    // rotation share an axis and can be combined in either order.
    XMFLOAT4 carRotF = GetTransform().GetRotationQuat();
    XMVECTOR carRot = XMLoadFloat4(&carRotF);
    XMVECTOR steerYaw = XMQuaternionRotationAxis(g_XMIdentityR1, m_steerAngle);
    XMVECTOR lineRot = XMQuaternionNormalize(XMQuaternionMultiply(carRot, steerYaw));

    XMFLOAT4 lineRotF;
    XMStoreFloat4(&lineRotF, lineRot);
    m_steerLine.GetTransform().SetPosition(GetTransform().GetPosition());
    m_steerLine.GetTransform().SetRotation(lineRotF);

    if (auto *pBasic = dynamic_cast<BasicEffect *>(&effect))
    {
        pBasic->SetRenderLines();
        m_steerLine.Draw(context, effect);
        pBasic->SetRenderDefault();
    }
}
