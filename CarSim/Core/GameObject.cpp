#include "GameObject.h"
#include "Core/Physics/PhysicsSystem.h"
#include "Rendering/Effects.h"
#include <ModelManager.h>
#include <DirectXMath.h>

JPH_SUPPRESS_WARNINGS

void GameObject::SetPosition(Vec3 position)
{
    m_transform.SetPosition(position.GetX(), position.GetY(), position.GetZ());

    // Once the rigidbody exists, SyncPhysicsToRender() overwrites m_transform from it every
    // frame, so a plain transform write here wouldn't stick -- teleport the physics body too.
    if (m_rigidbody.IsValid())
        m_rigidbody.SetPositionAndRotation(position, m_rigidbody.GetRotation());
}

void GameObject::SetRotation(const DirectX::XMFLOAT4 &rotation)
{
    m_transform.SetRotation(rotation);

    if (m_rigidbody.IsValid())
        m_rigidbody.SetPositionAndRotation(m_rigidbody.GetPosition(), JPH::Quat(rotation.x, rotation.y, rotation.z, rotation.w));
}

void GameObject::Init(JPH::Vec3 halfExtents, Rigidbody::Type type, JPH::Vec3 colliderOffset, float mass)
{
    DirectX::XMFLOAT3 pos = m_transform.GetPosition();
    m_rigidbody.Init(PhysicsSystem::Get().GetBodyInterface(),
                     halfExtents,
                     JPH::Vec3(pos.x, pos.y, pos.z),
                     type,
                     colliderOffset,
                     mass);

    m_colliderOffset = ToXMFLOAT3(colliderOffset);
}

void GameObject::Destroy()
{
    if (m_rigidbody.IsValid())
        m_rigidbody.Destroy(PhysicsSystem::Get().GetBodyInterface());
}

void GameObject::Draw(ID3D11DeviceContext *context, IEffect &effect)
{
    m_render.Draw(context, effect);
    for (auto &sub : m_subRenders)
        sub.Draw(context, effect);
}

void GameObject::SyncPhysicsToRender()
{
    if (!m_rigidbody.IsValid())
        return;

    JPH::Vec3 pos = m_rigidbody.GetPosition();
    JPH::Quat rot = m_rigidbody.GetRotation();

    m_transform.SetPosition(pos.GetX(), pos.GetY(), pos.GetZ());
    m_transform.SetRotation(DirectX::XMFLOAT4(
        rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW()));

    // Model is drawn at an offset from the true transform (set via SetRenderOffset),
    // rotated so the offset stays fixed relative to the object's facing direction.
    using namespace DirectX;
    XMVECTOR offsetWorld = XMVector3Rotate(XMLoadFloat3(&m_renderOffset), m_transform.GetRotationQuatXM());
    XMFLOAT3 renderPos;
    XMStoreFloat3(&renderPos, XMVectorAdd(m_transform.GetPositionXM(), offsetWorld));

    m_render.GetTransform().SetPosition(renderPos);
    m_render.GetTransform().SetRotation(m_transform.GetRotationQuat());
}
