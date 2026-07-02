#include "GameObject.h"
#include "PhysicsSystem.h"
#include "Rendering/Effects.h"
#include <ModelManager.h>
#include <DirectXMath.h>

JPH_SUPPRESS_WARNINGS

void GameObject::Init(JPH::Vec3 halfExtents, Rigidbody::Type type, JPH::Vec3 colliderOffset)
{
    DirectX::XMFLOAT3 pos = m_transform.GetPosition();
    m_rigidbody.Init(PhysicsSystem::Get().GetBodyInterface(),
                     halfExtents,
                     JPH::Vec3(pos.x, pos.y, pos.z),
                     type,
                     colliderOffset);

    m_colliderOffset = ToXMFLOAT3(colliderOffset);

    // Per-object model names: CreateFromGeometry() overwrites any existing
    // model stored under the same name, so a shared name would make every
    // GameObject's debug box end up showing whichever object initialized last.
    float w = halfExtents.GetX() * 2.0f;
    float h = halfExtents.GetY() * 2.0f;
    float d = halfExtents.GetZ() * 2.0f;
    Model* pBox = ModelManager::Get().CreateFromGeometry("__collider__:" + m_name, Geometry::CreateBox(w, h, d));
    pBox->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f));
    pBox->materials[0].Set<float>("$Opacity", 1.0f);
    m_debugBox.SetModel(pBox);

    Model* pMarker = ModelManager::Get().CreateFromGeometry("__origin__:" + m_name, Geometry::CreateSphere(0.1f, 8, 8));
    pMarker->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f));
    pMarker->materials[0].Set<float>("$Opacity", 1.0f);
    m_originMarker.SetModel(pMarker);
}

void GameObject::Draw(ID3D11DeviceContext* context, IEffect& effect)
{
    m_render.Draw(context, effect);
    for (auto& sub : m_subRenders)
        sub.Draw(context, effect);

    if (m_drawCollider && m_debugBox.GetModel())
    {
        using namespace DirectX;
        XMVECTOR colliderOffsetWorld = XMVector3Rotate(XMLoadFloat3(&m_colliderOffset), m_transform.GetRotationQuatXM());
        XMFLOAT3 colliderPos;
        XMStoreFloat3(&colliderPos, XMVectorAdd(m_transform.GetPositionXM(), colliderOffsetWorld));

        m_debugBox.GetTransform().SetPosition(colliderPos);
        m_debugBox.GetTransform().SetRotation(m_transform.GetRotationQuat());

        // Switch pass RS to wireframe so EffectPass::Apply() sets it correctly
        if (auto* pBasic = dynamic_cast<BasicEffect*>(&effect))
        {
            pBasic->SetRenderWireframe();
            m_debugBox.Draw(context, effect);
            pBasic->SetRenderDefault();
        }
    }

    // Origin marker (the object's true transform, e.g. rear axle center) always
    // shows in debug mode, drawn without depth testing so the model never occludes it.
    if (m_drawCollider && m_originMarker.GetModel())
    {
        m_originMarker.GetTransform().SetPosition(m_transform.GetPosition());

        if (auto* pBasic = dynamic_cast<BasicEffect*>(&effect))
        {
            pBasic->SetRenderNoDepthTest();
            m_originMarker.Draw(context, effect);
            pBasic->SetRenderDefault();
        }
    }
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
