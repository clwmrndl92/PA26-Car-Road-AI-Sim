#include "GameObject.h"
#include "PhysicsSystem.h"
#include "Rendering/Effects.h"
#include <ModelManager.h>
#include <DirectXMath.h>

JPH_SUPPRESS_WARNINGS

void GameObject::Init(JPH::Vec3 halfExtents, Rigidbody::Type type)
{
    DirectX::XMFLOAT3 pos = m_render.GetTransform().GetPosition();
    m_rigidbody.Init(PhysicsSystem::Get().GetBodyInterface(),
                     halfExtents,
                     JPH::Vec3(pos.x, pos.y, pos.z),
                     type);

    float w = halfExtents.GetX() * 2.0f;
    float h = halfExtents.GetY() * 2.0f;
    float d = halfExtents.GetZ() * 2.0f;
    Model* pBox = ModelManager::Get().CreateFromGeometry("__collider__", Geometry::CreateBox(w, h, d));
    pBox->materials[0].Set<DirectX::XMFLOAT4>("$DiffuseColor", DirectX::XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f));
    pBox->materials[0].Set<float>("$Opacity", 1.0f);
    m_debugBox.SetModel(pBox);
}

void GameObject::Draw(ID3D11DeviceContext* context, IEffect& effect)
{
    m_render.Draw(context, effect);
    for (auto& sub : m_subRenders)
        sub.Draw(context, effect);

    if (m_drawCollider && m_debugBox.GetModel())
    {
        m_debugBox.GetTransform().SetPosition(m_render.GetTransform().GetPosition());
        m_debugBox.GetTransform().SetRotation(m_render.GetTransform().GetRotationQuat());

        // Switch pass RS to wireframe so EffectPass::Apply() sets it correctly
        if (auto* pBasic = dynamic_cast<BasicEffect*>(&effect))
        {
            pBasic->SetRenderWireframe();
            m_debugBox.Draw(context, effect);
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

    m_render.GetTransform().SetPosition(pos.GetX(), pos.GetY(), pos.GetZ());
    m_render.GetTransform().SetRotation(DirectX::XMFLOAT4(
        rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW()));
}
