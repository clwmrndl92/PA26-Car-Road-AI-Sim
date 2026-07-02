#pragma once

#include <RenderObject.h>
#include <ModelManager.h>
#include <Transform.h>
#include <d3d11.h>
#include <string>
#include <vector>
#include "Rigidbody.h"
#include "MathUtil.h"

class GameObject
{
public:
    virtual ~GameObject() = default;

    void Init(JPH::Vec3 halfExtents, Rigidbody::Type type = Rigidbody::Type::Dynamic,
              JPH::Vec3 colliderOffset = JPH::Vec3::sZero());

    virtual void Update(float dt) { }
    void UpdateRender() { SyncPhysicsToRender(); }

    void Draw(ID3D11DeviceContext* context, IEffect& effect);

    void SetDrawCollider(bool draw) { m_drawCollider = draw; }
    void SetName(std::string name)        { m_name = std::move(name); }
    const std::string& GetName() const    { return m_name; }

    // True origin of the object (e.g. physics/steering reference point such as the rear axle center)
    Transform&       GetTransform()       { return m_transform; }
    const Transform& GetTransform() const { return m_transform; }

    // Local-space offset applied only to where the model is drawn, relative to GetTransform()
    void SetRenderOffset(const DirectX::XMFLOAT3& offset) { m_renderOffset = offset; }

    RenderObject& GetRender()    { return m_render; }
    Rigidbody&    GetRigidbody() { return m_rigidbody; }

    RenderObject& AddSubRender() { return m_subRenders.emplace_back(); }

protected:
    void SyncPhysicsToRender();

    Transform                 m_transform;
    DirectX::XMFLOAT3         m_renderOffset   = { 0.0f, 0.0f, 0.0f };
    DirectX::XMFLOAT3         m_colliderOffset = { 0.0f, 0.0f, 0.0f };

    RenderObject              m_render;
    std::vector<RenderObject> m_subRenders;
    RenderObject              m_debugBox;
    RenderObject              m_originMarker;
    Rigidbody                 m_rigidbody;
    std::string               m_name;
    bool                      m_drawCollider = false;
};
