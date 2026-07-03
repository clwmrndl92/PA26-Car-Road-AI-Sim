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
              JPH::Vec3 colliderOffset = JPH::Vec3::sZero(), float mass = 1.0f);

    virtual void Update(float dt) {}
    void UpdateRender() { SyncPhysicsToRender(); }

    virtual void Draw(ID3D11DeviceContext *context, IEffect &effect);

    void SetDrawCollider(bool draw) { m_drawCollider = draw; }
    void SetName(std::string name) { m_name = std::move(name); }
    const std::string &GetName() const { return m_name; }

    // True origin of the object (e.g. physics/steering reference point such as the rear axle
    // center). Car overrides both so external callers see the front axle instead.
    virtual DirectX::XMFLOAT3 GetPosition() const { return m_transform.GetPosition(); }
    virtual void SetPosition(float x, float y, float z);

    // Local-space offset applied only to where the model is drawn, relative to GetPosition()
    void SetRenderOffset(const DirectX::XMFLOAT3 &offset) { m_renderOffset = offset; }

    void SetModel(const Model *pModel) { m_render.SetModel(pModel); }
    DirectX::BoundingBox GetBoundingBox() const { return m_render.GetBoundingBox(); }

    RenderObject &AddSubRender() { return m_subRenders.emplace_back(); }

protected:
    void SyncPhysicsToRender();

    Transform m_transform;
    DirectX::XMFLOAT3 m_renderOffset = {0.0f, 0.0f, 0.0f};
    DirectX::XMFLOAT3 m_colliderOffset = {0.0f, 0.0f, 0.0f};

    RenderObject m_render;
    std::vector<RenderObject> m_subRenders;
    RenderObject m_debugBox;
    RenderObject m_originMarker;
    Rigidbody m_rigidbody;
    std::string m_name;
    bool m_drawCollider = false;
};
