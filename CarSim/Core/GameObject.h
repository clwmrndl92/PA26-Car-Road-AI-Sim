#pragma once

#include <RenderObject.h>
#include <ModelManager.h>
#include <d3d11.h>
#include "Rigidbody.h"

class PhysicsWorld;

class GameObject
{
public:
    virtual ~GameObject() = default;

    void Init(PhysicsWorld& physics, ModelManager& modelManager, JPH::Vec3 halfExtents, Rigidbody::Type type = Rigidbody::Type::Dynamic);

    virtual void Update(float dt) { SyncPhysicsToRender(); }

    void Draw(ID3D11DeviceContext* context, IEffect& effect);

    void SetDrawCollider(bool draw) { m_drawCollider = draw; }

    RenderObject& GetRender()    { return m_render; }
    Rigidbody&    GetRigidbody() { return m_rigidbody; }

protected:
    void SyncPhysicsToRender();

    RenderObject m_render;
    RenderObject m_debugBox;
    Rigidbody    m_rigidbody;
    bool         m_drawCollider = false;
};
