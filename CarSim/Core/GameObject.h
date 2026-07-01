#pragma once

#include <RenderObject.h>
#include <ModelManager.h>
#include <d3d11.h>
#include <string>
#include <vector>
#include "Rigidbody.h"

class GameObject
{
public:
    virtual ~GameObject() = default;

    void Init(JPH::Vec3 halfExtents, Rigidbody::Type type = Rigidbody::Type::Dynamic);

    virtual void Update(float dt) { }
    void UpdateRender() { SyncPhysicsToRender(); }

    void Draw(ID3D11DeviceContext* context, IEffect& effect);

    void SetDrawCollider(bool draw) { m_drawCollider = draw; }
    void SetName(std::string name)        { m_name = std::move(name); }
    const std::string& GetName() const    { return m_name; }

    RenderObject& GetRender()    { return m_render; }
    Rigidbody&    GetRigidbody() { return m_rigidbody; }

    RenderObject& AddSubRender() { return m_subRenders.emplace_back(); }

protected:
    void SyncPhysicsToRender();

    RenderObject              m_render;
    std::vector<RenderObject> m_subRenders;
    RenderObject              m_debugBox;
    Rigidbody                 m_rigidbody;
    std::string               m_name;
    bool                      m_drawCollider = false;
};
