//***************************************************************************************
// GameObject.h by X_Jun(MKXJun) (C) 2018-2022 All Rights Reserved.
// Licensed under the MIT License.
//
// Simple game object.
//***************************************************************************************

#pragma once

#ifndef GAME_OBJECT_H
#define GAME_OBJECT_H

#include "Geometry.h"
#include "Material.h"
#include "MeshData.h"
#include "Transform.h"
#include "IEffect.h"

struct Model;

class RenderObject
{
public:
    template <class T>
    using ComPtr = Microsoft::WRL::ComPtr<T>;


    RenderObject() = default;
    ~RenderObject() = default;

    RenderObject(const RenderObject&) = default;
    RenderObject& operator=(const RenderObject&) = default;

    RenderObject(RenderObject&&) = default;
    RenderObject& operator=(RenderObject&&) = default;

    // Get the object transform
    Transform& GetTransform();
    // Get the object transform (const)
    const Transform& GetTransform() const;

    //
    // Intersection tests
    //
    void FrustumCulling(const DirectX::BoundingFrustum& frustumInWorld);
    void CubeCulling(const DirectX::BoundingOrientedBox& obbInWorld);
    void CubeCulling(const DirectX::BoundingBox& aabbInWorld);
    bool InFrustum() const { return m_InFrustum; }

    //
    // Model
    //
    void SetModel(const Model* pModel);
    const Model* GetModel() const;

    DirectX::BoundingBox GetLocalBoundingBox() const;
    DirectX::BoundingBox GetLocalBoundingBox(size_t idx) const;
    DirectX::BoundingBox GetBoundingBox() const;
    DirectX::BoundingBox GetBoundingBox(size_t idx) const;
    DirectX::BoundingOrientedBox GetBoundingOrientedBox() const;
    DirectX::BoundingOrientedBox GetBoundingOrientedBox(size_t idx) const;
    //
    // Drawing
    //

    void SetVisible(bool visible) {
        m_InFrustum = visible;
        m_SubModelInFrustum.assign(m_SubModelInFrustum.size(), true);
    }

    // Draw the object
    void Draw(ID3D11DeviceContext* deviceContext, IEffect& effect);

protected:
    const Model* m_pModel = nullptr;
    std::vector<bool> m_SubModelInFrustum;
    Transform m_Transform = {};
    bool m_InFrustum = true;
};




#endif
