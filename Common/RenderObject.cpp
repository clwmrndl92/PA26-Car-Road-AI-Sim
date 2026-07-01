#include "XUtil.h"
#include "RenderObject.h"
#include "DXTrace.h"
#include "ModelManager.h"

using namespace DirectX;

struct InstancedData
{
    XMMATRIX world;
    XMMATRIX worldInvTranspose;
};

Transform& RenderObject::GetTransform()
{
    return m_Transform;
}

const Transform& RenderObject::GetTransform() const
{
    return m_Transform;
}

void RenderObject::FrustumCulling(const BoundingFrustum& frustumInWorld)
{
    size_t sz = m_pModel->meshdatas.size();
    m_InFrustum = false;
    m_SubModelInFrustum.resize(sz);
    for (size_t i = 0; i < sz; ++i)
    {
        BoundingOrientedBox box;
        BoundingOrientedBox::CreateFromBoundingBox(box, m_pModel->meshdatas[i].m_BoundingBox);
        box.Transform(box, m_Transform.GetLocalToWorldMatrixXM());
        m_SubModelInFrustum[i] = frustumInWorld.Intersects(box);
        m_InFrustum = m_InFrustum || m_SubModelInFrustum[i];
    }
}

void RenderObject::CubeCulling(const DirectX::BoundingOrientedBox& obbInWorld)
{
    size_t sz = m_pModel->meshdatas.size();
    m_InFrustum = false;
    m_SubModelInFrustum.resize(sz);
    for (size_t i = 0; i < sz; ++i)
    {
        BoundingOrientedBox box;
        BoundingOrientedBox::CreateFromBoundingBox(box, m_pModel->meshdatas[i].m_BoundingBox);
        box.Transform(box, m_Transform.GetLocalToWorldMatrixXM());
        m_SubModelInFrustum[i] = obbInWorld.Intersects(box);
        m_InFrustum = m_InFrustum || m_SubModelInFrustum[i];
    }
}

void RenderObject::CubeCulling(const DirectX::BoundingBox& aabbInWorld)
{
    size_t sz = m_pModel->meshdatas.size();
    m_InFrustum = false;
    m_SubModelInFrustum.resize(sz);
    for (size_t i = 0; i < sz; ++i)
    {
        BoundingBox box;
        m_pModel->meshdatas[i].m_BoundingBox.Transform(box, m_Transform.GetLocalToWorldMatrixXM());
        m_SubModelInFrustum[i] = aabbInWorld.Intersects(box);
        m_InFrustum = m_InFrustum || m_SubModelInFrustum[i];
    }
}

void RenderObject::SetModel(const Model* pModel)
{
    m_pModel = pModel;
}

const Model* RenderObject::GetModel() const
{
    return m_pModel;
}

BoundingBox RenderObject::GetLocalBoundingBox() const
{
    return m_pModel ? m_pModel->boundingbox : DirectX::BoundingBox(DirectX::XMFLOAT3(), DirectX::XMFLOAT3());
}

BoundingBox RenderObject::GetLocalBoundingBox(size_t idx) const
{
    if (!m_pModel || m_pModel->meshdatas.size() >= idx)
        return DirectX::BoundingBox(DirectX::XMFLOAT3(), DirectX::XMFLOAT3());
    return m_pModel->meshdatas[idx].m_BoundingBox;
}

BoundingBox RenderObject::GetBoundingBox() const
{
    if (!m_pModel)
        return DirectX::BoundingBox(DirectX::XMFLOAT3(), DirectX::XMFLOAT3());
    BoundingBox box = m_pModel->boundingbox;
    box.Transform(box, m_Transform.GetLocalToWorldMatrixXM());
    return box;
}

BoundingBox RenderObject::GetBoundingBox(size_t idx) const
{
    if (!m_pModel || m_pModel->meshdatas.size() >= idx)
        return DirectX::BoundingBox(DirectX::XMFLOAT3(), DirectX::XMFLOAT3());
    BoundingBox box = m_pModel->meshdatas[idx].m_BoundingBox;
    box.Transform(box, m_Transform.GetLocalToWorldMatrixXM());
    return box;
}

BoundingOrientedBox RenderObject::GetBoundingOrientedBox() const
{
    if (!m_pModel)
        return DirectX::BoundingOrientedBox(DirectX::XMFLOAT3(), DirectX::XMFLOAT3(), DirectX::XMFLOAT4(0.0f, 0.0f, 0.0f, 1.0f));
    BoundingOrientedBox obb;
    BoundingOrientedBox::CreateFromBoundingBox(obb, m_pModel->boundingbox);
    obb.Transform(obb, m_Transform.GetLocalToWorldMatrixXM());
    return obb;
}
BoundingOrientedBox RenderObject::GetBoundingOrientedBox(size_t idx) const
{
    if (!m_pModel || m_pModel->meshdatas.size() >= idx)
        return DirectX::BoundingOrientedBox(DirectX::XMFLOAT3(), DirectX::XMFLOAT3(), DirectX::XMFLOAT4(0.0f, 0.0f, 0.0f, 1.0f));
    BoundingOrientedBox obb;
    BoundingOrientedBox::CreateFromBoundingBox(obb, m_pModel->meshdatas[idx].m_BoundingBox);
    obb.Transform(obb, m_Transform.GetLocalToWorldMatrixXM());
    return obb;
}

void RenderObject::Draw(ID3D11DeviceContext * deviceContext, IEffect& effect)
{
    if (!m_InFrustum || !deviceContext)
        return;
    size_t sz = m_pModel->meshdatas.size();
    size_t fsz = m_SubModelInFrustum.size();
    for (size_t i = 0; i < sz; ++i)
    {
        if (i < fsz && !m_SubModelInFrustum[i])
            continue;

        IEffectMeshData* pEffectMeshData = dynamic_cast<IEffectMeshData*>(&effect);
        if (!pEffectMeshData)
            continue;

        IEffectMaterial* pEffectMaterial = dynamic_cast<IEffectMaterial*>(&effect);
        if (pEffectMaterial)
            pEffectMaterial->SetMaterial(m_pModel->materials[m_pModel->meshdatas[i].m_MaterialIndex]);

        IEffectTransform* pEffectTransform = dynamic_cast<IEffectTransform*>(&effect);
        if (pEffectTransform)
            pEffectTransform->SetWorldMatrix(m_Transform.GetLocalToWorldMatrixXM());

        effect.Apply(deviceContext);

        MeshDataInput input = pEffectMeshData->GetInputData(m_pModel->meshdatas[i]);
        {
            deviceContext->IASetInputLayout(input.pInputLayout);
            deviceContext->IASetPrimitiveTopology(input.topology);
            deviceContext->IASetVertexBuffers(0, (uint32_t)input.pVertexBuffers.size(), 
                input.pVertexBuffers.data(), input.strides.data(), input.offsets.data());
            deviceContext->IASetIndexBuffer(input.pIndexBuffer, input.indexCount > 65535 ? DXGI_FORMAT_R32_UINT : DXGI_FORMAT_R16_UINT, 0);

            deviceContext->DrawIndexed(input.indexCount, 0, 0);
        }
        
    }
}


