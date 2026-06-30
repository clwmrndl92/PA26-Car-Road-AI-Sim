//***************************************************************************************
// IEffects.h by X_Jun(MKXJun) (C) 2018-2022 All Rights Reserved.
// Licensed under the MIT License.
//
// Effect interface definitions.
//***************************************************************************************

#ifndef IEFFECT_H
#define IEFFECT_H

#include "WinMin.h"
#include <memory>
#include <vector>
#include <d3d11_1.h>
#include <DirectXMath.h>

class Material;
struct MeshData;

// Data from a single MeshData that must be bound to the input assembler stage
// The input layout, strides, offsets, and primitive topology are provided by the Effect Pass
// The remaining data comes from MeshData
struct MeshDataInput
{
    ID3D11InputLayout* pInputLayout = nullptr;
    D3D11_PRIMITIVE_TOPOLOGY topology = D3D11_PRIMITIVE_TOPOLOGY_UNDEFINED;
    std::vector<ID3D11Buffer*> pVertexBuffers;
    ID3D11Buffer* pIndexBuffer = nullptr;
    std::vector<uint32_t> strides;
    std::vector<uint32_t> offsets;
    uint32_t indexCount = 0;
};

class IEffect
{
public:
    IEffect() = default;
    virtual ~IEffect() = default;
    // Copy is not allowed, move is allowed
    IEffect(const IEffect&) = delete;
    IEffect& operator=(const IEffect&) = delete;
    IEffect(IEffect&&) = default;
    IEffect& operator=(IEffect&&) = default;

    // Update and bind constant buffers
    virtual void Apply(ID3D11DeviceContext * deviceContext) = 0;
};

class IEffectTransform
{
public:
    virtual void XM_CALLCONV SetWorldMatrix(DirectX::FXMMATRIX W) = 0;
    virtual void XM_CALLCONV SetViewMatrix(DirectX::FXMMATRIX V) = 0;
    virtual void XM_CALLCONV SetProjMatrix(DirectX::FXMMATRIX P) = 0;
};

class IEffectMaterial
{
public:
    virtual void SetMaterial(const Material& material) = 0;
};

class IEffectMeshData
{
public:
    virtual MeshDataInput GetInputData(const MeshData& meshData) = 0;
};


#endif
