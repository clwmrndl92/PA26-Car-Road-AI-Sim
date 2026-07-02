//***************************************************************************************
// Effects.h by X_Jun(MKXJun) (C) 2018-2022 All Rights Reserved.
// Licensed under the MIT License.
//
// Simple effect management framework.
//***************************************************************************************

#ifndef EFFECTS_H
#define EFFECTS_H

#include <IEffect.h>
#include <Material.h>
#include <MeshData.h>
#include <LightHelper.h>

class BasicEffect : public IEffect, public IEffectTransform,
    public IEffectMaterial, public IEffectMeshData
{
public:
    BasicEffect();
    virtual ~BasicEffect() override;

    BasicEffect(BasicEffect&& moveFrom) noexcept;
    BasicEffect& operator=(BasicEffect&& moveFrom) noexcept;

    // Returns the singleton instance
    static BasicEffect& Get();

    // Initializes all required resources
    bool InitAll(ID3D11Device* device);

    //
    // IEffectTransform
    //

    void XM_CALLCONV SetWorldMatrix(DirectX::FXMMATRIX W) override;
    void XM_CALLCONV SetViewMatrix(DirectX::FXMMATRIX V) override;
    void XM_CALLCONV SetProjMatrix(DirectX::FXMMATRIX P) override;

    //
    // IEffectMaterial
    //

    void SetMaterial(const Material& material) override;

    //
    // IEffectMeshData
    //

    MeshDataInput GetInputData(const MeshData& meshData) override;


    //
    // BasicEffect
    //

    // Draw with default render state
    void SetRenderDefault();
    // Draw with wireframe render state (for debug collider visualization)
    void SetRenderWireframe();
    // Draw with depth testing disabled (so debug markers show through other geometry)
    void SetRenderNoDepthTest();
    // Draw with line-list topology (for debug lines)
    void SetRenderLines();

    // Maximum number of lights per type
    static const int maxLights = 5;

    void SetDirLight(uint32_t pos, const DirectionalLight& dirLight);
    void SetPointLight(uint32_t pos, const PointLight& pointLight);
    void SetSpotLight(uint32_t pos, const SpotLight& spotLight);

    void SetEyePos(const DirectX::XMFLOAT3& eyePos);

    // Apply constant buffer and texture resource changes
    void Apply(ID3D11DeviceContext* deviceContext) override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};






#endif
