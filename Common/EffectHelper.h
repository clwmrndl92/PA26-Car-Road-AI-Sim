//***************************************************************************************
// EffectsHelper.h by X_Jun(MKXJun) (C) 2018-2022 All Rights Reserved.
// Licensed under the MIT License.
//
// Define some utility effect helper classes
//*************************************************************************************** 

#pragma once

#ifndef EFFECT_HELPER_H
#define EFFECT_HELPER_H

#include "WinMin.h"
#include <string_view>
#include <memory>
#include <wrl/client.h>
#include <d3dcompiler.h>
#include "Property.h"

struct ID3D11Device;
struct ID3D11DeviceContext;
struct ID3D11GeometryShader;
struct ID3D11ShaderResourceView;
struct ID3D11UnorderedAccessView;
struct ID3D11SamplerState;
struct ID3D11RasterizerState;
struct ID3D11DepthStencilState;
struct ID3D11BlendState;
//
// EffectHelper
//

// Render pass description
// Set shaders by specifying the names provided when shaders were added
struct EffectPassDesc
{
    std::string_view nameVS;
    std::string_view nameDS;
    std::string_view nameHS;
    std::string_view nameGS;
    std::string_view namePS;
    std::string_view nameCS;
};

// Constant buffer variable
// Non-COM component
struct IEffectConstantBufferVariable
{
    // Set an unsigned integer; also usable for bool
    virtual void SetUInt(uint32_t val) = 0;
    // Set a signed integer
    virtual void SetSInt(int val) = 0;
    // Set a float
    virtual void SetFloat(float val) = 0;

    // Set an unsigned integer vector with 1 to 4 components
    // Also works when the shader variable type is bool
    // Reads the first N components of data based on numComponents
    virtual void SetUIntVector(uint32_t numComponents, const uint32_t data[4]) = 0;

    // Set a signed integer vector with 1 to 4 components
    // Reads the first N components of data based on numComponents
    virtual void SetSIntVector(uint32_t numComponents, const int data[4]) = 0;

    // Set a float vector with 1 to 4 components
    // Reads the first N components of data based on numComponents
    virtual void SetFloatVector(uint32_t numComponents, const float data[4]) = 0;

    // Set an unsigned integer matrix; rows and columns can be 1–4
    // Input data must be unpadded; for example a 3x3 matrix can be passed as UINT[3][3]
    virtual void SetUIntMatrix(uint32_t rows, uint32_t cols, const uint32_t* noPadData) = 0;

    // Set a signed integer matrix; rows and columns can be 1–4
    // Input data must be unpadded; for example a 3x3 matrix can be passed as INT[3][3]
    virtual void SetSIntMatrix(uint32_t rows, uint32_t cols, const int* noPadData) = 0;

    // Set a float matrix; rows and columns can be 1–4
    // Input data must be unpadded; for example a 3x3 matrix can be passed as FLOAT[3][3]
    virtual void SetFloatMatrix(uint32_t rows, uint32_t cols, const float* noPadData) = 0;

    // Set arbitrary data; allows specifying a byte range
    virtual void SetRaw(const void* data, uint32_t byteOffset = 0, uint32_t byteCount = 0xFFFFFFFF) = 0;

    // Set a property
    virtual void Set(const Property& prop) = 0;

    // Get the most recently set value; allows specifying a byte range to read
    virtual HRESULT GetRaw(void* pOutput, uint32_t byteOffset = 0, uint32_t byteCount = 0xFFFFFFFF) = 0;

    virtual ~IEffectConstantBufferVariable() {}
};

// Render pass
// Non-COM component
class EffectHelper;
struct IEffectPass
{
    // Set rasterizer state
    virtual void SetRasterizerState(ID3D11RasterizerState* pRS) = 0;
    // Set blend state
    virtual void SetBlendState(ID3D11BlendState* pBS, const float blendFactor[4], uint32_t sampleMask) = 0;
    // Set depth stencil state
    virtual void SetDepthStencilState(ID3D11DepthStencilState* pDSS, uint32_t stencilValue) = 0;

    // Get a vertex shader uniform parameter for setting values
    virtual std::shared_ptr<IEffectConstantBufferVariable> VSGetParamByName(std::string_view paramName) = 0;
    // Get a domain shader uniform parameter for setting values
    virtual std::shared_ptr<IEffectConstantBufferVariable> DSGetParamByName(std::string_view paramName) = 0;
    // Get a hull shader uniform parameter for setting values
    virtual std::shared_ptr<IEffectConstantBufferVariable> HSGetParamByName(std::string_view paramName) = 0;
    // Get a geometry shader uniform parameter for setting values
    virtual std::shared_ptr<IEffectConstantBufferVariable> GSGetParamByName(std::string_view paramName) = 0;
    // Get a pixel shader uniform parameter for setting values
    virtual std::shared_ptr<IEffectConstantBufferVariable> PSGetParamByName(std::string_view paramName) = 0;
    // Get a compute shader uniform parameter for setting values
    virtual std::shared_ptr<IEffectConstantBufferVariable> CSGetParamByName(std::string_view paramName) = 0;
    // Get the owning EffectHelper
    virtual EffectHelper* GetEffectHelper() = 0;
    // Get the effect pass name
    virtual const std::string& GetPassName() = 0;

    // Apply shaders, constant buffers (including function parameters), samplers, shader resources, and read-write resources to the pipeline
    virtual void Apply(ID3D11DeviceContext* deviceContext) = 0;

    // Dispatch the compute shader
    // Takes thread counts; internally calculates the appropriate thread group counts based on the shader's thread group dimensions
    virtual void Dispatch(ID3D11DeviceContext* deviceContext, uint32_t threadX = 1, uint32_t threadY = 1, uint32_t threadZ = 1) = 0;

    virtual ~IEffectPass() {};
};

// Effect helper
// Manages shaders, samplers, shader resources, constant buffers, shader parameters, read-write resources, and render states
class EffectHelper
{
public:

    EffectHelper();
    ~EffectHelper();
    // Copy is not allowed, move is allowed
    EffectHelper(const EffectHelper&) = delete;
    EffectHelper& operator=(const EffectHelper&) = delete;
    EffectHelper(EffectHelper&&) = default;
    EffectHelper& operator=(EffectHelper&&) = default;

    // Set the cache directory for compiled shader files
    // If set to "", caching is disabled
    // If forceWrite is true, the compiled shader will always be overwritten on each run
    // By default, compiled shaders are not cached
    // forceWrite should be enabled while shader modifications are in progress
    void SetBinaryCacheDirectory(std::wstring_view cacheDir, bool forceWrite = false);

    // Compile a shader or read shader bytecode, in the following order:
    // 1. If the shader bytecode cache path is set and force-write is off, try reading ${cacheDir}/${shaderName}.cso first
    // 2. Otherwise read filename. If it is already bytecode, add it directly
    // 3. If filename is HLSL source, compile and add it. If caching is enabled, the bytecode is saved to ${cacheDir}/${shaderName}.cso
    // Notes:
    // 1. Different shaders sharing a constant buffer slot must have identical definitions
    // 2. Different shaders sharing global variables must have identical definitions
    // 3. Different shaders sharing a sampler, shader resource, or read-write resource slot must have identical definitions; otherwise set by slot only
    HRESULT CreateShaderFromFile(std::string_view shaderName, std::wstring_view filename, ID3D11Device* device,
        LPCSTR entryPoint = nullptr, LPCSTR shaderModel = nullptr, const D3D_SHADER_MACRO* pDefines = nullptr, ID3DBlob** ppShaderByteCode = nullptr);

    // Compile shader only
    static HRESULT CompileShaderFromFile(std::wstring_view filename, LPCSTR entryPoint, LPCSTR shaderModel, ID3DBlob** ppShaderByteCode, ID3DBlob** ppErrorBlob = nullptr,
        const D3D_SHADER_MACRO* pDefines = nullptr, ID3DInclude* pInclude = D3D_COMPILE_STANDARD_FILE_INCLUDE);

    // Add compiled shader binary and assign it an identifier name
    // This function does not save the shader binary to a file
    // Notes:
    // 1. Different shaders sharing a constant buffer register must have identical definitions
    // 2. Different shaders sharing global variables must have identical definitions
    // 3. Different shaders sharing a sampler, shader resource, or read-write resource slot must have identical definitions
    HRESULT AddShader(std::string_view name, ID3D11Device* device, ID3DBlob* blob);

    // Add a geometry shader with stream output and assign it an identifier name
    // This function does not save the shader binary to a file
    // Notes:
    // 1. Different shaders sharing a constant buffer slot must have identical definitions
    // 2. Different shaders sharing global variables must have identical definitions
    // 3. Different shaders sharing a sampler, shader resource, or read-write resource slot must have identical definitions; otherwise set by slot only
    HRESULT AddGeometryShaderWithStreamOutput(std::string_view name, ID3D11Device* device, ID3D11GeometryShader* gsWithSO, ID3DBlob* blob);

    // Clear all content
    void Clear();

    // Create a render pass
    HRESULT AddEffectPass(std::string_view effectPassName, ID3D11Device* device, const EffectPassDesc* pDesc);
    // Get a specific render pass
    std::shared_ptr<IEffectPass> GetEffectPass(std::string_view effectPassName);

    // Get a constant buffer variable for setting values
    std::shared_ptr<IEffectConstantBufferVariable> GetConstantBufferVariable(std::string_view name);

    // Set sampler state by slot
    void SetSamplerStateBySlot(uint32_t slot, ID3D11SamplerState* samplerState);
    // Set sampler state by name (if multiple names share the same slot, use set-by-slot instead)
    void SetSamplerStateByName(std::string_view name, ID3D11SamplerState* samplerState);
    // Map a sampler state name to its slot (returns -1 if not found)
    int MapSamplerStateSlot(std::string_view name);

    // Set shader resource by slot
    void SetShaderResourceBySlot(uint32_t slot, ID3D11ShaderResourceView* srv);
    // Set shader resource by name (if multiple names share the same slot, use set-by-slot instead)
    void SetShaderResourceByName(std::string_view name, ID3D11ShaderResourceView* srv);
    // Map a shader resource name to its slot (returns -1 if not found)
    int MapShaderResourceSlot(std::string_view name);

    // Set read-write resource by slot
    void SetUnorderedAccessBySlot(uint32_t slot, ID3D11UnorderedAccessView* uav, uint32_t* pInitialCount = nullptr);
    // Set read-write resource by name (if multiple names share the same slot, use set-by-slot instead)
    void SetUnorderedAccessByName(std::string_view name, ID3D11UnorderedAccessView* uav, uint32_t* pInitialCount = nullptr);
    // Map a read-write resource name to its slot (returns -1 if not found)
    int MapUnorderedAccessSlot(std::string_view name);


    // Set the debug object name
    void SetDebugObjectName(std::string name);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};



#endif
