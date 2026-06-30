//***************************************************************************************
// RenderStates.h by X_Jun(MKXJun) (C) 2018-2022 All Rights Reserved.
// Licensed under the MIT License.
//
// Provide some render states.
//***************************************************************************************

#pragma once

#ifndef RENDER_STATES_H
#define RENDER_STATES_H

#include "WinMin.h"
#include <wrl/client.h>
#include <d3d11_1.h>


class RenderStates
{
public:
    template <class T>
    using ComPtr = Microsoft::WRL::ComPtr<T>;

    static bool IsInit();

    static void InitAll(ID3D11Device * device);
    // ComPtr handles release automatically

public:
    static ComPtr<ID3D11RasterizerState> RSWireframe;		            // Rasterizer state: wireframe mode
    static ComPtr<ID3D11RasterizerState> RSNoCull;			            // Rasterizer state: no back-face culling
    static ComPtr<ID3D11RasterizerState> RSCullClockWise;	            // Rasterizer state: clockwise culling
    static ComPtr<ID3D11RasterizerState> RSShadow;						// Rasterizer state: depth bias mode

    static ComPtr<ID3D11SamplerState> SSPointClamp;						// Sampler state: point filtering with Clamp
    static ComPtr<ID3D11SamplerState> SSLinearWrap;			            // Sampler state: linear filtering with Wrap
    static ComPtr<ID3D11SamplerState> SSLinearClamp;					// Sampler state: linear filtering with Clamp
    static ComPtr<ID3D11SamplerState> SSAnistropicWrap16x;		        // Sampler state: 16x anisotropic filtering with Wrap
    static ComPtr<ID3D11SamplerState> SSAnistropicClamp2x;		        // Sampler state: 2x anisotropic filtering with Clamp
    static ComPtr<ID3D11SamplerState> SSAnistropicClamp4x;		        // Sampler state: 4x anisotropic filtering with Clamp
    static ComPtr<ID3D11SamplerState> SSAnistropicClamp8x;		        // Sampler state: 8x anisotropic filtering with Clamp
    static ComPtr<ID3D11SamplerState> SSAnistropicClamp16x;		        // Sampler state: 16x anisotropic filtering with Clamp
    static ComPtr<ID3D11SamplerState> SSShadowPCF;						// Sampler state: depth comparison with Border

    static ComPtr<ID3D11BlendState> BSTransparent;		                // Blend state: alpha blending
    static ComPtr<ID3D11BlendState> BSAlphaToCoverage;	                // Blend state: alpha-to-coverage
    static ComPtr<ID3D11BlendState> BSAdditive;			                // Blend state: additive blending
    static ComPtr<ID3D11BlendState> BSAlphaWeightedAdditive;            // Blend state: alpha-weighted additive blending


    static ComPtr<ID3D11DepthStencilState> DSSEqual;					// Depth/stencil state: only draw pixels with equal depth
    static ComPtr<ID3D11DepthStencilState> DSSLessEqual;                // Depth/stencil state: for traditional skybox rendering
    static ComPtr<ID3D11DepthStencilState> DSSGreaterEqual;             // Depth/stencil state: for reversed-Z rendering
    static ComPtr<ID3D11DepthStencilState> DSSNoDepthWrite;             // Depth/stencil state: test only, do not write depth
    static ComPtr<ID3D11DepthStencilState> DSSNoDepthTest;              // Depth/stencil state: disable depth testing
    static ComPtr<ID3D11DepthStencilState> DSSWriteStencil;		        // Depth/stencil state: no depth test, write stencil value
    static ComPtr<ID3D11DepthStencilState> DSSEqualStencil;	            // Depth/stencil state: reversed-Z, test stencil value
};



#endif
