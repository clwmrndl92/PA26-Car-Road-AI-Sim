
#pragma once

#ifndef D3DAPP_H
#define D3DAPP_H

#include <wrl/client.h>
#include <string>
#include <string_view>
#include "WinMin.h"
#include <d3d11_1.h>
#include <DirectXMath.h>
#include "CpuTimer.h"
#include "GpuTimer.h"

#include <imgui.h>
#include <imgui_impl_dx11.h>
#include <imgui_impl_win32.h>

class D3DApp
{
public:
    D3DApp(HINSTANCE hInstance, const std::wstring& windowName, int initWidth, int initHeight);
    virtual ~D3DApp();

    HINSTANCE AppInst()const;       // Get the application instance handle
    HWND      MainWnd()const;       // Get the main window handle
    float     AspectRatio()const;   // Get the screen aspect ratio

    int Run();                      // Run the program, execute the message event loop

    // Framework methods. Derived classes must override these to implement specific application requirements
    virtual bool Init();                      // This base class method initializes the window, Direct2D, and Direct3D
    virtual void OnResize();                  // This base class method is called when the window size changes
    virtual void UpdateScene(float dt) = 0;   // Subclasses must implement this method to update each frame
    virtual void DrawScene() = 0;             // Subclasses must implement this method to render each frame
    virtual LRESULT MsgProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam); // Window message callback function
    
protected:
    bool InitMainWindow();      // Window initialization
    bool InitDirect3D();        // Direct3D initialization
    bool InitImGui();           // ImGui initialization

    void CalculateFrameStats(); // Calculate frames per second and display in the window title
    ID3D11RenderTargetView* GetBackBufferRTV() { return m_pRenderTargetViews[m_FrameCount % m_BackBufferCount].Get(); }

protected:
    // Use template alias (C++11) to simplify type names
    template <class T>
    using ComPtr = Microsoft::WRL::ComPtr<T>;

    HINSTANCE m_hAppInst;        // Application instance handle
    HWND      m_hMainWnd;        // Main window handle
    bool      m_AppPaused;       // Whether the application is paused
    bool      m_Minimized;       // Whether the application is minimized
    bool      m_Maximized;       // Whether the application is maximized
    bool      m_Resizing;        // Whether the window size is changing

    bool m_IsDxgiFlipModel = false; // Whether the DXGI flip model is used
    UINT m_BackBufferCount = 0;		// Number of back buffers
    UINT m_FrameCount = 0;          // Current frame index
    ComPtr<ID3D11RenderTargetView> m_pRenderTargetViews[2];     // Render target views for all back buffers

    CpuTimer m_Timer;            // Timer

    // Direct3D 11
    ComPtr<ID3D11Device> m_pd3dDevice;                          // D3D11 device
    ComPtr<ID3D11DeviceContext> m_pd3dImmediateContext;	        // D3D11 device context
    ComPtr<IDXGISwapChain> m_pSwapChain;                        // D3D11 swap chain
    // Direct3D 11.1
    ComPtr<ID3D11Device1> m_pd3dDevice1;			    		// D3D11.1 device
    ComPtr<ID3D11DeviceContext1> m_pd3dImmediateContext1;		// D3D11.1 device context
    ComPtr<IDXGISwapChain1> m_pSwapChain1;						// D3D11.1 swap chain

    // Derived classes should set these custom initial parameters in their constructors
    std::wstring m_MainWndCaption;                              // Main window title
    int m_ClientWidth;                                          // Viewport width
    int m_ClientHeight;                                         // Viewport height
};

#endif // D3DAPP_H