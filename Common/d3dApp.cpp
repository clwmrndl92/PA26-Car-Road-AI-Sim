#include "d3dApp.h"
#include "DXTrace.h"
#include "XUtil.h"
#include <sstream>


#pragma warning(disable : 6031)

extern "C" {
// On hardware with multiple GPUs, prefer to run on the NVIDIA or AMD GPU
// Must be used in the .exe
__declspec(dllexport) uint32_t NvOptimusEnablement = 0x00000001;
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 0x00000001;
}

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

namespace {
// This is just used to forward Windows messages from a global window
// procedure to our member function window procedure because we cannot
// assign a member function to WNDCLASS::lpfnWndProc.
D3DApp* g_pd3dApp = nullptr;
}

LRESULT CALLBACK
MainWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    // Forward hwnd on because we can get messages (e.g., WM_CREATE)
    // before CreateWindow returns, and thus before m_hMainWnd is valid.
    return g_pd3dApp->MsgProc(hwnd, msg, wParam, lParam);
}

D3DApp::D3DApp(HINSTANCE hInstance, const std::wstring& windowName, int initWidth, int initHeight)
    : m_hAppInst(hInstance)
    , m_MainWndCaption(windowName)
    , m_ClientWidth(initWidth)
    , m_ClientHeight(initHeight)
    , m_hMainWnd(nullptr)
    , m_AppPaused(false)
    , m_Minimized(false)
    , m_Maximized(false)
    , m_Resizing(false)
{
    // Let a global pointer hold this class instance so we can call
    // the internal callback function from the Windows message procedure
    g_pd3dApp = this;
}

D3DApp::~D3DApp()
{
    // Restore all default settings
    if (m_pd3dImmediateContext)
        m_pd3dImmediateContext->ClearState();

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
}

HINSTANCE D3DApp::AppInst() const
{
    return m_hAppInst;
}

HWND D3DApp::MainWnd() const
{
    return m_hMainWnd;
}

float D3DApp::AspectRatio() const
{
    return static_cast<float>(m_ClientWidth) / m_ClientHeight;
}

int D3DApp::Run()
{
    MSG msg = { 0 };

    m_Timer.Reset();

    while (msg.message != WM_QUIT) {
        if (PeekMessage(&msg, 0, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        } else {
            m_Timer.Tick();

            if (!m_AppPaused) {
                CalculateFrameStats();
                ImGui_ImplDX11_NewFrame();
                ImGui_ImplWin32_NewFrame();
                ImGui::NewFrame();
                UpdateScene(m_Timer.DeltaTime());
                DrawScene();
                m_FrameCount++;
            } else {
                Sleep(100);
            }
        }
    }

    return (int)msg.wParam;
}

bool D3DApp::Init()
{

    if (!InitMainWindow())
        return false;

    if (!InitDirect3D())
        return false;

    if (!InitImGui())
        return false;

    return true;
}

void D3DApp::OnResize()
{
    assert(m_pd3dImmediateContext);
    assert(m_pd3dDevice);
    assert(m_pSwapChain);

    if (m_pd3dDevice1 != nullptr) {
        assert(m_pd3dImmediateContext1);
        assert(m_pd3dDevice1);
        assert(m_pSwapChain1);
    }

    // Reset the swap chain and recreate render target views
    for (UINT i = 0; i < m_BackBufferCount; ++i)
        m_pRenderTargetViews[i].Reset();
    HR(m_pSwapChain->ResizeBuffers(m_BackBufferCount, m_ClientWidth, m_ClientHeight, DXGI_FORMAT_R8G8B8A8_UNORM,
        m_IsDxgiFlipModel ? DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING : 0));
    m_FrameCount = 0;
}

LRESULT D3DApp::MsgProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (ImGui_ImplWin32_WndProcHandler(m_hMainWnd, msg, wParam, lParam))
        return true;

    switch (msg) {
        // WM_ACTIVATE is sent when the window is activated or deactivated. Losing focus
        // (e.g. alt-tab) intentionally does NOT pause the sim -- only minimize/resize do,
        // since those genuinely can't render (see WM_SIZE / WM_ENTERSIZEMOVE below).
    case WM_ACTIVATE:
        return 0;

        // WM_SIZE is sent when the user resizes the window.
    case WM_SIZE:
        // Save the new client area dimensions.
        m_ClientWidth = LOWORD(lParam);
        m_ClientHeight = HIWORD(lParam);
        if (m_pd3dDevice) {
            if (wParam == SIZE_MINIMIZED) {
                m_AppPaused = true;
                m_Minimized = true;
                m_Maximized = false;
            } else if (wParam == SIZE_MAXIMIZED) {
                m_AppPaused = false;
                m_Minimized = false;
                m_Maximized = true;
                OnResize();
            } else if (wParam == SIZE_RESTORED) {

                // Restoring from minimized state?
                if (m_Minimized) {
                    m_AppPaused = false;
                    m_Minimized = false;
                    OnResize();
                }

                // Restoring from maximized state?
                else if (m_Maximized) {
                    m_AppPaused = false;
                    m_Maximized = false;
                    OnResize();
                } else if (m_Resizing) {
                    // If user is dragging the resize bars, we do not resize
                    // the buffers here because as the user continuously
                    // drags the resize bars, a stream of WM_SIZE messages are
                    // sent to the window, and it would be pointless (and slow)
                    // to resize for each WM_SIZE message received from dragging
                    // the resize bars.  So instead, we reset after the user is
                    // done resizing the window and releases the resize bars, which
                    // sends a WM_EXITSIZEMOVE message.
                } else // API call such as SetWindowPos or m_pSwapChain->SetFullscreenState.
                {
                    OnResize();
                }
            }
        }
        return 0;

        // WM_EXITSIZEMOVE is sent when the user grabs the resize bars.
    case WM_ENTERSIZEMOVE:
        m_AppPaused = true;
        m_Resizing = true;
        m_Timer.Stop();
        return 0;

        // WM_EXITSIZEMOVE is sent when the user releases the resize bars.
        // Here we reset everything based on the new window dimensions.
    case WM_EXITSIZEMOVE:
        m_AppPaused = false;
        m_Resizing = false;
        m_Timer.Start();
        OnResize();
        return 0;

        // WM_DESTROY is sent when the window is being destroyed.
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;

        // The WM_MENUCHAR message is sent when a menu is active and the user presses
        // a key that does not correspond to any mnemonic or accelerator key.
    case WM_MENUCHAR:
        // Don't beep when we alt-enter.
        return MAKELRESULT(0, MNC_CLOSE);

        // Catch this message so to prevent the window from becoming too small.
    case WM_GETMINMAXINFO:
        ((MINMAXINFO*)lParam)->ptMinTrackSize.x = 200;
        ((MINMAXINFO*)lParam)->ptMinTrackSize.y = 200;
        return 0;
    }

    return DefWindowProc(hwnd, msg, wParam, lParam);
}

bool D3DApp::InitMainWindow()
{
    WNDCLASS wc;
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = MainWndProc;
    wc.cbClsExtra = 0;
    wc.cbWndExtra = 0;
    wc.hInstance = m_hAppInst;
    wc.hIcon = LoadIcon(0, IDI_APPLICATION);
    wc.hCursor = LoadCursor(0, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)GetStockObject(NULL_BRUSH);
    wc.lpszMenuName = 0;
    wc.lpszClassName = L"D3DWndClassName";

    if (!RegisterClass(&wc)) {
        MessageBox(0, L"RegisterClass Failed.", 0, 0);
        return false;
    }

    // Center the window on screen
    int screenWidth = GetSystemMetrics(SM_CXSCREEN);
    int screenHeight = GetSystemMetrics(SM_CYSCREEN);

    // Compute window rectangle dimensions based on requested client area dimensions.
    RECT R = { 0, 0, m_ClientWidth, m_ClientHeight };
    AdjustWindowRect(&R, WS_OVERLAPPEDWINDOW, false);
    int width = R.right - R.left;
    int height = R.bottom - R.top;

    m_hMainWnd = CreateWindow(L"D3DWndClassName", m_MainWndCaption.c_str(),
        WS_OVERLAPPEDWINDOW, (screenWidth - width) / 2, (screenHeight - height) / 2, width, height, 0, 0, m_hAppInst, 0);

    if (!m_hMainWnd) {
        MessageBox(0, L"CreateWindow Failed.", 0, 0);
        return false;
    }

    ShowWindow(m_hMainWnd, SW_SHOW);
    UpdateWindow(m_hMainWnd);

    return true;
}

bool D3DApp::InitDirect3D()
{
    HRESULT hr = S_OK;

    // Create D3D device and D3D device context
    UINT createDeviceFlags = 0;
#if defined(DEBUG) || defined(_DEBUG)
    createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
    // Driver type array
    D3D_DRIVER_TYPE driverTypes[] = {
        D3D_DRIVER_TYPE_HARDWARE,
        D3D_DRIVER_TYPE_WARP,
        D3D_DRIVER_TYPE_REFERENCE,
    };
    UINT numDriverTypes = ARRAYSIZE(driverTypes);

    // Feature level array
    D3D_FEATURE_LEVEL featureLevels[] = {
        D3D_FEATURE_LEVEL_11_1,
        D3D_FEATURE_LEVEL_11_0,
    };
    UINT numFeatureLevels = ARRAYSIZE(featureLevels);

    D3D_FEATURE_LEVEL featureLevel;
    D3D_DRIVER_TYPE d3dDriverType;
    for (UINT driverTypeIndex = 0; driverTypeIndex < numDriverTypes; driverTypeIndex++) {
        d3dDriverType = driverTypes[driverTypeIndex];
        hr = D3D11CreateDevice(nullptr, d3dDriverType, nullptr, createDeviceFlags, featureLevels, numFeatureLevels,
            D3D11_SDK_VERSION, m_pd3dDevice.GetAddressOf(), &featureLevel, m_pd3dImmediateContext.GetAddressOf());

        if (hr == E_INVALIDARG) {
            // The Direct3D 11.0 API does not recognize D3D_FEATURE_LEVEL_11_1, so we need to try feature level 11.0 and below
            hr = D3D11CreateDevice(nullptr, d3dDriverType, nullptr, createDeviceFlags, &featureLevels[1], numFeatureLevels - 1,
                D3D11_SDK_VERSION, m_pd3dDevice.GetAddressOf(), &featureLevel, m_pd3dImmediateContext.GetAddressOf());
        }

        if (SUCCEEDED(hr))
            break;
    }

    if (FAILED(hr)) {
        MessageBox(0, L"D3D11CreateDevice Failed.", 0, 0);
        return false;
    }

    // Check whether feature level 11.0 or 11.1 is supported
    if (featureLevel != D3D_FEATURE_LEVEL_11_0 && featureLevel != D3D_FEATURE_LEVEL_11_1) {
        MessageBox(0, L"Direct3D Feature Level 11_0 or 11_1 unsupported.", 0, 0);
        return false;
    }

    // Check the MSAA quality level support
    UINT quality;
    m_pd3dDevice->CheckMultisampleQualityLevels(
        DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, 4, &quality);
    assert(quality > 0);

    ComPtr<IDXGIDevice> dxgiDevice = nullptr;
    ComPtr<IDXGIAdapter> dxgiAdapter = nullptr;
    ComPtr<IDXGIFactory1> dxgiFactory1 = nullptr; // D3D11.0 (includes DXGI1.1) interface class
    ComPtr<IDXGIFactory2> dxgiFactory2 = nullptr; // D3D11.1 (includes DXGI1.2) specific interface class

    // To correctly create the DXGI swap chain, we first need to obtain the DXGI factory used to create the D3D device,
    // otherwise an error will occur:
    // "IDXGIFactory::CreateSwapChain: This function is being called with a device from a different IDXGIFactory."
    HR(m_pd3dDevice.As(&dxgiDevice));
    HR(dxgiDevice->GetAdapter(dxgiAdapter.GetAddressOf()));
    HR(dxgiAdapter->GetParent(IID_PPV_ARGS(dxgiFactory1.GetAddressOf())));

    // Check whether this object supports the IDXGIFactory2 interface
    hr = dxgiFactory1.As(&dxgiFactory2);
    // If it does, D3D11.1 is supported
    if (dxgiFactory2 != nullptr) {
        HR(m_pd3dDevice.As(&m_pd3dDevice1));
        HR(m_pd3dImmediateContext.As(&m_pd3dImmediateContext1));
        // Fill in the structures used to describe the swap chain
        DXGI_SWAP_CHAIN_DESC1 sd;
        ZeroMemory(&sd, sizeof(sd));
        sd.Width = m_ClientWidth;
        sd.Height = m_ClientHeight;
        sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        sd.SampleDesc.Count = 1;
        sd.SampleDesc.Quality = 0;
        sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
#if _WIN32_WINNT >= _WIN32_WINNT_WIN10
        m_BackBufferCount = 2;
        sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING;
        m_IsDxgiFlipModel = true;
#else
        m_BackBufferCount = 1;
        sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
        sd.Flags = 0;
#endif
        sd.BufferCount = m_BackBufferCount;

        DXGI_SWAP_CHAIN_FULLSCREEN_DESC fd;
        fd.RefreshRate.Numerator = 60;
        fd.RefreshRate.Denominator = 1;
        fd.Scaling = DXGI_MODE_SCALING_UNSPECIFIED;
        fd.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
        fd.Windowed = TRUE;
        // Create a swap chain for the current window
        HR(dxgiFactory2->CreateSwapChainForHwnd(m_pd3dDevice.Get(), m_hMainWnd, &sd, &fd, nullptr, m_pSwapChain1.GetAddressOf()));
        HR(m_pSwapChain1.As(&m_pSwapChain));
    } else {
        // Fill in DXGI_SWAP_CHAIN_DESC to describe the swap chain
        DXGI_SWAP_CHAIN_DESC sd;
        ZeroMemory(&sd, sizeof(sd));
        sd.BufferDesc.Width = m_ClientWidth;
        sd.BufferDesc.Height = m_ClientHeight;
        sd.BufferDesc.RefreshRate.Numerator = 60;
        sd.BufferDesc.RefreshRate.Denominator = 1;
        sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        sd.BufferDesc.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
        sd.BufferDesc.Scaling = DXGI_MODE_SCALING_UNSPECIFIED;
        sd.SampleDesc.Count = 1;
        sd.SampleDesc.Quality = 0;
        sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        sd.BufferCount = 1;
        sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
        sd.OutputWindow = m_hMainWnd;
        sd.Windowed = TRUE;
        sd.Flags = 0;
        m_BackBufferCount = 1;
        HR(dxgiFactory1->CreateSwapChain(m_pd3dDevice.Get(), &sd, m_pSwapChain.GetAddressOf()));
    }

    // Disable alt+enter fullscreen toggle
    dxgiFactory1->MakeWindowAssociation(m_hMainWnd, DXGI_MWA_NO_ALT_ENTER | DXGI_MWA_NO_WINDOW_CHANGES);

    // Set debug object names
#if (defined(DEBUG) || defined(_DEBUG)) && (GRAPHICS_DEBUGGER_OBJECT_NAME)
    SetDebugObjectName(m_pd3dImmediateContext.Get(), "ImmediateContext");
    SetDebugObjectName(m_pSwapChain.Get(), "SwapChain");
#endif

    // Whenever the window is resized, this OnResize function must be called. Call it now
    // to avoid code duplication
    OnResize();

    return true;
}

bool D3DApp::InitImGui()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable keyboard control
    io.ConfigFlags |= ImGuiConfigFlags_IsSRGB; // Mark that SRGB is used; ImGui source has been modified accordingly
    io.ConfigWindowsMoveFromTitleBarOnly = true; // Allow dragging only via the title bar

    // Set Dear ImGui style
    ImGui::StyleColorsDark();

    // Set platform/renderer backend
    ImGui_ImplWin32_Init(m_hMainWnd);
    ImGui_ImplDX11_Init(m_pd3dDevice.Get(), m_pd3dImmediateContext.Get());

    return true;
}

void D3DApp::CalculateFrameStats()
{
    // This code calculates frames per second and the time to render each frame, then displays it in the window title
    static int frameCnt = 0;
    static float timeElapsed = 0.0f;

    frameCnt++;

    if ((m_Timer.TotalTime() - timeElapsed) >= 1.0f) {
        float fps = (float)frameCnt; // fps = frameCnt / 1
        float mspf = 1000.0f / fps;

        std::wostringstream outs;
        outs.precision(6);
        outs << m_MainWndCaption << L"    "
             << L"FPS: " << fps << L"    "
             << L"Frame Time: " << mspf << L" (ms)";
        SetWindowText(m_hMainWnd, outs.str().c_str());

        // Reset for next average.
        frameCnt = 0;
        timeElapsed += 1.0f;
    }
}
