
#pragma once

#ifndef DEBUG_LAYER_H
#define DEBUG_LAYER_H

#include "WinMin.h"
#include <d3d11_1.h>
#include <wrl/client.h>
#include <vector>
#include <deque>
#include <string>


class DebugLayer
{
public:
    ~DebugLayer() { ClearMessages(); }

    HRESULT Init(ID3D11Device* device);

    // Report live objects
    void ReportLiveDeviceObjects(D3D11_RLDO_FLAGS detailLevel) { m_pDebug->ReportLiveDeviceObjects(detailLevel); }

    // Whether to suppress message output to the debug output window
    void MuteDebugOutput(bool mute) { m_pInfoQueue->SetMuteDebugOutput(mute); }

    // Cache all messages in the info queue and clear the ID3D11InfoQueue messages
    const std::vector<D3D11_MESSAGE*>& FetchMessages();

    // Clear all cached messages
    void ClearMessages();
    
    
private:
    Microsoft::WRL::ComPtr<ID3D11Debug> m_pDebug;
    Microsoft::WRL::ComPtr<ID3D11InfoQueue> m_pInfoQueue;
    std::vector<D3D11_MESSAGE*> m_pMessages;
};





#endif