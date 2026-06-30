//***************************************************************************************
// GameObject.h by X_Jun(MKXJun) (C) 2018-2022 All Rights Reserved.
// Licensed under the MIT License.
//
// Retrieve the interval between two timestamps of the GPU.
//***************************************************************************************

#pragma once

#ifndef GPU_TIMER_H
#define GPU_TIMER_H

#include <cassert>
#include <cstdint>
#include <deque>
#include <wrl/client.h>
#include "WinMin.h"
#include <d3d11_1.h>

struct GpuTimerInfo
{
    D3D11_QUERY_DATA_TIMESTAMP_DISJOINT disjointData{}; // Frequency / disjoint info
    uint64_t startData = 0;                             // Start timestamp
    uint64_t stopData = 0;                              // Stop timestamp
    Microsoft::WRL::ComPtr<ID3D11Query> disjointQuery;  // Disjoint query
    Microsoft::WRL::ComPtr<ID3D11Query> startQuery;     // Start timestamp query
    Microsoft::WRL::ComPtr<ID3D11Query> stopQuery;      // Stop timestamp query
    bool isStopped = false;                             // Whether the stop timestamp has been inserted
};

class GpuTimer
{
public:
    GpuTimer() = default;
    
    // When recentCount is 0, averages all recorded intervals
    // Otherwise averages the most recent N frame intervals
    void Init(ID3D11Device* device, ID3D11DeviceContext* deviceContext, size_t recentCount = 0);

    // Reset the average elapsed time
    // When recentCount is 0, averages all recorded intervals
    // Otherwise averages the most recent N frame intervals
    void Reset(ID3D11DeviceContext* deviceContext, size_t recentCount = 0);
    // Insert the start timestamp into the command queue
    HRESULT Start();
    // Insert the stop timestamp into the command queue
    void Stop();
    // Try to retrieve the interval (non-blocking)
    bool TryGetTime(double* pOut);
    // Force retrieval of the interval (may block)
    double GetTime();
    // Calculate the average elapsed time
    double AverageTime()
    {
        if (m_RecentCount)
            return m_AccumTime / m_DeltaTimes.size();
        else
            return m_AccumTime / m_AccumCount;
    }

private:
    
    static bool GetQueryDataHelper(ID3D11DeviceContext* pContext, bool loopUntilDone, ID3D11Query* query, void* data, uint32_t dataSize);
    

    std::deque<double> m_DeltaTimes;    // Recorded intervals for the most recent N frames
    double m_AccumTime = 0.0;           // Accumulated total of query intervals
    size_t m_AccumCount = 0;            // Number of completed readbacks
    size_t m_RecentCount = 0;           // Keep the most recent N frames; 0 means keep all

    std::deque<GpuTimerInfo> m_Queries; // Cache of pending queries
    Microsoft::WRL::ComPtr<ID3D11Device> m_pDevice;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext> m_pImmediateContext;
};

#endif // GAMETIMER_H