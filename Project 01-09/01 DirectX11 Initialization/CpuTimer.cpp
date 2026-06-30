#include "WinMin.h"
#include "CpuTimer.h"

CpuTimer::CpuTimer()
{
    __int64 countsPerSec{};
    QueryPerformanceFrequency((LARGE_INTEGER*)&countsPerSec);
    m_SecondsPerCount = 1.0 / (double)countsPerSec;
}


float CpuTimer::TotalTime()const
{
    // If Stop() was called, we don't need to count the paused time.
    // m_StopTime - m_BaseTime may include previously accumulated paused time,
    // so we subtract it from m_StopTime.
    //
    //                     |<-- paused time -->|
    // ----*---------------*-----------------*------------*------------*------> time
    //  m_BaseTime       m_StopTime        startTime     m_StopTime    m_CurrTime

    if( m_Stopped )
    {
        return (float)(((m_StopTime - m_PausedTime)-m_BaseTime)*m_SecondsPerCount);
    }

    // m_CurrTime - m_BaseTime includes paused time, but we don't want to count it.
    // So we subtract the previously accumulated paused time from m_CurrTime.
    //
    //  (m_CurrTime - m_PausedTime) - m_BaseTime
    //
    //                     |<-- paused time -->|
    // ----*---------------*-----------------*------------*------> time
    //  m_BaseTime       m_StopTime        startTime     m_CurrTime
    
    else
    {
        return (float)(((m_CurrTime-m_PausedTime)-m_BaseTime)*m_SecondsPerCount);
    }
}

float CpuTimer::DeltaTime()const
{
    return (float)m_DeltaTime;
}

void CpuTimer::Reset()
{
    __int64 currTime{};
    QueryPerformanceCounter((LARGE_INTEGER*)&currTime);

    m_BaseTime = currTime;
    m_PrevTime = currTime;
    m_StopTime = 0;
    m_PausedTime = 0;   // Must reset to 0 when Reset() is called multiple times
    m_Stopped  = false;
}

void CpuTimer::Start()
{
    __int64 startTime{};
    QueryPerformanceCounter((LARGE_INTEGER*)&startTime);


    // Accumulate elapsed time from pause start to pause end
    //
    //                     |<-------d------->|
    // ----*---------------*-----------------*------------> time
    //  m_BaseTime       m_StopTime        startTime     

    if( m_Stopped )
    {
        m_PausedTime += (startTime - m_StopTime);

        m_PrevTime = startTime;
        m_StopTime = 0;
        m_Stopped  = false;
    }
}

void CpuTimer::Stop()
{
    if( !m_Stopped )
    {
        __int64 currTime{};
        QueryPerformanceCounter((LARGE_INTEGER*)&currTime);

        m_StopTime = currTime;
        m_Stopped  = true;
    }
}

void CpuTimer::Tick()
{
    if( m_Stopped )
    {
        m_DeltaTime = 0.0;
        return;
    }

    __int64 currTime{};
    QueryPerformanceCounter((LARGE_INTEGER*)&currTime);
    m_CurrTime = currTime;

    // Frame interval between current tick and previous tick
    m_DeltaTime = (m_CurrTime - m_PrevTime)*m_SecondsPerCount;

    m_PrevTime = m_CurrTime;

    if(m_DeltaTime < 0.0)
    {
        m_DeltaTime = 0.0;
    }
}

bool CpuTimer::IsStopped() const
{
    return m_Stopped;
}

