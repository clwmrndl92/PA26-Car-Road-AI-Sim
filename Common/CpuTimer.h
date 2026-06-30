//***************************************************************************************
// CpuTimer.h by Frank Luna (C) 2011 All Rights Reserved.
// Modify name from GameTimer.cpp
// CPU timer
//***************************************************************************************

#pragma once

#ifndef CPU_TIMER_H
#define CPU_TIMER_H

class CpuTimer
{
public:
    CpuTimer();
 
    float TotalTime()const;     // Returns elapsed time since Reset() was called, excluding paused periods
    float DeltaTime()const;		// Returns the time between frames

    void Reset();               // Call before timing starts or when a reset is needed
    void Start();               // Call when starting timing or resuming from a pause
    void Stop();                // Call when a pause is needed
    void Tick();                // Call at the beginning of each frame
    bool IsStopped() const;     // Whether the timer is paused/stopped

private:
    double m_SecondsPerCount = 0.0;
    double m_DeltaTime = -1.0;

    __int64 m_BaseTime = 0;
    __int64 m_PausedTime = 0;
    __int64 m_StopTime = 0;
    __int64 m_PrevTime = 0;
    __int64 m_CurrTime = 0;

    bool m_Stopped = false;
};

#endif // GAMETIMER_H