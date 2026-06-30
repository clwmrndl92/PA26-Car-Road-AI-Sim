//***************************************************************************************
// DXTrace.h by X_Jun(MKXJun) (C) 2018-2022 All Rights Reserved.
// Licensed under the MIT License.
//
// DirectX Error Tracing
//***************************************************************************************

#pragma once

#ifndef DXTRACE_H
#define DXTRACE_H

#include "WinMin.h"

// ------------------------------
// DXTraceW function
// ------------------------------
// Outputs formatted error information to the debug output window, with an optional error message box
// [In]strFile          Current file name, typically the macro __FILEW__
// [In]hlslFileName     Current line number, typically the macro __LINE__
// [In]hr               The HRESULT value returned when a function call fails
// [In]strMsg           A string to help locate the problem during debugging, typically L#x (may be NULL)
// [In]bPopMsgBox       If TRUE, pops up a message box to report the error
// Return value: the hr parameter
HRESULT WINAPI DXTraceW(_In_z_ const WCHAR* strFile, _In_ DWORD dwLine, _In_ HRESULT hr, _In_opt_ const WCHAR* strMsg, _In_ bool bPopMsgBox);


// ------------------------------
// HR macro
// ------------------------------
// Error notification and tracing in Debug mode
#if defined(DEBUG) | defined(_DEBUG)
    #ifndef HR
    #define HR(x)												\
    {															\
        HRESULT hr = (x);										\
        if(FAILED(hr))											\
        {														\
            DXTraceW(__FILEW__, (DWORD)__LINE__, hr, L#x, true);\
        }														\
    }
    #endif
#else
    #ifndef HR
    #define HR(x) (x)
    #endif 
#endif



#endif
