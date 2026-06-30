//***************************************************************************************
// XUtil.h by X_Jun(MKXJun) (C) 2018-2022 All Rights Reserved.
// Licensed under the MIT License.
//
// utility tools.
//***************************************************************************************

#pragma once

#ifndef XUTIL_H
#define XUTIL_H

#include <DirectXMath.h>
#include <vector>
#include <string>
#include <string_view>
#include <algorithm>

//
// Macro definitions
//

#define LEN_AND_STR(STR) ((UINT)(sizeof(STR) - 1)), (STR)

// Whether to enable graphics debugger object names
#if (defined(DEBUG) || defined(_DEBUG)) && !defined(GRAPHICS_DEBUGGER_OBJECT_NAME)
#define GRAPHICS_DEBUGGER_OBJECT_NAME 1
#endif

//
// Set debug object name
//

template<class IObject>
inline void SetDebugObjectName(IObject* pObject, std::string_view name)
{
    pObject->SetPrivateData(WKPDID_D3DDebugObjectName, (uint32_t)name.size(), name.data());
}

//
// Text conversion functions
//

// Do not delete the following
#pragma warning(push)
#pragma warning(disable: 28251)
extern "C" __declspec(dllimport) int __stdcall MultiByteToWideChar(unsigned int cp, unsigned long flags, const char* str, int cbmb, wchar_t* widestr, int cchwide);
extern "C" __declspec(dllimport) int __stdcall WideCharToMultiByte(unsigned int cp, unsigned long flags, const wchar_t* widestr, int cchwide, char* str, int cbmb, const char* defchar, int* used_default);
#pragma warning(pop)

inline std::wstring UTF8ToWString(std::string_view utf8str)
{
    if (utf8str.empty()) return std::wstring();
    int cbMultiByte = static_cast<int>(utf8str.size());
    int req = MultiByteToWideChar(65001, 0, utf8str.data(), cbMultiByte, nullptr, 0);
    std::wstring res(req, 0);
    MultiByteToWideChar(65001, 0, utf8str.data(), cbMultiByte, &res[0], req);
    return res;
}

inline std::string WStringToUTF8(std::wstring_view wstr)
{
    if (wstr.empty()) return std::string();
    int cbMultiByte = static_cast<int>(wstr.size());
    int req = WideCharToMultiByte(65001, 0, wstr.data(), cbMultiByte, nullptr, 0, nullptr, nullptr);
    std::string res(req, 0);
    WideCharToMultiByte(65001, 0, wstr.data(), cbMultiByte, &res[0], req, nullptr, nullptr);
    return res;
}

//
// String to hash ID
//

using XID = size_t;
inline XID StringToID(std::string_view str)
{
    static std::hash<std::string_view> hash;
    return hash(str);
}

//
// Math utility functions
//

namespace XMath
{
    // ------------------------------
    // InverseTranspose function
    // ------------------------------
    inline DirectX::XMMATRIX XM_CALLCONV InverseTranspose(DirectX::FXMMATRIX M)
    {
        using namespace DirectX;

        // The inverse-transpose of the world matrix applies only to normals;
        // we also don't need the translation component of the world matrix,
        // because leaving it in would produce incorrect results when later
        // multiplied by the view matrix or similar transforms

        XMMATRIX A = M;
        A.r[3] = g_XMIdentityR3;

        return XMMatrixTranspose(XMMatrixInverse(nullptr, A));
    }

    inline float Lerp(float a, float b, float t)
    {
        return (1.0f - t) * a + t * b;
    }
}


#endif
