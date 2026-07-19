#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <DirectXMath.h>
#include <string>

using Vec3 = JPH::Vec3;

inline DirectX::XMFLOAT3 ToXMFLOAT3(Vec3 v)
{
    return DirectX::XMFLOAT3(v.GetX(), v.GetY(), v.GetZ());
}

inline Vec3 ToVec3(const DirectX::XMFLOAT3 &v)
{
    return Vec3(v.x, v.y, v.z);
}

inline std::string ToString(Vec3 v)
{
    return std::to_string(v.GetX()) + " " + std::to_string(v.GetY()) + " " + std::to_string(v.GetZ());
}
inline std::string ToString(float f)
{
    return std::to_string(f);
}
inline std::string ToString(int f)
{
    return std::to_string(f);
}

constexpr float ToRadians(float degrees)
{
    return degrees * (3.14159265358979323846f / 180.0f);
}

constexpr float ToDegrees(float radians)
{
    return radians * (180.0f / 3.14159265358979323846f);
}