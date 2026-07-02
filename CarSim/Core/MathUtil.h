#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <DirectXMath.h>

inline DirectX::XMFLOAT3 ToXMFLOAT3(JPH::Vec3 v)
{
    return DirectX::XMFLOAT3(v.GetX(), v.GetY(), v.GetZ());
}
