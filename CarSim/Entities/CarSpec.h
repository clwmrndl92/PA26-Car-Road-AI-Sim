#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>

struct CarSpec
{
    const char* modelPath;
    JPH::Vec3   halfExtents;
    float       maxSpeed;
    float       acceleration;
};

enum class CarType
{
    Sedan,
    Truck,
};

const CarSpec& GetCarSpec(CarType type);
