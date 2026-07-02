#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>

struct CarSpec
{
    const char *modelPath;
    JPH::Vec3 halfExtents;
    JPH::Vec3 renderOffset;   // where the model is drawn, relative to the car's transform
    JPH::Vec3 colliderOffset; // where the collider is placed, relative to the car's transform
    float wheelbase;          // distance between front and rear axle centers, for the bicycle model
    float mass;               // kg, used as the rigidbody's mass for force-based driving
};

enum class CarType
{
    Car0,
    Car1,
};

const CarSpec &GetCarSpec(CarType type);
