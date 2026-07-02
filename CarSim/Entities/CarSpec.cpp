#include "CarSpec.h"

const CarSpec &GetCarSpec(CarType type)
{
    static const CarSpec specs[] = {
        // model path, half extents, render offset, collider offset, wheelbase, mass
        {"Model\\car_1.obj", JPH::Vec3(0.9919f, 0.9674f, 2.1204f), JPH::Vec3(0.0f, 0.0f, 1.5f), JPH::Vec3(0.0f, 0.96f, 1.5f), 3.0f, 1300.0f},
        {"Model\\car_2.obj", JPH::Vec3(1.3421f, 0.9073f, 2.8342f), JPH::Vec3(0.0f, 0.0f, 1.5f), JPH::Vec3(0.0f, 0.9f, 1.5f), 3.40f, 1600.0f},
    };
    return specs[static_cast<size_t>(type)];
}
