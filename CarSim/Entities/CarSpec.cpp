#include "CarSpec.h"

const CarSpec& GetCarSpec(CarType type)
{
    static const CarSpec specs[] = {
        { "Model\\car_1.obj", JPH::Vec3(0.9919f, 0.9674f, 2.1204f), 5.0f, 0.1f },
        { "Model\\car_2.obj", JPH::Vec3(1.3421f, 0.9073f, 2.8342f), 3.0f, 0.05f },
    };
    return specs[static_cast<size_t>(type)];
}
