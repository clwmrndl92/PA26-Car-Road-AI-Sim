#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>

namespace Layers {
    static constexpr JPH::ObjectLayer STATIC  = 0;
    static constexpr JPH::ObjectLayer DYNAMIC = 1;
    static constexpr JPH::uint NUM_LAYERS     = 2;
}

namespace BPLayers {
    static constexpr JPH::BroadPhaseLayer STATIC  { 0 };
    static constexpr JPH::BroadPhaseLayer DYNAMIC { 1 };
    static constexpr JPH::uint NUM_LAYERS = 2;
}
