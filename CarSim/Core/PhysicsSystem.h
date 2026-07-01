#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include "PhysicsLayers.h"

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return BPLayers::NUM_LAYERS; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override
    {
        return layer == Layers::STATIC ? BPLayers::STATIC : BPLayers::DYNAMIC;
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override
    {
        return layer == BPLayers::STATIC ? "STATIC" : "DYNAMIC";
    }
#endif
};

class ObjectVsBPLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer bpLayer) const override
    {
        if (layer == Layers::STATIC)  return bpLayer == BPLayers::DYNAMIC;
        if (layer == Layers::DYNAMIC) return true;
        return false;
    }
};

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter
{
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override
    {
        if (a == Layers::STATIC)  return b == Layers::DYNAMIC;
        if (a == Layers::DYNAMIC) return true;
        return false;
    }
};
class PhysicsSystem
{
public:
    PhysicsSystem();
    ~PhysicsSystem();

    static PhysicsSystem& Get();

    void Init();
    void Update(float dt);
    void Shutdown();

    JPH::BodyInterface& GetBodyInterface();

private:
    struct JoltInitializer {
        JoltInitializer() { JPH::RegisterDefaultAllocator(); }
    };

    JoltInitializer              m_joltInit;     // must be declared before m_tempAllocator
    JPH::TempAllocatorImpl       m_tempAllocator{ 16 * 1024 * 1024 };
    JPH::JobSystemThreadPool     m_jobSystem;
    BPLayerInterfaceImpl         m_bpLayerInterface;
    ObjectVsBPLayerFilterImpl    m_objVsBPFilter;
    ObjectLayerPairFilterImpl    m_objLayerFilter;
    JPH::PhysicsSystem           m_physicsSystem;

    static constexpr JPH::uint MAX_BODIES       = 2048;
    static constexpr JPH::uint MAX_BODY_PAIRS   = 4096;
    static constexpr JPH::uint MAX_CONTACT_CONSTRAINTS = 2048;
};
