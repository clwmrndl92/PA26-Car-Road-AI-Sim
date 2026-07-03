#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <mutex>
#include <unordered_set>
#include "PhysicsLayers.h"

// Records bodies that started touching something during the last physics step. Jolt calls
// OnContactAdded once per pair, from worker threads, only at the moment contact begins
// (not while it persists), so this doubles as a one-shot "collision just happened" signal.
class CarContactListener final : public JPH::ContactListener
{
public:
    void OnContactAdded(const JPH::Body &inBody1, const JPH::Body &inBody2,
                         const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override;

    void Clear();
    bool HasNewContact(JPH::BodyID id) const;

private:
    mutable std::mutex             m_mutex;
    std::unordered_set<JPH::BodyID> m_newContacts;
};

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

    // True if `id` started touching another body during the most recent Update(). One-shot per
    // contact start, not per frame the bodies remain touching.
    bool HasNewContact(JPH::BodyID id) const { return m_contactListener.HasNewContact(id); }

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
    CarContactListener           m_contactListener;

    static constexpr JPH::uint MAX_BODIES       = 2048;
    static constexpr JPH::uint MAX_BODY_PAIRS   = 4096;
    static constexpr JPH::uint MAX_CONTACT_CONSTRAINTS = 2048;
};
