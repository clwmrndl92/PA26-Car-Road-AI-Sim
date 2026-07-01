#include "PhysicsSystem.h"

#include <Jolt/Core/StreamWrapper.h>

JPH_SUPPRESS_WARNINGS

namespace { PhysicsSystem* s_pInstance = nullptr; }

PhysicsSystem::PhysicsSystem()
{
    if (s_pInstance)
        throw std::exception("PhysicsSystem is a singleton!");
    s_pInstance = this;
}

PhysicsSystem::~PhysicsSystem()
{
    s_pInstance = nullptr;
}

PhysicsSystem& PhysicsSystem::Get()
{
    if (!s_pInstance)
        throw std::exception("PhysicsSystem needs an instance!");
    return *s_pInstance;
}

void PhysicsSystem::Init()
{
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    m_jobSystem.Init(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers,
        std::thread::hardware_concurrency() - 1);

    m_physicsSystem.Init(
        MAX_BODIES, 0,
        MAX_BODY_PAIRS,
        MAX_CONTACT_CONSTRAINTS,
        m_bpLayerInterface,
        m_objVsBPFilter,
        m_objLayerFilter
    );

    m_physicsSystem.SetGravity(JPH::Vec3(0.0f, -9.81f, 0.0f));
}

void PhysicsSystem::Update(float dt)
{
    // Fixed timestep: max 1 collision step per frame
    m_physicsSystem.Update(dt, 1, &m_tempAllocator, &m_jobSystem);
}

void PhysicsSystem::Shutdown()
{
    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
}

JPH::BodyInterface& PhysicsSystem::GetBodyInterface()
{
    return m_physicsSystem.GetBodyInterface();
}
