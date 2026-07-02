#include "Rigidbody.h"
#include "PhysicsLayers.h"

JPH_SUPPRESS_WARNINGS

void Rigidbody::Init(JPH::BodyInterface& bodyInterface, JPH::Vec3 halfExtents, JPH::Vec3 position, Type type,
                      JPH::Vec3 colliderOffset, float mass)
{
    m_bodyInterface = &bodyInterface;

    JPH::EMotionType motionType = (type == Type::Dynamic)
        ? JPH::EMotionType::Dynamic
        : JPH::EMotionType::Static;

    JPH::ObjectLayer layer = (type == Type::Dynamic)
        ? Layers::DYNAMIC
        : Layers::STATIC;

    // Box is offset within the body's own frame, so the body still rotates around
    // `position`, not around the box's center.
    JPH::BodyCreationSettings settings(
        new JPH::RotatedTranslatedShapeSettings(colliderOffset, JPH::Quat::sIdentity(), new JPH::BoxShape(halfExtents)),
        JPH::RVec3(position),
        JPH::Quat::sIdentity(),
        motionType,
        layer
    );

    settings.mRestitution = 0.4f;

    if (type == Type::Dynamic)
    {
        // Take mass from `mass` but keep Jolt's shape-derived inertia tensor, scaled to that mass.
        settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = mass;
    }

    m_bodyId = bodyInterface.CreateAndAddBody(settings, JPH::EActivation::Activate);
}

void Rigidbody::Destroy(JPH::BodyInterface& bodyInterface)
{
    if (!m_bodyId.IsInvalid())
    {
        bodyInterface.RemoveBody(m_bodyId);
        bodyInterface.DestroyBody(m_bodyId);
        m_bodyId = JPH::BodyID();
    }
}

JPH::Vec3 Rigidbody::GetPosition() const
{
    return JPH::Vec3(m_bodyInterface->GetPosition(m_bodyId));
}

JPH::Quat Rigidbody::GetRotation() const
{
    return m_bodyInterface->GetRotation(m_bodyId);
}

void Rigidbody::SetPositionAndRotation(JPH::Vec3 position, JPH::Quat rotation)
{
    m_bodyInterface->SetPositionAndRotation(m_bodyId, JPH::RVec3(position), rotation, JPH::EActivation::Activate);
}

JPH::Vec3 Rigidbody::GetLinearVelocity() const
{
    return m_bodyInterface->GetLinearVelocity(m_bodyId);
}

void Rigidbody::SetLinearVelocity(JPH::Vec3 velocity)
{
    m_bodyInterface->SetLinearVelocity(m_bodyId, velocity);
}

JPH::Vec3 Rigidbody::GetAngularVelocity() const
{
    return m_bodyInterface->GetAngularVelocity(m_bodyId);
}

void Rigidbody::SetAngularVelocity(JPH::Vec3 angularVelocity)
{
    m_bodyInterface->SetAngularVelocity(m_bodyId, angularVelocity);
}

void Rigidbody::AddForce(JPH::Vec3 force)
{
    m_bodyInterface->AddForce(m_bodyId, force);
}
