#include "Rigidbody.h"
#include "PhysicsLayers.h"

JPH_SUPPRESS_WARNINGS

void Rigidbody::Init(JPH::BodyInterface& bodyInterface, JPH::Vec3 halfExtents, JPH::Vec3 position, Type type,
                      JPH::Vec3 colliderOffset)
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

JPH::Vec3 Rigidbody::GetLinearVelocity() const
{
    return m_bodyInterface->GetLinearVelocity(m_bodyId);
}

void Rigidbody::SetLinearVelocity(JPH::Vec3 velocity)
{
    m_bodyInterface->SetLinearVelocity(m_bodyId, velocity);
}

void Rigidbody::AddForce(JPH::Vec3 force)
{
    m_bodyInterface->AddForce(m_bodyId, force);
}
