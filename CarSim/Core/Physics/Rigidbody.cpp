#include "Core/Physics/Rigidbody.h"
#include "PhysicsLayers.h"

JPH_SUPPRESS_WARNINGS

void Rigidbody::Init(JPH::BodyInterface &bodyInterface, JPH::Vec3 halfExtents, JPH::Vec3 position, Type type,
                     JPH::Vec3 colliderOffset, float mass, JPH::EAllowedDOFs allowedDOFs)
{
    m_bodyInterface = &bodyInterface;

    JPH::EMotionType motionType = JPH::EMotionType::Static;
    if (type == Type::Dynamic)
        motionType = JPH::EMotionType::Dynamic;
    else if (type == Type::Kinematic)
        motionType = JPH::EMotionType::Kinematic;

    // Kinematic도 Dynamic과 마찬가지로 "움직이는" 레이어 -- Static(도로/장애물)과의 충돌 감지가
    // 필요하다(HasNewContact가 이걸로 장애물 접촉을 잡아낸다).
    JPH::ObjectLayer layer = (type == Type::Static) ? Layers::STATIC : Layers::DYNAMIC;

    // Jolt always simulates/rotates a body around Shape::GetCenterOfMass(), regardless of any
    // mass/inertia override -- so just offsetting the box via RotatedTranslatedShape makes the
    // body rotate around the box's center, not `position`. OffsetCenterOfMassShape cancels that
    // out (-colliderOffset) so the shape's center of mass -- and therefore the rotation pivot --
    // lands back on `position`, while the collider itself stays visually offset.
    JPH::BodyCreationSettings settings(
        new JPH::OffsetCenterOfMassShapeSettings(-colliderOffset,
                                                 new JPH::RotatedTranslatedShapeSettings(colliderOffset, JPH::Quat::sIdentity(), new JPH::BoxShape(halfExtents))),
        JPH::RVec3(position),
        JPH::Quat::sIdentity(),
        motionType,
        layer);
    settings.mAllowedDOFs = allowedDOFs; // 예: 차량은 RotationY만 허용해 충돌/중력에 의한 피치·롤(전복)을 막는다

    if (type == Type::Dynamic)
    {
        // Take mass from `mass` but keep Jolt's shape-derived inertia tensor, scaled to that mass.
        settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = mass;
    }

    m_bodyId = bodyInterface.CreateAndAddBody(settings, JPH::EActivation::Activate);
}

void Rigidbody::Destroy(JPH::BodyInterface &bodyInterface)
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
