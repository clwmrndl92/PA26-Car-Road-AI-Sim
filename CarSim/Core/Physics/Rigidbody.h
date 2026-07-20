#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>

class Rigidbody
{
public:
    enum class Type { Static, Dynamic, Kinematic };

    // position stays the body's rotation pivot (its center of mass); colliderOffset shifts only
    // the collision box within that frame (see Init's OffsetCenterOfMassShape usage).
    void    Init(JPH::BodyInterface& bodyInterface, JPH::Vec3 halfExtents, JPH::Vec3 position, Type type,
                 JPH::Vec3 colliderOffset = JPH::Vec3::sZero(), float mass = 1.0f);
    void    Destroy(JPH::BodyInterface& bodyInterface);

    JPH::Vec3   GetPosition() const;
    JPH::Quat   GetRotation() const;
    void        SetPositionAndRotation(JPH::Vec3 position, JPH::Quat rotation);
    JPH::Vec3   GetLinearVelocity() const;
    void        SetLinearVelocity(JPH::Vec3 velocity);
    JPH::Vec3   GetAngularVelocity() const;
    void        SetAngularVelocity(JPH::Vec3 angularVelocity);
    void        AddForce(JPH::Vec3 force);

    bool        IsValid() const { return !m_bodyId.IsInvalid(); }
    JPH::BodyID GetBodyID() const { return m_bodyId; }

private:
    JPH::BodyInterface* m_bodyInterface = nullptr;
    JPH::BodyID         m_bodyId;
};
