#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

class Rigidbody
{
public:
    enum class Type { Static, Dynamic };

    void    Init(JPH::BodyInterface& bodyInterface, JPH::Vec3 halfExtents, JPH::Vec3 position, Type type);
    void    Destroy(JPH::BodyInterface& bodyInterface);

    JPH::Vec3   GetPosition() const;
    JPH::Quat   GetRotation() const;
    JPH::Vec3   GetLinearVelocity() const;
    void        SetLinearVelocity(JPH::Vec3 velocity);
    void        AddForce(JPH::Vec3 force);

    bool IsValid() const { return !m_bodyId.IsInvalid(); }

private:
    JPH::BodyInterface* m_bodyInterface = nullptr;
    JPH::BodyID         m_bodyId;
};
