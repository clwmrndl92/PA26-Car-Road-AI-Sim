//***************************************************************************************
// Transform.h by X_Jun(MKXJun) (C) 2018-2022 All Rights Reserved.
// Licensed under the MIT License.
//
// Describe object scale, rotation(quaternion-based) and translation
//***************************************************************************************

#pragma once

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <DirectXMath.h>

class Transform
{
public:
    Transform() = default;
    Transform(const DirectX::XMFLOAT3& scale, const DirectX::XMFLOAT3& rotation, const DirectX::XMFLOAT3& position) 
        : m_Scale(scale), m_Position(position) 
    {
        SetRotation(rotation);
    }
    ~Transform() = default;

    Transform(const Transform&) = default;
    Transform& operator=(const Transform&) = default;

    Transform(Transform&&) = default;
    Transform& operator=(Transform&&) = default;

    // Get the object scale
    DirectX::XMFLOAT3 GetScale() const { return m_Scale; }
    // Get the object scale (XMVECTOR)
    DirectX::XMVECTOR GetScaleXM() const { return XMLoadFloat3(&m_Scale); }

    // Get the object Euler angles (radians)
    // The object rotates in Z-X-Y axis order
    DirectX::XMFLOAT3 GetRotation() const 
    {
        float sinX = 2 * (m_Rotation.w * m_Rotation.x - m_Rotation.y * m_Rotation.z);
        float sinY_cosX = 2 * (m_Rotation.w * m_Rotation.y + m_Rotation.x * m_Rotation.z);
        float cosY_cosX = 1 - 2 * (m_Rotation.x * m_Rotation.x + m_Rotation.y * m_Rotation.y);
        float sinZ_cosX = 2 * (m_Rotation.w * m_Rotation.z + m_Rotation.x * m_Rotation.y);
        float cosZ_cosX = 1 - 2 * (m_Rotation.x * m_Rotation.x + m_Rotation.z * m_Rotation.z);

        DirectX::XMFLOAT3 rotation;
        // Special handling for gimbal lock
        if (fabs(fabs(sinX) - 1.0f) < 1e-5f)
        {
            rotation.x = copysignf(DirectX::XM_PI / 2, sinX);
            rotation.y = 2.0f * atan2f(m_Rotation.y, m_Rotation.w);
            rotation.z = 0.0f;
        }
        else
        {
            rotation.x = asinf(sinX);
            rotation.y = atan2f(sinY_cosX, cosY_cosX);
            rotation.z = atan2f(sinZ_cosX, cosZ_cosX);
        }

        return rotation;
    }
    // Get the object rotation quaternion
    DirectX::XMFLOAT4 GetRotationQuat() const { return m_Rotation; }
    // Get the object Euler angles (radians, XMVECTOR)
    // The object rotates in Z-X-Y axis order
    DirectX::XMVECTOR GetRotationXM() const { auto rot = GetRotation(); return XMLoadFloat3(&rot); }
    // Get the object rotation quaternion (XMVECTOR)
    DirectX::XMVECTOR GetRotationQuatXM() const { return XMLoadFloat4(&m_Rotation); }

    // Get the object position
    DirectX::XMFLOAT3 GetPosition() const { return m_Position; }
    // Get the object position (XMVECTOR)
    DirectX::XMVECTOR GetPositionXM() const { return XMLoadFloat3(&m_Position); }

    // Get the right direction axis
    DirectX::XMFLOAT3 GetRightAxis() const 
    {
        using namespace DirectX;
        XMMATRIX R = XMMatrixRotationQuaternion(XMLoadFloat4(&m_Rotation));
        XMFLOAT3 right;
        XMStoreFloat3(&right, R.r[0]);
        return right;
    }
    // Get the right direction axis (XMVECTOR)
    DirectX::XMVECTOR GetRightAxisXM() const 
    { 
        DirectX::XMFLOAT3 right = GetRightAxis(); 
        return DirectX::XMLoadFloat3(&right);
    }

    // Get the up direction axis
    DirectX::XMFLOAT3 GetUpAxis() const
    {
        using namespace DirectX;
        XMMATRIX R = XMMatrixRotationQuaternion(XMLoadFloat4(&m_Rotation));
        XMFLOAT3 up;
        XMStoreFloat3(&up, R.r[1]);
        return up;
    }
    // Get the up direction axis (XMVECTOR)
    DirectX::XMVECTOR GetUpAxisXM() const 
    {
        DirectX::XMFLOAT3 up = GetUpAxis();
        return DirectX::XMLoadFloat3(&up);
    }

    // Get the forward direction axis
    DirectX::XMFLOAT3 GetForwardAxis() const
    {
        using namespace DirectX;
        XMMATRIX R = XMMatrixRotationQuaternion(XMLoadFloat4(&m_Rotation));
        XMFLOAT3 forward;
        XMStoreFloat3(&forward, R.r[2]);
        return forward;
    }
    // Get the forward direction axis (XMVECTOR)
    DirectX::XMVECTOR GetForwardAxisXM() const
    {
        DirectX::XMFLOAT3 forward = GetForwardAxis();
        return DirectX::XMLoadFloat3(&forward);
    }

    // Get the local-to-world transform matrix
    DirectX::XMFLOAT4X4 GetLocalToWorldMatrix() const
    {
        DirectX::XMFLOAT4X4 res;
        DirectX::XMStoreFloat4x4(&res, GetLocalToWorldMatrixXM());
        return res;
    }
    // Get the local-to-world transform matrix (XMMATRIX)
    DirectX::XMMATRIX GetLocalToWorldMatrixXM() const
    {
        using namespace DirectX;
        DirectX::XMVECTOR scaleVec = XMLoadFloat3(&m_Scale);
        DirectX::XMVECTOR quateration = XMLoadFloat4(&m_Rotation);
        DirectX::XMVECTOR positionVec = XMLoadFloat3(&m_Position);
        DirectX::XMMATRIX World = XMMatrixAffineTransformation(scaleVec, g_XMZero, quateration, positionVec);
        return World;
    }

    // Get the world-to-local transform matrix
    DirectX::XMFLOAT4X4 GetWorldToLocalMatrix() const
    {
        DirectX::XMFLOAT4X4 res;
        DirectX::XMStoreFloat4x4(&res, GetWorldToLocalMatrixXM());
        return res;
    }
    // Get the world-to-local transform matrix (XMMATRIX)
    DirectX::XMMATRIX GetWorldToLocalMatrixXM() const
    {
        DirectX::XMMATRIX InvWorld = DirectX::XMMatrixInverse(nullptr, GetLocalToWorldMatrixXM());
        return InvWorld;
    }

    // Set the object scale
    void SetScale(const DirectX::XMFLOAT3& scale) { m_Scale = scale; }
    // Set the object scale
    void SetScale(float x, float y, float z) { m_Scale = DirectX::XMFLOAT3(x, y, z); }

    // Set the object Euler angles (radians)
    // The object will rotate in Z-X-Y axis order
    void SetRotation(const DirectX::XMFLOAT3& eulerAnglesInRadian)
    {
        auto quat = DirectX::XMQuaternionRotationRollPitchYawFromVector(DirectX::XMLoadFloat3(&eulerAnglesInRadian));
        DirectX::XMStoreFloat4(&m_Rotation, quat);
    }
    // Set the object Euler angles (radians)
    // The object will rotate in Z-X-Y axis order
    void SetRotation(float x, float y, float z)
    {
        auto quat = DirectX::XMQuaternionRotationRollPitchYaw(x, y, z);
        DirectX::XMStoreFloat4(&m_Rotation, quat);
    }

    // Set the object position
    void SetPosition(const DirectX::XMFLOAT3& position) { m_Position = position; }
    // Set the object position
    void SetPosition(float x, float y, float z) { m_Position = DirectX::XMFLOAT3(x, y, z); }

    // Rotate the object by the given Euler angles
    void Rotate(const DirectX::XMFLOAT3& eulerAnglesInRadian)
    {
        using namespace DirectX;
        auto newQuat = XMQuaternionRotationRollPitchYawFromVector(XMLoadFloat3(&eulerAnglesInRadian));
        auto quat = XMLoadFloat4(&m_Rotation);
        XMStoreFloat4(&m_Rotation, XMQuaternionMultiply(quat, newQuat));
    }
    // Rotate around an axis with the origin as the pivot
    void RotateAxis(const DirectX::XMFLOAT3& axis, float radian)
    {
        using namespace DirectX;
        auto newQuat = XMQuaternionRotationAxis(XMLoadFloat3(&axis), radian);
        auto quat = XMLoadFloat4(&m_Rotation);
        XMStoreFloat4(&m_Rotation, XMQuaternionMultiply(quat, newQuat));
    }
    // Rotate around an axis with the given point as the pivot
    void RotateAround(const DirectX::XMFLOAT3& point, const DirectX::XMFLOAT3& axis, float radian)
    {
        using namespace DirectX;
        XMVECTOR quat = XMLoadFloat4(&m_Rotation);
        XMVECTOR positionVec = XMLoadFloat3(&m_Position);
        XMVECTOR centerVec = XMLoadFloat3(&point);

        // Rotate using point as the origin
        XMMATRIX RT = XMMatrixRotationQuaternion(quat) * XMMatrixTranslationFromVector(positionVec - centerVec);
        RT *= XMMatrixRotationAxis(XMLoadFloat3(&axis), radian);
        RT *= XMMatrixTranslationFromVector(centerVec);
        XMStoreFloat4(&m_Rotation, XMQuaternionRotationMatrix(RT));
        XMStoreFloat3(&m_Position, RT.r[3]);
    }
    // Translate along a given direction
    void Translate(const DirectX::XMFLOAT3& direction, float magnitude)
    {
        using namespace DirectX;
        XMVECTOR directionVec = XMVector3Normalize(XMLoadFloat3(&direction));
        XMVECTOR newPosition = XMVectorMultiplyAdd(XMVectorReplicate(magnitude), directionVec, XMLoadFloat3(&m_Position));
        XMStoreFloat3(&m_Position, newPosition);
    }

    // Look at a target point
    void LookAt(const DirectX::XMFLOAT3& target, const DirectX::XMFLOAT3& up = { 0.0f, 1.0f, 0.0f })
    {
        using namespace DirectX;
        XMMATRIX View = XMMatrixLookAtLH(XMLoadFloat3(&m_Position), XMLoadFloat3(&target), XMLoadFloat3(&up));
        XMMATRIX InvView = XMMatrixInverse(nullptr, View);
        XMStoreFloat4(&m_Rotation, XMQuaternionRotationMatrix(InvView));
    }
    // Look along a given direction
    void LookTo(const DirectX::XMFLOAT3& direction, const DirectX::XMFLOAT3& up = { 0.0f, 1.0f, 0.0f })
    {
        using namespace DirectX;
        XMMATRIX View = XMMatrixLookToLH(XMLoadFloat3(&m_Position), XMLoadFloat3(&direction), XMLoadFloat3(&up));
        XMMATRIX InvView = XMMatrixInverse(nullptr, View);
        XMStoreFloat4(&m_Rotation, XMQuaternionRotationMatrix(InvView));
    }

    // Get Euler angles from a rotation matrix
    static DirectX::XMFLOAT3 GetEulerAnglesFromRotationMatrix(const DirectX::XMFLOAT4X4& rotationMatrix)
    {
        DirectX::XMFLOAT3 rotation{};
        // Special handling for gimbal lock
        if (fabs(1.0f - fabs(rotationMatrix(2, 1))) < 1e-5f)
        {
            rotation.x = copysignf(DirectX::XM_PIDIV2, -rotationMatrix(2, 1));
            rotation.y = -atan2f(rotationMatrix(0, 2), rotationMatrix(0, 0));
            return rotation;
        }

        // Solve for Euler angles from the rotation matrix
        float c = sqrtf(1.0f - rotationMatrix(2, 1) * rotationMatrix(2, 1));
        // Guard against r[2][1] exceeding 1
        if (isnan(c))
            c = 0.0f;
        
        rotation.z = atan2f(rotationMatrix(0, 1), rotationMatrix(1, 1));
        rotation.x = atan2f(-rotationMatrix(2, 1), c);
        rotation.y = atan2f(rotationMatrix(2, 0), rotationMatrix(2, 2));
        return rotation;
    }

private:
    DirectX::XMFLOAT3 m_Scale = { 1.0f, 1.0f, 1.0f };				// Scale
    DirectX::XMFLOAT4 m_Rotation = { 0.0f, 0.0f, 0.0f, 1.0f };		// Rotation quaternion
    DirectX::XMFLOAT3 m_Position = {};								// Position
};

#endif


