//***************************************************************************************
// Camera.h by X_Jun(MKXJun) (C) 2018-2022 All Rights Reserved.
// Licensed under the MIT License.
//
// Provide 1st person(free view) and 3rd person cameras.
//***************************************************************************************

#pragma once

#ifndef CAMERA_H
#define CAMERA_H

#include "WinMin.h"
#include <d3d11_1.h>
#include <DirectXMath.h>
#include "Transform.h"


class Camera
{
public:
    Camera() = default;
    virtual ~Camera() = 0;

    //
    // Get camera position
    //

    DirectX::XMVECTOR GetPositionXM() const;
    DirectX::XMFLOAT3 GetPosition() const;

    //
    // Get camera rotation
    //

    // Get Euler angle (in radians) of rotation around the X axis
    float GetRotationX() const;
    // Get Euler angle (in radians) of rotation around the Y axis
    float GetRotationY() const;

    //
    // Get camera axis vectors
    //

    DirectX::XMVECTOR GetRightAxisXM() const;
    DirectX::XMFLOAT3 GetRightAxis() const;
    DirectX::XMVECTOR GetUpAxisXM() const;
    DirectX::XMFLOAT3 GetUpAxis() const;
    DirectX::XMVECTOR GetLookAxisXM() const;
    DirectX::XMFLOAT3 GetLookAxis() const;

    //
    // Get matrices
    //

    DirectX::XMMATRIX GetLocalToWorldMatrixXM() const;
    DirectX::XMMATRIX GetViewMatrixXM() const;
    DirectX::XMMATRIX GetProjMatrixXM(bool reversedZ = false) const;
    DirectX::XMMATRIX GetViewProjMatrixXM(bool reversedZ = false) const;

    // Get viewport
    D3D11_VIEWPORT GetViewPort() const;

    float GetNearZ() const;
    float GetFarZ() const;
    float GetFovY() const;
    float GetAspectRatio() const;

    // Set frustum
    void SetFrustum(float fovY, float aspect, float nearZ, float farZ);

    // Set viewport
    void SetViewPort(const D3D11_VIEWPORT& viewPort);
    void SetViewPort(float topLeftX, float topLeftY, float width, float height, float minDepth = 0.0f, float maxDepth = 1.0f);

protected:

    // Camera transform
    Transform m_Transform = {};

    // Frustum properties
    float m_NearZ = 0.0f;
    float m_FarZ = 0.0f;
    float m_Aspect = 0.0f;
    float m_FovY = 0.0f;

    // Current viewport
    D3D11_VIEWPORT m_ViewPort = {};

};

class FirstPersonCamera : public Camera
{
public:
    FirstPersonCamera() = default;
    ~FirstPersonCamera() override;

    // Set camera position
    void SetPosition(float x, float y, float z);
    void SetPosition(const DirectX::XMFLOAT3& pos);
    // Set camera orientation
    void LookAt(const DirectX::XMFLOAT3& pos, const DirectX::XMFLOAT3& target,const DirectX::XMFLOAT3& up);
    void LookTo(const DirectX::XMFLOAT3& pos, const DirectX::XMFLOAT3& to, const DirectX::XMFLOAT3& up);
    // Strafe
    void Strafe(float d);
    // Walk (planar movement)
    void Walk(float d);
    // Move forward (in the forward direction)
    void MoveForward(float d);
    // Translate
    void Translate(const DirectX::XMFLOAT3& dir, float magnitude);
    // Look up/down
    // Positive rad looks up
    // Negative rad looks down
    void Pitch(float rad);
    // Look left/right
    // Positive rad looks right
    // Negative rad looks left
    void RotateY(float rad);
};

class ThirdPersonCamera : public Camera
{
public:
    ThirdPersonCamera() = default;
    ~ThirdPersonCamera() override;

    // Get the position of the currently tracked object
    DirectX::XMFLOAT3 GetTargetPosition() const;
    // Get the distance to the tracked object
    float GetDistance() const;
    // Rotate vertically around the object (note: Euler angle around x axis is clamped to [0, pi/3])
    void RotateX(float rad);
    // Rotate horizontally around the object
    void RotateY(float rad);
    // Move closer to the object
    void Approach(float dist);
    // Set initial rotation around the X axis (note: Euler angle around x axis is clamped to [0, pi/3])
    void SetRotationX(float rad);
    // Set initial rotation around the Y axis
    void SetRotationY(float rad);
    // Set and bind the position of the object to track
    void SetTarget(const DirectX::XMFLOAT3& target);
    // Set initial distance
    void SetDistance(float dist);
    // Set minimum and maximum allowed distances
    void SetDistanceMinMax(float minDist, float maxDist);

private:
    DirectX::XMFLOAT3 m_Target = {};
    float m_Distance = 0.0f;
    // Minimum allowed distance, maximum allowed distance
    float m_MinDist = 0.0f, m_MaxDist = 0.0f;
};


#endif
