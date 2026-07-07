#include "Camera.h"
using namespace DirectX;

Camera::~Camera()
{
}

XMVECTOR Camera::GetPositionXM() const
{
    return m_Transform.GetPositionXM();
}

XMFLOAT3 Camera::GetPosition() const
{
    return m_Transform.GetPosition();
}

float Camera::GetRotationX() const
{
    return m_Transform.GetRotation().x;
}

float Camera::GetRotationY() const
{
    return m_Transform.GetRotation().y;
}

XMVECTOR Camera::GetRightAxisXM() const
{
    return m_Transform.GetRightAxisXM();
}

XMFLOAT3 Camera::GetRightAxis() const
{
    return m_Transform.GetRightAxis();
}

XMVECTOR Camera::GetUpAxisXM() const
{
    return m_Transform.GetUpAxisXM();
}

XMFLOAT3 Camera::GetUpAxis() const
{
    return m_Transform.GetUpAxis();
}

XMVECTOR Camera::GetLookAxisXM() const
{
    return m_Transform.GetForwardAxisXM();
}

XMFLOAT3 Camera::GetLookAxis() const
{
    return m_Transform.GetForwardAxis();
}

DirectX::XMMATRIX Camera::GetLocalToWorldMatrixXM() const
{
    return m_Transform.GetLocalToWorldMatrixXM();
}

XMMATRIX Camera::GetViewMatrixXM() const
{
    return m_Transform.GetWorldToLocalMatrixXM();
}

XMMATRIX Camera::GetProjMatrixXM(bool reversedZ) const
{
    if (reversedZ)
        return XMMatrixPerspectiveFovLH(m_FovY, m_Aspect, m_FarZ, m_NearZ);
    else
        return XMMatrixPerspectiveFovLH(m_FovY, m_Aspect, m_NearZ, m_FarZ);
}

XMMATRIX Camera::GetViewProjMatrixXM(bool reversedZ) const
{
    return GetViewMatrixXM() * GetProjMatrixXM(reversedZ);
}

D3D11_VIEWPORT Camera::GetViewPort() const
{
    return m_ViewPort;
}

float Camera::GetNearZ() const
{
    return m_NearZ;
}

float Camera::GetFarZ() const
{
    return m_FarZ;
}

float Camera::GetFovY() const
{
    return m_FovY;
}

float Camera::GetAspectRatio() const
{
    return m_Aspect;
}

void Camera::SetFrustum(float fovY, float aspect, float nearZ, float farZ)
{
    m_FovY = fovY;
    m_Aspect = aspect;
    m_NearZ = nearZ;
    m_FarZ = farZ;
}

void Camera::SetViewPort(const D3D11_VIEWPORT &viewPort)
{
    m_ViewPort = viewPort;
}

void Camera::SetViewPort(float topLeftX, float topLeftY, float width, float height, float minDepth, float maxDepth)
{
    m_ViewPort.TopLeftX = topLeftX;
    m_ViewPort.TopLeftY = topLeftY;
    m_ViewPort.Width = width;
    m_ViewPort.Height = height;
    m_ViewPort.MinDepth = minDepth;
    m_ViewPort.MaxDepth = maxDepth;
}

// ******************
// First-person / free-view camera
//

FreeCamera::~FreeCamera()
{
}

void FreeCamera::SetPosition(float x, float y, float z)
{
    SetPosition(XMFLOAT3(x, y, z));
}

void FreeCamera::SetPosition(const XMFLOAT3 &pos)
{
    m_Transform.SetPosition(pos);
}

void FreeCamera::LookAt(const XMFLOAT3 &pos, const XMFLOAT3 &target, const XMFLOAT3 &up)
{
    m_Transform.SetPosition(pos);
    m_Transform.LookAt(target, up);
}

void FreeCamera::LookTo(const XMFLOAT3 &pos, const XMFLOAT3 &to, const XMFLOAT3 &up)
{
    m_Transform.SetPosition(pos);
    m_Transform.LookTo(to, up);
}

void FreeCamera::Strafe(float d)
{
    m_Transform.Translate(m_Transform.GetRightAxis(), d);
}

void FreeCamera::Walk(float d)
{
    XMVECTOR rightVec = m_Transform.GetRightAxisXM();
    XMVECTOR frontVec = XMVector3Normalize(XMVector3Cross(rightVec, g_XMIdentityR1));
    XMFLOAT3 front;
    XMStoreFloat3(&front, frontVec);
    m_Transform.Translate(front, d);
}

void FreeCamera::MoveForward(float d)
{
    m_Transform.Translate(m_Transform.GetForwardAxis(), d);
}

void FreeCamera::Translate(const DirectX::XMFLOAT3 &dir, float magnitude)
{
    m_Transform.Translate(dir, magnitude);
}

void FreeCamera::Pitch(float rad)
{
    XMFLOAT3 rotation = m_Transform.GetRotation();
    // Clamp the rotation around the x axis to [-7pi/18, 7pi/18]
    rotation.x += rad;
    if (rotation.x > XM_PI * 7 / 18)
        rotation.x = XM_PI * 7 / 18;
    else if (rotation.x < -XM_PI * 7 / 18)
        rotation.x = -XM_PI * 7 / 18;

    m_Transform.SetRotation(rotation);
}

void FreeCamera::RotateY(float rad)
{
    XMFLOAT3 rotation = m_Transform.GetRotation();
    rotation.y = XMScalarModAngle(rotation.y + rad);
    m_Transform.SetRotation(rotation);
}

// ******************
// Third-person camera
//

FocusCamera::~FocusCamera()
{
}

XMFLOAT3 FocusCamera::GetTargetPosition() const
{
    return m_Target;
}

float FocusCamera::GetDistance() const
{
    return m_Distance;
}

void FocusCamera::RotateX(float rad)
{
    XMFLOAT3 rotation = m_Transform.GetRotation();
    // Clamp the rotation around the x axis to [0, pi/3]
    rotation.x += rad;
    if (rotation.x < 0.0f)
        rotation.x = 0.0f;
    else if (rotation.x > XM_PI / 3)
        rotation.x = XM_PI / 3;

    m_Transform.SetRotation(rotation);
    m_Transform.SetPosition(m_Target);
    m_Transform.Translate(m_Transform.GetForwardAxis(), -m_Distance);
}

void FocusCamera::RotateY(float rad)
{
    XMFLOAT3 rotation = m_Transform.GetRotation();
    rotation.y = XMScalarModAngle(rotation.y + rad);

    m_Transform.SetRotation(rotation);
    m_Transform.SetPosition(m_Target);
    m_Transform.Translate(m_Transform.GetForwardAxis(), -m_Distance);
}

void FocusCamera::Approach(float dist)
{
    m_Distance += dist;
    // Clamp the distance to [m_MinDist, m_MaxDist]
    if (m_Distance < m_MinDist)
        m_Distance = m_MinDist;
    else if (m_Distance > m_MaxDist)
        m_Distance = m_MaxDist;

    m_Transform.SetPosition(m_Target);
    m_Transform.Translate(m_Transform.GetForwardAxis(), -m_Distance);
}

void FocusCamera::SetRotationX(float rad)
{
    XMFLOAT3 rotation = m_Transform.GetRotation();
    // Clamp the rotation around the x axis to [0, pi/3]
    rotation.x = rad;
    if (rotation.x < 0.0f)
        rotation.x = 0.0f;
    else if (rotation.x > XM_PI / 3)
        rotation.x = XM_PI / 3;

    m_Transform.SetRotation(rotation);
    m_Transform.SetPosition(m_Target);
    m_Transform.Translate(m_Transform.GetForwardAxis(), -m_Distance);
}

void FocusCamera::SetRotationY(float rad)
{
    XMFLOAT3 rotation = m_Transform.GetRotation();
    rotation.y = XMScalarModAngle(rad);
    m_Transform.SetRotation(rotation);
    m_Transform.SetPosition(m_Target);
    m_Transform.Translate(m_Transform.GetForwardAxis(), -m_Distance);
}

void FocusCamera::SetTarget(const XMFLOAT3 &target)
{
    m_Target = target;
}

void FocusCamera::SetDistance(float dist)
{
    m_Distance = dist;
}

void FocusCamera::SetDistanceMinMax(float minDist, float maxDist)
{
    m_MinDist = minDist;
    m_MaxDist = maxDist;
}
