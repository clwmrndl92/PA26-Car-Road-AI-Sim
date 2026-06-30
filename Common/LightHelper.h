#ifndef LIGHTHELPER_H
#define LIGHTHELPER_H

#include <cstring>
#include <DirectXMath.h>


// Directional light
struct DirectionalLight
{
    DirectionalLight() = default;

    DirectionalLight(const DirectionalLight&) = default;
    DirectionalLight& operator=(const DirectionalLight&) = default;

    DirectionalLight(DirectionalLight&&) = default;
    DirectionalLight& operator=(DirectionalLight&&) = default;

    DirectionalLight(const DirectX::XMFLOAT4& _ambient, const DirectX::XMFLOAT4& _diffuse, const DirectX::XMFLOAT4& _specular,
        const DirectX::XMFLOAT3& _direction) :
        ambient(_ambient), diffuse(_diffuse), specular(_specular), direction(_direction), pad() {}

    DirectX::XMFLOAT4 ambient;
    DirectX::XMFLOAT4 diffuse;
    DirectX::XMFLOAT4 specular;
    DirectX::XMFLOAT3 direction;
    float pad; // Pad to make the struct size a multiple of 16, for easy HLSL array binding
};

// Point light
struct PointLight
{
    PointLight() = default;

    PointLight(const PointLight&) = default;
    PointLight& operator=(const PointLight&) = default;

    PointLight(PointLight&&) = default;
    PointLight& operator=(PointLight&&) = default;

    PointLight(const DirectX::XMFLOAT4& _ambient, const DirectX::XMFLOAT4& _diffuse, const DirectX::XMFLOAT4& _specular,
        const DirectX::XMFLOAT3& _position, float _range, const DirectX::XMFLOAT3& _att) :
        ambient(_ambient), diffuse(_diffuse), specular(_specular), position(_position), range(_range), att(_att), pad() {}

    DirectX::XMFLOAT4 ambient;
    DirectX::XMFLOAT4 diffuse;
    DirectX::XMFLOAT4 specular;

    // Packed into a 4D vector: (position, range)
    DirectX::XMFLOAT3 position;
    float range;

    // Packed into a 4D vector: (A0, A1, A2, pad)
    DirectX::XMFLOAT3 att;
    float pad; // Pad to make the struct size a multiple of 16, for easy HLSL array binding
};

// Spotlight
struct SpotLight
{
    SpotLight() = default;

    SpotLight(const SpotLight&) = default;
    SpotLight& operator=(const SpotLight&) = default;

    SpotLight(SpotLight&&) = default;
    SpotLight& operator=(SpotLight&&) = default;

    SpotLight(const DirectX::XMFLOAT4& _ambient, const DirectX::XMFLOAT4& _diffuse, const DirectX::XMFLOAT4& _specular,
        const DirectX::XMFLOAT3& _position, float _range, const DirectX::XMFLOAT3& _direction,
        float _spot, const DirectX::XMFLOAT3& _att) :
        ambient(_ambient), diffuse(_diffuse), specular(_specular), 
        position(_position), range(_range), direction(_direction), spot(_spot), att(_att), pad() {}

    DirectX::XMFLOAT4 ambient;
    DirectX::XMFLOAT4 diffuse;
    DirectX::XMFLOAT4 specular;

    // Packed into a 4D vector: (position, range)
    DirectX::XMFLOAT3 position;
    float range;

    // Packed into a 4D vector: (direction, spot)
    DirectX::XMFLOAT3 direction;
    float spot;

    // Packed into a 4D vector: (att, pad)
    DirectX::XMFLOAT3 att;
    float pad; // Pad to make the struct size a multiple of 16, for easy HLSL array binding
};

// Object surface material
struct PhongMaterial
{
    PhongMaterial() = default;

    PhongMaterial(const PhongMaterial&) = default;
    PhongMaterial& operator=(const PhongMaterial&) = default;

    PhongMaterial(PhongMaterial&&) = default;
    PhongMaterial& operator=(PhongMaterial&&) = default;

    PhongMaterial(const DirectX::XMFLOAT4& _ambient, const DirectX::XMFLOAT4& _diffuse, const DirectX::XMFLOAT4& _specular,
        const DirectX::XMFLOAT4& _reflect) :
        ambient(_ambient), diffuse(_diffuse), specular(_specular), reflect(_reflect) {}

    DirectX::XMFLOAT4 ambient;
    DirectX::XMFLOAT4 diffuse;
    DirectX::XMFLOAT4 specular; // w = specular intensity
    DirectX::XMFLOAT4 reflect;
};

#endif
