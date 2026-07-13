
#pragma once

#include <vector>
#include <string>
#include <map>
#include <DirectXMath.h>
#include <functional>

struct GeometryData
{
    std::vector<DirectX::XMFLOAT3> vertices;
    std::vector<DirectX::XMFLOAT3> normals;
    std::vector<DirectX::XMFLOAT2> texcoords;
    std::vector<DirectX::XMFLOAT4> tangents;
    std::vector<uint32_t> indices32;
    std::vector<uint16_t> indices16;
};

namespace Geometry
{
    // Create sphere mesh data; larger levels and slices yield higher precision.
    GeometryData CreateSphere(float radius = 1.0f, uint32_t levels = 20, uint32_t slices = 20);

    // Create box mesh data
    GeometryData CreateBox(float width = 2.0f, float height = 2.0f, float depth = 2.0f);

    // Create cylinder mesh data; larger slices yield higher precision.
    GeometryData CreateCylinder(float radius = 1.0f, float height = 2.0f, uint32_t slices = 20, uint32_t stacks = 10, float texU = 1.0f, float texV = 1.0f);

    // Create cone mesh data; larger slices yield higher precision.
    GeometryData CreateCone(float radius = 1.0f, float height = 2.0f, uint32_t slices = 20);


    // Create a plane
    GeometryData CreatePlane(const DirectX::XMFLOAT2& planeSize, const DirectX::XMFLOAT2& maxTexCoord = { 1.0f, 1.0f });
    GeometryData CreatePlane(float width = 10.0f, float depth = 10.0f, float texU = 1.0f, float texV = 1.0f);

    // Create a grid
    GeometryData CreateGrid(const DirectX::XMFLOAT2& gridSize, const DirectX::XMUINT2& slices, const DirectX::XMFLOAT2& maxTexCoord,
        const std::function<float(float, float)>& heightFunc = [](float x, float z) { return 0.0f; },
        const std::function<DirectX::XMFLOAT3(float, float)>& normalFunc = [](float x, float z) { return DirectX::XMFLOAT3(0.0f, 1.0f, 0.0f); },
        const std::function<DirectX::XMFLOAT4(float, float)>& colorFunc = [](float x, float z) { return DirectX::XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f); });

    // Create a single line segment. Draw with a line-list topology (see BasicEffect::SetRenderLines).
    GeometryData CreateLine(const DirectX::XMFLOAT3& from, const DirectX::XMFLOAT3& to);

    // Create a grid of unit-spaced lines lying flat in the local XZ plane (y = 0), centered at the origin.
    // Draw with a line-list topology (see BasicEffect::SetRenderLines).
    GeometryData CreateLineGrid(float size = 500.0f, float spacing = 1.0f);

    // Create a connected polyline through the given points, as consecutive line segments.
    // Draw with a line-list topology (see BasicEffect::SetRenderLines).
    GeometryData CreatePolyline(const std::vector<DirectX::XMFLOAT3>& points);

    // Create a flat quad-strip ribbon of constant width following a centerline path in the XZ plane
    // (y taken from each point). Useful for roads/lanes built from spline sample points.
    GeometryData CreateRibbon(const std::vector<DirectX::XMFLOAT3>& centerPoints, float width);

}






