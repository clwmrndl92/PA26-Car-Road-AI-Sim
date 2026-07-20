#include "Geometry.h"
#include <algorithm>

namespace Geometry
{
    constexpr float PI = 3.1415926f;
    //
    // Implementation of geometry methods
    //

    GeometryData CreateSphere(float radius, uint32_t levels, uint32_t slices)
    {
        using namespace DirectX;

        GeometryData geoData;

        uint32_t vertexCount = 2 + (levels - 1) * (slices + 1);
        uint32_t indexCount = 6 * (levels - 1) * slices;
        geoData.vertices.resize(vertexCount);
        geoData.normals.resize(vertexCount);
        geoData.texcoords.resize(vertexCount);
        geoData.tangents.resize(vertexCount);
        if (indexCount > 65535)
            geoData.indices32.resize(indexCount);
        else
            geoData.indices16.resize(indexCount);

        uint32_t vIndex = 0, iIndex = 0;

        float phi = 0.0f, theta = 0.0f;
        float per_phi = PI / levels;
        float per_theta = 2 * PI / slices;
        float x, y, z;

        // Insert top apex vertex
        geoData.vertices[vIndex] = XMFLOAT3(0.0f, radius, 0.0f);
        geoData.normals[vIndex] = XMFLOAT3(0.0f, 1.0f, 0.0f);
        geoData.tangents[vIndex] = XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f);
        geoData.texcoords[vIndex++] = XMFLOAT2(0.0f, 0.0f);

        for (uint32_t i = 1; i < levels; ++i)
        {
            phi = per_phi * i;
            // slices + 1 vertices are needed because the start and end points must be at the same position but have different texture coordinates
            for (uint32_t j = 0; j <= slices; ++j)
            {
                theta = per_theta * j;
                x = radius * sinf(phi) * cosf(theta);
                y = radius * cosf(phi);
                z = radius * sinf(phi) * sinf(theta);
                // Compute local position, normal, tangent, and texture coordinates
                XMFLOAT3 pos = XMFLOAT3(x, y, z);

                geoData.vertices[vIndex] = pos;
                XMStoreFloat3(&geoData.normals[vIndex], XMVector3Normalize(XMLoadFloat3(&pos)));
                geoData.tangents[vIndex] = XMFLOAT4(-sinf(theta), 0.0f, cosf(theta), 1.0f);
                geoData.texcoords[vIndex++] = XMFLOAT2(theta / 2 / PI, phi / PI);
            }
        }

        // Insert bottom apex vertex
        geoData.vertices[vIndex] = XMFLOAT3(0.0f, -radius, 0.0f);
        geoData.normals[vIndex] = XMFLOAT3(0.0f, -1.0f, 0.0f);
        geoData.tangents[vIndex] = XMFLOAT4(-1.0f, 0.0f, 0.0f, 1.0f);
        geoData.texcoords[vIndex++] = XMFLOAT2(0.0f, 1.0f);

        // Insert indices
        if (levels > 1)
        {
            for (uint32_t j = 1; j <= slices; ++j)
            {
                if (indexCount > 65535)
                {
                    geoData.indices32[iIndex++] = 0;
                    geoData.indices32[iIndex++] = j % (slices + 1) + 1;
                    geoData.indices32[iIndex++] = j;
                }
                else
                {
                    geoData.indices16[iIndex++] = 0;
                    geoData.indices16[iIndex++] = j % (slices + 1) + 1;
                    geoData.indices16[iIndex++] = j;
                }
            }
        }

        for (uint32_t i = 1; i < levels - 1; ++i)
        {
            for (uint32_t j = 1; j <= slices; ++j)
            {
                if (indexCount > 65535)
                {
                    geoData.indices32[iIndex++] = (i - 1) * (slices + 1) + j;
                    geoData.indices32[iIndex++] = (i - 1) * (slices + 1) + j % (slices + 1) + 1;
                    geoData.indices32[iIndex++] = i * (slices + 1) + j % (slices + 1) + 1;

                    geoData.indices32[iIndex++] = i * (slices + 1) + j % (slices + 1) + 1;
                    geoData.indices32[iIndex++] = i * (slices + 1) + j;
                    geoData.indices32[iIndex++] = (i - 1) * (slices + 1) + j;
                }
                else
                {
                    geoData.indices16[iIndex++] = (i - 1) * (slices + 1) + j;
                    geoData.indices16[iIndex++] = (i - 1) * (slices + 1) + j % (slices + 1) + 1;
                    geoData.indices16[iIndex++] = i * (slices + 1) + j % (slices + 1) + 1;

                    geoData.indices16[iIndex++] = i * (slices + 1) + j % (slices + 1) + 1;
                    geoData.indices16[iIndex++] = i * (slices + 1) + j;
                    geoData.indices16[iIndex++] = (i - 1) * (slices + 1) + j;
                }
            }
        }

        // Insert bottom cap indices
        if (levels > 1)
        {
            for (uint32_t j = 1; j <= slices; ++j)
            {
                if (indexCount > 65535)
                {
                    geoData.indices32[iIndex++] = (levels - 2) * (slices + 1) + j;
                    geoData.indices32[iIndex++] = (levels - 2) * (slices + 1) + j % (slices + 1) + 1;
                    geoData.indices32[iIndex++] = (levels - 1) * (slices + 1) + 1;
                }
                else
                {
                    geoData.indices16[iIndex++] = (levels - 2) * (slices + 1) + j;
                    geoData.indices16[iIndex++] = (levels - 2) * (slices + 1) + j % (slices + 1) + 1;
                    geoData.indices16[iIndex++] = (levels - 1) * (slices + 1) + 1;
                }
            }
        }

        return geoData;
    }

    GeometryData CreateBox(float width, float height, float depth)
    {
        using namespace DirectX;

        GeometryData geoData;

        geoData.vertices.resize(24);
        geoData.normals.resize(24);
        geoData.tangents.resize(24);
        geoData.texcoords.resize(24);

        float w2 = width / 2, h2 = height / 2, d2 = depth / 2;

        // Right face (+X face)
        geoData.vertices[0] = XMFLOAT3(w2, -h2, -d2);
        geoData.vertices[1] = XMFLOAT3(w2, h2, -d2);
        geoData.vertices[2] = XMFLOAT3(w2, h2, d2);
        geoData.vertices[3] = XMFLOAT3(w2, -h2, d2);
        // Left face (-X face)
        geoData.vertices[4] = XMFLOAT3(-w2, -h2, d2);
        geoData.vertices[5] = XMFLOAT3(-w2, h2, d2);
        geoData.vertices[6] = XMFLOAT3(-w2, h2, -d2);
        geoData.vertices[7] = XMFLOAT3(-w2, -h2, -d2);
        // Top face (+Y face)
        geoData.vertices[8] = XMFLOAT3(-w2, h2, -d2);
        geoData.vertices[9] = XMFLOAT3(-w2, h2, d2);
        geoData.vertices[10] = XMFLOAT3(w2, h2, d2);
        geoData.vertices[11] = XMFLOAT3(w2, h2, -d2);
        // Bottom face (-Y face)
        geoData.vertices[12] = XMFLOAT3(w2, -h2, -d2);
        geoData.vertices[13] = XMFLOAT3(w2, -h2, d2);
        geoData.vertices[14] = XMFLOAT3(-w2, -h2, d2);
        geoData.vertices[15] = XMFLOAT3(-w2, -h2, -d2);
        // Back face (+Z face)
        geoData.vertices[16] = XMFLOAT3(w2, -h2, d2);
        geoData.vertices[17] = XMFLOAT3(w2, h2, d2);
        geoData.vertices[18] = XMFLOAT3(-w2, h2, d2);
        geoData.vertices[19] = XMFLOAT3(-w2, -h2, d2);
        // Front face (-Z face)
        geoData.vertices[20] = XMFLOAT3(-w2, -h2, -d2);
        geoData.vertices[21] = XMFLOAT3(-w2, h2, -d2);
        geoData.vertices[22] = XMFLOAT3(w2, h2, -d2);
        geoData.vertices[23] = XMFLOAT3(w2, -h2, -d2);

        for (size_t i = 0; i < 4; ++i)
        {
            // Right face (+X face)
            geoData.normals[i] = XMFLOAT3(1.0f, 0.0f, 0.0f);
            geoData.tangents[i] = XMFLOAT4(0.0f, 0.0f, 1.0f, 1.0f);
            // Left face (-X face)
            geoData.normals[i + 4] = XMFLOAT3(-1.0f, 0.0f, 0.0f);
            geoData.tangents[i + 4] = XMFLOAT4(0.0f, 0.0f, -1.0f, 1.0f);
            // Top face (+Y face)
            geoData.normals[i + 8] = XMFLOAT3(0.0f, 1.0f, 0.0f);
            geoData.tangents[i + 8] = XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f);
            // Bottom face (-Y face)
            geoData.normals[i + 12] = XMFLOAT3(0.0f, -1.0f, 0.0f);
            geoData.tangents[i + 12] = XMFLOAT4(-1.0f, 0.0f, 0.0f, 1.0f);
            // Back face (+Z face)
            geoData.normals[i + 16] = XMFLOAT3(0.0f, 0.0f, 1.0f);
            geoData.tangents[i + 16] = XMFLOAT4(-1.0f, 0.0f, 0.0f, 1.0f);
            // Front face (-Z face)
            geoData.normals[i + 20] = XMFLOAT3(0.0f, 0.0f, -1.0f);
            geoData.tangents[i + 20] = XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f);
        }

        for (size_t i = 0; i < 6; ++i)
        {
            geoData.texcoords[i * 4] = XMFLOAT2(0.0f, 1.0f);
            geoData.texcoords[i * 4 + 1] = XMFLOAT2(0.0f, 0.0f);
            geoData.texcoords[i * 4 + 2] = XMFLOAT2(1.0f, 0.0f);
            geoData.texcoords[i * 4 + 3] = XMFLOAT2(1.0f, 1.0f);
        }

        geoData.indices16.resize(36);

        uint16_t indices[] = {
            0, 1, 2, 2, 3, 0,       // Right face (+X face)
            4, 5, 6, 6, 7, 4,       // Left face (-X face)
            8, 9, 10, 10, 11, 8,    // Top face (+Y face)
            12, 13, 14, 14, 15, 12, // Bottom face (-Y face)
            16, 17, 18, 18, 19, 16, // Back face (+Z face)
            20, 21, 22, 22, 23, 20  // Front face (-Z face)
        };
        memcpy_s(geoData.indices16.data(), sizeof indices, indices, sizeof indices);

        return geoData;
    }

    GeometryData CreateCylinder(float radius, float height, uint32_t slices, uint32_t stacks, float texU, float texV)
    {
        using namespace DirectX;

        GeometryData geoData;
        uint32_t vertexCount = (slices + 1) * (stacks + 3) + 2;
        uint32_t indexCount = 6 * slices * (stacks + 1);

        geoData.vertices.resize(vertexCount);
        geoData.normals.resize(vertexCount);
        geoData.tangents.resize(vertexCount);
        geoData.texcoords.resize(vertexCount);

        if (indexCount > 65535)
            geoData.indices32.resize(indexCount);
        else
            geoData.indices16.resize(indexCount);

        float h2 = height / 2;
        float theta = 0.0f;
        float per_theta = 2 * PI / slices;
        float stackHeight = height / stacks;
        //
        // Side surface
        //
        {
            // Lay out the side vertices from bottom to top
            size_t vIndex = 0;
            for (size_t i = 0; i < stacks + 1; ++i)
            {
                float y = -h2 + i * stackHeight;
                // Vertices for the current ring
                for (size_t j = 0; j <= slices; ++j)
                {
                    theta = j * per_theta;
                    float u = theta / 2 / PI;
                    float v = 1.0f - (float)i / stacks;

                    geoData.vertices[vIndex] = XMFLOAT3(radius * cosf(theta), y, radius * sinf(theta)), XMFLOAT3(cosf(theta), 0.0f, sinf(theta));
                    geoData.normals[vIndex] = XMFLOAT3(cosf(theta), 0.0f, sinf(theta));
                    geoData.tangents[vIndex] = XMFLOAT4(-sinf(theta), 0.0f, cosf(theta), 1.0f);
                    geoData.texcoords[vIndex++] = XMFLOAT2(u * texU, v * texV);
                }
            }

            // Insert indices
            size_t iIndex = 0;
            for (uint32_t i = 0; i < stacks; ++i)
            {
                for (uint32_t j = 0; j < slices; ++j)
                {
                    if (indexCount > 65535)
                    {
                        geoData.indices32[iIndex++] = i * (slices + 1) + j;
                        geoData.indices32[iIndex++] = (i + 1) * (slices + 1) + j;
                        geoData.indices32[iIndex++] = (i + 1) * (slices + 1) + j + 1;

                        geoData.indices32[iIndex++] = i * (slices + 1) + j;
                        geoData.indices32[iIndex++] = (i + 1) * (slices + 1) + j + 1;
                        geoData.indices32[iIndex++] = i * (slices + 1) + j + 1;
                    }
                    else
                    {
                        geoData.indices16[iIndex++] = i * (slices + 1) + j;
                        geoData.indices16[iIndex++] = (i + 1) * (slices + 1) + j;
                        geoData.indices16[iIndex++] = (i + 1) * (slices + 1) + j + 1;

                        geoData.indices16[iIndex++] = i * (slices + 1) + j;
                        geoData.indices16[iIndex++] = (i + 1) * (slices + 1) + j + 1;
                        geoData.indices16[iIndex++] = i * (slices + 1) + j + 1;
                    }
                }
            }
        }

        //
        // Top and bottom caps
        //
        {
            size_t vIndex = (slices + 1) * (stacks + 1), iIndex = 6 * slices * stacks;
            uint32_t offset = static_cast<uint32_t>(vIndex);

            // Insert top cap center vertex
            geoData.vertices[vIndex] = XMFLOAT3(0.0f, h2, 0.0f);
            geoData.normals[vIndex] = XMFLOAT3(0.0f, 1.0f, 0.0f);
            geoData.tangents[vIndex] = XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f);
            geoData.texcoords[vIndex++] = XMFLOAT2(0.5f, 0.5f);

            // Insert vertices along the top cap circle
            for (uint32_t i = 0; i <= slices; ++i)
            {
                theta = i * per_theta;
                float u = cosf(theta) * radius / height + 0.5f;
                float v = sinf(theta) * radius / height + 0.5f;
                geoData.vertices[vIndex] = XMFLOAT3(radius * cosf(theta), h2, radius * sinf(theta));
                geoData.normals[vIndex] = XMFLOAT3(0.0f, 1.0f, 0.0f);
                geoData.tangents[vIndex] = XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f);
                geoData.texcoords[vIndex++] = XMFLOAT2(u, v);
            }

            // Insert bottom cap center vertex
            geoData.vertices[vIndex] = XMFLOAT3(0.0f, -h2, 0.0f);
            geoData.normals[vIndex] = XMFLOAT3(0.0f, -1.0f, 0.0f);
            geoData.tangents[vIndex] = XMFLOAT4(-1.0f, 0.0f, 0.0f, 1.0f);
            geoData.texcoords[vIndex++] = XMFLOAT2(0.5f, 0.5f);

            // Insert vertices along the bottom cap circle
            for (uint32_t i = 0; i <= slices; ++i)
            {
                theta = i * per_theta;
                float u = cosf(theta) * radius / height + 0.5f;
                float v = sinf(theta) * radius / height + 0.5f;
                geoData.vertices[vIndex] = XMFLOAT3(radius * cosf(theta), -h2, radius * sinf(theta));
                geoData.normals[vIndex] = XMFLOAT3(0.0f, -1.0f, 0.0f);
                geoData.tangents[vIndex] = XMFLOAT4(-1.0f, 0.0f, 0.0f, 1.0f);
                geoData.texcoords[vIndex++] = XMFLOAT2(u, v);
            }

            // Insert top cap triangle indices
            for (uint32_t i = 1; i <= slices; ++i)
            {
                if (indexCount > 65535)
                {
                    geoData.indices32[iIndex++] = offset;
                    geoData.indices32[iIndex++] = offset + i % (slices + 1) + 1;
                    geoData.indices32[iIndex++] = offset + i;
                }
                else
                {
                    geoData.indices16[iIndex++] = offset;
                    geoData.indices16[iIndex++] = offset + i % (slices + 1) + 1;
                    geoData.indices16[iIndex++] = offset + i;
                }
            }

            // Insert bottom cap triangle indices
            offset += slices + 2;
            for (uint32_t i = 1; i <= slices; ++i)
            {
                if (indexCount > 65535)
                {
                    geoData.indices32[iIndex++] = offset;
                    geoData.indices32[iIndex++] = offset + i;
                    geoData.indices32[iIndex++] = offset + i % (slices + 1) + 1;
                }
                else
                {
                    geoData.indices16[iIndex++] = offset;
                    geoData.indices16[iIndex++] = offset + i;
                    geoData.indices16[iIndex++] = offset + i % (slices + 1) + 1;
                }
            }
        }

        return geoData;
    }

    GeometryData CreateCone(float radius, float height, uint32_t slices)
    {
        using namespace DirectX;

        GeometryData geoData;

        uint32_t vertexCount = 3 * slices + 1;
        uint32_t indexCount = 6 * slices;
        geoData.vertices.resize(vertexCount);
        geoData.normals.resize(vertexCount);
        geoData.tangents.resize(vertexCount);
        geoData.texcoords.resize(vertexCount);

        if (indexCount > 65535)
            geoData.indices32.resize(indexCount);
        else
            geoData.indices16.resize(indexCount);

        float h2 = height / 2;
        float theta = 0.0f;
        float per_theta = 2 * PI / slices;
        float len = sqrtf(height * height + radius * radius);

        //
        // Cone lateral surface
        //
        {
            size_t iIndex = 0;
            size_t vIndex = 0;

            // Insert cone apex vertices (same position for each, but with different normals and tangents)
            for (uint32_t i = 0; i < slices; ++i)
            {
                theta = i * per_theta + per_theta / 2;
                geoData.vertices[vIndex] = XMFLOAT3(0.0f, h2, 0.0f);
                geoData.normals[vIndex] = XMFLOAT3(radius * cosf(theta) / len, height / len, radius * sinf(theta) / len);
                geoData.tangents[vIndex] = XMFLOAT4(-sinf(theta), 0.0f, cosf(theta), 1.0f);
                geoData.texcoords[vIndex++] = XMFLOAT2(0.5f, 0.5f);
            }

            // Insert cone lateral surface base vertices
            for (uint32_t i = 0; i < slices; ++i)
            {
                theta = i * per_theta;
                geoData.vertices[vIndex] = XMFLOAT3(radius * cosf(theta), -h2, radius * sinf(theta));
                geoData.normals[vIndex] = XMFLOAT3(radius * cosf(theta) / len, height / len, radius * sinf(theta) / len);
                geoData.tangents[vIndex] = XMFLOAT4(-sinf(theta), 0.0f, cosf(theta), 1.0f);
                geoData.texcoords[vIndex++] = XMFLOAT2(cosf(theta) / 2 + 0.5f, sinf(theta) / 2 + 0.5f);
            }

            // Insert indices
            for (uint32_t i = 0; i < slices; ++i)
            {
                if (indexCount > 65535)
                {
                    geoData.indices32[iIndex++] = i;
                    geoData.indices32[iIndex++] = slices + (i + 1) % slices;
                    geoData.indices32[iIndex++] = slices + i % slices;
                }
                else
                {
                    geoData.indices16[iIndex++] = i;
                    geoData.indices16[iIndex++] = slices + (i + 1) % slices;
                    geoData.indices16[iIndex++] = slices + i % slices;
                }
            }
        }

        //
        // Cone base
        //
        {
            size_t iIndex = 3 * (size_t)slices;
            size_t vIndex = 2 * (size_t)slices;

            // Insert cone base vertices
            for (uint32_t i = 0; i < slices; ++i)
            {
                theta = i * per_theta;

                geoData.vertices[vIndex] = XMFLOAT3(radius * cosf(theta), -h2, radius * sinf(theta)),
                geoData.normals[vIndex] = XMFLOAT3(0.0f, -1.0f, 0.0f);
                geoData.tangents[vIndex] = XMFLOAT4(-1.0f, 0.0f, 0.0f, 1.0f);
                geoData.texcoords[vIndex++] = XMFLOAT2(cosf(theta) / 2 + 0.5f, sinf(theta) / 2 + 0.5f);
            }
            // Insert cone base center vertex
            geoData.vertices[vIndex] = XMFLOAT3(0.0f, -h2, 0.0f),
            geoData.normals[vIndex] = XMFLOAT3(0.0f, -1.0f, 0.0f);
            geoData.texcoords[vIndex++] = XMFLOAT2(0.5f, 0.5f);

            // Insert indices
            uint32_t offset = 2 * slices;
            for (uint32_t i = 0; i < slices; ++i)
            {
                if (indexCount > 65535)
                {
                    geoData.indices32[iIndex++] = offset + slices;
                    geoData.indices32[iIndex++] = offset + i % slices;
                    geoData.indices32[iIndex++] = offset + (i + 1) % slices;
                }
                else
                {
                    geoData.indices16[iIndex++] = offset + slices;
                    geoData.indices16[iIndex++] = offset + i % slices;
                    geoData.indices16[iIndex++] = offset + (i + 1) % slices;
                }
            }
        }

        return geoData;
    }

    GeometryData CreatePlane(const DirectX::XMFLOAT2 &planeSize, const DirectX::XMFLOAT2 &maxTexCoord)
    {
        return CreatePlane(planeSize.x, planeSize.y, maxTexCoord.x, maxTexCoord.y);
    }

    GeometryData CreatePlane(float width, float depth, float texU, float texV)
    {
        using namespace DirectX;

        GeometryData geoData;

        geoData.vertices.resize(4);
        geoData.normals.resize(4);
        geoData.tangents.resize(4);
        geoData.texcoords.resize(4);

        uint32_t vIndex = 0;
        geoData.vertices[vIndex] = XMFLOAT3(-width / 2, 0.0f, -depth / 2);
        geoData.normals[vIndex] = XMFLOAT3(0.0f, 1.0f, 0.0f);
        geoData.tangents[vIndex] = XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f);
        geoData.texcoords[vIndex++] = XMFLOAT2(0.0f, texV);

        geoData.vertices[vIndex] = XMFLOAT3(-width / 2, 0.0f, depth / 2);
        geoData.normals[vIndex] = XMFLOAT3(0.0f, 1.0f, 0.0f);
        geoData.tangents[vIndex] = XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f);
        geoData.texcoords[vIndex++] = XMFLOAT2(0.0f, 0.0f);

        geoData.vertices[vIndex] = XMFLOAT3(width / 2, 0.0f, depth / 2);
        geoData.normals[vIndex] = XMFLOAT3(0.0f, 1.0f, 0.0f);
        geoData.tangents[vIndex] = XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f);
        geoData.texcoords[vIndex++] = XMFLOAT2(texU, 0.0f);

        geoData.vertices[vIndex] = XMFLOAT3(width / 2, 0.0f, -depth / 2);
        geoData.normals[vIndex] = XMFLOAT3(0.0f, 1.0f, 0.0f);
        geoData.tangents[vIndex] = XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f);
        geoData.texcoords[vIndex++] = XMFLOAT2(texU, texV);

        geoData.indices16 = {0, 1, 2, 2, 3, 0};

        return geoData;
    }

    GeometryData CreateGrid(const DirectX::XMFLOAT2 &gridSize, const DirectX::XMUINT2 &slices, const DirectX::XMFLOAT2 &maxTexCoord,
                            const std::function<float(float, float)> &heightFunc,
                            const std::function<DirectX::XMFLOAT3(float, float)> &normalFunc,
                            const std::function<DirectX::XMFLOAT4(float, float)> &colorFunc)
    {
        using namespace DirectX;

        GeometryData geoData;
        uint32_t vertexCount = (slices.x + 1) * (slices.y + 1);
        uint32_t indexCount = 6 * slices.x * slices.y;
        geoData.vertices.resize(vertexCount);
        geoData.normals.resize(vertexCount);
        geoData.tangents.resize(vertexCount);
        geoData.texcoords.resize(vertexCount);
        if (indexCount > 65535)
            geoData.indices32.resize(indexCount);
        else
            geoData.indices16.resize(indexCount);

        uint32_t vIndex = 0;
        uint32_t iIndex = 0;

        float sliceWidth = gridSize.x / slices.x;
        float sliceDepth = gridSize.y / slices.y;
        float leftBottomX = -gridSize.x / 2;
        float leftBottomZ = -gridSize.y / 2;
        float posX, posZ;
        float sliceTexWidth = maxTexCoord.x / slices.x;
        float sliceTexDepth = maxTexCoord.y / slices.y;

        XMFLOAT3 normal;
        XMFLOAT4 tangent;
        // Create grid vertices
        //  __ __
        // | /| /|
        // |/_|/_|
        // | /| /|
        // |/_|/_|
        for (uint32_t z = 0; z <= slices.y; ++z)
        {
            posZ = leftBottomZ + z * sliceDepth;
            for (uint32_t x = 0; x <= slices.x; ++x)
            {
                posX = leftBottomX + x * sliceWidth;
                // Compute the normal and normalize it
                normal = normalFunc(posX, posZ);
                XMStoreFloat3(&normal, XMVector3Normalize(XMLoadFloat3(&normal)));
                // Compute the unit tangent of the line formed by the normal plane and the z=posZ plane, keeping the w component at 1.0f
                XMStoreFloat4(&tangent, XMVector3Normalize(XMVectorSet(normal.y, -normal.x, 0.0f, 0.0f)) + g_XMIdentityR3);

                geoData.vertices[vIndex] = XMFLOAT3(posX, heightFunc(posX, posZ), posZ);
                geoData.normals[vIndex] = normal;
                geoData.tangents[vIndex] = tangent;
                geoData.texcoords[vIndex++] = XMFLOAT2(x * sliceTexWidth, maxTexCoord.y - z * sliceTexDepth);
            }
        }
        // Insert indices
        for (uint32_t i = 0; i < slices.y; ++i)
        {
            for (uint32_t j = 0; j < slices.x; ++j)
            {
                if (indexCount > 65535)
                {
                    geoData.indices32[iIndex++] = i * (slices.x + 1) + j;
                    geoData.indices32[iIndex++] = (i + 1) * (slices.x + 1) + j;
                    geoData.indices32[iIndex++] = (i + 1) * (slices.x + 1) + j + 1;

                    geoData.indices32[iIndex++] = (i + 1) * (slices.x + 1) + j + 1;
                    geoData.indices32[iIndex++] = i * (slices.x + 1) + j + 1;
                    geoData.indices32[iIndex++] = i * (slices.x + 1) + j;
                }
                else
                {
                    geoData.indices16[iIndex++] = i * (slices.x + 1) + j;
                    geoData.indices16[iIndex++] = (i + 1) * (slices.x + 1) + j;
                    geoData.indices16[iIndex++] = (i + 1) * (slices.x + 1) + j + 1;

                    geoData.indices16[iIndex++] = (i + 1) * (slices.x + 1) + j + 1;
                    geoData.indices16[iIndex++] = i * (slices.x + 1) + j + 1;
                    geoData.indices16[iIndex++] = i * (slices.x + 1) + j;
                }
            }
        }

        return geoData;
    }

    GeometryData CreateLine(const DirectX::XMFLOAT3 &from, const DirectX::XMFLOAT3 &to)
    {
        using namespace DirectX;

        GeometryData geoData;
        geoData.vertices = {from, to};
        geoData.normals = {XMFLOAT3(0.0f, 1.0f, 0.0f), XMFLOAT3(0.0f, 1.0f, 0.0f)};
        geoData.texcoords = {XMFLOAT2(0.0f, 0.0f), XMFLOAT2(0.0f, 0.0f)};
        geoData.indices16 = {0, 1};
        return geoData;
    }

    GeometryData CreateLineGrid(float size, float spacing)
    {
        using namespace DirectX;

        GeometryData geoData;
        int lineCount = static_cast<int>(size / spacing) + 1;
        float half = size / 2.0f;

        std::vector<uint32_t> indices;
        for (int i = 0; i < lineCount; ++i)
        {
            float offset = -half + i * spacing;

            uint32_t base = static_cast<uint32_t>(geoData.vertices.size());
            geoData.vertices.push_back(XMFLOAT3(-half, 0.0f, offset)); // line parallel to X axis
            geoData.vertices.push_back(XMFLOAT3(half, 0.0f, offset));
            indices.push_back(base);
            indices.push_back(base + 1);

            base = static_cast<uint32_t>(geoData.vertices.size());
            geoData.vertices.push_back(XMFLOAT3(offset, 0.0f, -half)); // line parallel to Z axis
            geoData.vertices.push_back(XMFLOAT3(offset, 0.0f, half));
            indices.push_back(base);
            indices.push_back(base + 1);
        }

        // RenderObject::Draw() picks the index buffer format (16 vs 32-bit) from indexCount alone,
        // so the populated array must match that same 65535 threshold or the GPU misreads the buffer.
        if (indices.size() > 65535)
            geoData.indices32 = std::move(indices);
        else
            geoData.indices16.assign(indices.begin(), indices.end());

        geoData.normals.resize(geoData.vertices.size(), XMFLOAT3(0.0f, 1.0f, 0.0f));
        geoData.texcoords.resize(geoData.vertices.size(), XMFLOAT2(0.0f, 0.0f));

        return geoData;
    }

    GeometryData CreateLineList(const std::vector<std::pair<DirectX::XMFLOAT3, DirectX::XMFLOAT3>> &segments)
    {
        using namespace DirectX;

        GeometryData geoData;
        if (segments.empty())
            return geoData;

        std::vector<uint32_t> indices;
        indices.reserve(segments.size() * 2);
        for (const auto &segment : segments)
        {
            uint32_t base = static_cast<uint32_t>(geoData.vertices.size());
            geoData.vertices.push_back(segment.first);
            geoData.vertices.push_back(segment.second);
            indices.push_back(base);
            indices.push_back(base + 1);
        }

        // RenderObject::Draw() picks the index buffer format (16 vs 32-bit) from indexCount alone,
        // so the populated array must match that same 65535 threshold or the GPU misreads the buffer.
        if (indices.size() > 65535)
            geoData.indices32 = std::move(indices);
        else
            geoData.indices16.assign(indices.begin(), indices.end());

        geoData.normals.resize(geoData.vertices.size(), XMFLOAT3(0.0f, 1.0f, 0.0f));
        geoData.texcoords.resize(geoData.vertices.size(), XMFLOAT2(0.0f, 0.0f));

        return geoData;
    }

    GeometryData CreatePolyline(const std::vector<DirectX::XMFLOAT3> &points)
    {
        using namespace DirectX;

        GeometryData geoData;
        if (points.size() < 2)
            return geoData;

        geoData.vertices = points;
        geoData.normals.assign(points.size(), XMFLOAT3(0.0f, 1.0f, 0.0f));
        geoData.texcoords.assign(points.size(), XMFLOAT2(0.0f, 0.0f));

        std::vector<uint32_t> indices;
        indices.reserve((points.size() - 1) * 2);
        for (uint32_t i = 0; i + 1 < points.size(); ++i)
        {
            indices.push_back(i);
            indices.push_back(i + 1);
        }

        // RenderObject::Draw() picks the index buffer format (16 vs 32-bit) from indexCount alone,
        // so the populated array must match that same 65535 threshold or the GPU misreads the buffer.
        if (indices.size() > 65535)
            geoData.indices32 = std::move(indices);
        else
            geoData.indices16.assign(indices.begin(), indices.end());

        return geoData;
    }

    GeometryData CreateQuad(const DirectX::XMFLOAT3 &p0, const DirectX::XMFLOAT3 &p1,
                            const DirectX::XMFLOAT3 &p2, const DirectX::XMFLOAT3 &p3)
    {
        using namespace DirectX;

        GeometryData geoData;
        geoData.vertices = {p0, p1, p2, p3};

        XMVECTOR normalVec = XMVector3Normalize(XMVector3Cross(
            XMVectorSubtract(XMLoadFloat3(&p1), XMLoadFloat3(&p0)),
            XMVectorSubtract(XMLoadFloat3(&p2), XMLoadFloat3(&p0))));
        XMFLOAT3 normal;
        XMStoreFloat3(&normal, normalVec);

        geoData.normals.assign(4, normal);
        geoData.tangents.assign(4, XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f));
        geoData.texcoords = {XMFLOAT2(0.0f, 1.0f), XMFLOAT2(0.0f, 0.0f), XMFLOAT2(1.0f, 0.0f), XMFLOAT2(1.0f, 1.0f)};
        geoData.indices16 = {0, 1, 2, 2, 3, 0};

        return geoData;
    }

    GeometryData CreateRibbon(const std::vector<DirectX::XMFLOAT3> &centerPoints, float width)
    {
        using namespace DirectX;

        GeometryData geoData;
        size_t n = centerPoints.size();
        if (n < 2)
            return geoData;

        float halfWidth = width * 0.5f;

        geoData.vertices.resize(n * 2);
        geoData.normals.resize(n * 2, XMFLOAT3(0.0f, 1.0f, 0.0f));
        geoData.tangents.resize(n * 2);
        geoData.texcoords.resize(n * 2);

        float distanceAccum = 0.0f;
        for (size_t i = 0; i < n; ++i)
        {
            // Central difference for interior points gives a smoother offset direction at joints;
            // endpoints fall back to the single adjacent segment.
            XMVECTOR dir;
            if (i == 0)
                dir = XMLoadFloat3(&centerPoints[1]) - XMLoadFloat3(&centerPoints[0]);
            else if (i == n - 1)
                dir = XMLoadFloat3(&centerPoints[n - 1]) - XMLoadFloat3(&centerPoints[n - 2]);
            else
                dir = XMLoadFloat3(&centerPoints[i + 1]) - XMLoadFloat3(&centerPoints[i - 1]);
            dir = XMVector3Normalize(dir);

            // Rotate the direction 90 degrees around Y to get the sideways offset in the XZ plane.
            XMFLOAT3 dirF;
            XMStoreFloat3(&dirF, dir);
            XMVECTOR side = XMVector3Normalize(XMVectorSet(dirF.z, 0.0f, -dirF.x, 0.0f)) * halfWidth;

            XMVECTOR center = XMLoadFloat3(&centerPoints[i]);
            XMVECTOR left = center - side;
            XMVECTOR right = center + side;

            if (i > 0)
            {
                XMVECTOR prev = XMLoadFloat3(&centerPoints[i - 1]);
                distanceAccum += XMVectorGetX(XMVector3Length(center - prev));
            }

            XMStoreFloat3(&geoData.vertices[i * 2 + 0], left);
            XMStoreFloat3(&geoData.vertices[i * 2 + 1], right);
            geoData.tangents[i * 2 + 0] = XMFLOAT4(dirF.x, dirF.y, dirF.z, 1.0f);
            geoData.tangents[i * 2 + 1] = XMFLOAT4(dirF.x, dirF.y, dirF.z, 1.0f);
            geoData.texcoords[i * 2 + 0] = XMFLOAT2(0.0f, distanceAccum);
            geoData.texcoords[i * 2 + 1] = XMFLOAT2(1.0f, distanceAccum);
        }

        // Winding matches CreatePlane: (left@i, left@i+1, right@i+1), (right@i+1, right@i, left@i)
        // so the ribbon faces up (+Y) just like a flat plane laid along +Z.
        std::vector<uint32_t> indices;
        indices.reserve((n - 1) * 6);
        for (size_t i = 0; i + 1 < n; ++i)
        {
            uint32_t l0 = static_cast<uint32_t>(i * 2 + 0);
            uint32_t r0 = static_cast<uint32_t>(i * 2 + 1);
            uint32_t l1 = static_cast<uint32_t>((i + 1) * 2 + 0);
            uint32_t r1 = static_cast<uint32_t>((i + 1) * 2 + 1);

            indices.push_back(l0);
            indices.push_back(l1);
            indices.push_back(r1);

            indices.push_back(r1);
            indices.push_back(r0);
            indices.push_back(l0);
        }

        if (indices.size() > 65535)
            geoData.indices32 = std::move(indices);
        else
            geoData.indices16.assign(indices.begin(), indices.end());

        return geoData;
    }

    GeometryData CreateDashedRibbon(const std::vector<DirectX::XMFLOAT3> &centerPoints, float width,
                                    float dashLength, float dashGap)
    {
        using namespace DirectX;

        GeometryData combined;
        if (centerPoints.size() < 2 || dashLength <= 0.0f || dashGap < 0.0f)
            return combined;

        std::vector<uint32_t> combinedIndices;

        // Appends one dash's ribbon (built via CreateRibbon) into the combined mesh,
        // offsetting its indices so all dashes end up in a single draw call.
        std::vector<XMFLOAT3> currentDash;
        auto flushDash = [&]()
        {
            if (currentDash.size() >= 2)
            {
                GeometryData seg = CreateRibbon(currentDash, width);
                uint32_t vertexOffset = static_cast<uint32_t>(combined.vertices.size());

                combined.vertices.insert(combined.vertices.end(), seg.vertices.begin(), seg.vertices.end());
                combined.normals.insert(combined.normals.end(), seg.normals.begin(), seg.normals.end());
                combined.tangents.insert(combined.tangents.end(), seg.tangents.begin(), seg.tangents.end());
                combined.texcoords.insert(combined.texcoords.end(), seg.texcoords.begin(), seg.texcoords.end());

                if (!seg.indices32.empty())
                {
                    for (uint32_t idx : seg.indices32)
                        combinedIndices.push_back(idx + vertexOffset);
                }
                else
                {
                    for (uint16_t idx : seg.indices16)
                        combinedIndices.push_back(idx + vertexOffset);
                }
            }
            currentDash.clear();
        };

        // Walk the centerline by arc length, alternating "on" (dash) / "off" (gap) phases of
        // fixed length; each "on" phase becomes its own sub-polyline fed through CreateRibbon.
        bool inDash = true;
        float phaseDistance = 0.0f;
        currentDash.push_back(centerPoints[0]);

        for (size_t i = 0; i + 1 < centerPoints.size(); ++i)
        {
            XMVECTOR a = XMLoadFloat3(&centerPoints[i]);
            XMVECTOR b = XMLoadFloat3(&centerPoints[i + 1]);
            float segLen = XMVectorGetX(XMVector3Length(b - a));
            if (segLen <= 1e-6f)
                continue;

            float segTraveled = 0.0f;
            while (segTraveled < segLen)
            {
                float phaseLimit = inDash ? dashLength : dashGap;
                float remainingInPhase = phaseLimit - phaseDistance;
                float remainingInSeg = segLen - segTraveled;
                float step = std::min(remainingInPhase, remainingInSeg);

                segTraveled += step;
                phaseDistance += step;

                XMVECTOR pt = a + (b - a) * (segTraveled / segLen);
                XMFLOAT3 ptF;
                XMStoreFloat3(&ptF, pt);

                if (inDash)
                    currentDash.push_back(ptF);

                if (phaseDistance >= phaseLimit - 1e-5f)
                {
                    if (inDash)
                        flushDash();
                    inDash = !inDash;
                    phaseDistance = 0.0f;
                    if (inDash)
                        currentDash.push_back(ptF);
                }
            }
        }
        flushDash(); // line may end mid-dash

        if (combinedIndices.size() > 65535)
            combined.indices32 = std::move(combinedIndices);
        else
            combined.indices16.assign(combinedIndices.begin(), combinedIndices.end());

        return combined;
    }

}
