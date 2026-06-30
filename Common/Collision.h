//***************************************************************************************
// Collision.h by X_Jun(MKXJun) (C) 2018-2022 All Rights Reserved.
// Licensed under the MIT License.
//
// Provide encapsulated objects and collision detection methods
// Note: WireFrameData has not yet been fully tested; it may be moved to Geometry.h in the future
// Provide encapsulated collision classes and detection method.
//***************************************************************************************

#pragma once

#ifndef COLLISION_H
#define COLLISION_H

#include <DirectXCollision.h>
#include <vector>
#include "Vertex.h"
#include "Camera.h"


struct Ray
{
	Ray();
	Ray(const DirectX::XMFLOAT3& origin, const DirectX::XMFLOAT3& direction);

	static Ray ScreenToRay(const Camera& camera, float screenX, float screenY);

	bool Hit(const DirectX::BoundingBox& box, float* pOutDist = nullptr, float maxDist = FLT_MAX);
	bool Hit(const DirectX::BoundingOrientedBox& box, float* pOutDist = nullptr, float maxDist = FLT_MAX);
	bool Hit(const DirectX::BoundingSphere& sphere, float* pOutDist = nullptr, float maxDist = FLT_MAX);
	bool XM_CALLCONV Hit(DirectX::FXMVECTOR V0, DirectX::FXMVECTOR V1, DirectX::FXMVECTOR V2, float* pOutDist = nullptr, float maxDist = FLT_MAX);

	DirectX::XMFLOAT3 origin;		// Ray origin
	DirectX::XMFLOAT3 direction;	// Unit direction vector
};


class Collision
{
public:

	// Wireframe vertex/index arrays
	struct WireFrameData
	{
		std::vector<VertexPosColor> vertexVec;		// Vertex array
		std::vector<uint32_t> indexVec;				// Index array
	};

	//
	// Creation of bounding box wireframes
	//

	// Create AABB wireframe
	static WireFrameData CreateBoundingBox(const DirectX::BoundingBox& box, const DirectX::XMFLOAT4& color);
	// Create OBB wireframe
	static WireFrameData CreateBoundingOrientedBox(const DirectX::BoundingOrientedBox& box, const DirectX::XMFLOAT4& color);
	// Create bounding sphere wireframe
	static WireFrameData CreateBoundingSphere(const DirectX::BoundingSphere& sphere, const DirectX::XMFLOAT4& color, int slices = 20);
	// Create frustum wireframe
	static WireFrameData CreateBoundingFrustum(const DirectX::BoundingFrustum& frustum, const DirectX::XMFLOAT4& color);

	// Frustum culling
	static std::vector<Transform> XM_CALLCONV FrustumCulling(
		const std::vector<Transform>& transforms, const DirectX::BoundingBox& localBox, DirectX::FXMMATRIX View, DirectX::CXMMATRIX Proj);

    // Frustum culling
    static void XM_CALLCONV FrustumCulling(
        std::vector<Transform>& dest, const std::vector<Transform>& src, 
        const DirectX::BoundingBox& localBox, DirectX::FXMMATRIX View, DirectX::CXMMATRIX Proj);

private:
	static WireFrameData CreateFromCorners(const DirectX::XMFLOAT3(&corners)[8], const DirectX::XMFLOAT4& color);
};





#endif
