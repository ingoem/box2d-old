/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_POLYGON_SHAPE_H
#define B2_POLYGON_SHAPE_H

#include "b2Shape.h"

/// Convex polygon, vertices must be in CCW order.
struct b2PolygonDef : public b2ShapeDef
{
	b2PolygonDef()
	{
		type = e_polygonShape;
		vertexCount = 0;
	}

	/// Build vertices to represent a box.
	/// @param extents the half-width vector
	/// @param transform the local transform of the box
	void SetAsBox(const b2Vec2& extents, const b2XForm& transform);

	b2Vec2 vertices[b2_maxPolygonVertices];
	int32 vertexCount;
};


/// A convex polygon.
class b2PolygonShape : public b2Shape
{
public:
	/// @see b2Shape::TestPoint
	bool TestPoint(const b2XForm& transform, const b2Vec2& p) const;

	/// @see b2Shape::TestSegment
	bool TestSegment(	const b2XForm& transform,
		float32* lambda,
		b2Vec2* normal,
		const b2Segment& segment,
		float32 maxLambda) const;

	/// @see b2Shape::ComputeAABB
	void ComputeAABB(b2AABB* aabb, const b2XForm& transform) const;

	/// @see b2Shape::ComputeSweptAABB
	void ComputeSweptAABB(	b2AABB* aabb,
		const b2XForm& transform1,
		const b2XForm& transform2) const;

	/// @see b2Shape::ComputeMass
	void ComputeMass(b2MassData* massData) const;

	//--------------- Internals Below -------------------
	
	b2PolygonShape(const b2ShapeDef* def);

	void ApplyOffset(const b2Vec2& offset);

	b2Vec2 Centroid(const b2XForm& xf) const;
	b2Vec2 Support(const b2XForm& xf, const b2Vec2& d) const;

	// Local position of the polygon centroid.
	b2Vec2 m_centroid;

	b2OBB m_obb;

	b2Vec2 m_vertices[b2_maxPolygonVertices];
	b2Vec2 m_normals[b2_maxPolygonVertices];
	b2Vec2 m_coreVertices[b2_maxPolygonVertices];
	int32 m_vertexCount;
};

#endif
