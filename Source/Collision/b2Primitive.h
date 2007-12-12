/*
* Copyright (c) 2007 Erin Catto http://www.gphysics.com
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

#ifndef B2_PRIMITIVE_H
#define B2_PRIMITIVE_H

#include "b2Collision.h"

struct b2AABB;

// Primitives are the basic geometric entities used in collision detection.

struct b2PointPrimitive
{
	b2Vec2 Centroid() const;
	b2Vec2 Support(const b2Vec2& d) const;
	b2Vec2 m_position;
};

struct b2SegmentPrimitive
{
	b2Vec2 Centroid() const;
	b2Vec2 Support(const b2Vec2& d) const;
	bool TestSegment(float32* lambda, b2Vec2* normal, const b2SegmentPrimitive& segment, float32 maxLambda);
	b2Vec2 m_p1;
	b2Vec2 m_p2;
};

struct b2CirclePrimitive
{
	bool TestPoint(const b2Vec2& p) const;
	bool TestSegment(float32* lambda, b2Vec2* normal, const b2SegmentPrimitive& segment, float32 maxLambda);
	void ComputeAABB(b2AABB* aabb) const;

	b2Vec2 m_position;
	float32 m_radius;
};

// A convex polygon.
struct b2PolygonPrimitive
{
	void Initialize(const b2Vec2& position, const b2Mat22& R, const b2Vec2* vertices, int32 vertexCount);
	b2Vec2 Centroid() const;
	b2Vec2 Support(const b2Vec2& d) const;
	bool TestPoint(const b2Vec2& p) const;
	bool TestSegment(float32* lambda, b2Vec2* normal, const b2SegmentPrimitive& segment, float32 maxLambda);
	void ComputeAABB(b2AABB* aabb) const;

	b2Mat22 m_R;
	b2Vec2 m_position;
	b2Vec2 m_centroid;	// centroid in local coordinates
	b2Vec2 m_vertices[b2_maxPolygonVertices];
	b2Vec2 m_coreVertices[b2_maxPolygonVertices];
	b2Vec2 m_normals[b2_maxPolygonVertices];
	int32 m_vertexCount;
	float32 m_minRadius;
	float32 m_maxRadius;
};

// ------------------------- Inline Functions ---------------------------------

inline b2Vec2 b2PointPrimitive::Centroid() const
{
	return m_position;
}

inline b2Vec2 b2PointPrimitive::Support(const b2Vec2& d) const
{
	NOT_USED(d);
	return m_position;
}

inline b2Vec2 b2SegmentPrimitive::Centroid() const
{
	return 0.5f * (m_p1 + m_p2);
}

inline b2Vec2 b2SegmentPrimitive::Support(const b2Vec2& d) const
{
	float32 value1 = b2Dot(d, m_p1);
	float32 value2 = b2Dot(d, m_p2);
	if (value1 > value2)
	{
		return m_p1;
	}
	else
	{
		return m_p2;
	}
}

inline bool b2CirclePrimitive::TestPoint(const b2Vec2& p) const
{
	b2Vec2 d = p - m_position;
	return b2Dot(d, d) <= m_radius * m_radius;
}

inline void b2CirclePrimitive::ComputeAABB(b2AABB* aabb) const
{
	aabb->minVertex.Set(m_position.x - m_radius, m_position.y - m_radius);
	aabb->maxVertex.Set(m_position.x + m_radius, m_position.y + m_radius);
}

inline b2Vec2 b2PolygonPrimitive::Centroid() const
{
	return m_position + b2Mul(m_R, m_centroid);
}

#endif
