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

#ifndef B2_COLLISION_H
#define B2_COLLISION_H

#include "../Common/b2Math.h"
#include <climits>

class b2Shape;
class b2CircleShape;
class b2PolygonShape;

const uint8 b2_nullFeature = UCHAR_MAX;

/// Contact ids to facilitate warm starting.
union b2ContactID
{
	struct Features
	{
		uint8 referenceFace;
		uint8 incidentEdge;
		uint8 incidentVertex;
		uint8 flip;
	} features;
	uint32 key;
};

struct b2ContactPoint
{
	b2Vec2 position;
	float32 separation;
	float32 normalImpulse;
	float32 tangentImpulse;
	b2ContactID id;
};

// A manifold for two touching convex shapes.
struct b2Manifold
{
	b2ContactPoint points[b2_maxManifoldPoints];
	b2Vec2 normal;
	int32 pointCount;
};

struct b2Segment
{
	bool TestSegment(float32* lambda, b2Vec2* normal, const b2Segment& segment, float32 maxLambda) const;

	b2Vec2 p1, p2;
};

struct b2AABB
{
	bool IsValid() const;

	b2Vec2 minVertex, maxVertex;
};

struct b2OBB
{
	b2Mat22 R;
	b2Vec2 center;
	b2Vec2 extents;
};

/// This describes the motion of a body/shape for TOI computation.
struct b2Sweep
{
	/// Get the transform at a specific time.
	/// @param toi the normalized time in [0,1].
	b2XForm GetXForm(float32 toi) const;

	b2Vec2 position, velocity;
	float32 theta, omega;
};

/// Compute the collision manifold between two circles.
void b2CollideCircles(b2Manifold* manifold,
					  const b2CircleShape* circle1, const b2XForm& xf1,
					  const b2CircleShape* circle2, const b2XForm& xf2);

/// Compute the collision manifold between a polygon and a circle.
void b2CollidePolygonAndCircle(b2Manifold* manifold,
							   const b2PolygonShape* polygon, const b2XForm& xf1,
							   const b2CircleShape* circle, const b2XForm& xf2);

/// Compute the collision manifold between two circles.
void b2CollidePolygons(b2Manifold* manifold,
					   const b2PolygonShape* polygon1, const b2XForm& xf1,
					   const b2PolygonShape* polygon2, const b2XForm& xf2);

/// Compute the distance between two shapes and the closest points.
/// @return the distance between the shapes or zero if they are overlapped/touching.
float32 b2Distance(b2Vec2* x1, b2Vec2* x2,
				   const b2Shape* shape1, const b2XForm& xf1,
				   const b2Shape* shape2, const b2XForm& xf2);

/// Compute the distance between two shapes and the closest points.
/// @return the fraction between [0,1] in which the shapes first touch.
/// t=0 means the shapes begin touching/overlapped, and t=1 means the shapes don't touch.
float32 b2TimeOfImpact(b2Vec2* point1, b2Vec2* point2,
					   const b2Shape* shape1, const b2Sweep& sweep1,
					   const b2Shape* shape2, const b2Sweep& sweep2,
					   float32 maxTOI);

inline bool b2AABB::IsValid() const
{
	b2Vec2 d = maxVertex - minVertex;
	bool valid = d.x >= 0.0f && d.y >= 0;
	valid = valid && minVertex.IsValid() && maxVertex.IsValid();
	return valid;
}

inline bool b2TestOverlap(const b2AABB& a, const b2AABB& b)
{
	b2Vec2 d1, d2;
	d1 = b.minVertex - a.maxVertex;
	d2 = a.minVertex - b.maxVertex;

	if (d1.x > 0.0f || d1.y > 0.0f)
		return false;

	if (d2.x > 0.0f || d2.y > 0.0f)
		return false;

	return true;
}

inline b2XForm b2Sweep::GetXForm(float32 toi) const
{
	b2XForm xf;
	xf.position = position + toi * velocity;
	xf.R.Set(theta + toi * omega);
	return xf;
}

#endif
