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

#include "b2Shape.h"
#include "Engine/Dynamics/b2Body.h"
#include "Engine/Dynamics/b2World.h"
#include "Engine/Common/b2BlockAllocator.h"

#include <new.h>

// Polygon mass, centroid, and inertia.
// Let rho be the polygon density in mass per unit area.
// Then:
// mass = rho * int(dA)
// centroid.x = (1/mass) * rho * int(x * dA)
// centroid.y = (1/mass) * rho * int(y * dA)
// I = rho * int((x*x + y*y) * dA)
//
// We can compute these integrals by summing all the integrals
// for each triangle of the polygon. To evaluate the integral
// for a single triangle, we make a change of variables to
// the (u,v) coordinates of the triangle:
// x = x0 + e1x * u + e2x * v
// y = y0 + e1y * u + e2y * v
// where 0 <= u && 0 <= v && u + v <= 1.
//
// We integrate u from [0,1-v] and then v from [0,1].
// We also need to use the Jacobian of the transformation:
// D = cross(e1, e2)
//
// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
//
// The rest of the derivation is handled by computer algebra.
static void PolyMass(b2MassData* massData, const b2Vec2* vs, int32 count, float32 rho)
{
	b2Assert(count >= 3);

	b2Vec2 center; center.Set(0.0f, 0.0f);
	float32 area = 0.0f;
	float32 I = 0.0f;

	b2Vec2 a; a.Set(0.0f, 0.0f);
#if 0
	for (int32 i = 0; i < count; ++i)
	{
		a += vs[i];
	}
	a *= 1.0f / count;
#endif

	const float32 inv3 = 1.0f / 3.0f;

	for (int32 i = 0; i < count; ++i)
	{
		// Triangle vertices.
		b2Vec2 p1 = a;
		b2Vec2 p2 = vs[i];
		b2Vec2 p3 = i + 1 < count ? vs[i+1] : vs[0];

		b2Vec2 e1 = p2 - p1;
		b2Vec2 e2 = p3 - p1;

		float32 D = b2Cross(e1, e2);

		float32 triangleArea = 0.5f * D;
		area += triangleArea;

		// Area weighted centroid
		center += triangleArea * inv3 * (p1 + p2 + p3);

		float32 px = p1.x, py = p1.y;
		float32 ex1 = e1.x, ey1 = e1.y;
		float32 ex2 = e2.x, ey2 = e2.y;

		float32 intx2 = inv3 * (0.25f * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5f*px*px;
		float32 inty2 = inv3 * (0.25f * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5f*py*py;

		I += D * (intx2 + inty2);
	}

	// Total mass
	massData->mass = rho * area;

	// Center of mass
	center *= 1.0f / area;
	massData->center = center;

	// Inertia tensor relative to the center.
	I = rho * (I - area * b2Dot(center, center));
	massData->I = I;
}

void b2ShapeDescription::ComputeMass(b2MassData* massData) const
{
	if (density == 0.0f)
	{
		massData->mass = 0.0f;
		massData->center.Set(0.0f, 0.0f);
		massData->I = 0.0f;
	}

	switch (type)
	{
	case e_circleShape:
		massData->mass = density * b2_pi * circle.m_radius * circle.m_radius;
		massData->center.Set(0.0f, 0.0f);
		massData->I = 0.5f * (massData->mass) * circle.m_radius * circle.m_radius;
		break;

	case e_boxShape:
		massData->mass = 4.0f * density * box.m_extents.x * box.m_extents.y;
		massData->center.Set(0.0f, 0.0f);
		massData->I = massData->mass / 3.0f * b2Dot(box.m_extents, box.m_extents);
		break;

	case e_polyShape:
		PolyMass(massData, poly.m_vertices, poly.m_vertexCount, density);
		break;

	default:
		massData->mass = 0.0f;
		massData->center.Set(0.0f, 0.0f);
		massData->I = 0.0f;
		break;
	}
}

b2Shape* b2Shape::Create(const b2ShapeDescription* description,
					 b2Body* body, const b2Vec2& center, const b2MassData* massData)
{
	switch (description->type)
	{
	case e_circleShape:
		{
			void* mem = body->m_world->m_blockAllocator.Allocate(sizeof(b2CircleShape));
			return new (mem) b2CircleShape(description, body, center);
		}

	case e_boxShape:
	case e_polyShape:
		{
			void* mem = body->m_world->m_blockAllocator.Allocate(sizeof(b2PolyShape));
			return new (mem) b2PolyShape(description, body, center, massData);
		}
	}

	b2Assert(false);
	return NULL;
}

void b2Shape::Destroy(b2Shape*& shape)
{
	b2BlockAllocator& allocator = shape->m_body->m_world->m_blockAllocator;

	switch (shape->m_type)
	{
	case e_circleShape:
		shape->~b2Shape();
		allocator.Free(shape, sizeof(b2CircleShape));
		break;

	case e_polyShape:
		shape->~b2Shape();
		allocator.Free(shape, sizeof(b2PolyShape));
		break;

	default:
		b2Assert(false);
	}

	shape = NULL;
}

b2Shape::b2Shape(const b2ShapeDescription* description, b2Body* body, const b2Vec2& center)
{
	m_localPosition = description->localPosition - center;
	m_localRotation = description->localRotation;
	m_friction = description->friction;
	m_restitution = description->restitution;
	m_body = body;

	m_position = m_body->m_position + b2Mul(m_body->m_R, m_localPosition);
	m_rotation = m_body->m_rotation + m_localRotation;
	m_R.Set(m_rotation);

	m_proxyId = b2_nullProxy;
}

b2Shape::~b2Shape()
{
	m_body->m_world->m_broadPhase->DestroyProxy(m_proxyId);
}

b2CircleShape::b2CircleShape(const b2ShapeDescription* description, b2Body* body, const b2Vec2& center)
: b2Shape(description, body, center)
{
	b2Assert(description->type == e_circleShape);
	m_type = e_circleShape;
	m_radius = description->circle.m_radius;

	b2AABB aabb;
	aabb.minVertex.Set(m_position.x - m_radius, m_position.y - m_radius);
	aabb.maxVertex.Set(m_position.x + m_radius, m_position.y + m_radius);

	m_body->m_world->m_broadPhase->CreateProxy(aabb, this);
}

void b2CircleShape::UpdateProxy()
{
	b2AABB aabb;
	aabb.minVertex.Set(m_position.x - m_radius, m_position.y - m_radius);
	aabb.maxVertex.Set(m_position.x + m_radius, m_position.y + m_radius);

	m_body->m_world->m_broadPhase->MoveProxy(m_proxyId, aabb);
}

bool b2CircleShape::TestPoint(const b2Vec2& p)
{
	b2Vec2 d = p - m_position;
	return b2Dot(d, d) <= m_radius * m_radius;
}

b2PolyShape::b2PolyShape(const b2ShapeDescription* description, b2Body* body,
					 const b2Vec2& center, const b2MassData* massData)
: b2Shape(description, body, center)
{
	b2Assert(description->type == e_boxShape || description->type == e_polyShape);
	m_type = e_polyShape;

	if (description->type == e_boxShape)
	{
		m_vertexCount = 4;
		b2Vec2 h = description->box.m_extents;
		m_vertices[0].Set(h.x, h.y);
		m_vertices[1].Set(-h.x, h.y);
		m_vertices[2].Set(-h.x, -h.y);
		m_vertices[3].Set(h.x, -h.y);
		m_normals[0].Set(0.0f, 1.0f);
		m_normals[1].Set(-1.0f, 0.0f);
		m_normals[2].Set(0.0f, -1.0f);
		m_normals[3].Set(1.0f, 0.0f);
		m_next[0] = 1;
		m_next[1] = 2;
		m_next[2] = 3;
		m_next[3] = 0;

		m_extents = h;
	}
	else
	{
		b2AABB aabb;
		aabb.minVertex.Set(FLT_MAX, FLT_MAX);
		aabb.maxVertex.Set(-FLT_MAX, -FLT_MAX);
		m_vertexCount = description->poly.m_vertexCount;
		b2Assert(3 <= m_vertexCount && m_vertexCount <= b2_maxPolyVertices);
		for (int32 i = 0; i < m_vertexCount; ++i)
		{
			m_vertices[i] = description->poly.m_vertices[i];

			aabb.minVertex = b2Min(aabb.minVertex, description->poly.m_vertices[i]);
			aabb.maxVertex = b2Max(aabb.maxVertex, description->poly.m_vertices[i]);
		}
		b2Vec2 offset = 0.5f * (aabb.minVertex + aabb.maxVertex);

		b2Assert(m_localRotation == 0.0f); // TODO_ERIN handle local rotation

		m_localPosition += offset;
		for (int32 i = 0; i < m_vertexCount; ++i)
		{
			// Shift the vertices so the shape position is the centroid.
			m_vertices[i] = description->poly.m_vertices[i] - offset;
			m_next[i] = i + 1 < m_vertexCount ? i + 1 : 0;
			b2Vec2 vNext = description->poly.m_vertices[m_next[i]] - offset;
			b2Vec2 edge = vNext - m_vertices[i];
			m_normals[i] = b2Cross(edge, 1.0f);
			m_normals[i].Normalize();
		}

		for (int32 i = 0; i < m_vertexCount; ++i)
		{
			// Ensure the polygon in convex.
			b2Assert(b2Cross(m_normals[i], m_normals[m_next[i]]) > 0.0f);
		}

		m_extents = 0.5f * (aabb.maxVertex - aabb.minVertex);
	}

	b2Mat22 absR = b2Abs(m_R);
	b2Vec2 h = b2Mul(absR, m_extents);
	b2AABB aabb;
	aabb.minVertex = m_position - h;
	aabb.maxVertex = m_position + h;
	m_proxyId = m_body->m_world->m_broadPhase->CreateProxy(aabb, this);
}

void b2PolyShape::UpdateProxy()
{
	b2Mat22 absR = b2Abs(m_R);
	b2Vec2 h = b2Mul(absR, m_extents);
	b2AABB aabb;
	aabb.minVertex = m_position - h;
	aabb.maxVertex = m_position + h;
	m_body->m_world->m_broadPhase->MoveProxy(m_proxyId, aabb);
}

bool b2PolyShape::TestPoint(const b2Vec2& p)
{
	b2Vec2 pLocal = b2MulT(m_R, p - m_position);

	for (int32 i = 0; i < m_vertexCount; ++i)
	{
		float32 dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
		if (dot > 0.0f)
		{
			return false;
		}
	}

	return true;
}