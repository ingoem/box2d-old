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

#ifndef B2_SHAPE_H
#define B2_SHAPE_H

#include "Engine/Common/b2Math.h"

struct b2Body;

struct b2MassData
{
	float32 mass;
	b2Vec2 center;
	float32 I;
};

enum b2ShapeType
{
	e_unknownShape = -1,
	e_circleShape,
	e_boxShape,
	e_polyShape,
	e_meshShape,
	e_shapeTypeCount,
};

struct b2BoxData
{
	b2Vec2 m_extents;
};

struct b2CircleData
{
	float32 m_radius;
};

// Convex polygon, vertices must be in CCW order.
struct b2PolyData
{
	b2Vec2 m_vertices[b2_maxPolyVertices];
	int32 m_vertexCount;
};

struct b2ShapeDescription
{
	b2ShapeDescription()
	{
		type = e_unknownShape;
		localPosition.Set(0.0f, 0.0f);
		localRotation = 0.0f;
		friction = 0.2f;
		restitution = 0.0f;
		density = 0.0f;
	}

	void ComputeMass(b2MassData* massData) const;

	b2ShapeType type;
	b2Vec2 localPosition;
	float32 localRotation;
	float32 friction;
	float32 restitution;
	float32 density;

	union
	{
		b2BoxData box;
		b2CircleData circle;
		b2PolyData poly;
	};
};

// Shapes are created automatically when a body is created.
struct b2Shape
{
	static b2Shape* Create(	const b2ShapeDescription* description,
							b2Body* body, const b2Vec2& center,
							const b2MassData* massData);

	static void Destroy(b2Shape*& shape);

	b2Shape(const b2ShapeDescription* description, b2Body* body, const b2Vec2& center);
	virtual ~b2Shape();

	virtual void UpdateProxy() = 0;

	virtual bool TestPoint(const b2Vec2& p) = 0;

	b2ShapeType m_type;

	b2Body* m_body;
	uint16 m_proxyId;

	// Position in world
	b2Vec2 m_position;
	float32 m_rotation;
	b2Mat22 m_R;

	// Local position in parent body
	b2Vec2 m_localPosition;
	float32 m_localRotation;

	float32 m_friction;
	float32 m_restitution;

	b2Shape* m_next;
};

struct b2CircleShape : public b2Shape
{
	b2CircleShape(const b2ShapeDescription* description, b2Body* body, const b2Vec2& center);

	void UpdateProxy();
	bool TestPoint(const b2Vec2& p);

	float32 m_radius;
};

struct b2PolyShape : public b2Shape
{
	b2PolyShape(	const b2ShapeDescription* description, b2Body* body,
		const b2Vec2& center, const b2MassData* massData);

	void UpdateProxy();
	bool TestPoint(const b2Vec2& p);

	b2Vec2 m_extents;
	b2Vec2 m_vertices[b2_maxPolyVertices];
	int32 m_vertexCount;
	b2Vec2 m_normals[b2_maxPolyVertices];
	int32 m_next[b2_maxPolyVertices];
};

#endif
