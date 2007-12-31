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
#include "b2CircleShape.h"
#include "b2PolygonShape.h"
#include "../b2Collision.h"
#include "../b2Broadphase.h"
#include "../../Common/b2BlockAllocator.h"

#include <new>

b2Shape* b2Shape::Create(const b2ShapeDef* def, b2BlockAllocator* allocator)
{
	switch (def->type)
	{
	case e_circleShape:
		{
			void* mem = allocator->Allocate(sizeof(b2CircleShape));
			return new (mem) b2CircleShape(def);
		}

	case e_polygonShape:
		{
			void* mem = allocator->Allocate(sizeof(b2PolygonShape));
			return new (mem) b2PolygonShape(def);
		}

	default:
		b2Assert(false);
		return NULL;
	}
}

void b2Shape::Destroy(b2Shape* s, b2BlockAllocator* allocator)
{
	switch (s->m_type)
	{
	case e_circleShape:
		s->~b2Shape();
		allocator->Free(s, sizeof(b2CircleShape));
		break;

	case e_polygonShape:
		s->~b2Shape();
		allocator->Free(s, sizeof(b2PolygonShape));
		break;

	default:
		b2Assert(false);
	}
}

b2Shape::b2Shape(const b2ShapeDef* def)
{
	m_userData = def->userData;
	m_friction = def->friction;
	m_restitution = def->restitution;
	m_density = def->density;
	m_body = NULL;

	m_bodyNext = NULL;
	m_worldPrev = NULL;
	m_worldNext = NULL;

	m_proxyId = b2_nullProxy;
	m_minRadius = 0.0f;
	m_maxRadius = 0.0f;

	m_categoryBits = def->categoryBits;
	m_maskBits = def->maskBits;
	m_groupIndex = def->groupIndex;

	m_isSensor = def->isSensor;
}

b2Shape::~b2Shape()
{
	b2Assert(m_proxyId == b2_nullProxy);
}

void b2Shape::Attach(b2Body* body, const b2Vec2& offset)
{
	// Are you trying to reuse a shape or a body definition?
	b2Assert(m_body == NULL);
	m_body = body;
	ApplyOffset(offset);
}

void b2Shape::CreateProxy(b2BroadPhase* broadPhase, const b2XForm& transform)
{
	b2Assert(m_proxyId == b2_nullProxy);

	b2AABB aabb;
	ComputeAABB(&aabb, transform);

	bool inRange = broadPhase->InRange(aabb);

	// You are creating a shape outside the world box.
	b2Assert(inRange);

	if (inRange)
	{
		m_proxyId = broadPhase->CreateProxy(aabb, this);
	}
	else
	{
		m_proxyId = b2_nullProxy;
	}
}

void b2Shape::DestroyProxy(b2BroadPhase* broadPhase)
{
	if (m_proxyId != b2_nullProxy)
	{
		broadPhase->DestroyProxy(m_proxyId);
		m_proxyId = b2_nullProxy;
	}
}

bool b2Shape::Synchronize(b2BroadPhase* broadPhase, const b2XForm& transform1, const b2XForm& transform2)
{
	if (m_proxyId == b2_nullProxy)
	{	
		return false;
	}

	// Compute an AABB that covers the swept shape (may miss some rotation effect).
	b2AABB aabb;
	ComputeSweptAABB(&aabb, transform1, transform2);

	if (broadPhase->InRange(aabb))
	{
		broadPhase->MoveProxy(m_proxyId, aabb);
		return true;
	}
	else
	{
		return false;
	}
}

bool b2Shape::ResetProxy(b2BroadPhase* broadPhase, const b2XForm& transform)
{
	if (m_proxyId == b2_nullProxy)
	{	
		return false;
	}

	broadPhase->DestroyProxy(m_proxyId);

	b2AABB aabb;
	ComputeAABB(&aabb, transform);

	if (broadPhase->InRange(aabb))
	{
		m_proxyId = broadPhase->CreateProxy(aabb, this);
		return true;
	}
	else
	{
		m_proxyId = b2_nullProxy;
		return false;
	}
}