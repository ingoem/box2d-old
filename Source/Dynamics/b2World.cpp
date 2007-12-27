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

#include "b2World.h"
#include "b2Body.h"
#include "b2Island.h"
#include "Joints/b2PulleyJoint.h"
#include "Contacts/b2Contact.h"
#include "../Collision/b2Collision.h"
#include "../Collision/Shapes/b2CircleShape.h"
#include "../Collision/Shapes/b2PolygonShape.h"
#include <new>

int32 b2World::s_enablePositionCorrection = 1;
int32 b2World::s_enableWarmStarting = 1;

b2World::b2World(const b2AABB& worldAABB, const b2Vec2& gravity, bool doSleep)
{
	m_destructionListener = NULL;
	m_boundaryListener = NULL;
	m_contactFilter = &b2_defaultFilter;
	m_contactListener = NULL;
	m_debugDraw = NULL;

	m_shapeList = NULL;
	m_bodyList = NULL;
	m_contactList = NULL;
	m_jointList = NULL;

	m_shapeCount = 0;
	m_bodyCount = 0;
	m_contactCount = 0;
	m_jointCount = 0;

	m_allowSleep = doSleep;
	m_gravity = gravity;

	m_lock = false;

	m_contactManager.m_world = this;
	void* mem = b2Alloc(sizeof(b2BroadPhase));
	m_broadPhase = new (mem) b2BroadPhase(worldAABB, &m_contactManager);

	b2BodyDef bd;
	m_groundBody = Create(&bd);
}

b2World::~b2World()
{
	Destroy(m_groundBody);
	m_broadPhase->~b2BroadPhase();
	b2Free(m_broadPhase);
}

void b2World::SetListener(b2DestructionListener* listener)
{
	m_destructionListener = listener;
}

void b2World::SetListener(b2BoundaryListener* listener)
{
	m_boundaryListener = listener;
}

void b2World::SetFilter(b2ContactFilter* filter)
{
	m_contactFilter = filter;
}

void b2World::SetListener(b2ContactListener* listener)
{
	m_contactListener = listener;
}

void b2World::SetDebugDraw(b2DebugDraw* debugDraw)
{
	m_debugDraw = debugDraw;
}

b2Shape* b2World::Create(const b2ShapeDef* def)
{
	b2Assert(m_lock == false);
	if (m_lock == true)
	{
		return NULL;
	}

	b2Shape* s = b2Shape::Create(def, &m_blockAllocator);

	// Connect to doubly linked shape list.
	s->m_worldPrev = NULL;
	s->m_worldNext = m_shapeList;

	if (m_shapeList)
	{
		m_shapeList->m_worldPrev = s;
	}

	m_shapeList = s;
	
	++m_shapeCount;

	return s;
}

void b2World::Destroy(b2Shape* s)
{
	b2Assert(m_shapeCount > 0);
	b2Assert(m_lock == false);
	if (m_lock == true)
	{
		return;
	}

	// Remove from doubly linked list.
	if (s->m_worldPrev)
	{
		s->m_worldPrev->m_worldNext = s->m_worldNext;
	}

	if (s->m_worldNext)
	{
		s->m_worldNext->m_worldPrev = s->m_worldPrev;
	}

	if (s == m_shapeList)
	{
		m_shapeList = s->m_worldNext;
	}

	--m_shapeCount;

	if (s->m_body != NULL)
	{
		// Remove from the body's singly linked list.
		b2Assert(s->m_body->m_shapeCount > 0);
		b2Shape** node = &s->m_body->m_shapeList;
		bool found = false;
		while (*node != NULL)
		{
			if (*node == s)
			{
				*node = s->m_bodyNext;
				found = true;
				break;
			}

			node = &(*node)->m_bodyNext;
		}
		b2Assert(found);

		--s->m_body->m_shapeCount;
	}

	s->DestroyProxy(m_broadPhase);
	b2Shape::Destroy(s, &m_blockAllocator);
	s = NULL;
}

b2Body* b2World::Create(const b2BodyDef* def)
{
	b2Assert(m_lock == false);
	if (m_lock == true)
	{
		return NULL;
	}

	void* mem = m_blockAllocator.Allocate(sizeof(b2Body));
	b2Body* b = new (mem) b2Body(def, this);

	// Add to doubly linked list.
	b->m_prev = NULL;
	b->m_next = m_bodyList;
	if (m_bodyList)
	{
		m_bodyList->m_prev = b;
	}
	m_bodyList = b;
	++m_bodyCount;

	return b;
}

void b2World::Destroy(b2Body* b)
{
	b2Assert(m_bodyCount > 0);
	b2Assert(m_lock == false);
	if (m_lock == true)
	{
		return;
	}

	// Delete the attached joints.
	b2JointNode* jn = b->m_jointList;
	while (jn)
	{
		b2JointNode* jn0 = jn;
		jn = jn->next;

		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(jn0->joint);
		}

		Destroy(jn0->joint);
	}

	// Delete the attached shapes.
	b2Shape* s = b->m_shapeList;
	while (s)
	{
		b2Shape* s0 = s;
		s = s->m_bodyNext;

		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(s0);
		}

		Destroy(s0);
	}

	// Remove from normal body list.
	if (b->m_prev)
	{
		b->m_prev->m_next = b->m_next;
	}

	if (b->m_next)
	{
		b->m_next->m_prev = b->m_prev;
	}

	if (b == m_bodyList)
	{
		m_bodyList = b->m_next;
	}

	--m_bodyCount;
	b->~b2Body();
	m_blockAllocator.Free(b, sizeof(b2Body));
}

b2Joint* b2World::Create(const b2JointDef* def)
{
	b2Assert(m_lock == false);

	b2Joint* j = b2Joint::Create(def, &m_blockAllocator);

	// Connect to the world list.
	j->m_prev = NULL;
	j->m_next = m_jointList;
	if (m_jointList)
	{
		m_jointList->m_prev = j;
	}
	m_jointList = j;
	++m_jointCount;

	// Connect to the bodies' doubly linked lists.
	j->m_node1.joint = j;
	j->m_node1.other = j->m_body2;
	j->m_node1.prev = NULL;
	j->m_node1.next = j->m_body1->m_jointList;
	if (j->m_body1->m_jointList) j->m_body1->m_jointList->prev = &j->m_node1;
	j->m_body1->m_jointList = &j->m_node1;

	j->m_node2.joint = j;
	j->m_node2.other = j->m_body1;
	j->m_node2.prev = NULL;
	j->m_node2.next = j->m_body2->m_jointList;
	if (j->m_body2->m_jointList) j->m_body2->m_jointList->prev = &j->m_node2;
	j->m_body2->m_jointList = &j->m_node2;

	// If the joint prevents collisions, then reset collision filtering.
	if (def->collideConnected == false)
	{
		// Reset the proxies on the body with the minimum number of shapes.
		b2Body* b = def->body1->m_shapeCount < def->body2->m_shapeCount ? def->body1 : def->body2;
		for (b2Shape* s = b->m_shapeList; s; s = s->m_bodyNext)
		{
			s->ResetProxy(m_broadPhase, b->m_xf);
		}
	}

	return j;
}

void b2World::Destroy(b2Joint* j)
{
	b2Assert(m_lock == false);

	bool collideConnected = j->m_collideConnected;

	// Remove from the doubly linked list.
	if (j->m_prev)
	{
		j->m_prev->m_next = j->m_next;
	}

	if (j->m_next)
	{
		j->m_next->m_prev = j->m_prev;
	}

	if (j == m_jointList)
	{
		m_jointList = j->m_next;
	}

	// Disconnect from island graph.
	b2Body* body1 = j->m_body1;
	b2Body* body2 = j->m_body2;

	// Wake up connected bodies.
	body1->WakeUp();
	body2->WakeUp();

	// Remove from body 1.
	if (j->m_node1.prev)
	{
		j->m_node1.prev->next = j->m_node1.next;
	}

	if (j->m_node1.next)
	{
		j->m_node1.next->prev = j->m_node1.prev;
	}

	if (&j->m_node1 == body1->m_jointList)
	{
		body1->m_jointList = j->m_node1.next;
	}

	j->m_node1.prev = NULL;
	j->m_node1.next = NULL;

	// Remove from body 2
	if (j->m_node2.prev)
	{
		j->m_node2.prev->next = j->m_node2.next;
	}

	if (j->m_node2.next)
	{
		j->m_node2.next->prev = j->m_node2.prev;
	}

	if (&j->m_node2 == body2->m_jointList)
	{
		body2->m_jointList = j->m_node2.next;
	}

	j->m_node2.prev = NULL;
	j->m_node2.next = NULL;

	b2Joint::Destroy(j, &m_blockAllocator);

	b2Assert(m_jointCount > 0);
	--m_jointCount;

	// If the joint prevents collisions, then reset collision filtering.
	if (collideConnected == false)
	{
		// Reset the proxies on the body with the minimum number of shapes.
		b2Body* b = body1->m_shapeCount < body2->m_shapeCount ? body1 : body2;
		for (b2Shape* s = b->m_shapeList; s; s = s->m_bodyNext)
		{
			s->ResetProxy(m_broadPhase, b->m_xf);
		}
	}
}

void b2World::Integrate(const b2TimeStep& step)
{
	// Size the island for the worst case.
	b2Island island(m_bodyCount, m_contactCount, m_jointCount, this);

	// Clear all the island flags.
	for (b2Body* b = m_bodyList; b; b = b->m_next)
	{
		b->m_flags &= ~b2Body::e_islandFlag;
	}
	for (b2Contact* c = m_contactList; c; c = c->m_next)
	{
		c->m_flags &= ~b2Contact::e_islandFlag;
	}
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->m_islandFlag = false;
	}

	// Build and simulate all awake islands.
	int32 stackSize = m_bodyCount;
	b2Body** stack = (b2Body**)m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));
	for (b2Body* seed = m_bodyList; seed; seed = seed->m_next)
	{
		if (seed->m_flags & (b2Body::e_staticFlag | b2Body::e_islandFlag | b2Body::e_sleepFlag | b2Body::e_frozenFlag))
		{
			continue;
		}

		// Reset island and stack.
		island.Clear();
		int32 stackCount = 0;
		stack[stackCount++] = seed;
		seed->m_flags |= b2Body::e_islandFlag;

		// Perform a depth first search (DFS) on the constraint graph.
		while (stackCount > 0)
		{
			// Grab the next body off the stack and add it to the island.
			b2Body* b = stack[--stackCount];
			island.Add(b);

			// Make sure the body is awake.
			b->m_flags &= ~b2Body::e_sleepFlag;

			// To keep islands as small as possible, we don't
			// propagate islands across static bodies.
			if (b->m_flags & b2Body::e_staticFlag)
			{
				continue;
			}

			// Search all contacts connected to this body.
			for (b2ContactNode* cn = b->m_contactList; cn; cn = cn->next)
			{
				if (cn->contact->m_flags & b2Contact::e_islandFlag)
				{
					continue;
				}

				island.Add(cn->contact);
				cn->contact->m_flags |= b2Contact::e_islandFlag;

				b2Body* other = cn->other;
				if (other->m_flags & b2Body::e_islandFlag)
				{
					continue;
				}

				b2Assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->m_flags |= b2Body::e_islandFlag;
			}

			// Search all joints connect to this body.
			for (b2JointNode* jn = b->m_jointList; jn; jn = jn->next)
			{
				if (jn->joint->m_islandFlag == true)
				{
					continue;
				}

				island.Add(jn->joint);
				jn->joint->m_islandFlag = true;

				b2Body* other = jn->other;
				if (other->m_flags & b2Body::e_islandFlag)
				{
					continue;
				}

				b2Assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->m_flags |= b2Body::e_islandFlag;
			}
		}

		island.Integrate(step, m_gravity);

		// Post solve cleanup.
		for (int32 i = 0; i < island.m_bodyCount; ++i)
		{
			// Allow static bodies to participate in other islands.
			b2Body* b = island.m_bodies[i];
			if (b->m_flags & b2Body::e_staticFlag)
			{
				b->m_flags &= ~b2Body::e_islandFlag;
			}
		}
	}

	m_stackAllocator.Free(stack);

	// Synchronize shapes.
	b2Body* b = m_bodyList;
	while (b)
	{
		uint32 skipFlags = b2Body::e_sleepFlag | b2Body::e_staticFlag | b2Body::e_frozenFlag;
		if (b->m_flags & skipFlags)
		{
			b = b->GetNext();
			continue;
		}
		
		// Update shapes (for broad-phase). If the shapes go out of
		// the world AABB then shapes and contacts may be destroyed,
		// including contacts that are
		bool inRange = b->SynchronizeShapes();

		// Did the body's shapes leave the world?
		if (inRange == false && m_boundaryListener != NULL)
		{
			b2BoundaryListener::Response response = m_boundaryListener->Violation(b);
			if (response == b2BoundaryListener::e_destroyBody)
			{
				b2Body* bNuke = b;
				b = b->GetNext();
				Destroy(bNuke);
				continue;
			}
		}

		b = b->GetNext();
	}

	// Commit shape proxy movements to the broad-phase so that new contacts are created.
	m_broadPhase->Commit();
}

void b2World::SolvePositionConstraints(const b2TimeStep& step)
{
	m_positionIterationCount = 0;
	if (step.dt == 0.0f)
	{
		return;
	}

	// Size the island for the worst case.
	b2Island island(m_bodyCount, m_contactCount, m_jointCount, this);

	// Clear all the island flags.
	for (b2Body* b = m_bodyList; b; b = b->m_next)
	{
		b->m_flags &= ~b2Body::e_islandFlag;
	}
	for (b2Contact* c = m_contactList; c; c = c->m_next)
	{
		c->m_flags &= ~b2Contact::e_islandFlag;
	}
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->m_islandFlag = false;
	}

	// Build and simulate all awake islands.
	int32 stackSize = m_bodyCount;
	b2Body** stack = (b2Body**)m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));
	for (b2Body* seed = m_bodyList; seed; seed = seed->m_next)
	{
		if (seed->m_flags & (b2Body::e_staticFlag | b2Body::e_islandFlag | b2Body::e_sleepFlag | b2Body::e_frozenFlag))
		{
			continue;
		}

		// Reset island and stack.
		island.Clear();
		int32 stackCount = 0;
		stack[stackCount++] = seed;
		seed->m_flags |= b2Body::e_islandFlag;

		// Perform a depth first search (DFS) on the constraint graph.
		while (stackCount > 0)
		{
			// Grab the next body off the stack and add it to the island.
			b2Body* b = stack[--stackCount];
			island.Add(b);

			// Make sure the body is awake.
			b->m_flags &= ~b2Body::e_sleepFlag;

			// To keep islands as small as possible, we don't
			// propagate islands across static bodies.
			if (b->m_flags & b2Body::e_staticFlag)
			{
				continue;
			}

			// Search all contacts connected to this body.
			for (b2ContactNode* cn = b->m_contactList; cn; cn = cn->next)
			{
				if (cn->contact->m_flags & b2Contact::e_islandFlag)
				{
					continue;
				}

				island.Add(cn->contact);
				cn->contact->m_flags |= b2Contact::e_islandFlag;

				b2Body* other = cn->other;
				if (other->m_flags & b2Body::e_islandFlag)
				{
					continue;
				}

				b2Assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->m_flags |= b2Body::e_islandFlag;
			}

			// Search all joints connect to this body.
			for (b2JointNode* jn = b->m_jointList; jn; jn = jn->next)
			{
				if (jn->joint->m_islandFlag == true)
				{
					continue;
				}

				island.Add(jn->joint);
				jn->joint->m_islandFlag = true;

				b2Body* other = jn->other;
				if (other->m_flags & b2Body::e_islandFlag)
				{
					continue;
				}

				b2Assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->m_flags |= b2Body::e_islandFlag;
			}
		}

		island.SolvePositionConstraints(step);

		m_positionIterationCount = b2Max(m_positionIterationCount, island.m_positionIterationCount);

		if (m_allowSleep)
		{
			island.UpdateSleep(step);
		}

		// Post solve cleanup.
		for (int32 i = 0; i < island.m_bodyCount; ++i)
		{
			// Allow static bodies to participate in other islands.
			b2Body* b = island.m_bodies[i];
			if (b->m_flags & b2Body::e_staticFlag)
			{
				b->m_flags &= ~b2Body::e_islandFlag;
			}
		}
	}

	m_stackAllocator.Free(stack);
}

void b2World::Step(float32 dt, int32 iterations)
{
	b2TimeStep step;
	step.dt = dt;
	step.iterations	= iterations;
	if (dt > 0.0f)
	{
		step.inv_dt = 1.0f / dt;
	}
	else
	{
		step.inv_dt = 0.0f;
	}
	
	// Integrate velocities, solve velocity constraints, and integrate positions.
	Integrate(step);

	// Update contacts.
	m_contactManager.Collide(step);

	// Project positions onto the constraint manifold.
	if (s_enablePositionCorrection)
	{
		SolvePositionConstraints(step);
	}

	DebugDraw();
}

int32 b2World::Query(const b2AABB& aabb, b2Shape** shapes, int32 maxCount)
{
	void** results = (void**)m_stackAllocator.Allocate(maxCount * sizeof(void*));

	int32 count = m_broadPhase->Query(aabb, results, maxCount);

	for (int32 i = 0; i < count; ++i)
	{
		shapes[i] = (b2Shape*)results[i];
	}

	m_stackAllocator.Free(results);
	return count;
}

void b2World::DrawShape(b2Shape* shape, const b2XForm& xf, const b2Color& color, bool core)
{
	b2Color coreColor(0.9f, 0.6f, 0.6f);

	switch (shape->m_type)
	{
	case e_circleShape:
		{
			b2CircleShape* circle = (b2CircleShape*)shape;

			b2Vec2 center = b2Mul(xf, circle->GetLocalPosition());
			float32 radius = circle->GetRadius();
			b2Vec2 axis = xf.R.col1;

			m_debugDraw->DrawSolidCircle(center, radius, axis, color);

			if (core)
			{
				m_debugDraw->DrawCircle(center, radius - b2_toiSlop, coreColor);
			}
		}
		break;

	case e_polygonShape:
		{
			b2PolygonShape* poly = (b2PolygonShape*)shape;
			int32 vertexCount = poly->GetVertexCount();
			const b2Vec2* localVertices = poly->GetVertices();

			b2Assert(vertexCount < b2_maxPolygonVertices);
			b2Vec2 vertices[b2_maxPolygonVertices];

			for (int32 i = 0; i < vertexCount; ++i)
			{
				vertices[i] = b2Mul(xf, localVertices[i]);
			}

			m_debugDraw->DrawSolidPolygon(vertices, vertexCount, color);

			if (core)
			{
				const b2Vec2* localCoreVertices = poly->GetCoreVertices();
				for (int32 i = 0; i < vertexCount; ++i)
				{
					vertices[i] = b2Mul(xf, localCoreVertices[i]);
				}
				m_debugDraw->DrawPolygon(vertices, vertexCount, coreColor);
			}
		}
		break;
	}
}

void b2World::DrawJoint(b2Joint* joint)
{
	b2Body* b1 = joint->GetBody1();
	b2Body* b2 = joint->GetBody2();
	const b2XForm& xf1 = b1->GetXForm();
	const b2XForm& xf2 = b2->GetXForm();
	b2Vec2 x1 = xf1.position;
	b2Vec2 x2 = xf2.position;
	b2Vec2 p1 = joint->GetAnchor1();
	b2Vec2 p2 = joint->GetAnchor2();

	b2Color color(0.5f, 0.8f, 0.8f);

	switch (joint->GetType())
	{
	case e_distanceJoint:
		m_debugDraw->DrawSegment(p1, p2, color);
		break;

	case e_pulleyJoint:
		{
			b2PulleyJoint* pulley = (b2PulleyJoint*)joint;
			b2Vec2 s1 = pulley->GetGroundAnchor1();
			b2Vec2 s2 = pulley->GetGroundAnchor2();
			m_debugDraw->DrawSegment(s1, p1, color);
			m_debugDraw->DrawSegment(s2, p2, color);
			m_debugDraw->DrawSegment(s1, s2, color);
		}
		break;

	case e_mouseJoint:
		// don't draw this
		break;

	default:
		m_debugDraw->DrawSegment(x1, p1, color);
		m_debugDraw->DrawSegment(p1, p2, color);
		m_debugDraw->DrawSegment(x2, p2, color);
	}
}

void b2World::DebugDraw()
{
	if (m_debugDraw == NULL)
	{
		return;
	}

	uint32 flags = m_debugDraw->GetFlags();

	if (flags & b2DebugDraw::e_shapeBit)
	{
		bool core = (flags & b2DebugDraw::e_coreShapeBit) == b2DebugDraw::e_coreShapeBit;

		for (b2Body* b = m_bodyList; b; b = b->GetNext())
		{
			const b2XForm& xf = b->GetXForm();
			for (b2Shape* s = b->GetShapeList(); s; s = s->GetBodyNext())
			{
				if (b->IsStatic())
				{
					DrawShape(s, xf, b2Color(0.5f, 0.9f, 0.5f), core);
				}
				else if (b->IsSleeping())
				{
					DrawShape(s, xf, b2Color(0.5f, 0.5f, 0.9f), core);
				}
				else
				{
					DrawShape(s, xf, b2Color(0.9f, 0.9f, 0.9f), core);
				}
			}
		}
	}

	if (flags & b2DebugDraw::e_jointBit)
	{
		for (b2Joint* j = m_jointList; j; j = j->GetNext())
		{
			if (j->GetType() != e_mouseJoint)
			{
				DrawJoint(j);
			}
		}
	}

	if (flags & b2DebugDraw::e_contactPointBit)
	{
		b2Color color(0.9f, 0.5f, 0.5f);
		for (b2Contact* c = m_contactList; c; c = c->GetNext())
		{
			b2Manifold* ms = c->GetManifolds();
			for (int32 i = 0; i < c->GetManifoldCount(); ++i)
			{
				b2Manifold* m = ms + i;
				for (int j = 0; j < m->pointCount; ++j)
				{
					m_debugDraw->DrawPoint(m->points[j].position, color);
				}
			}
		}
	}

	if (flags & b2DebugDraw::e_contactNormalBit)
	{
		b2Color color(0.4f, 0.9f, 0.4f);
		for (b2Contact* c = m_contactList; c; c = c->GetNext())
		{
			b2Manifold* ms = c->GetManifolds();
			for (int32 i = 0; i < c->GetManifoldCount(); ++i)
			{
				b2Manifold* m = ms + i;
				for (int j = 0; j < m->pointCount; ++j)
				{
					m_debugDraw->DrawAxis(m->points[j].position, m->normal, color);
				}
			}
		}
	}

	if (flags & b2DebugDraw::e_contactImpulseBit)
	{
		b2Color color(0.9f, 0.9f, 0.3f);
		for (b2Contact* c = m_contactList; c; c = c->GetNext())
		{
			b2Manifold* ms = c->GetManifolds();
			for (int32 i = 0; i < c->GetManifoldCount(); ++i)
			{
				b2Manifold* m = ms + i;
				for (int j = 0; j < m->pointCount; ++j)
				{
					m_debugDraw->DrawAxis(m->points[j].position, m->points[j].normalImpulse * m->normal, color);
				}
			}
		}
	}

	if (flags & b2DebugDraw::e_frictionImpulseBit)
	{
		b2Color color(0.9f, 0.9f, 0.3f);
		for (b2Contact* c = m_contactList; c; c = c->GetNext())
		{
			b2Manifold* ms = c->GetManifolds();
			for (int32 i = 0; i < c->GetManifoldCount(); ++i)
			{
				b2Manifold* m = ms + i;
				b2Vec2 tangent = b2Cross(m->normal, 1.0f);

				for (int j = 0; j < m->pointCount; ++j)
				{
					m_debugDraw->DrawAxis(m->points[j].position, m->points[j].tangentImpulse * tangent, color);
				}
			}
		}
	}

	if (flags & b2DebugDraw::e_pairBit)
	{
		b2BroadPhase* bp = m_broadPhase;
		b2Vec2 invQ;
		invQ.Set(1.0f / bp->m_quantizationFactor.x, 1.0f / bp->m_quantizationFactor.y);
		b2Color color(0.9f, 0.9f, 0.3f);

		for (int32 i = 0; i < b2_tableCapacity; ++i)
		{
			uint16 index = bp->m_pairManager.m_hashTable[i];
			while (index != b2_nullPair)
			{
				b2Pair* pair = bp->m_pairManager.m_pairs + index;
				b2Proxy* p1 = bp->m_proxyPool + pair->proxyId1;
				b2Proxy* p2 = bp->m_proxyPool + pair->proxyId2;

				b2AABB b1, b2;
				b1.minVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p1->lowerBounds[0]].value;
				b1.minVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p1->lowerBounds[1]].value;
				b1.maxVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p1->upperBounds[0]].value;
				b1.maxVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p1->upperBounds[1]].value;
				b2.minVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p2->lowerBounds[0]].value;
				b2.minVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p2->lowerBounds[1]].value;
				b2.maxVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p2->upperBounds[0]].value;
				b2.maxVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p2->upperBounds[1]].value;

				b2Vec2 x1 = 0.5f * (b1.minVertex + b1.maxVertex);
				b2Vec2 x2 = 0.5f * (b2.minVertex + b2.maxVertex);

				m_debugDraw->DrawSegment(x1, x2, color);

				index = pair->next;
			}
		}
	}

	if (flags & b2DebugDraw::e_aabbBit)
	{
		b2BroadPhase* bp = m_broadPhase;
		b2Vec2 invQ;
		invQ.Set(1.0f / bp->m_quantizationFactor.x, 1.0f / bp->m_quantizationFactor.y);
		b2Color color(0.9f, 0.3f, 0.9f);
		for (int32 i = 0; i < b2_maxProxies; ++i)
		{
			b2Proxy* p = bp->m_proxyPool + i;
			if (p->IsValid() == false)
			{
				continue;
			}

			b2AABB b;
			b.minVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p->lowerBounds[0]].value;
			b.minVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p->lowerBounds[1]].value;
			b.maxVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p->upperBounds[0]].value;
			b.maxVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p->upperBounds[1]].value;

			b2Vec2 vs[4];
			vs[0].Set(b.minVertex.x, b.minVertex.y);
			vs[1].Set(b.maxVertex.x, b.minVertex.y);
			vs[2].Set(b.maxVertex.x, b.maxVertex.y);
			vs[3].Set(b.minVertex.x, b.maxVertex.y);

			m_debugDraw->DrawPolygon(vs, 4, color);
		}
	}

	if (flags & b2DebugDraw::e_obbBit)
	{
		b2Color color(0.5f, 0.3f, 0.5f);

		for (b2Body* b = m_bodyList; b; b = b->GetNext())
		{
			const b2XForm& xf = b->GetXForm();
			for (b2Shape* s = b->GetShapeList(); s; s = s->GetBodyNext())
			{
				if (s->GetType() != e_polygonShape)
				{
					continue;
				}

				b2PolygonShape* poly = (b2PolygonShape*)s;
				const b2OBB& obb = poly->GetOBB();
				b2Vec2 h = obb.extents;
				b2Vec2 vs[4];
				vs[0].Set(-h.x, -h.y);
				vs[1].Set( h.x, -h.y);
				vs[2].Set( h.x,  h.y);
				vs[3].Set(-h.x,  h.y);

				for (int32 i = 0; i < 4; ++i)
				{
					vs[i] = obb.center + b2Mul(obb.R, vs[i]);
					vs[i] = b2Mul(xf, vs[i]);
				}

				m_debugDraw->DrawPolygon(vs, 4, color);
			}
		}
	}
}
