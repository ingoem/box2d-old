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

#include "b2Contact.h"
#include "b2CircleContact.h"
#include "b2PolyAndCircleContact.h"
#include "b2PolyContact.h"
#include "b2ContactSolver.h"
#include "../../Collision/b2Collision.h"
#include "../../Collision/Shapes/b2Shape.h"
#include "../../Common/b2BlockAllocator.h"
#include "../../Dynamics/b2World.h"
#include "../../Dynamics/b2Body.h"

b2ContactRegister b2Contact::s_registers[e_shapeTypeCount][e_shapeTypeCount];
bool b2Contact::s_initialized = false;

void b2Contact::InitializeRegisters()
{
	AddType(b2CircleContact::Create, b2CircleContact::Destroy, e_circleShape, e_circleShape);
	AddType(b2PolyAndCircleContact::Create, b2PolyAndCircleContact::Destroy, e_polygonShape, e_circleShape);
	AddType(b2PolygonContact::Create, b2PolygonContact::Destroy, e_polygonShape, e_polygonShape);
}

void b2Contact::AddType(b2ContactCreateFcn* createFcn, b2ContactDestroyFcn* destoryFcn,
					  b2ShapeType type1, b2ShapeType type2)
{
	b2Assert(e_unknownShape < type1 && type1 < e_shapeTypeCount);
	b2Assert(e_unknownShape < type2 && type2 < e_shapeTypeCount);
	
	s_registers[type1][type2].createFcn = createFcn;
	s_registers[type1][type2].destroyFcn = destoryFcn;
	s_registers[type1][type2].primary = true;

	if (type1 != type2)
	{
		s_registers[type2][type1].createFcn = createFcn;
		s_registers[type2][type1].destroyFcn = destoryFcn;
		s_registers[type2][type1].primary = false;
	}
}

b2Contact* b2Contact::Create(b2Shape* shape1, b2Shape* shape2, b2BlockAllocator* allocator)
{
	if (s_initialized == false)
	{
		InitializeRegisters();
		s_initialized = true;
	}

	b2ShapeType type1 = shape1->m_type;
	b2ShapeType type2 = shape2->m_type;

	b2Assert(e_unknownShape < type1 && type1 < e_shapeTypeCount);
	b2Assert(e_unknownShape < type2 && type2 < e_shapeTypeCount);
	
	b2ContactCreateFcn* createFcn = s_registers[type1][type2].createFcn;
	if (createFcn)
	{
		if (s_registers[type1][type2].primary)
		{
			return createFcn(shape1, shape2, allocator);
		}
		else
		{
			b2Contact* c = createFcn(shape2, shape1, allocator);
			for (int32 i = 0; i < c->GetManifoldCount(); ++i)
			{
				b2Manifold* m = c->GetManifolds() + i;
				m->normal = -m->normal;
			}
			return c;
		}
	}
	else
	{
		return NULL;
	}
}

void b2Contact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
{
	b2Assert(s_initialized == true);

	if (contact->GetManifoldCount() > 0)
	{
		contact->GetShape1()->GetBody()->WakeUp();
		contact->GetShape2()->GetBody()->WakeUp();
	}

	b2ShapeType type1 = contact->GetShape1()->GetType();
	b2ShapeType type2 = contact->GetShape2()->GetType();

	b2Assert(e_unknownShape < type1 && type1 < e_shapeTypeCount);
	b2Assert(e_unknownShape < type2 && type2 < e_shapeTypeCount);

	b2ContactDestroyFcn* destroyFcn = s_registers[type1][type2].destroyFcn;
	destroyFcn(contact, allocator);
}

b2Contact::b2Contact(b2Shape* s1, b2Shape* s2)
{
	m_flags = 0;

	m_shape1 = s1;
	m_shape2 = s2;

	m_manifoldCount = 0;

	m_friction = sqrtf(m_shape1->m_friction * m_shape2->m_friction);
	m_restitution = b2Max(m_shape1->m_restitution, m_shape2->m_restitution);
	m_prev = NULL;
	m_next = NULL;

	m_node1.contact = NULL;
	m_node1.prev = NULL;
	m_node1.next = NULL;
	m_node1.other = NULL;

	m_node2.contact = NULL;
	m_node2.prev = NULL;
	m_node2.next = NULL;
	m_node2.other = NULL;
}

void b2Contact::Update(b2ContactListener* listener)
{
	int32 oldCount = m_manifoldCount;

	// The oldCount might be positive due to a TOI event. We should
	// still notify the user that contact has begun.
	bool toiBegin = (m_flags & e_toiBeginFlag) != 0;

	Evaluate();

	m_flags &= ~(e_beginFlag | e_persistFlag | e_endFlag | e_toiBeginFlag);
	if ((oldCount == 0 || toiBegin) && m_manifoldCount > 0)
	{
		m_flags |= e_beginFlag;
	}
	else if (oldCount > 0 && m_manifoldCount == 0)
	{
		m_flags |= e_endFlag;
	}
	else if (m_manifoldCount > 0)
	{
		m_flags |= e_persistFlag;
	}

	b2Body* body1 = m_shape1->GetBody();
	b2Body* body2 = m_shape2->GetBody();
	if (body1->IsStatic() || body1->IsBullet() || body2->IsStatic() || body2->IsBullet())
	{
		m_flags &= ~e_slowFlag;
	}
	else
	{
		m_flags |= e_slowFlag;
	}

	if (listener)
	{
		b2SolverTweaks tweaks;
		tweaks.friction = m_friction;
		tweaks.restitution = m_restitution;
		tweaks.nonSolid = bool(m_flags & e_nonSolidFlag);

		if (m_flags & e_beginFlag)
		{
			listener->Tweak(&tweaks, GetManifolds(), m_manifoldCount, m_shape1, m_shape2, true);
		}
		else if (m_flags & e_endFlag)
		{
			listener->End(m_shape1, m_shape2);
		}
		else if (m_flags & e_persistFlag)
		{
			listener->Tweak(&tweaks, GetManifolds(), m_manifoldCount, m_shape1, m_shape2, false);
		}

		m_friction = tweaks.friction;
		m_restitution = tweaks.restitution;
		if (tweaks.nonSolid)
		{
			m_flags |= e_nonSolidFlag;
		}
		else
		{
			m_flags &= ~e_nonSolidFlag;
		}
	}
}

float32 b2Contact::TimeOfImpact(b2ContactListener* listener)
{
	b2Body* b1 = m_shape1->GetBody();
	b2Body* b2 = m_shape2->GetBody();
	b2Sweep sweep1, sweep2;
	b1->GetSweep(&sweep1);
	b2->GetSweep(&sweep2);
	float32 toi1 = b1->GetTOI();
	float32 toi2 = b2->GetTOI();
	b2Vec2 point1, point2;

	// Use maxTOI as an early out of the TOI calculation.
	float32 maxTOI = b2Max(toi1, toi2);

	float32 toi = b2TimeOfImpact(&point1, &point2, m_shape1, sweep1, m_shape2, sweep2, maxTOI);
	if (toi > 0.0f && toi < maxTOI)
	{
		bool apply = true;
		if (listener)
		{
			b2XForm xf1 = sweep1.GetXForm(toi);
			b2XForm xf2 = sweep2.GetXForm(toi);
			apply = listener->TOI(m_shape1, xf1, point1, m_shape2, xf2, point2);
		}

		if (apply)
		{
			b1->SetTOI(toi);
			b2->SetTOI(toi);
		}
		else
		{
			toi = 1.0f;
		}

		return toi;
	}

	// The time of impact had an early out, so the result is meaningless.
	return 1.0f;
}

void b2Contact::ResolveTOI(b2StackAllocator* allocator)
{
	b2Body* b1 = m_shape1->GetBody();
	b2Body* b2 = m_shape2->GetBody();

	// Has either body already been resolved?
	bool resolved1 = (b1->m_flags & b2Body::e_toiResolvedFlag) != 0;
	bool resolved2 = (b2->m_flags & b2Body::e_toiResolvedFlag) != 0;
	b2Assert(resolved1 == false || resolved2 == false);

	// Use mass factors to avoid repositioning already resolved bodies.
	float32 factor1 = 1.0f, factor2 = 1.0f;
	if (resolved1)
	{
		factor1 = 0.0f;
	}

	if (resolved2)
	{
		factor2 = 0.0f;
	}

	b2Assert(b1->GetTOI() == b2->GetTOI());

	float32 toi = b1->GetTOI();

	b1->m_xf.position = (1.0f - toi) * b1->m_position0 + toi * b1->m_xf.position;
	b1->m_angle = (1.0f - toi) * b1->m_angle0 + toi * b1->m_angle;
	b1->m_xf.R.Set(b1->m_angle);

	b2->m_xf.position = (1.0f - toi) * b2->m_position0 + toi * b2->m_xf.position;
	b2->m_angle = (1.0f - toi) * b2->m_angle0 + toi * b2->m_angle;
	b2->m_xf.R.Set(b2->m_angle);

	int32 oldCount = m_manifoldCount;

	Evaluate();

	b2Assert(m_manifoldCount > 0);

	if (oldCount == 0)
	{
		m_flags |= e_toiBeginFlag;
	}

	b2Contact* contact = this;
	b2ContactSolver solver(&contact, 1, allocator);

	const int32 k_maxIterations = 100;
	const float32 k_toiBaumgarte = 0.5f;

	for (int32 i = 0; i < k_maxIterations; ++i)
	{
		bool success = solver.SolvePositionConstraints(k_toiBaumgarte, factor1, factor2);
		if (success)
		{
			break;
		}
	}

	// Mark the bodies as resolved.
	b1->m_flags |= b2Body::e_toiResolvedFlag;
	b1->m_position0 = b1->m_xf.position;
	b1->m_angle0 = b1->m_angle;

	b2->m_flags |= b2Body::e_toiResolvedFlag;
	b2->m_position0 = b2->m_xf.position;
	b2->m_angle0 = b2->m_angle;
}