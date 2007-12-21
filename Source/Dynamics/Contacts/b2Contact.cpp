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
	AddType(b2PolyContact::Create, b2PolyContact::Destroy, e_polygonShape, e_polygonShape);
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
		contact->m_shape1->m_body->WakeUp();
		contact->m_shape2->m_body->WakeUp();
	}

	b2ShapeType type1 = contact->m_shape1->m_type;
	b2ShapeType type2 = contact->m_shape2->m_type;

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

	Evaluate();

	if (listener)
	{
		b2SolverTweaks tweaks;
		tweaks.friction = m_friction;
		tweaks.restitution = m_restitution;
		tweaks.nonSolid = bool(m_flags & e_nonSolidFlag);
		if (oldCount == 0 && m_manifoldCount > 0)
		{
			listener->Begin(&tweaks, GetManifolds(), m_shape1, m_shape2);
		}
		else if (oldCount > 0 && m_manifoldCount == 0)
		{
			listener->End(m_shape1, m_shape2);
		}
		else if (m_manifoldCount > 0)
		{
			listener->Persist(&tweaks, GetManifolds(), m_shape1, m_shape2);
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

float32 b2Contact::ComputeTOI()
{
	return b2TimeOfImpact(m_shape1, m_shape2);
}
