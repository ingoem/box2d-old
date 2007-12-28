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

#include "b2Body.h"
#include "b2World.h"
#include "Joints/b2Joint.h"
//#include "Contacts/b2Contact.h"
#include "../Collision/Shapes/b2Shape.h"

b2Body::b2Body(const b2BodyDef* bd, b2World* world)
{
	m_flags = 0;
	if (bd->isBullet)
	{
		m_flags |= e_bulletFlag;
	}
	m_xf.position = bd->position;
	m_angle = bd->rotation;
	m_xf.R.Set(m_angle);
	m_position0 = m_xf.position;
	m_angle0 = m_angle;
	m_world = world;

	m_jointList = NULL;
	m_contactList = NULL;
	m_prev = NULL;
	m_next = NULL;

	m_linearDamping = b2Clamp(1.0f - bd->linearDamping, 0.0f, 1.0f);
	m_angularDamping = b2Clamp(1.0f - bd->angularDamping, 0.0f, 1.0f);

	m_force.Set(0.0f, 0.0f);
	m_torque = 0.0f;

	// Compute the center of mass velocity.
	m_linearVelocity.SetZero();
	m_angularVelocity = 0.0f;

	m_sleepTime = 0.0f;
	if (bd->allowSleep)
	{
		m_flags |= e_allowSleepFlag;
	}
	if (bd->isSleeping)
	{
		m_flags |= e_sleepFlag;
	}

	m_userData = bd->userData;

	// Compute the total mass and COM.
	m_shapeList = bd->shapes;
	m_shapeCount = 0;
	m_mass = 0.0f;
	m_I = 0.0f;
	m_center.SetZero();
	for (b2Shape* s = m_shapeList; s; s = s->m_bodyNext)
	{
		b2MassData massData;
		s->ComputeMass(&massData);
		m_mass += massData.mass;
		m_center += massData.mass * massData.center;
		m_I += massData.I;
		++m_shapeCount;
	}

	// Compute center of mass, and shift the origin to the COM.
	b2Vec2 offset;
	if (m_mass > 0.0f)
	{
		m_center *= 1.0f / m_mass;
		m_xf.position += b2Mul(m_xf.R, m_center);

		offset = -m_center;
		m_I -= m_mass * b2Dot(m_center, m_center);
		b2Assert(m_I > 0.0f);

		m_invMass = 1.0f / m_mass;
		m_invI = 1.0f / m_I;
	}
	else
	{
		offset = b2Vec2_zero;
		m_flags |= e_staticFlag;
		m_invMass = 0.0f;
		m_invI = 0.0f;
	}

	if (bd->preventRotation == true)
	{
		m_I = 0.0f;
		m_invI = 0.0f;
	}

	// Attach shapes and add them to the broad-phase.
	// Careful: this invokes user callbacks. So this
	// should go last in the body constructor.
	for (b2Shape* s = m_shapeList; s; s = s->m_bodyNext)
	{
		s->Attach(this, offset);
		s->CreateProxy(m_world->m_broadPhase, m_xf);
	}
}

b2Body::~b2Body()
{
	// shapes and joints are destroyed in b2World::Destroy
}

void b2Body::AddShape(b2Shape* shape, bool recomputeMass)
{
	b2Assert(m_world->m_lock == false);
	if (m_world->m_lock == true)
	{
		return;
	}

	b2Assert(shape->m_body == NULL);
	b2Assert(shape->m_bodyNext == NULL);
	B2_NOT_USED(shape);
	B2_NOT_USED(recomputeMass);
	// TODO_ERIN
}

void b2Body::RemoveShape(b2Shape* shape, bool recomputeMass)
{
	b2Assert(m_world->m_lock == false);
	if (m_world->m_lock == true)
	{
		return;
	}

	B2_NOT_USED(shape);
	B2_NOT_USED(recomputeMass);
	// TODO_ERIN
}

void b2Body::RecomputeMass()
{
	// TODO_ERIN
}

bool b2Body::SetOriginPosition(const b2Vec2& position, float angle)
{
	b2Assert(m_world->m_lock == false);
	if (m_world->m_lock == true)
	{
		return true;
	}

	if (IsFrozen())
	{
		return false;
	}

	m_angle = angle;
	m_xf.R.Set(m_angle);
	m_xf.position = position + b2Mul(m_xf.R, m_center);

	m_position0 = m_xf.position;
	m_angle0 = m_angle;

	bool freeze = false;
	for (b2Shape* s = m_shapeList; s; s = s->m_bodyNext)
	{
		bool inRange = s->Synchronize(m_world->m_broadPhase, m_xf, m_xf);

		if (inRange == false)
		{
			freeze = true;
			break;
		}
	}

	if (freeze == true)
	{
		m_flags |= e_frozenFlag;
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0f;
		for (b2Shape* s = m_shapeList; s; s = s->m_bodyNext)
		{
			s->DestroyProxy(m_world->m_broadPhase);
		}

		// Failure
		return false;
	}

	// Success
	m_world->m_broadPhase->Commit();
	return true;
}

bool b2Body::SynchronizeShapes()
{
	b2XForm xf0(m_position0, b2Mat22(m_angle0));

	bool inRange = true;
	for (b2Shape* s = m_shapeList; s; s = s->m_bodyNext)
	{
		inRange = s->Synchronize(m_world->m_broadPhase, xf0, m_xf);
		if (inRange == false)
		{
			break;
		}
	}

	if (inRange == false)
	{
		m_flags |= e_frozenFlag;
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0f;
		for (b2Shape* s = m_shapeList; s; s = s->m_bodyNext)
		{
			s->DestroyProxy(m_world->m_broadPhase);
		}

		// Failure
		return false;
	}

	// Success
	return true;
}
