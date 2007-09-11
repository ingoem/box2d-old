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

#include "b2Island.h"
#include "b2Body.h"
#include "b2World.h"
#include "Contacts/b2Contact.h"
#include "Contacts/b2ContactSolver.h"
#include "Joints/b2Joint.h"
#include "Engine/Common/b2StackAllocator.h"

b2Island::b2Island(int32 bodyCapacity, int32 contactCapacity, int32 jointCapacity, b2StackAllocator* allocator)
{
	m_bodyCapacity = bodyCapacity;
	m_contactCapacity = contactCapacity;
	m_jointCapacity	 = jointCapacity;
	m_bodyCount = 0;
	m_contactCount = 0;
	m_jointCount = 0;

	m_bodies = (b2Body**)allocator->Allocate(bodyCapacity * sizeof(b2Body*));
	m_contacts = (b2Contact**)allocator->Allocate(contactCapacity	 * sizeof(b2Contact*));
	m_joints = (b2Joint**)allocator->Allocate(jointCapacity * sizeof(b2Joint*));

	m_allocator = allocator;
}

b2Island::~b2Island()
{
	// Warning: the order should reverse the constructor order.
	m_allocator->Free(m_joints);
	m_allocator->Free(m_contacts);
	m_allocator->Free(m_bodies);
}

void b2Island::Clear()
{
	m_bodyCount = 0;
	m_contactCount = 0;
	m_jointCount = 0;
}

void b2Island::Solve(b2Vec2 gravity, int32 iterations, float32 dt)
{
	for (int32 i = 0; i < m_bodyCount; ++i)
	{
		b2Body* b = m_bodies[i];

		if (b->m_invMass == 0.0f)
			continue;

		b->m_linearVelocity += dt * (gravity + b->m_invMass * b->m_force);
		b->m_angularVelocity += dt * b->m_invI * b->m_torque;
	}

	float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

	b2ContactSolver contactSolver(m_contacts, m_contactCount, inv_dt, m_allocator);

	// Pre-solve
	contactSolver.PreSolve();

	for (int32 i = 0; i < m_jointCount; ++i)
	{
		m_joints[i]->PreSolve();
	}

	// Solve velocity constraints.
	for (int32 i = 0; i < iterations; ++i)
	{
		contactSolver.SolveVelocityConstraints();

		for (int32 j = 0; j < m_jointCount; ++j)
		{
			m_joints[j]->SolveVelocityConstraints(dt);
		}
	}

	// Integrate positions.
	for (int32 i = 0; i < m_bodyCount; ++i)
	{
		b2Body* b = m_bodies[i];

		b->m_position += dt * b->m_linearVelocity;
		b->m_rotation += dt * b->m_angularVelocity;

		// Prevent large rotations to keep sin/cos accurate.
		while (b->m_rotation < -20.0f * b2_pi)
		{
			b->m_rotation += 2.0f * b2_pi;
		}
		while (b->m_rotation > 20.0f * b2_pi)
		{
			b->m_rotation -= 2.0f * b2_pi;
		}

		b->m_R.Set(b->m_rotation);
	}

	// Solve position constraints.
	if (b2World::s_enablePositionCorrection)
	{
		for (m_positionIterations = 0; m_positionIterations < iterations; ++m_positionIterations)
		{
			bool contactsOkay = contactSolver.SolvePositionConstraints(b2_contactBaumgarte);

			bool jointsOkay = true;
			for (int i = 0; i < m_jointCount; ++i)
			{
				jointsOkay = m_joints[i]->SolvePositionConstraints();
			}

			if (contactsOkay && jointsOkay)
			{
				break;
			}
		}
	}

	// Post-solve.
	contactSolver.PostSolve();

	// Synchronize shapes and reset forces.
	for (int32 i = 0; i < m_bodyCount; ++i)
	{
		b2Body* b = m_bodies[i];
		b->SynchronizeShapes();
		b->m_force.Set(0.0f, 0.0f);
		b->m_torque = 0.0f;
	}
}

void b2Island::UpdateSleep(float32 dt)
{
	float32 minSleepTime = FLT_MAX;

	float32 linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
	float32 angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;

	for (int32 i = 0; i < m_bodyCount; ++i)
	{
		b2Body* b = m_bodies[i];
		if (b->m_invMass == 0.0f)
		{
			continue;
		}

		if (b->m_allowSleep == false)
		{
			b->m_sleepTime = 0.0f;
			minSleepTime = 0.0f;
		}

		if (b->m_allowSleep == false ||
			b->m_angularVelocity * b->m_angularVelocity > angTolSqr ||
			b2Dot(b->m_linearVelocity, b->m_linearVelocity) > linTolSqr)
		{
			b->m_sleepTime = 0.0f;
			minSleepTime = 0.0f;
		}
		else
		{
			b->m_sleepTime += dt;
			minSleepTime = b2Min(minSleepTime, b->m_sleepTime);
		}
	}

	if (minSleepTime >= b2_timeToSleep)
	{
		for (int32 i = 0; i < m_bodyCount; ++i)
		{
			b2Body* b = m_bodies[i];
			b->m_isSleeping = true;
		}
	}
}