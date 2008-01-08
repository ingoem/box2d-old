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

#include "b2ContactSolver.h"
#include "b2Contact.h"
#include "../b2Body.h"
#include "../b2World.h"
#include "../../Common/b2StackAllocator.h"

b2ContactSolver::b2ContactSolver(b2Contact** contacts, int32 contactCount, b2StackAllocator* allocator)
{
	m_allocator = allocator;

	m_constraintCount = 0;
	for (int32 i = 0; i < contactCount; ++i)
	{
		b2Assert(contacts[i]->IsSolid());
		m_constraintCount += contacts[i]->GetManifoldCount();
	}

	m_constraints = (b2ContactConstraint*)m_allocator->Allocate(m_constraintCount * sizeof(b2ContactConstraint));

	int32 count = 0;
	for (int32 i = 0; i < contactCount; ++i)
	{
		b2Contact* contact = contacts[i];

		b2Body* b1 = contact->m_shape1->m_body;
		b2Body* b2 = contact->m_shape2->m_body;
		int32 manifoldCount = contact->GetManifoldCount();
		b2Manifold* manifolds = contact->GetManifolds();
		float32 friction = contact->m_friction;
		float32 restitution = contact->m_restitution;

		b2Vec2 v1 = b1->m_linearVelocity;
		b2Vec2 v2 = b2->m_linearVelocity;
		float32 w1 = b1->m_angularVelocity;
		float32 w2 = b2->m_angularVelocity;

		for (int32 j = 0; j < manifoldCount; ++j)
		{
			b2Manifold* manifold = manifolds + j;

			b2Assert(manifold->pointCount > 0);

			const b2Vec2 normal = manifold->normal;

			b2Assert(count < m_constraintCount);
			b2ContactConstraint* c = m_constraints + count;
			c->body1 = b1;
			c->body2 = b2;
			c->manifold = manifold;
			c->normal = normal;
			c->pointCount = manifold->pointCount;
			c->friction = friction;
			c->restitution = restitution;

			for (int32 k = 0; k < c->pointCount; ++k)
			{
				b2ContactPoint* cp = manifold->points + k;
				b2ContactConstraintPoint* ccp = c->points + k;

				ccp->normalImpulse = cp->normalImpulse;
				ccp->tangentImpulse = cp->tangentImpulse;
				ccp->separation = cp->separation;
				ccp->positionImpulse = 0.0f;

				b2Vec2 r1 = cp->position - b1->m_xf.position;
				b2Vec2 r2 = cp->position - b2->m_xf.position;

				ccp->localAnchor1 = b2MulT(b1->m_xf.R, r1);
				ccp->localAnchor2 = b2MulT(b2->m_xf.R, r2);

				float32 r1Sqr = b2Dot(r1, r1);
				float32 r2Sqr = b2Dot(r2, r2);
				float32 rn1 = b2Dot(r1, normal);
				float32 rn2 = b2Dot(r2, normal);

				float32 kNormal = b1->m_invMass + b2->m_invMass;
				kNormal += b1->m_invI * (r1Sqr - rn1 * rn1) + b2->m_invI * (r2Sqr - rn2 * rn2);
				b2Assert(kNormal > FLT_EPSILON);
				ccp->normalMass = 1.0f / kNormal;

				float32 kEqualized = b1->m_mass * b1->m_invMass + b2->m_mass * b2->m_invMass;
				kEqualized += b1->m_mass * b1->m_invI * (r1Sqr - rn1 * rn1) + b2->m_mass * b2->m_invI * (r2Sqr - rn2 * rn2);
				b2Assert(kEqualized > FLT_EPSILON);
				ccp->equalizedMass = 1.0f / kEqualized;

				b2Vec2 tangent = b2Cross(normal, 1.0f);

				float32 rt1 = b2Dot(r1, tangent);
				float32 rt2 = b2Dot(r2, tangent);
				float32 kTangent = b1->m_invMass + b2->m_invMass;
				kTangent += b1->m_invI * (r1Sqr - rt1 * rt1) + b2->m_invI * (r2Sqr - rt2 * rt2);
				b2Assert(kTangent > FLT_EPSILON);
				ccp->tangentMass = 1.0f /  kTangent;

				// Setup a velocity bias for restitution.
				ccp->velocityBias = 0.0f;
				if (ccp->separation > 0.0f)
				{
					ccp->velocityBias = -60.0f * ccp->separation; // TODO_ERIN b2TimeStep
				}

				float32 vRel = b2Dot(c->normal, v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1));
				if (vRel < -b2_velocityThreshold)
				{
					ccp->velocityBias += -c->restitution * vRel;
				}
			}

			++count;
		}
	}

	b2Assert(count == m_constraintCount);
}

b2ContactSolver::~b2ContactSolver()
{
	m_allocator->Free(m_constraints);
}

void b2ContactSolver::InitVelocityConstraints()
{
	// Warm start.
	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;

		b2Body* b1 = c->body1;
		b2Body* b2 = c->body2;
		float32 invMass1 = b1->m_invMass;
		float32 invI1 = b1->m_invI;
		float32 invMass2 = b2->m_invMass;
		float32 invI2 = b2->m_invI;
		b2Vec2 normal = c->normal;
		b2Vec2 tangent = b2Cross(normal, 1.0f);

		if (b2World::s_enableWarmStarting)
		{
			for (int32 j = 0; j < c->pointCount; ++j)
			{
				b2ContactConstraintPoint* ccp = c->points + j;
				b2Vec2 P = ccp->normalImpulse * normal + ccp->tangentImpulse * tangent;
				b2Vec2 r1 = b2Mul(b1->m_xf.R, ccp->localAnchor1);
				b2Vec2 r2 = b2Mul(b2->m_xf.R, ccp->localAnchor2);
				b1->m_angularVelocity -= invI1 * b2Cross(r1, P);
				b1->m_linearVelocity -= invMass1 * P;
				b2->m_angularVelocity += invI2 * b2Cross(r2, P);
				b2->m_linearVelocity += invMass2 * P;
			}
		}
		else
		{
			for (int32 j = 0; j < c->pointCount; ++j)
			{
				b2ContactConstraintPoint* ccp = c->points + j;
				ccp->normalImpulse = 0.0f;
				ccp->tangentImpulse = 0.0f;
			}
		}
	}
}

void b2ContactSolver::SolveVelocityConstraints()
{
	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;
		b2Body* b1 = c->body1;
		b2Body* b2 = c->body2;
		float32 invMass1 = b1->m_invMass;
		float32 invI1 = b1->m_invI;
		float32 invMass2 = b2->m_invMass;
		float32 invI2 = b2->m_invI;
		b2Vec2 normal = c->normal;
		b2Vec2 tangent = b2Cross(normal, 1.0f);

		// Solve normal constraints
		for (int32 j = 0; j < c->pointCount; ++j)
		{
			b2ContactConstraintPoint* ccp = c->points + j;

			b2Vec2 r1 = b2Mul(b1->m_xf.R, ccp->localAnchor1);
			b2Vec2 r2 = b2Mul(b2->m_xf.R, ccp->localAnchor2);

			// Relative velocity at contact
			b2Vec2 dv = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2) - b1->m_linearVelocity - b2Cross(b1->m_angularVelocity, r1);

			// Compute normal impulse
			float32 vn = b2Dot(dv, normal);
			float32 lambda = -ccp->normalMass * (vn - ccp->velocityBias);

			// b2Clamp the accumulated impulse
			float32 newImpulse = b2Max(ccp->normalImpulse + lambda, 0.0f);
			lambda = newImpulse - ccp->normalImpulse;

			// Apply contact impulse
			b2Vec2 P = lambda * normal;

			b1->m_linearVelocity -= invMass1 * P;
			b1->m_angularVelocity -= invI1 * b2Cross(r1, P);

			b2->m_linearVelocity += invMass2 * P;
			b2->m_angularVelocity += invI2 * b2Cross(r2, P);

			ccp->normalImpulse = newImpulse;
		}

		// Solve tangent constraints
		for (int32 j = 0; j < c->pointCount; ++j)
		{
			b2ContactConstraintPoint* ccp = c->points + j;

			b2Vec2 r1 = b2Mul(b1->m_xf.R, ccp->localAnchor1);
			b2Vec2 r2 = b2Mul(b2->m_xf.R, ccp->localAnchor2);

			// Relative velocity at contact
			b2Vec2 dv = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2) - b1->m_linearVelocity - b2Cross(b1->m_angularVelocity, r1);

			// Compute tangent impulse
			float32 vt = b2Dot(dv, tangent);
			float32 lambda = ccp->tangentMass * (-vt);

			// b2Clamp the accumulated impulse
			float32 maxFriction = c->friction * ccp->normalImpulse;
			float32 newImpulse = b2Clamp(ccp->tangentImpulse + lambda, -maxFriction, maxFriction);
			lambda = newImpulse - ccp->tangentImpulse;

			// Apply contact impulse
			b2Vec2 P = lambda * tangent;

			b1->m_linearVelocity -= invMass1 * P;
			b1->m_angularVelocity -= invI1 * b2Cross(r1, P);

			b2->m_linearVelocity += invMass2 * P;
			b2->m_angularVelocity += invI2 * b2Cross(r2, P);

			ccp->tangentImpulse = newImpulse;
		}
	}
}

void b2ContactSolver::FinalizeVelocityConstraints()
{
	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;
		b2Manifold* m = c->manifold;

		for (int32 j = 0; j < c->pointCount; ++j)
		{
			m->points[j].normalImpulse = c->points[j].normalImpulse;
			m->points[j].tangentImpulse = c->points[j].tangentImpulse;
		}
	}
}

bool b2ContactSolver::SolvePositionConstraints(float32 baumgarte)
{
	float32 minSeparation = 0.0f;

	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;
		b2Body* b1 = c->body1;
		b2Body* b2 = c->body2;
		float32 invMass1 = b1->m_mass * b1->m_invMass;
		float32 invI1 = b1->m_mass * b1->m_invI;
		float32 invMass2 = b2->m_mass * b2->m_invMass;
		float32 invI2 = b2->m_mass * b2->m_invI;
		
		b2Vec2 normal = c->normal;

		// Solver normal constraints
		for (int32 j = 0; j < c->pointCount; ++j)
		{
			b2ContactConstraintPoint* ccp = c->points + j;

			b2Vec2 r1 = b2Mul(b1->m_xf.R, ccp->localAnchor1);
			b2Vec2 r2 = b2Mul(b2->m_xf.R, ccp->localAnchor2);

			b2Vec2 p1 = b1->m_xf.position + r1;
			b2Vec2 p2 = b2->m_xf.position + r2;
			b2Vec2 dp = p2 - p1;

			// Approximate the current separation.
			float32 separation = b2Dot(dp, normal) + ccp->separation;

			// Track max constraint error.
			minSeparation = b2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			float32 C = baumgarte * b2Clamp(separation + b2_linearSlop, -b2_maxLinearCorrection, 0.0f);

			// Compute normal impulse
			float32 dImpulse = -ccp->equalizedMass * C;

			// b2Clamp the accumulated impulse
			float32 impulse0 = ccp->positionImpulse;
			ccp->positionImpulse = b2Max(impulse0 + dImpulse, 0.0f);
			dImpulse = ccp->positionImpulse - impulse0;

			b2Vec2 impulse = dImpulse * normal;

			b1->m_xf.position -= invMass1 * impulse;
			b1->m_angle -= invI1 * b2Cross(r1, impulse);
			b1->m_xf.R.Set(b1->m_angle);

			b2->m_xf.position += invMass2 * impulse;
			b2->m_angle += invI2 * b2Cross(r2, impulse);
			b2->m_xf.R.Set(b2->m_angle);
		}
	}

	return minSeparation >= -1.5f * b2_linearSlop;
}


void b2SolveContactPoint(b2TOIPoint* tp, b2Body* b1, b2Body* b2,
						 float32 restitution, float32 friction, float32 baumgarte)
{
	b2Vec2 r1 = tp->position - b1->m_xf.position;
	b2Vec2 r2 = tp->position - b2->m_xf.position;

	float32 r1Sqr = b2Dot(r1, r1);
	float32 r2Sqr = b2Dot(r2, r2);
	float32 rn1 = b2Dot(r1, tp->normal);
	float32 rn2 = b2Dot(r2, tp->normal);

	float32 kNormal = b1->m_invMass + b2->m_invMass;
	kNormal += b1->m_invI * (r1Sqr - rn1 * rn1) + b2->m_invI * (r2Sqr - rn2 * rn2);
	b2Assert(kNormal > FLT_EPSILON);
	float32 normalMass = 1.0f / kNormal;

	b2Vec2 tangent = b2Cross(tp->normal, 1.0f);

	float32 rt1 = b2Dot(r1, tangent);
	float32 rt2 = b2Dot(r2, tangent);
	float32 kTangent = b1->m_invMass + b2->m_invMass;
	kTangent += b1->m_invI * (r1Sqr - rt1 * rt1) + b2->m_invI * (r2Sqr - rt2 * rt2);
	b2Assert(kTangent > FLT_EPSILON);
	float32 tangentMass = 1.0f /  kTangent;

	// Setup a velocity bias for restitution.
	b2Vec2 v1 = b1->m_linearVelocity;
	float32 w1 = b1->m_angularVelocity;
	b2Vec2 v2 = b2->m_linearVelocity;
	float32 w2 = b2->m_angularVelocity;

	float32 velocityBias = -baumgarte * tp->separation;
	b2Vec2 dv = v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1);
	float32 vn = b2Dot(tp->normal, dv);
	if (vn < -b2_velocityThreshold)
	{
		velocityBias += -restitution * vn;
	}

	float32 invMass1 = b1->m_invMass;
	float32 invI1 = b1->m_invI;
	float32 invMass2 = b2->m_invMass;
	float32 invI2 = b2->m_invI;

	tp->normalImpulse = b2Max(0.0f, -normalMass * (vn - velocityBias));

	// Apply contact impulse
	b2Vec2 P = tp->normalImpulse * tp->normal;

	v1 -= invMass1 * P;
	w1 -= invI1 * b2Cross(r1, P);

	v2 += invMass2 * P;
	w2 += invI2 * b2Cross(r2, P);

	// Relative velocity at contact
	dv = v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1);

	// Compute tangent impulse
	float32 vt = b2Dot(dv, tangent);
	tp->tangentImpulse = tangentMass * (-vt);

	// b2Clamp the accumulated impulse
	float32 maxFriction = friction * tp->normalImpulse;
	tp->tangentImpulse = b2Clamp(tp->tangentImpulse, -maxFriction, maxFriction);

	// Apply contact impulse
	P = tp->tangentImpulse * tangent;

	v1 -= invMass1 * P;
	w1 -= invI1 * b2Cross(r1, P);

	v2 += invMass2 * P;
	w2 += invI2 * b2Cross(r2, P);

	b1->m_linearVelocity = v1;
	b1->m_angularVelocity = w1;

	b2->m_linearVelocity = v2;
	b2->m_angularVelocity = w2;
}
