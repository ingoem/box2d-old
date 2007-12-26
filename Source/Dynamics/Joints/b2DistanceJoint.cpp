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

#include "b2DistanceJoint.h"
#include "../b2Body.h"
#include "../b2World.h"

void b2DistanceJointDef::Build(	const b2XForm& xf1, const b2XForm& xf2,
								const b2Vec2& anchor1, const b2Vec2& anchor2)
{
	localAnchor1 = b2MulT(xf1, anchor1);
	localAnchor2 = b2MulT(xf2, anchor2);
	b2Vec2 d = anchor2 - anchor1;
	length = d.Length();
}

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2


b2DistanceJoint::b2DistanceJoint(const b2DistanceJointDef* def)
: b2Joint(def)
{
	m_localAnchor1 = def->localAnchor1;
	m_localAnchor2 = def->localAnchor2;
	m_length = def->length;
	m_impulse = 0.0f;
}

void b2DistanceJoint::InitVelocityConstraints()
{
	// Compute the effective mass matrix.
	b2Vec2 r1 = b2Mul(m_body1->m_xf.R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(m_body2->m_xf.R, m_localAnchor2);
	m_u = m_body2->m_xf.position + r2 - m_body1->m_xf.position - r1;

	// Handle singularity.
	float32 length = m_u.Length();
	if (length > b2_linearSlop)
	{
		m_u *= 1.0f / length;
	}
	else
	{
		m_u.Set(0.0f, 0.0f);
	}

	float32 cr1u = b2Cross(r1, m_u);
	float32 cr2u = b2Cross(r2, m_u);
	m_mass = m_body1->m_invMass + m_body1->m_invI * cr1u * cr1u + m_body2->m_invMass + m_body2->m_invI * cr2u * cr2u;
	b2Assert(m_mass > FLT_EPSILON);
	m_mass = 1.0f / m_mass;

	if (b2World::s_enableWarmStarting)
	{
		b2Vec2 P = m_impulse * m_u;
		m_body1->m_linearVelocity -= m_body1->m_invMass * P;
		m_body1->m_angularVelocity -= m_body1->m_invI * b2Cross(r1, P);
		m_body2->m_linearVelocity += m_body2->m_invMass * P;
		m_body2->m_angularVelocity += m_body2->m_invI * b2Cross(r2, P);
	}
	else
	{
		m_impulse = 0.0f;
	}
}

void b2DistanceJoint::SolveVelocityConstraints(const b2TimeStep& step)
{
	B2_NOT_USED(step);

	b2Vec2 r1 = b2Mul(m_body1->m_xf.R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(m_body2->m_xf.R, m_localAnchor2);

	// Cdot = dot(u, v + cross(w, r))
	b2Vec2 v1 = m_body1->m_linearVelocity + b2Cross(m_body1->m_angularVelocity, r1);
	b2Vec2 v2 = m_body2->m_linearVelocity + b2Cross(m_body2->m_angularVelocity, r2);
	float32 Cdot = b2Dot(m_u, v2 - v1);
	float32 impulse = -m_mass * Cdot;
	m_impulse += impulse;

	b2Vec2 P = impulse * m_u;
	m_body1->m_linearVelocity -= m_body1->m_invMass * P;
	m_body1->m_angularVelocity -= m_body1->m_invI * b2Cross(r1, P);
	m_body2->m_linearVelocity += m_body2->m_invMass * P;
	m_body2->m_angularVelocity += m_body2->m_invI * b2Cross(r2, P);
}

bool b2DistanceJoint::SolvePositionConstraints()
{
	b2Vec2 r1 = b2Mul(m_body1->m_xf.R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(m_body2->m_xf.R, m_localAnchor2);
	b2Vec2 d = m_body2->m_xf.position + r2 - m_body1->m_xf.position - r1;

	float32 length = d.Normalize();
	float32 C = length - m_length;
	C = b2Clamp(C, -b2_maxLinearCorrection, b2_maxLinearCorrection);

	float32 impulse = -m_mass * C;
	m_u = d;
	b2Vec2 P = impulse * m_u;

	m_body1->m_xf.position -= m_body1->m_invMass * P;
	m_body1->m_angle -= m_body1->m_invI * b2Cross(r1, P);
	m_body2->m_xf.position += m_body2->m_invMass * P;
	m_body2->m_angle += m_body2->m_invI * b2Cross(r2, P);

	m_body1->m_xf.R.Set(m_body1->m_angle);
	m_body2->m_xf.R.Set(m_body2->m_angle);

	return b2Abs(C) < b2_linearSlop;
}

b2Vec2 b2DistanceJoint::GetAnchor1() const
{
	return b2Mul(m_body1->m_xf, m_localAnchor1);
}

b2Vec2 b2DistanceJoint::GetAnchor2() const
{
	return b2Mul(m_body2->m_xf, m_localAnchor2);
}

b2Vec2 b2DistanceJoint::GetReactionForce(float32 invTimeStep) const
{
	b2Vec2 F = (m_impulse * invTimeStep) * m_u;
	return F;
}

float32 b2DistanceJoint::GetReactionTorque(float32 invTimeStep) const
{
	B2_NOT_USED(invTimeStep);
	return 0.0f;
}
