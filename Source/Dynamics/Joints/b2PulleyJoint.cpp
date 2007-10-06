/*
* Copyright (c) 2007 Erin Catto http://www.gphysics.com
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

#include "b2PulleyJoint.h"
#include "../b2Body.h"
#include "../b2World.h"

#include <stdio.h>

// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C = length1 + ratio * length2 - length0
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = dot(u1, v1 + cross(w1, r1)) + ratio * dot(u2, v2 + cross(w2, r2))
// J = [u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)
//
// Limit:
// C = maxLength - length
// u = (p - s) / norm(p - s)
// Cdot = -dot(u, v + cross(w, r))
// K = invMass + invI * cross(r, u)^2
// 0 <= impulse

b2PulleyJoint::b2PulleyJoint(const b2PulleyJointDef* def)
: b2Joint(def)
{
	m_ground = m_body1->m_world->m_groundBody;
	m_groundAnchor1 = def->groundPoint1 - m_ground->m_position;
	m_groundAnchor2 = def->groundPoint2 - m_ground->m_position;
	m_localAnchor1 = b2MulT(m_body1->m_R, def->anchorPoint1 - m_body1->m_position);
	m_localAnchor2 = b2MulT(m_body2->m_R, def->anchorPoint2 - m_body2->m_position);

	m_ratio = def->ratio;

	b2Vec2 d1 = def->groundPoint1 - def->anchorPoint1;
	b2Vec2 d2 = def->groundPoint2 - def->anchorPoint2;

	float32 length1 = b2Max(0.5f * b2_minPulleyLength, d1.Length());
	float32 length2 = b2Max(0.5f * b2_minPulleyLength, d2.Length());

	m_length = length1 + length2;

	m_maxLength1 = b2Clamp(def->maxLength1, length1, m_length - 0.5f * b2_minPulleyLength);
	m_maxLength2 = b2Clamp(def->maxLength1, length1, m_length / m_ratio - 0.5f * b2_minPulleyLength);

	m_impulse = 0.0f;
	m_limitImpulse = 0.0f;
	m_motorImpulse = 0.0f;

	m_maxMotorForce = def->motorForce;
	m_motorSpeed = def->motorSpeed;
	m_enableMotor = def->enableMotor;
}

void b2PulleyJoint::PreSolve()
{
	// u1 = (p1 - s1) / norm(p1 - s1)
	// u2 = (p2 - s2) / norm(p2 - s2)
	// Cdot = dot(u1, v1 + cross(w1, r1)) + ratio * dot(u2, v2 + cross(w2, r2))
	// J = [u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
	// K = J * invM * JT
	//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

	b2Vec2 s1 = m_ground->m_position + m_groundAnchor1;
	b2Vec2 s2 = m_ground->m_position + m_groundAnchor2;

	b2Vec2 r1 = b2Mul(m_body1->m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(m_body2->m_R, m_localAnchor2);

	b2Vec2 p1 = m_body1->m_position + r1;
	b2Vec2 p2 = m_body2->m_position	+ r2;

	m_u1 = p1 - s1;
	m_u2 = p2 - s2;

	// TODO use fast inv sqrt
	float32 length1 = m_u1.Length();
	float32 length2 = m_u2.Length();

	if (length1 > b2_linearSlop)
	{
		m_u1 *= 1.0f / length1;
	}
	else
	{
		m_u1.SetZero();
	}

	if (length2 > b2_linearSlop)
	{
		m_u2 *= 1.0f / length2;
	}
	else
	{
		m_u2.SetZero();
	}


#if 0
	// Compute the effective mass matrix.
	m_u = m_body2->m_position + r2 - m_body1->m_position - r1;

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

	// Warm starting.
	b2Vec2 P = m_impulse * m_u;
	m_body1->m_linearVelocity -= m_body1->m_invMass * P;
	m_body1->m_angularVelocity -= m_body1->m_invI * b2Cross(r1, P);
	m_body2->m_linearVelocity += m_body2->m_invMass * P;
	m_body2->m_angularVelocity += m_body2->m_invI * b2Cross(r2, P);
#endif
}

void b2PulleyJoint::SolveVelocityConstraints(float32 dt)
{
	NOT_USED(dt);
#if 0
	b2Vec2 r1 = b2Mul(m_body1->m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(m_body2->m_R, m_localAnchor2);

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
#endif
}

bool b2PulleyJoint::SolvePositionConstraints()
{
#if 0
	b2Vec2 r1 = b2Mul(m_body1->m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(m_body2->m_R, m_localAnchor2);
	b2Vec2 d = m_body2->m_position + r2 - m_body1->m_position - r1;
	float32 C = d.Length() - m_length;
	float32 impulse = -m_mass * C;
	b2Vec2 P = impulse * m_u;
	m_body1->m_position -= m_body1->m_invMass * P;
	m_body1->m_rotation -= m_body1->m_invI * b2Cross(r1, P);
	m_body2->m_position += m_body2->m_invMass * P;
	m_body2->m_rotation += m_body2->m_invI * b2Cross(r2, P);

	m_body1->m_R.Set(m_body1->m_rotation);
	m_body2->m_R.Set(m_body2->m_rotation);

	return b2Abs(C) < b2_linearSlop;
#endif
	return true;
}

b2Vec2 b2PulleyJoint::GetAnchor1() const
{
	return m_body1->m_position + b2Mul(m_body1->m_R, m_localAnchor1);
}

b2Vec2 b2PulleyJoint::GetAnchor2() const
{
	return m_body2->m_position + b2Mul(m_body2->m_R, m_localAnchor2);
}

b2Vec2 b2PulleyJoint::GetReactionForce(float32 invTimeStep) const
{
	NOT_USED(invTimeStep);
	b2Vec2 F(0.0f, 0.0f); // = (m_impulse * invTimeStep) * m_u;
	return F;
}

float32 b2PulleyJoint::GetReactionTorque(float32 invTimeStep) const
{
	NOT_USED(invTimeStep);
	return 0.0f;
}
