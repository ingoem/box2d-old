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

#include "b2RevoluteJoint.h"
#include "../b2Body.h"
#include "../b2World.h"

#include "../b2Island.h"

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

b2RevoluteJoint::b2RevoluteJoint(const b2RevoluteJointDef* def)
: b2Joint(def)
{
	m_localAnchor1 = b2MulT(m_body1->m_R, def->anchorPoint - m_body1->m_position);
	m_localAnchor2 = b2MulT(m_body2->m_R, def->anchorPoint - m_body2->m_position);

	m_intialAngle = m_body2->m_rotation - m_body1->m_rotation;

	m_ptpImpulse.Set(0.0f, 0.0f);
	m_motorImpulse = 0.0f;
	m_limitImpulse = 0.0f;
	m_limitPositionImpulse = 0.0f;

	m_lowerAngle = def->lowerAngle;
	m_upperAngle = def->upperAngle;
	m_maxMotorTorque = def->motorTorque;
	m_motorSpeed = def->motorSpeed;
	m_enableLimit = def->enableLimit;
	m_enableMotor = def->enableMotor;
}

void b2RevoluteJoint::PreSolve()
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	// Compute the effective mass matrix.
	b2Vec2 r1 = b2Mul(b1->m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(b2->m_R, m_localAnchor2);

	b2Vec2 p1 = b1->m_position + r1;
	b2Vec2 p2 = b2->m_position + r2;
	m_ptpC = p2 - p1;

	// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	float32 invMass1 = b1->m_invMass, invMass2 = b2->m_invMass;
	float32 invI1 = b1->m_invI, invI2 = b2->m_invI;

	b2Mat22 K1;
	K1.col1.x = invMass1 + invMass2;	K1.col2.x = 0.0f;
	K1.col1.y = 0.0f;					K1.col2.y = invMass1 + invMass2;

	b2Mat22 K2;
	K2.col1.x =  invI1 * r1.y * r1.y;	K2.col2.x = -invI1 * r1.x * r1.y;
	K2.col1.y = -invI1 * r1.x * r1.y;	K2.col2.y =  invI1 * r1.x * r1.x;

	b2Mat22 K3;
	K3.col1.x =  invI2 * r2.y * r2.y;	K3.col2.x = -invI2 * r2.x * r2.y;
	K3.col1.y = -invI2 * r2.x * r2.y;	K3.col2.y =  invI2 * r2.x * r2.x;

	b2Mat22 K = K1 + K2 + K3;
	m_ptpMass = K.Invert();

	m_motorMass = 1.0f / (invI1 + invI2);

	if (m_enableMotor == false)
	{
		m_motorImpulse = 0.0f;
	}

	if (m_enableLimit)
	{
		float32 jointAngle = b2->m_rotation - b1->m_rotation - m_intialAngle;
		if (b2Abs(m_upperAngle - m_lowerAngle) < 2.0f * b2_angularSlop)
		{
			m_limitState = e_equalLimits;
		}
		else if (jointAngle <= m_lowerAngle)
		{
			if (m_limitState != e_atLowerLimit)
			{
				m_limitImpulse = 0.0f;
			}
			m_limitState = e_atLowerLimit;
		}
		else if (jointAngle >= m_upperAngle)
		{
			if (m_limitState != e_atUpperLimit)
			{
				m_limitImpulse = 0.0f;
			}
			m_limitState = e_atUpperLimit;
		}
		else
		{
			m_limitState = e_inactiveLimit;
			m_limitImpulse = 0.0f;
		}
	}
	else
	{
		m_limitImpulse = 0.0f;
	}

	if (b2World::s_enableWarmStarting)
	{
		b1->m_linearVelocity -= invMass1 * m_ptpImpulse;
		b1->m_angularVelocity -= invI1 * (b2Cross(r1, m_ptpImpulse) + m_motorImpulse + m_limitImpulse);

		b2->m_linearVelocity += invMass2 * m_ptpImpulse;
		b2->m_angularVelocity += invI2 * (b2Cross(r2, m_ptpImpulse) + m_motorImpulse + m_limitImpulse);
	}
	else
	{
		m_ptpImpulse.SetZero();
		m_motorImpulse = 0.0f;
		m_limitImpulse = 0.0f;
	}

	m_limitPositionImpulse = 0.0f;
}

void b2RevoluteJoint::SolveVelocityConstraints(const b2StepInfo* step)
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	b2Vec2 r1 = b2Mul(b1->m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(b2->m_R, m_localAnchor2);

	// Solve point-to-point constraint
#if 0
	b2Vec2 ptpCdot = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2) - b1->m_linearVelocity - b2Cross(b1->m_angularVelocity, r1) + 0.2f * step->inv_dt * m_ptpC;
#else
	b2Vec2 ptpCdot = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2) - b1->m_linearVelocity - b2Cross(b1->m_angularVelocity, r1);
#endif
	b2Vec2 ptpImpulse = -b2Mul(m_ptpMass, ptpCdot);
	m_ptpImpulse += ptpImpulse;

	b1->m_linearVelocity -= b1->m_invMass * ptpImpulse;
	b1->m_angularVelocity -= b1->m_invI * b2Cross(r1, ptpImpulse);

	b2->m_linearVelocity += b2->m_invMass * ptpImpulse;
	b2->m_angularVelocity += b2->m_invI * b2Cross(r2, ptpImpulse);

	if (m_enableMotor && m_limitState != e_equalLimits)
	{
		float32 motorCdot = b2->m_angularVelocity - b1->m_angularVelocity - m_motorSpeed;
		float32 motorImpulse = -m_motorMass * motorCdot;
		float32 oldMotorImpulse = m_motorImpulse;
		m_motorImpulse = b2Clamp(m_motorImpulse + motorImpulse, -step->dt * m_maxMotorTorque, step->dt * m_maxMotorTorque);
		motorImpulse = m_motorImpulse - oldMotorImpulse;
		b1->m_angularVelocity -= b1->m_invI * motorImpulse;
		b2->m_angularVelocity += b2->m_invI * motorImpulse;
	}

	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		float32 limitCdot = b2->m_angularVelocity - b1->m_angularVelocity;
		float32 limitImpulse = -m_motorMass * limitCdot;

		if (m_limitState == e_equalLimits)
		{
			m_limitImpulse += limitImpulse;
		}
		else if (m_limitState == e_atLowerLimit)
		{
			float32 oldLimitImpulse = m_limitImpulse;
			m_limitImpulse = b2Max(m_limitImpulse + limitImpulse, 0.0f);
			limitImpulse = m_limitImpulse - oldLimitImpulse;
		}
		else if (m_limitState == e_atUpperLimit)
		{
			float32 oldLimitImpulse = m_limitImpulse;
			m_limitImpulse = b2Min(m_limitImpulse + limitImpulse, 0.0f);
			limitImpulse = m_limitImpulse - oldLimitImpulse;
		}

		b1->m_angularVelocity -= b1->m_invI * limitImpulse;
		b2->m_angularVelocity += b2->m_invI * limitImpulse;
	}
}

/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than b2_linearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

void b2RevoluteJoint::PreparePositionSolver()
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	b2Vec2 r1 = b2Mul(b1->m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(b2->m_R, m_localAnchor2);

	b2Vec2 p1 = b1->m_position + r1;
	b2Vec2 p2 = b2->m_position + r2;
	m_ptpC = p2 - p1;
	//b2Vec2 dpMax(b2_maxLinearCorrection, b2_maxLinearCorrection);
	//m_ptpC = b2Clamp(m_ptpC, -dpMax, dpMax);

	float32 invMass1 = b1->m_invMass, invMass2 = b2->m_invMass;
	float32 invI1 = b1->m_invI, invI2 = b2->m_invI;

	b2Mat22 K1;
	K1.col1.x = invMass1 + invMass2;	K1.col2.x = 0.0f;
	K1.col1.y = 0.0f;					K1.col2.y = invMass1 + invMass2;

	b2Mat22 K2;
	K2.col1.x =  invI1 * r1.y * r1.y;	K2.col2.x = -invI1 * r1.x * r1.y;
	K2.col1.y = -invI1 * r1.x * r1.y;	K2.col2.y =  invI1 * r1.x * r1.x;

	b2Mat22 K3;
	K3.col1.x =  invI2 * r2.y * r2.y;	K3.col2.x = -invI2 * r2.x * r2.y;
	K3.col1.y = -invI2 * r2.x * r2.y;	K3.col2.y =  invI2 * r2.x * r2.x;

	b2Mat22 K = K1 + K2 + K3;
	m_ptpMass = K.Invert();

	m_r1 = r1;
	m_r2 = r2;
}

bool b2RevoluteJoint::SolvePositionConstraints()
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	float32 positionError = 0.0f;
#if 0
	// Solve point-to-point position error.
	b2Vec2 r1 = b2Mul(b1->m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(b2->m_R, m_localAnchor2);

	b2Vec2 p1 = b1->m_position + r1;
	b2Vec2 p2 = b2->m_position + r2;
	b2Vec2 ptpC = p2 - p1;

	positionError = ptpC.Length();

	// Prevent overly large corrections.
	//b2Vec2 dpMax(b2_maxLinearCorrection, b2_maxLinearCorrection);
	//ptpC = b2Clamp(ptpC, -dpMax, dpMax);

	b2Vec2 impulse = -b2Mul(m_ptpMass, ptpC);

	b1->m_position -= b1->m_invMass * impulse;
	b1->m_rotation -= b1->m_invI * b2Cross(r1, impulse);
	b1->m_R.Set(b1->m_rotation);

	b2->m_position += b2->m_invMass * impulse;
	b2->m_rotation += b2->m_invI * b2Cross(r2, impulse);
	b2->m_R.Set(b2->m_rotation);

#elif 1
	// Solve point-to-point position error.
	b2Vec2 r1 = b2Mul(b1->m_R, m_localAnchor1);
	b2Vec2 r2 = b2Mul(b2->m_R, m_localAnchor2);

	b2Vec2 p1 = b1->m_position + r1;
	b2Vec2 p2 = b2->m_position + r2;
	b2Vec2 ptpC = p2 - p1;

	positionError = ptpC.Length();

	// Prevent overly large corrections.
	//b2Vec2 dpMax(b2_maxLinearCorrection, b2_maxLinearCorrection);
	//ptpC = b2Clamp(ptpC, -dpMax, dpMax);

	float32 invMass1 = b1->m_invMass, invMass2 = b2->m_invMass;
	float32 invI1 = b1->m_invI, invI2 = b2->m_invI;

	b2Mat22 K1;
	K1.col1.x = invMass1 + invMass2;	K1.col2.x = 0.0f;
	K1.col1.y = 0.0f;					K1.col2.y = invMass1 + invMass2;

	b2Mat22 K2;
	K2.col1.x =  invI1 * r1.y * r1.y;	K2.col2.x = -invI1 * r1.x * r1.y;
	K2.col1.y = -invI1 * r1.x * r1.y;	K2.col2.y =  invI1 * r1.x * r1.x;

	b2Mat22 K3;
	K3.col1.x =  invI2 * r2.y * r2.y;	K3.col2.x = -invI2 * r2.x * r2.y;
	K3.col1.y = -invI2 * r2.x * r2.y;	K3.col2.y =  invI2 * r2.x * r2.x;

	b2Mat22 K = K1 + K2 + K3;
	m_ptpMass = K.Invert();

	b2Vec2 impulse = -b2Mul(m_ptpMass, ptpC);

	b1->m_position -= b1->m_invMass * impulse;
	b1->m_rotation -= b1->m_invI * b2Cross(r1, impulse);
	b1->m_R.Set(b1->m_rotation);

	b2->m_position += b2->m_invMass * impulse;
	b2->m_rotation += b2->m_invI * b2Cross(r2, impulse);
	b2->m_R.Set(b2->m_rotation);
#elif 0
	b2Vec2 ptpCdot = b2->m_dp + b2Cross(b2->m_dr, m_r2) - b1->m_dp - b2Cross(b1->m_dr, m_r1) + 0.2f * m_ptpC;
	b2Vec2 ptpImpulse = -b2Mul(m_ptpMass, ptpCdot);

	b2Vec2 dp1 = b1->m_invMass * ptpImpulse;
	b2Vec2 dp2 = b2->m_invMass * ptpImpulse;

	positionError = 10000.0f; //b2Max(dp1.Length(), dp2.Length());

	b1->m_dp -= dp1;
	b1->m_dr -= b1->m_invI * b2Cross(m_r1, ptpImpulse);

	b2->m_dp += dp2;
	b2->m_dr += b2->m_invI * b2Cross(m_r2, ptpImpulse);
#endif

	// Handle limits.
	float32 angularError = 0.0f;

	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		float32 angle = b2->m_rotation - b1->m_rotation - m_intialAngle;
		float32 limitImpulse = 0.0f;

		if (m_limitState == e_equalLimits)
		{
			// Prevent large angular corrections
			float32 limitC = b2Clamp(angle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
			limitImpulse = -m_motorMass * limitC;
			angularError = b2Abs(limitC);
		}
		else if (m_limitState == e_atLowerLimit)
		{
			float32 limitC = angle - m_lowerAngle;
			angularError = b2Max(0.0f, -limitC);

			// Prevent large angular corrections and allow some slop.
			limitC = b2Clamp(limitC + b2_angularSlop, -b2_maxAngularCorrection, 0.0f);
			limitImpulse = -m_motorMass * limitC;
			float32 oldLimitImpulse = m_limitPositionImpulse;
			m_limitPositionImpulse = b2Max(m_limitPositionImpulse + limitImpulse, 0.0f);
			limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
		}
		else if (m_limitState == e_atUpperLimit)
		{
			float32 limitC = angle - m_upperAngle;
			angularError = b2Max(0.0f, limitC);

			// Prevent large angular corrections and allow some slop.
			limitC = b2Clamp(limitC - b2_angularSlop, 0.0f, b2_maxAngularCorrection);
			limitImpulse = -m_motorMass * limitC;
			float32 oldLimitImpulse = m_limitPositionImpulse;
			m_limitPositionImpulse = b2Min(m_limitPositionImpulse + limitImpulse, 0.0f);
			limitImpulse = m_limitPositionImpulse - oldLimitImpulse;
		}

		b1->m_rotation -= b1->m_invI * limitImpulse;
		b1->m_R.Set(b1->m_rotation);
		b2->m_rotation += b2->m_invI * limitImpulse;
		b2->m_R.Set(b2->m_rotation);
	}

	return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
}

b2Vec2 b2RevoluteJoint::GetAnchor1() const
{
	b2Body* b1 = m_body1;
	return b1->m_position + b2Mul(b1->m_R, m_localAnchor1);
}

b2Vec2 b2RevoluteJoint::GetAnchor2() const
{
	b2Body* b2 = m_body2;
	return b2->m_position + b2Mul(b2->m_R, m_localAnchor2);
}

float32 b2RevoluteJoint::GetJointAngle() const
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;
	return b2->m_rotation - b1->m_rotation;
}

float32 b2RevoluteJoint::GetJointSpeed() const
{
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;
	return b2->m_angularVelocity - b1->m_angularVelocity;
}

float32 b2RevoluteJoint::GetMotorTorque(float32 invTimeStep) const
{
	return invTimeStep * m_motorImpulse;
}

void b2RevoluteJoint::SetMotorSpeed(float32 speed)
{
	m_motorSpeed = speed;
}

void b2RevoluteJoint::SetMotorTorque(float32 torque)
{
	m_maxMotorTorque = torque;
}

b2Vec2 b2RevoluteJoint::GetReactionForce(float32 invTimeStep) const
{
	return invTimeStep * m_ptpImpulse;
}

float32 b2RevoluteJoint::GetReactionTorque(float32 invTimeStep) const
{
	return invTimeStep * m_limitImpulse;
}
