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

#ifndef B2_MOUSE_JOINT_H
#define B2_MOUSE_JOINT_H

#include "b2Joint.h"

struct b2Body;
class b2BlockAllocator;

struct b2MouseDescription : public b2JointDescription
{
	b2MouseDescription()
	{
		type = e_mouseJoint;
		target.Set(0.0f, 0.0f);
		motorForce = 0.0f;
		beta = 0.2f;
		length = 1.0f;
	}

	b2Vec2 target;
	float32 beta;
	float32 motorForce;
	float32 length;
};

struct b2MouseJoint : public b2Joint
{
	b2MouseJoint(const b2MouseDescription* description);

	void PreSolve();
	void SolveVelocityConstraints(float32 dt);
	bool SolvePositionConstraints() { return true; }

	b2Vec2 GetAnchor1() const;
	b2Vec2 GetAnchor2() const;
	float32 GetMotorForce(float32 inv_dt) const { return m_impulse * inv_dt; }
	void SetTarget(const b2Vec2& target);

	b2Vec2 m_localAnchor;
	b2Vec2 m_target;
	b2Vec2 m_u;
	float32 m_positionError;
	float32 m_impulse;

	float32 m_mEff;		// effective mass
	float32 m_motorForce;
	float32 m_length;
	float32 m_beta;
};

#endif
