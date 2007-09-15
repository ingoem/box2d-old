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

#ifndef B2_DISTANCE_JOINT_H
#define B2_DISTANCE_JOINT_H

#include "b2Joint.h"

struct b2Body;

struct b2DistanceJointDef : public b2JointDef
{
	b2DistanceJointDef()
	{
		type = e_distanceJoint;
		anchorPoint1.Set(0.0f, 0.0f);
		anchorPoint2.Set(0.0f, 0.0f);
	}

	b2Vec2 anchorPoint1;
	b2Vec2 anchorPoint2;
};

struct b2DistanceJoint : public b2Joint
{
	b2DistanceJoint(const b2DistanceJointDef* data);

	void PreSolve();
	void SolveVelocityConstraints(float32 dt);
	bool SolvePositionConstraints();
	b2Vec2 GetAnchor1() const;
	b2Vec2 GetAnchor2() const;

	b2Vec2 m_localAnchor1;
	b2Vec2 m_localAnchor2;
	b2Vec2 m_u;
	float32 m_impulse;
	float32 m_mass;	// effective mass for the constraint.
	float32 m_length;
};

#endif
