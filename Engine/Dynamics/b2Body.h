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

#ifndef BODY_H
#define BODY_H

#include "Engine/Common/b2Math.h"
#include "Engine/Dynamics/Joints/b2Joint.h"
#include "Engine/Collision/b2Shape.h"

#include <string.h>

struct b2Joint;
struct b2Contact;
struct b2World;
struct b2JointNode;
struct b2ContactNode;

struct b2BodyDef
{
	b2BodyDef()
	{
		memset(shapes, 0, sizeof(shapes));
		position.Set(0.0f, 0.0f);
		rotation = 0.0f;
		linearVelocity.Set(0.0f, 0.0f);
		angularVelocity = 0.0f;
		allowSleep = true;
		isSleeping = false;
	}

	b2ShapeDef* shapes[b2_maxShapesPerBody];
	b2Vec2 position;
	float32 rotation;
	b2Vec2 linearVelocity;
	float32 angularVelocity;
	bool allowSleep;
	bool isSleeping;

	void AddShape(b2ShapeDef* shape);
};

// A rigid body. Internal computation are done in terms
// of the center of mass position. The center of mass may
// be offset from the body's origin.
struct b2Body
{
	// Set the position of the body's origin and rotation (radians).
	// This breaks any contacts and wakes the other bodies.
	void SetOriginPosition(const b2Vec2& position, float32 rotation);

	// Get the position of the body's origin.
	b2Vec2 GetOriginPosition() const { return m_position - b2Mul(m_R, m_center); }

	// Get the rotation in radians.
	float32 GetRotation() const { return m_rotation; }

	// Set/Get the linear velocity of the center of mass.
	void SetLinearVelocity(const b2Vec2& v);
	b2Vec2 GetLinearVelocity() const;

	// Set/Get the angular velocity.
	void SetAngularVelocity(float32 w);
	float32 GetAngularVelocity() const;

	// Apply a force to the center of mass. Additive.
	void ApplyForce(const b2Vec2& force);

	// Apply a torque. Additive.
	void ApplyTorque(float32 torque);

	// Is this body static (immovable)?
	bool IsStatic() const { return m_invMass == 0.0f; }

	// Is this body sleeping (not simulating).
	bool IsSleeping() const;

	// Wake up this body so it will begin simulating.
	void WakeUp();

	//--------------- Internal Usage -------------------

	b2Body(const b2BodyDef* bd, b2World* world);
	~b2Body();

	void SynchronizeShapes();

	bool IsConnected(const b2Body* other) const;

	b2Vec2 m_position;	// center of mass position
	float32 m_rotation;
	b2Mat22 m_R;

	b2Vec2 m_linearVelocity;
	float32 m_angularVelocity;

	b2Vec2 m_force;
	float32 m_torque;

	b2Vec2 m_center;	// local vector from client origin to center of mass

	b2World* m_world;
	b2Body* m_prev;
	b2Body* m_next;

	b2Shape* m_shapeList;

	b2JointNode* m_jointList;
	b2ContactNode* m_contactList;

	float32 m_mass, m_invMass;
	float32 m_I, m_invI;

	float32 m_sleepTime;

	bool m_allowSleep;
	bool m_isSleeping;

	bool m_islandFlag;
};

inline void b2BodyDef::AddShape(b2ShapeDef* shape)
{
	for (int32 i = 0; i < b2_maxShapesPerBody; ++i)
	{
		if (shapes[i] == NULL)
		{
			shapes[i] = shape;
			break;
		}
	}
}

inline bool b2Body::IsSleeping() const
{
	return m_isSleeping;
}

inline void b2Body::WakeUp()
{
	m_isSleeping = false;
	m_sleepTime = 0.0f;
}

inline bool b2Body::IsConnected(const b2Body* other) const
{
	for (b2JointNode* jn = m_jointList; jn; jn = jn->next)
	{
		if (jn->other == other)
			return true;
	}

	return false;
}

#endif
