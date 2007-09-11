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

struct b2BodyDescription
{
	b2BodyDescription()
	{
		memset(shapes, 0, sizeof(shapes));
		position.Set(0.0f, 0.0f);
		rotation = 0.0f;
		linearVelocity.Set(0.0f, 0.0f);
		angularVelocity = 0.0f;
		allowSleep = true;
		isSleeping = false;
	}

	b2ShapeDescription* shapes[b2_maxShapesPerBody];
	b2Vec2 position;
	float32 rotation;
	b2Vec2 linearVelocity;
	float32 angularVelocity;
	bool allowSleep;
	bool isSleeping;

	void AddShape(b2ShapeDescription* shape);
};

// A rigid body. Internal computation are done in terms
// of the center of mass position. When compound shapes are
// used the center does not correspond to root position.
struct b2Body
{
	b2Body(const b2BodyDescription* bd, b2World* world);
	~b2Body();

	void SetRootPosition(const b2Vec2& position, float rotation);
	b2Vec2 GetRootPosition() const { return m_position - b2Mul(m_R, m_center); }

	bool IsStatic() const { return m_invMass == 0.0f; }
	bool IsSleeping() const;
	void WakeUp();

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

inline void b2BodyDescription::AddShape(b2ShapeDescription* shape)
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
