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

#ifndef B2_WORLD_H
#define B2_WORLD_H

#include "Engine/Common/b2Math.h"
#include "Engine/Common/b2BlockAllocator.h"
#include "Engine/Common/b2StackAllocator.h"
#include "b2ContactManager.h"
#include "b2WorldCallbacks.h"

struct b2AABB;
struct b2BodyDef;
struct b2JointDef;
struct b2Body;
struct b2Joint;
struct b2Shape;
struct b2Contact;
class b2BroadPhase;

struct b2World
{
	b2World(const b2AABB& worldAABB, const b2Vec2& gravity, bool doSleep);
	~b2World();

	// Set a callback to notify you when a joint is implicitly destroyed
	// when an attached body is destroyed.
	void SetJointDestroyedCallback(b2JointDestroyedCallback* callback);

	b2Body* CreateBody(const b2BodyDef* def);
	void DestroyBody(b2Body* body);

	b2Joint* CreateJoint(const b2JointDef* def);
	void DestroyJoint(b2Joint* joint);

	// The world provides a single ground body with no collision shapes. You
	// can use this to simplify the creation of joints.
	b2Body* GetGroundBody();

	void Step(float32 timeStep, int32 iterations);

	// Query the world for all shapes that potentially overlap the
	// provided AABB. You provide a shape pointer buffer of specified
	// size. The number of shapes found is returned.
	int32 Query(const b2AABB& aabb, b2Shape** shapes, int32 maxCount);

	// You can use these to iterate over all the bodies, joints, and contacts.
	b2Body* GetBodyList();
	b2Joint* GetJointList();
	b2Contact* GetContactList();

	//--------------- Internals Below -------------------

	b2BlockAllocator m_blockAllocator;
	b2StackAllocator m_stackAllocator;

	b2BroadPhase* m_broadPhase;
	b2ContactManager m_contactManager;

	b2Body* m_bodyList;
	b2Contact* m_contactList;
	b2Joint* m_jointList;

	int32 m_bodyCount;
	int32 m_contactCount;
	int32 m_jointCount;

	b2Vec2 m_gravity;
	bool m_doSleep;

	b2Body* m_groundBody;

	b2JointDestroyedCallback* m_jointDestroyedCallback;

	static int32 s_enablePositionCorrection;
	static int32 s_enableWarmStarting;
};

inline b2Body* b2World::GetGroundBody()
{
	return m_groundBody;
}

inline b2Body* b2World::GetBodyList()
{
	return m_bodyList;
}

inline b2Joint* b2World::GetJointList()
{
	return m_jointList;
}

inline b2Contact* b2World::GetContactList()
{
	return m_contactList;
}

#endif
