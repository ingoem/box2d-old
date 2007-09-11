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

struct b2AABB;
struct b2BodyDescription;
struct b2JointDescription;
struct b2Body;
struct b2Joint;
struct b2Shape;
struct b2Contact;
class b2BroadPhase;

struct b2World
{
	b2World(const b2AABB& worldAABB, const b2Vec2& gravity, bool doSleep);
	~b2World();

	b2Body* CreateBody(const b2BodyDescription* description);
	void DestroyBody(b2Body* body);

	b2Joint* CreateJoint(const b2JointDescription* description);
	void DestroyJoint(b2Joint* joint);

	void Step(float32 dt, int32 iterations);

	int32 Query(const b2AABB& aabb, b2Shape** shapes, int32 maxCount);

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

	static int32 s_enablePositionCorrection;
	static int32 s_enableWarmStarting;
};

#endif
