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

#include "../Common/b2Math.h"
#include "../Common/b2BlockAllocator.h"
#include "../Common/b2StackAllocator.h"
#include "b2ContactManager.h"
#include "b2WorldCallbacks.h"

struct b2AABB;
struct b2ShapeDef;
struct b2BodyDef;
struct b2JointDef;
class b2Body;
class b2Joint;
class b2Shape;
class b2Contact;
class b2BroadPhase;

struct b2TimeStep
{
	float32 dt;			// time step
	float32 inv_dt;		// inverse time step (0 if dt == 0).
	int32 maxIterations;
};

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
class b2World
{
public:
	/// Construct a world object.
	/// @param worldAABB a bounding box that completely encompasses all your shapes.
	/// @param gravity the world gravity vector.
	/// @param doSleep improve performance by not simulating inactive bodies.
	b2World(const b2AABB& worldAABB, const b2Vec2& gravity, bool doSleep);

	/// Destruct the world. All physics entities are destroyed and all heap memory is released.
	~b2World();

	/// Register a destruction listener.
	void SetListener(b2DestructionListener* listener);

	/// Register a broad-phase boundary listener.
	void SetListener(b2BoundaryListener* listener);

	/// Register a contact filter to provide specific control over collision.
	/// Otherwise the default filter is used (b2_defaultFilter).
	void SetFilter(b2ContactFilter* filter);

	/// Register a contact event listener
	void SetListener(b2ContactListener* listener);

	/// Register a routine for debug drawing. The debug draw functions are called
	/// inside the b2World::Step method, so make sure your renderer is ready to
	/// consume draw commands when you call Step().
	void SetDebugDraw(b2DebugDraw* debugDraw);

	/// Create a rigid body given a definition. No reference to the definition
	/// is retained.
	/// @warning This function is locked during callbacks.
	b2Shape* Create(const b2ShapeDef* def);

	/// Destroy a rigid body given a definition. No reference to the definition
	/// is retained. This function is locked during callbacks. This automatically
	/// deletes all associated shapes and joints.
	/// @warning This function is locked during callbacks.
	void Destroy(b2Shape* shape);

	/// Create a rigid body given a definition. No reference to the definition
	/// is retained.
	/// @warning This function is locked during callbacks.
	b2Body* Create(const b2BodyDef* def);

	/// Destroy a rigid body given a definition. No reference to the definition
	/// is retained. This function is locked during callbacks.
	/// @warning This automatically deletes all associated shapes and joints.
	/// @warning This function is locked during callbacks.
	void Destroy(b2Body* body);

	/// Create a joint to constrain bodies together. No reference to the definition
	/// is retained. This may cause the connected bodies to cease colliding.
	/// @warning This function is locked during callbacks.
	b2Joint* Create(const b2JointDef* def);

	/// Destroy a joint. This may cause the connected bodies to begin colliding.
	/// @warning This function is locked during callbacks.
	void Destroy(b2Joint* joint);

	/// The world provides a single static ground body with no collision shapes.
	/// You can use this to simplify the creation of joints.
	b2Body* GetGroundBody();

	/// Take a time step. This performs collision detection, integration,
	/// and constraint solution.
	/// @param timeStep the amount of time to simulate, this should not vary.
	/// @param iterations the number of iterations to be used by the constraint solver.
	void Step(float32 timeStep, int32 iterations);

	/// Query the world for all shapes that potentially overlap the
	/// provided AABB. You provide a shape pointer buffer of specified
	/// size. The number of shapes found is returned.
	/// @param aabb the query box.
	/// @param shapes a user allocated shape pointer array of size maxCount (or greater).
	/// @param maxCount the capacity of the shapes array.
	/// @return the number of shapes found in aabb.
	int32 Query(const b2AABB& aabb, b2Shape** shapes, int32 maxCount);

	/// Get the world shape list. These shapes may or may not be attached to bodies.
	/// With the returned shape, use b2Shape::GetWorldNext to get the next shape in
	/// the world list. A NULL shape indicates the end of the list.
	/// @return the head of the world shape list.
	b2Shape* GetShapeList();

	/// Get the world shape list. These shapes may or may not be attached to bodies.
	/// With the returned shape, use b2Shape::GetWorldNext to get the next shape in
	/// the world list. A NULL shape indicates the end of the list.
	/// @return the head of the world shape list.
	b2Body* GetBodyList();

	/// Get the world shape list. These shapes may or may not be attached to bodies.
	/// With the returned shape, use b2Shape::GetWorldNext to get the next shape in
	/// the world list. A NULL shape indicates the end of the list.
	/// @return the head of the world shape list.
	b2Joint* GetJointList();

	/// Get the world shape list. These shapes may or may not be attached to bodies.
	/// With the returned shape, use b2Shape::GetWorldNext to get the next shape in
	/// the world list. A NULL shape indicates the end of the list.
	/// @return the head of the world shape list.
	b2Contact* GetContactList();

public:
	//--------------- Internals Below -------------------
	// Internal yet public to make life easier.

	void Solve(const b2TimeStep& step);
	void SolveTOI();

	void DrawJoint(b2Joint* joint);
	void DrawShape(b2Shape* shape, const b2XForm& xf, const b2Color& color, bool core);
	void DrawDebugData();

	b2BlockAllocator m_blockAllocator;
	b2StackAllocator m_stackAllocator;

	bool m_lock;

	b2BroadPhase* m_broadPhase;
	b2ContactManager m_contactManager;

	b2Shape* m_shapeList;
	b2Body* m_bodyList;
	b2Contact* m_contactList;
	b2Joint* m_jointList;

	int32 m_shapeCount;
	int32 m_bodyCount;
	int32 m_contactCount;
	int32 m_jointCount;

	b2Vec2 m_gravity;
	bool m_allowSleep;

	b2Body* m_groundBody;

	b2DestructionListener* m_destructionListener;
	b2BoundaryListener* m_boundaryListener;
	b2ContactFilter* m_contactFilter;
	b2ContactListener* m_contactListener;
	b2DebugDraw* m_debugDraw;

	int32 m_positionIterationCount;

	// This is for debugging the solver.
	static int32 s_enablePositionCorrection;

	// This is for debugging the solver.
	static int32 s_enableWarmStarting;
};

inline b2Body* b2World::GetGroundBody()
{
	return m_groundBody;
}

inline b2Shape* b2World::GetShapeList()
{
	return m_shapeList;
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
