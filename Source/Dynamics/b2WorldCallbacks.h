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

#ifndef B2_WORLD_CALLBACKS_H
#define B2_WORLD_CALLBACKS_H

#include "../Common/b2Settings.h"

struct b2Manifold;
struct b2Vec2;
struct b2XForm;
class b2Shape;
class b2Body;
class b2Joint;

/// Joints and shapes are destroyed when their associated
/// body is destroyed. Implement this listener so that you
/// may nullify references to these joints and shapes.
class b2DestructionListener
{
public:
	virtual ~b2DestructionListener() {}

	/// Called when any joint is about to be destroyed due
	/// to the destruction of one of its attached bodies.
	virtual void SayGoodbye(b2Joint* joint) = 0;

	/// Called when any shape is about to be destroyed due
	/// to the destruction of its parent body.
	virtual void SayGoodbye(b2Shape* shape) = 0;
};


/// This is called when a body's shape passes outside of the world boundary. If you
/// override this and pass back e_destroyBody, you must nullify your copies of the
/// body pointer.
class b2BoundaryListener
{
public:
	enum Response
	{
		e_freezeBody,
		e_destroyBody,
	};

	virtual ~b2BoundaryListener() {}

	/// This is called for each body that leaves the world boundary.
	virtual Response Violation(b2Body* body) = 0;
};


/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
class b2ContactFilter
{
public:
	virtual ~b2ContactFilter() {}

	/// Return true if contact calculations should be performed between these two shapes.
	virtual bool ShouldCollide(b2Shape* shape1, b2Shape* shape2);
};

/// The default contact filter.
extern b2ContactFilter b2_defaultFilter;


/// This structure allows you to tweak the contact solver behavior.
struct b2SolverTweaks
{
	/// Set this to true if you want the contact solver to ignore this contact.
	bool nonSolid;

	/// Override the default friction determined from the shapes.
	float32 friction;

	/// Override the default restitution determined from the shapes.
	float32 restitution;
};

/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
/// You can also use this class to tweak contact settings. These tweaks persist until
/// you tweak the settings again or the contact is destroyed.
class b2ContactListener
{
public:
	virtual ~b2ContactListener() {}

	/// Called when a time of impact (TOI) event occurs.
	/// @param point1 closest point on shape1.
	/// @param point2 closest point on shape2.
	/// @param shape1 the first shape in the contact.
	/// @param shape2 the second shape in the contact.
	/// @return false if this TOI event should be ignored.
	virtual bool TOI(const b2Vec2& point1, const b2Vec2& point2, const b2Shape* shape1, const b2Shape* shape2) = 0;

	/// Called when a new non-empty contact manifold is created. The manifold normal points from shape1 to shape2.
	/// @param tweaks allows you to adjust the contact solver behavior.
	/// @param manifold the contact geometry. You may adjust this, but be careful.
	/// @param shape1 the first shape in the contact.
	/// @param shape2 the second shape in the contact.
	virtual void Begin(b2SolverTweaks* tweaks, b2Manifold* manifold, const b2Shape* shape1, const b2Shape* shape2) = 0;

	/// Called when contact persists. The manifold normal points from shape1 to shape2.
	/// @param tweaks allows you to adjust the contact solver behavior.
	/// @param manifold the contact geometry. You may adjust this, but be careful.
	/// @param shape1 the first shape in the contact.
	/// @param shape2 the second shape in the contact.
	virtual void Persist(b2SolverTweaks* tweaks, b2Manifold* manifold, const b2Shape* shape1, const b2Shape* shape2) = 0;

	/// Called when contact ends. This does not mean the contact object is destroyed, it just
	/// means that there are no contact points.
	/// @param shape1 the first shape in the contact.
	/// @param shape2 the second shape in the contact.
	virtual void End(const b2Shape* shape1, const b2Shape* shape2) = 0;
};

/// Color for debug drawing. Each value has the range [0,1].
struct b2Color
{
	b2Color() {}
	b2Color(float32 r, float32 g, float32 b) : r(r), g(g), b(b) {}
	float32 r, g, b;
};

/// Implement and register this class with a b2World to provide debug drawing of physics
/// entities in your game.
class b2DebugDraw
{
public:
	b2DebugDraw::b2DebugDraw();

	virtual ~b2DebugDraw() {}

	enum
	{
		e_shapeBit				= 0x0001, ///< draw shapes
		e_jointBit				= 0x0002, ///< draw joint connections
		e_coreShapeBit			= 0x0004, ///< draw core (TOI) shapes
		e_aabbBit				= 0x0008, ///< draw axis aligned bounding boxes
		e_obbBit				= 0x0010, ///< draw oriented bounding boxes
		e_pairBit				= 0x0020, ///< draw broad-phase pairs
		e_contactPointBit		= 0x0040, ///< draw contact points
		e_contactNormalBit		= 0x0080, ///< draw contact normals
		e_contactImpulseBit		= 0x0100, ///< draw contact impulses
		e_frictionImpulseBit	= 0x0200, ///< draw friction impulses
		e_centerOfMassBit		= 0x0400, ///< draw center of mass frame
	};

	/// Set the drawing flags.
	void SetFlags(uint32 flags);

	/// Get the drawing flags.
	uint32 GetFlags() const;
	
	/// Append flags to the current flags.
	void AppendFlags(uint32 flags);

	/// Clear flags from the current flags.
	void ClearFlags(uint32 flags);

	/// Draw a closed polygon provided in CCW order.
	virtual void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) = 0;

	/// Draw a solid closed polygon provided in CCW order.
	virtual void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) = 0;

	/// Draw a circle.
	virtual void DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color) = 0;
	
	/// Draw a solid circle.
	virtual void DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color) = 0;
	
	/// Draw a point. For example, a contact point.
	virtual void DrawPoint(const b2Vec2& p, const b2Color& color) = 0;

	/// Draw a line segment.
	virtual void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) = 0;

	/// Draw a unit length axis. Choose your own length scale.
	/// @param point the axis origin in world coordinates.
	/// @param axis a unit vector in world coordinates.
	virtual void DrawAxis(const b2Vec2& point, const b2Vec2& axis, const b2Color& color) = 0;

	/// Draw a transform. Choose your own length scale.
	/// @param xf a transform.
	virtual void DrawXForm(const b2XForm& xf) = 0;

	/// Draw an impulse. Choose your own length scale.
	/// @param point the impulse point of application in world coordinates.
	/// @param axis the impulse vector in world coordinates.
	virtual void DrawImpulse(const b2Vec2& point, const b2Vec2& impulse, const b2Color& color) = 0;

protected:
	uint32 m_drawFlags;
};

#endif
