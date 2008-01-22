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

#ifndef B2_BODY_H
#define B2_BODY_H

#include "../Common/b2Math.h"
#include "../Collision/Shapes/b2Shape.h"
#include "Joints/b2Joint.h"

#include <memory>

class b2Joint;
class b2Contact;
class b2World;
struct b2JointNode;
struct b2ContactNode;

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions.
struct b2BodyDef
{
	/// The type of body.
	enum Type
	{
		e_staticBody,	///< A static body should not move and has infinite mass.
		e_dynamicBody,	///< A regular moving body.
	};

	/// This constructor sets the body definition default values.
	b2BodyDef()
	{
		type = e_staticBody;
		massData.center.SetZero();
		massData.mass = 0.0f;
		massData.I = 0.0f;
		userData = NULL;
		position.Set(0.0f, 0.0f);
		angle = 0.0f;
		linearDamping = 0.0f;
		angularDamping = 0.0f;
		allowSleep = true;
		isSleeping = false;
		preventRotation = false;
		isBullet = false;
	}

	/// We need the body type to setup collision filtering correctly, so
	/// that static bodies don't collide with each other. You can't change
	/// this once a body is created.
	Type type;

	/// You can use this to initialized the mass properties of the body.
	/// If you prefer, you can set the mass properties after the shapes
	/// have been added using b2Body::SetMassFromShapes.
	b2MassData massData;

	/// Use this to store application specific body data.
	void* userData;

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	b2Vec2 position;

	/// The world angle of the body in radians.
	float32 angle;

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	float32 linearDamping;

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	float32 angularDamping;

	/// Set this flag to false if this body should never fall asleep. Note that
	/// this increases CPU usage.
	bool allowSleep;

	/// Is this body initially sleeping?
	bool isSleeping;

	/// Should this body be prevented from rotating? Useful for characters.
	bool preventRotation;

	/// Is this a fast moving body that should be prevented from tunneling through
	/// other moving bodies? Note that all bodies are prevented from tunneling through
	/// static bodies.
	/// @warning You should use this flag sparingly since it increases processing time.
	bool isBullet;
};

/// A rigid body.
class b2Body
{
public:
	/// Add a shape. The shape must be created with b2World::Create.
	/// The shape must have not been already added to this or any other
	/// body. This function may be expensive so you should try to add
	/// shapes using the body definition.
	/// @param shape the shape to be added.
	void AddShape(b2Shape* shape);

	/// Convenience function, automatically creates a shape and adds it.
	/// @param shapeDef the shape definition.
	b2Shape* AddShape(b2ShapeDef* shapeDef);

	/// Remove a shape. It is up to you to destroy the shape with
	/// b2World::Destroy. This removes the shape from the broad-phase and
	/// therefore destroys any contacts associated with this shape. If you
	/// don't want to keep the shape, you can just call b2World::Destroy
	/// instead.
	/// @param shape the shape to be removed.
	void RemoveShape(b2Shape* shape);

	/// Set the mass properties. Note that this changes the center of mass position.
	/// If you are not sure how to compute mass properties, use SetMassFromShapes.
	/// @param massData the mass properties.
	void SetMass(const b2MassData* massData);

	/// Compute the mass properties from the attached shapes. You typically call this
	/// after adding all the shapes. If you add or remove shapes later, you may want
	/// to call this again. Note that this changes the center of mass position.
	void SetMassFromShapes();

	/// Set the position of the body's origin and rotation (radians).
	/// This breaks any contacts and wakes the other bodies.
	/// @param position the new world position of the body's origin (not necessarily
	/// the center of mass).
	/// @param angle the new world rotation angle of the body in radians.
	/// @return false if the movement put a shape outside the world. In this case the
	/// body is automatically frozen.
	bool SetOriginPosition(const b2Vec2& position, float32 angle);

	/// Get the position of the body's origin. The body's origin does not
	/// necessarily coincide with the center of mass. It depends on how the
	/// shapes are created.
	/// @return the world position of the body's origin.
	b2Vec2 GetOriginPosition() const;

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	float32 GetAngle() const;

	/// Get the rotation matrix.
	/// @return the world rotation matrix.
	const b2Mat22& GetRotationMatrix() const;

	/// Get the body's transform, which includes the center of mass position.
	/// The body's center of mass does not necessarily coincide with the body's origin.
	/// @return the world transform of the center of mass frame.
	const b2XForm& GetXForm() const;

	/// Set the linear velocity of the center of mass.
	/// @param v the new linear velocity of the center of mass.
	void SetLinearVelocity(const b2Vec2& v);

	/// Get the linear velocity of the center of mass.
	/// @return the linear velocity of the center of mass.
	b2Vec2 GetLinearVelocity() const;

	/// Set the angular velocity.
	/// @param omega the new angular velocity in radians/second.
	void SetAngularVelocity(float32 omega);

	/// Get the angular velocity.
	/// @return the angular velocity in radians/second.
	float32 GetAngularVelocity() const;

	/// Apply a force at a world point. Additive. If the force is not
	/// applied at the center of mass, it will generate a torque and
	/// affect the angular velocity. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param point the world position of the point of application.
	void ApplyForce(const b2Vec2& force, const b2Vec2& point);

	/// Apply a torque. Additive. This affects the angular velocity
	/// without affecting the linear velocity of the center of mass.
	/// This wakes up the body.
	/// @param torque about the z-axis (out of the screen), usually in N-m.
	void ApplyTorque(float32 torque);

	/// Apply an impulse at a point. This immediately modifies the velocity.
	/// It also modifies the angular velocity if the point of application
	/// is not at the center of mass. This wakes up the body.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param point the world position of the point of application.
	void ApplyImpulse(const b2Vec2& impulse, const b2Vec2& point);

	/// Get the total mass of the body.
	/// @return the mass, usually in kilograms (kg).
	float32 GetMass() const;

	/// Get the central rotational inertia of the body.
	/// @return the rotational inertia, usually in kg-m^2.
	float32 GetInertia() const;

	/// Get the world coordinates of a point given the local coordinates.
	/// @param localPoint a point on the body measured relative the the body's origin.
	/// @return the same point expressed in world coordinates.
	b2Vec2 GetWorldPoint(const b2Vec2& localPoint);

	/// Get the world coordinates of a vector given the local coordinates.
	/// @param localVector a vector fixed in the body.
	/// @return the same vector expressed in world coordinates.
	b2Vec2 GetWorldVector(const b2Vec2& localVector);

	/// Gets a local point relative to the body's origin given a world point.
	/// @param a point in world coordinates.
	/// @return the corresponding local point relative to the body's origin.
	b2Vec2 GetLocalPoint(const b2Vec2& worldPoint);

	/// Gets a local vector given a world vector.
	/// @param a vector in world coordinates.
	/// @return the corresponding local vector.
	b2Vec2 GetLocalVector(const b2Vec2& worldVector);

	/// Is this body treated like a bullet for continuous collision detection?
	bool IsBullet() const;

	/// Should this body be treated like a bullet for continuous collision detection?
	void SetBullet(bool flag);

	/// Is this body static (immovable)?
	bool IsStatic() const;

	/// Is this body frozen?
	bool IsFrozen() const;

	/// Is this body sleeping (not simulating).
	bool IsSleeping() const;

	/// You can disable sleeping on this body.
	void AllowSleeping(bool flag);

	/// Wake up this body so it will begin simulating.
	void WakeUp();

	/// Get the list of all shapes attached to this body.
	b2Shape* GetShapeList();

	/// Get the list of all joints attached to this body.
	b2JointNode* GetJointList();

	/// Get the list of all contacts attached to this body.
	b2ContactNode* GetContactList();

	/// Get the next body in the world's body list.
	b2Body* GetNext();

	/// Get the user data pointer that was provided in the body definition.
	void* GetUserData();

	//--------------- Internals Below -------------------

	// m_flags
	enum
	{
		e_staticFlag		= 0x0001,
		e_frozenFlag		= 0x0002,
		e_islandFlag		= 0x0004,
		e_sleepFlag			= 0x0008,
		e_allowSleepFlag	= 0x0010,
		e_bulletFlag		= 0x0020,
		e_fixedRotationFlag	= 0x0040,
	};

	b2Body(const b2BodyDef* bd, b2World* world);
	~b2Body();

	void ComputeMass();

	bool SynchronizeShapes();

	// This is used to prevent connected bodies from colliding.
	// It may lie, depending on the collideConnected flag.
	bool IsConnected(const b2Body* other) const;

	void GetSweep(b2Sweep* sweep) const;
	void GetSweep(b2Sweep* sweep, float32 t) const;

	void SetTOI(float32 toi);
	float32 GetTOI() const;

	void SetStepFraction(float32 t);

	uint32 m_flags;

	b2XForm m_xf;	// center of mass transform
	float32 m_angle;

	// Conservative advancement data.
	b2Vec2 m_position0;
	float32 m_angle0;
	float32 m_t;

	b2Vec2 m_linearVelocity;
	float32 m_angularVelocity;

	b2Vec2 m_force;
	float32 m_torque;

	b2Vec2 m_center;	// local vector from client origin to center of mass

	b2World* m_world;
	b2Body* m_prev;
	b2Body* m_next;

	b2Shape* m_shapeList;
	int32 m_shapeCount;

	b2JointNode* m_jointList;
	b2ContactNode* m_contactList;

	float32 m_mass, m_invMass;
	float32 m_I, m_invI;

	float32 m_linearDamping;
	float32 m_angularDamping;

	float32 m_sleepTime;

	void* m_userData;
};

inline b2Vec2 b2Body::GetOriginPosition() const
{
	return b2Mul(m_xf, -m_center);
}

inline const b2Mat22& b2Body::GetRotationMatrix() const
{
	return m_xf.R;
}

inline float32 b2Body::GetAngle() const
{
	return m_angle;
}

inline const b2XForm& b2Body::GetXForm() const
{
	return m_xf;
}

inline void b2Body::SetLinearVelocity(const b2Vec2& v)
{
	m_linearVelocity = v;
}

inline b2Vec2 b2Body::GetLinearVelocity() const
{
	return m_linearVelocity;
}

inline void b2Body::SetAngularVelocity(float32 w)
{
	m_angularVelocity = w;
}

inline float32 b2Body::GetAngularVelocity() const
{
	return m_angularVelocity;
}

inline float32 b2Body::GetMass() const
{
	return m_mass;
}

inline float32 b2Body::GetInertia() const
{
	return m_I;
}

inline b2Vec2 b2Body::GetWorldPoint(const b2Vec2& localPoint)
{
	return b2Mul(m_xf, localPoint);
}

inline b2Vec2 b2Body::GetWorldVector(const b2Vec2& localVector)
{
	return b2Mul(m_xf.R, localVector);
}

inline b2Vec2 b2Body::GetLocalPoint(const b2Vec2& worldPoint)
{
	return b2MulT(m_xf, worldPoint);
}

inline b2Vec2 b2Body::GetLocalVector(const b2Vec2& worldVector)
{
	return b2MulT(m_xf.R, worldVector);
}

inline bool b2Body::IsBullet() const
{
	return (m_flags & e_bulletFlag) == e_bulletFlag;
}

inline void b2Body::SetBullet(bool flag)
{
	if (flag)
	{
		m_flags |= e_bulletFlag;
	}
	else
	{
		m_flags &= ~e_bulletFlag;
	}
}

inline bool b2Body::IsStatic() const
{
	return (m_flags & e_staticFlag) == e_staticFlag;
}

inline bool b2Body::IsFrozen() const
{
	return (m_flags & e_frozenFlag) == e_frozenFlag;
}

inline bool b2Body::IsSleeping() const
{
	return (m_flags & e_sleepFlag) == e_sleepFlag;
}

inline void b2Body::AllowSleeping(bool flag)
{
	if (flag)
	{
		m_flags |= e_allowSleepFlag;
	}
	else
	{
		m_flags &= ~e_allowSleepFlag;
		WakeUp();
	}
}

inline void b2Body::WakeUp()
{
	m_flags &= ~e_sleepFlag;
	m_sleepTime = 0.0f;
}

inline b2Shape* b2Body::GetShapeList()
{
	return m_shapeList;
}

inline b2ContactNode* b2Body::GetContactList()
{
	return m_contactList;
}

inline b2JointNode* b2Body::GetJointList()
{
	return m_jointList;
}

inline b2Body* b2Body::GetNext()
{
	return m_next;
}

inline void* b2Body::GetUserData()
{
	return m_userData;
}

inline bool b2Body::IsConnected(const b2Body* other) const
{
	for (b2JointNode* jn = m_jointList; jn; jn = jn->next)
	{
		if (jn->other == other)
			return jn->joint->m_collideConnected == false;
	}

	return false;
}

inline void b2Body::ApplyForce(const b2Vec2& force, const b2Vec2& point)
{
	WakeUp();
	m_force += force;
	m_torque += b2Cross(point - m_xf.position, force);
}

inline void b2Body::ApplyTorque(float32 torque)
{
	WakeUp();
	m_torque += torque;
}

inline void b2Body::ApplyImpulse(const b2Vec2& impulse, const b2Vec2& point)
{
	WakeUp();
	m_linearVelocity += m_invMass * impulse;
	m_angularVelocity += m_invI * b2Cross(point - m_xf.position, impulse);
}

inline void b2Body::GetSweep(b2Sweep* sweep) const
{
	sweep->position = m_position0;
	sweep->angle = m_angle0;
	sweep->velocity = m_xf.position - m_position0;
	sweep->omega = m_angle - m_angle0;
}

inline void b2Body::GetSweep(b2Sweep* sweep, float32 t) const
{
	float32 dt = t - m_t;
	b2Assert(dt >= 0.0f);

	b2Vec2 p0 = m_position0 + dt * (m_xf.position - m_position0);
	float32 a0 = m_angle0 + dt * (m_angle - m_angle0);

	sweep->position = p0;
	sweep->angle = a0;
	sweep->velocity = m_xf.position - p0;
	sweep->omega = m_angle - a0;
}

inline void b2Body::SetTOI(float32 toi)
{
	m_t = toi;
}

inline float32 b2Body::GetTOI() const
{
	return m_t;
}

inline void b2Body::SetStepFraction(float32 t)
{
	b2Assert(m_t < 1.0f);
	float32 s = (t - m_t) / (1.0f - m_t);
	m_position0 += s * (m_xf.position - m_position0);
	m_angle0 += s * (m_angle - m_angle0);
	m_xf.position = m_position0;
	m_angle = m_angle0;
	m_xf.R.Set(m_angle);
	m_t = t;
}

#endif
