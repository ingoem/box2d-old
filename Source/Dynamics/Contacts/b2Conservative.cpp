/*
* Copyright (c) 2007 Erin Catto http://www.gphysics.com
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

#include "b2Conservative.h"
#include "../../Collision/b2Collision.h"
#include "../../Collision/b2Shape.h"
#include "../b2Body.h"

// This algorithm uses conservative advancement to compute the time of
// impact (TOI) of two shapes.
// Refs: Bullet, Young Kim
bool b2Conservative(b2Shape* shape1, b2Shape* shape2, bool touching)
{
	b2Body* body1 = shape1->GetBody();
	b2Body* body2 = shape2->GetBody();

	float32 toi1 = body1->m_toi;
	float32 toi2 = body2->m_toi;
	float32 bestTOI = b2Min(toi1, toi2);
	NOT_USED(bestTOI);

	b2Vec2 v1 = body1->m_position - body1->m_position0;
	float32 omega1 = body1->m_rotation - body1->m_rotation0;
	b2Vec2 v2 = body2->m_position - body2->m_position0;
	float32 omega2 = body2->m_rotation - body2->m_rotation0;

	float32 r1 = shape1->GetMaxRadius();
	float32 r2 = shape2->GetMaxRadius();

	b2Vec2 p1Start = body1->m_position0;
	float32 a1Start = body1->m_rotation0;

	b2Vec2 p2Start = body2->m_position0;
	float32 a2Start = body2->m_rotation0;

	b2Vec2 p1 = p1Start;
	float32 a1 = a1Start;
	b2Vec2 p2 = p2Start;
	float32 a2 = a2Start;

	b2Mat22 R1(a1), R2(a2);

	shape1->QuickSync(p1, R1);
	shape2->QuickSync(p2, R2);

	float32 s1 = 0.0f;
	const int32 maxIterations = 1000;
	b2Vec2 d;
	float32 invRelativeVelocity = 0.0f;
	bool hit = true;
	b2Vec2 x1, x2;
	NOT_USED(touching);
	for (int32 iter = 0; iter < maxIterations; ++iter)
	{
		// Get the accurate distance between shapes.
		float32 distance = b2Distance(&x1, &x2, shape1, shape2);
		if (distance < b2_linearSlop)
		{
			if (iter == 0)
			{
				hit = false;
			}
			else
			{
				hit = true;
			}
			break;
		}

		if (iter == 0)
		{
			b2Vec2 d = x2 - x1;
			d.Normalize();
			float32 relativeVelocity = b2Dot(d, v1 - v2) + b2Abs(omega1) * r1 + b2Abs(omega2) * r2;
			if (b2Abs(relativeVelocity) < FLT_EPSILON)
			{
				hit = false;
				break;
			}

			invRelativeVelocity = 1.0f / relativeVelocity;
		}

		// Get the conservative movement.
		float32 ds = distance * invRelativeVelocity;
		float32 s2 = s1 + ds;

		if (s2 < 0.0f || 1.0f < s2)
		{
			hit = false;
			break;
		}

		if (s2 < (1.0f + 100.0f * FLT_EPSILON) * s1)
		{
			hit = true;
			break;
		}

		s1 = s2;

		// Move forward conservatively.
		p1 = p1Start + s1 * v1;
		a1 = a1Start + s1 * omega1;
		p2 = p2Start + s1 * v2;
		a2 = a2Start + s1 * omega2;

		R1.Set(a1);
		R2.Set(a2);
		shape1->QuickSync(p1, R1);
		shape2->QuickSync(p2, R2);
	}

	if (hit)
	{
		// Hit, move bodies to safe position and re-sync shapes.
		b2Vec2 d = x2 - x1;
		float32 length = d.Length();
		if (length > FLT_EPSILON)
		{
			d *= 5.0f * b2_linearSlop / length;
		}

		// TODO_ERIN include angular impulse.

		float32 m1 = body1->GetMass();
		float32 m2 = body2->GetMass();
		float32 m = m1 + m2;
		b2Assert(m > 0.0f);
		m = 1.0f / m;

		body1->WakeUp();
		body1->m_position = p1 - (m1 * m) * d;
		body1->m_rotation = a1;
		body1->m_R.Set(a1);
		body1->QuickSyncShapes();

		body2->WakeUp();
		body2->m_position = p2 + (m2 * m) * d;
		body2->m_rotation = a2;
		body2->m_R.Set(a2);
		body2->QuickSyncShapes();

		return true;
	}

	// No hit, restore shapes.
	shape1->QuickSync(body1->m_position, body1->m_R);
	shape2->QuickSync(body2->m_position, body2->m_R);
	return false;
}
