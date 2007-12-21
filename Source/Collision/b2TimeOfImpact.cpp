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

#include "b2Collision.h"
#include "Shapes/b2Shape.h"

// This algorithm uses conservative advancement to compute the time of
// impact (TOI) of two shapes.
// Refs: Bullet, Young Kim
template <typename T1, typename T2>
float32 b2TimeOfImpact(const T1* shape1, const b2Sweep& sweep1,
					   const T2* shape2, const b2Sweep& sweep2)
{
	b2Vec2 p1Start = sweep1.position;
	float32 a1Start = sweep1.theta;
	b2Vec2 v1 = sweep1.velocity;
	float32 omega1 = sweep1.omega;

	b2Vec2 p2Start = sweep2.position;
	float32 a2Start = sweep2.theta;
	b2Vec2 v2 = sweep2.velocity;
	float32 omega2 = sweep2.omega;

	float32 r1 = shape1->GetMaxRadius();
	float32 r2 = shape2->GetMaxRadius();

	b2Vec2 p1 = p1Start;
	float32 a1 = a1Start;
	b2Vec2 p2 = p2Start;
	float32 a2 = a2Start;

	b2XForm xf1, xf2;
	xf1.position = p1;
	xf1.R.Set(a1);

	xf2.position = p2;
	xf2.R.Set(a2);

	float32 t1 = 0.0f;
	const int32 maxIterations = 50;
	for (int32 iter = 0; iter < maxIterations; ++iter)
	{
		// Get the accurate distance between shapes.
		b2Vec2 x1, x2;
		float32 distance = b2Distance(&x1, &x2, shape1, xf1, shape2, xf2);
		if (distance < b2_linearSlop)
		{
			break;
		}

		// Compute upper bound on remaining movement.
		b2Vec2 d = x2 - x1;
		d.Normalize();
		float32 relativeVelocity = (1.0f - t1) * (b2Dot(d, v1 - v2) + b2Abs(omega1) * r1 + b2Abs(omega2) * r2);
		if (b2Abs(relativeVelocity) < FLT_EPSILON)
		{
			t1 = 1.0f;
			break;
		}

		// Get the conservative time increment.
		float32 dt = distance / relativeVelocity;
		float32 t2 = t1 + dt;

		// The shapes may be moving apart.
		if (t2 < 0.0f || 1.0f < t2)
		{
			t1 = 1.0f;
			break;
		}

		// Ensure significant advancement.
		if (t2 < (1.0f + 100.0f * FLT_EPSILON) * t1)
		{
			break;
		}

		t1 = t2;

		// Move forward conservatively.
		p1 = p1Start + t1 * v1;
		a1 = a1Start + t1 * omega1;
		p2 = p2Start + t1 * v2;
		a2 = a2Start + t1 * omega2;

		xf1.position = p1;
		xf1.R.Set(a1);

		xf2.position = p2;
		xf2.R.Set(a2);
	}

	return t1;
}
