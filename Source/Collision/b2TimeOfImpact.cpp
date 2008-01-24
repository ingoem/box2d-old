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
float32 b2TimeOfImpact(const b2Shape* shape1, const b2Sweep& sweep1,
					   const b2Shape* shape2, const b2Sweep& sweep2,
					   float32 maxTOI)
{
	b2Vec2 x1Start = sweep1.position;
	float32 a1Start = sweep1.angle;
	b2Vec2 v1 = sweep1.velocity;
	float32 omega1 = sweep1.omega;

	b2Vec2 x2Start = sweep2.position;
	float32 a2Start = sweep2.angle;
	b2Vec2 v2 = sweep2.velocity;
	float32 omega2 = sweep2.omega;

	float32 r1 = shape1->GetMaxRadius();
	float32 r2 = shape2->GetMaxRadius();

	b2Vec2 x1 = x1Start;
	float32 a1 = a1Start;
	b2Vec2 x2 = x2Start;
	float32 a2 = a2Start;

	b2XForm xf1, xf2;
	xf1.position = x1;
	xf1.R.Set(a1);

	xf2.position = x2;
	xf2.R.Set(a2);

	b2Vec2 p1, p2;
	float32 t1 = 0.0f;
	const int32 maxIterations = 20;
	int32 iter = 0;
	b2Vec2 normal = b2Vec2_zero;
	float32 distance = 0.0f;
	float32 targetDistance = 0.0f;
	for(;;)
	{
		// Get the accurate distance between shapes.
		distance = b2Distance(&p1, &p2, shape1, xf1, shape2, xf2);

		if (iter == 0)
		{
			if (distance > 2.0f * b2_toiSlop)
			{
				targetDistance = 1.5f * b2_toiSlop;
			}
			else
			{
				targetDistance = b2Max(0.05f * b2_toiSlop, distance - 0.5f * b2_toiSlop);
			}
		}

		if (distance - targetDistance < 0.05f * b2_toiSlop || iter == maxIterations)
		{
			break;
		}

		normal = p2 - p1;
		normal.Normalize();

		// Compute upper bound on remaining movement.
		float32 approachVelocityBound = b2Dot(normal, v1 - v2) + b2Abs(omega1) * r1 + b2Abs(omega2) * r2;
		if (b2Abs(approachVelocityBound) < FLT_EPSILON)
		{
			t1 = 1.0f;
			break;
		}

		// Get the conservative time increment. Don't advance all the way.
		float32 dt = (distance - targetDistance) / approachVelocityBound;
		//float32 dt = (distance - 0.5f * b2_linearSlop) / approachVelocityBound;
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

		if (t2 > maxTOI)
		{
			t1 = 1.0f;
			break;
		}

		t1 = t2;

		// Move forward conservatively.
		x1 = x1Start + t1 * v1;
		a1 = a1Start + t1 * omega1;
		x2 = x2Start + t1 * v2;
		a2 = a2Start + t1 * omega2;

		xf1.position = x1;
		xf1.R.Set(a1);

		xf2.position = x2;
		xf2.R.Set(a2);

		++iter;
	}

	// Debug
	if (iter == maxIterations)
	{
		iter += 0;
	}

	return t1;
}
