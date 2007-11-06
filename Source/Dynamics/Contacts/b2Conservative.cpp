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

void b2Conservative(b2Shape* shape1, b2Shape* shape2)
{
	b2Body* body1 = shape1->GetBody();
	b2Body* body2 = shape2->GetBody();
	b2Vec2 v1 = body1->m_position - body1->m_position0;
	float32 w1 = body1->m_rotation - body1->m_rotation0;
	b2Vec2 v2 = body2->m_position - body2->m_position0;
	float32 w2 = body2->m_rotation - body2->m_rotation0;

	float32 r1 = shape1->GetMaxRadius();
	float32 r2 = shape2->GetMaxRadius();

	const int32 maxIterations = 10;
	for (int32 iter = 0; iter < maxIterations; ++iter)
	{
		// Get the accurate distance between shapes.
		b2Vec2 x1, x2;
		float32 distance = b2Distance(&x1, &x2, shape1, shape2);
		if (distance == 0.0f)
		{
			b2Assert(false);
			break;
		}

		b2Vec2 d = x2 - x1;
		d.Normalize();

		// Get the conservative movement.
		float32 movement = b2Dot(d, v1 - v2) + w1 * r1 + w2 * r2;

		if (movement <= distance)
		{
			break;
		}

		float32 dt = distance / movement;
		dt;
	}

		



}
