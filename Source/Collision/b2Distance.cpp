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
#include "b2Shape.h"

#if 1

float32 b2Distance(b2Vec2* p1Out, b2Vec2* p2Out, const b2Shape* shape1, const b2Shape* shape2)
{
	b2Vec2 p1s[3], p2s[3];
	b2Vec2 points[3];
	int32 pointCount = 0;

	b2Vec2 v = shape2->m_position - shape1->m_position;

	const int32 maxIterations = 20;
	for (int32 iter = 0; iter < maxIterations; ++iter)
	{
		b2Vec2 w1 = shape1->Support(-v);
		b2Vec2 w2 = shape2->Support(v);
		b2Vec2 w = w2 - w1;
		float32 vSqr = b2Dot(v, v);
		if (vSqr - b2Dot(v, w) <= 0.01f * vSqr) // or w in points
		{
			if (pointCount == 0)
			{
				*p1Out = w1;
				*p2Out = w2;
			}
			return sqrtf(vSqr);
		}

		switch (pointCount)
		{
		case 0:
			p1s[0] = w1;
			p2s[0] = w2;
			points[0] = w;
			*p1Out = p1s[0];
			*p2Out = p2s[0];
			++pointCount;
			break;
			
		case 1:
			p1s[1] = w1;
			p2s[1] = w2;
			points[1] = w;
			//pointCount = ProcessTwo(p1Out, p2Out, p1s, ps2, points);
			break;

		case 2:
			p1s[2] = w1;
			p2s[2] = w2;
			points[2] = w;
			//pointCount = ProcessThree(p1Out, p2Out, p1s, p2s, points);
			break;
		}

		float32 maxSqr = -FLT_MAX;
		for (int32 i = 0; i < pointCount; ++i)
		{
			maxSqr = b2Max(maxSqr, b2Dot(points[i], points[i]));
		}

		if (pointCount == 3 || vSqr <= 0.01f * maxSqr)
		{
			break;
		}
	}

	return 0.0f;
}

#endif