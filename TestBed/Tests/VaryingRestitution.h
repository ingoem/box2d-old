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

#ifndef VARYING_RESTITUTION_H
#define VARYING_RESTITUTION_H

#include "TestBed/Framework/Test.h"
#include "Engine/Dynamics/b2Body.h"
#include "Engine/Dynamics/b2World.h"
#include "Engine/Collision/b2Shape.h"

class VaryingRestitution : public Test
{
public:

	VaryingRestitution()
	{
		{
			b2BoxDef sd;
			sd.type = e_boxShape;
			sd.extents.Set(50.0f, 10.0f);

			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			bd.AddShape(&sd);
			m_world->CreateBody(&bd);
		}

		{
			b2PolyDef sd;
			sd.type = e_polyShape;
			sd.vertexCount = 8;
			float32 w = 1.5f;
			float32 b = w / (2.0f + sqrtf(2.0f));
			float32 s = sqrtf(2.0f) * b;
			sd.vertices[0].Set(0.5f * s, 0.0f);
			sd.vertices[1].Set(0.5f * w, b);
			sd.vertices[2].Set(0.5f * w, b + s);
			sd.vertices[3].Set(0.5f * s, w);
			sd.vertices[4].Set(-0.5f * s, w);
			sd.vertices[5].Set(-0.5f * w, b + s);
			sd.vertices[6].Set(-0.5f * w, b);
			sd.vertices[7].Set(-0.5f * s, 0.0f);
			sd.density = 5.0f;

			b2BodyDef bd;
			bd.AddShape(&sd);

			float32 restitution[7] = {0.0f, 0.1f, 0.3f, 0.5f, 0.75f, 0.9f, 1.0f};

			for (int32 i = 0; i < 7; ++i)
			{
				sd.restitution = restitution[i];
				bd.position.Set(-10.0f + 3.0f * i, 10.0f);
				m_world->CreateBody(&bd);
			}
		}
	}

	static Test* Create()
	{
		return new VaryingRestitution;
	}
};

#endif
