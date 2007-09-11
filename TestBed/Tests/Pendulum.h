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

#ifndef PENDULUM_H
#define PENDULUM_H

#include "TestBed/Framework/Test.h"
#include "Engine/Dynamics/b2Body.h"
#include "Engine/Dynamics/b2World.h"
#include "Engine/Collision/b2Shape.h"
#include "Engine/Dynamics/Joints/b2RevoluteJoint.h"

class Pendulum : public Test
{
public:
	Pendulum()
	{
		b2Body* ground = NULL;
		{
			b2ShapeDescription sd;
			sd.type = e_boxShape;
			sd.box.m_extents.Set(50.0f, 10.0f);

			b2BodyDescription bd;
			bd.position.Set(0.0f, -10.0f);
			bd.AddShape(&sd);
			ground = m_world->CreateBody(&bd);
		}

		{
			b2ShapeDescription sd;
			sd.type = e_boxShape;
			sd.box.m_extents.Set(0.375f, 0.125f);
			sd.density = 20.0f;
			sd.friction = 0.2f;

			b2BodyDescription bd;
			bd.AddShape(&sd);

			b2RevoluteDescription jd;

			const float32 y = 25.0f;
			b2Body* prevBody = ground;
			for (int32 i = 0; i < 30; ++i)
			{
				bd.position.Set(0.5f + i, y);
				b2Body* body = m_world->CreateBody(&bd);

				jd.anchorPoint.Set((float32)i, y);
				jd.body1 = prevBody;
				jd.body2 = body;
				m_world->CreateJoint(&jd);

				prevBody = body;
			}
		}
	}

	static Test* Create()
	{
		return new Pendulum;
	}
};

#endif
