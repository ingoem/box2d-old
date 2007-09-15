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

#ifndef COMPOUND_SHAPES_H
#define COMPOUND_SHAPES_H

#include "TestBed/Framework/Test.h"
#include "Engine/Dynamics/b2Body.h"
#include "Engine/Collision/b2Shape.h"

class CompoundShapes : public Test
{
public:
	CompoundShapes()
	{
		{
			b2BoxDef sd;
			sd.type = e_boxShape;
			sd.extents.Set(50.0f, 10.0f);

			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			bd.AddShape(&sd);
			b2Body* body = m_world->CreateBody(&bd);
		}

		{
			b2BoxDef sd1;
			sd1.type = e_boxShape;
			sd1.extents.Set(0.25f, 0.5f);
			//sd1.localPosition.Set(-0.5f, 0.0f);
			sd1.density = 1.0f;

			b2BoxDef sd2;
			sd2.type = e_boxShape;
			sd2.extents.Set(0.25f, 0.5f);
			//sd2.localPosition.Set(0.5f, 0.0f);
			sd2.localPosition.Set(0.0f, -0.5f);
			sd2.localRotation = 0.5f * b2_pi;
			sd2.density = 1.0f;

			b2BodyDef bd;
			bd.AddShape(&sd1);
			bd.AddShape(&sd2);

			for (int i = 0; i < 100; ++i)
			{
				float32 x = b2Random(-0.1f, 0.1f);
				bd.position.Set(x, 1.05f + 1.5f * i);
				//bd.position.Set(0.0f, 0.45f);
				bd.rotation = b2Random(-b2_pi, b2_pi);

				b2Body* body = m_world->CreateBody(&bd);
			}
		}
	}

	static Test* Create()
	{
		return new CompoundShapes;
	}
};

#endif
