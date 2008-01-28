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

#ifndef BIPED_TEST_H
#define BIPED_TEST_H

#include "Biped.h"

class BipedTest : public Test
{
public:

	BipedTest()
	{
		const float32 k_restitution = 1.4f;

		{
			b2BodyDef bd;
			bd.type = b2BodyDef::e_staticBody;
			bd.position.Set(0.0f, 20.0f);
			b2Body* body = m_world->Create(&bd);

			b2PolygonDef sd;
			sd.density = 0.0f;
			sd.restitution = k_restitution;

			sd.SetAsBox(0.1f, 10.0f, b2Vec2(-10.0f, 0.0f), 0.0f);
			b2Shape* shape1 = m_world->Create(&sd);

			sd.SetAsBox(0.1f, 10.0f, b2Vec2(10.0f, 0.0f), 0.0f);
			b2Shape* shape2 = m_world->Create(&sd);

			sd.SetAsBox(0.1f, 10.0f, b2Vec2(0.0f, -10.0f), 0.5f * b2_pi);
			b2Shape* shape3 = m_world->Create(&sd);

			sd.SetAsBox(0.1f, 10.0f, b2Vec2(0.0f, 10.0f), -0.5f * b2_pi);
			b2Shape* shape4 = m_world->Create(&sd);

			body->AddShape(shape1);
			body->AddShape(shape2);
			body->AddShape(shape3);
			body->AddShape(shape4);
		}

		m_biped = new Biped(m_world, b2Vec2(0.0f, 20.0f));

		for (int32 i = 0; i < 8; ++i)
		{
			b2BodyDef bd;
			bd.type = b2BodyDef::e_dynamicBody;
			bd.position.Set(5.0f, 20.0f + i);
			bd.isBullet = true;
			b2Body* body = m_world->Create(&bd);
			body->SetLinearVelocity(b2Vec2(0.0f, -100.0f));
			body->SetAngularVelocity(b2Random(-50.0f, 50.0f));

			b2CircleDef sd;
			sd.radius = 0.25f;
			sd.density = 15.0f;
			sd.restitution = k_restitution;
			b2Shape* shape = m_world->Create(&sd);
			body->AddShape(shape);
			body->SetMassFromShapes();
		}
	}

	~BipedTest()
	{
		delete m_biped;
	}

	static Test* Create()
	{
		return new BipedTest;
	}

	Biped* m_biped;
};

#endif
