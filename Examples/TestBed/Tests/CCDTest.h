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

#ifndef CCD_TEST_H
#define CCD_TEST_H

class CCDTest : public Test
{
public:

	CCDTest()
	{
#if 0
		m_world->m_gravity.SetZero();

		{
			b2PolygonDef sd;
			sd.SetAsBox(0.1f, 10.0f);
			sd.density = 0.0f;
			b2Shape* shape = m_world->Create(&sd);

			b2BodyDef bd;
			bd.position.Set(0.0f, 20.0f);
			bd.AddShape(shape);
			m_world->Create(&bd);
		}

		{
			b2PolygonDef sd;
			sd.SetAsBox(0.1f, 2.0f);
			sd.density = 1.0f;
			sd.restitution = 0.0f;
			b2Shape* shape = m_world->Create(&sd);

			m_angularVelocity = b2Random(-50.0f, 50.0f);
			//m_angularVelocity = 39.596241f;

			b2BodyDef bd;
			bd.position.Set(50.0f, 20.0f);
			bd.AddShape(shape);
			b2Body* body = m_world->Create(&bd);
			body->SetLinearVelocity(b2Vec2(-200.0f, 0.0f));
			body->SetAngularVelocity(m_angularVelocity);
		}
#elif 0
		{
			b2PolygonDef sd;
			sd.SetAsBox(10.0f, 0.1f);
			sd.density = 0.0f;

			b2BodyDef bd;
			bd.position.Set(0.0f, -0.2f);
			bd.AddShape(m_world->Create(&sd));
			m_world->Create(&bd);
		}

		{
			b2PolygonDef sd;
			sd.SetAsBox(2.0f, 0.1f);
			sd.density = 1.0f;
			sd.restitution = 0.0f;

			b2BodyDef bd1;
			bd1.isBullet = true;
			bd1.allowSleep = false;
			bd1.position.Set(0.0f, 20.0f);
			bd1.AddShape(m_world->Create(&sd));
			b2Body* b1 = m_world->Create(&bd1);
			b1->SetLinearVelocity(b2Vec2(0.0f, -200.0f));

			sd.SetAsBox(1.0f, 0.1f);
			b2BodyDef bd2;
			bd2.isBullet = true;
			bd2.allowSleep = false;
			bd2.position.Set(0.0f, 20.2f);
			bd2.AddShape(m_world->Create(&sd));
			b2Body* b2 = m_world->Create(&bd2);
			b2->SetLinearVelocity(b2Vec2(0.0f, -201.0f));

			//sd.SetAsBox(0.5f, 0.1f);
			//b2BodyDef bd3;
			//bd3.isBullet = true;
			//bd3.allowSleep = false;
			//bd3.position.Set(0.0f, 20.4f);
			//bd3.AddShape(m_world->Create(&sd));
			//b2Body* b3 = m_world->Create(&bd3);
			//b3->SetLinearVelocity(b2Vec2(0.0f, -202.0f));
		}
#else
		const float32 k_restitution = 1.1f;

		{
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

			b2BodyDef bd;
			bd.position.Set(0.0f, 20.0f);
			bd.AddShape(shape1);
			bd.AddShape(shape2);
			bd.AddShape(shape3);
			bd.AddShape(shape4);
			m_world->Create(&bd);
		}

		{
			b2PolygonDef sd;
			sd.SetAsBox(0.1f, 4.0f);
			sd.density = 1.0f;
			sd.restitution = 0.8f;
			b2Shape* shape = m_world->Create(&sd);

			m_angularVelocity = b2Random(-50.0f, 50.0f);

			b2BodyDef bd;
			bd.position.Set(-5.0f, 20.0f);
			bd.isBullet = true;
			bd.AddShape(shape);
			b2Body* body = m_world->Create(&bd);
			body->SetAngularVelocity(m_angularVelocity);
		}

		{
			b2CircleDef sd;
			sd.radius = 0.25f;
			sd.density = 1.0f;
			sd.restitution = 0.8f;
			b2Shape* shape = m_world->Create(&sd);

			m_angularVelocity = b2Random(-50.0f, 50.0f);

			b2BodyDef bd;
			bd.position.Set(5.0f, 25.0f);
			bd.isBullet = true;
			bd.AddShape(shape);
			b2Body* body = m_world->Create(&bd);
			body->SetAngularVelocity(m_angularVelocity);
		}
#endif
	}

	static Test* Create()
	{
		return new CCDTest;
	}

	float32 m_angularVelocity;
};

#endif
