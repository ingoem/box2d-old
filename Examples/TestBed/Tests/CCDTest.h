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
#if 1
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

			//m_angularVelocity = b2Random(-50.0f, 50.0f);
			m_angularVelocity = -35.268715f;

			b2BodyDef bd;
			bd.position.Set(50.0f, 20.0f);
			bd.AddShape(shape);
			b2Body* body = m_world->Create(&bd);
			body->SetLinearVelocity(b2Vec2(-200.0f, 0.0f));
			body->SetAngularVelocity(m_angularVelocity);
		}
#else
		{
			b2PolygonDef sd;
			sd.SetAsBox(10.0f, 0.1f);
			sd.density = 0.0f;

			b2BodyDef bd;
			bd.position.Set(0.0f, -0.2f);
			bd.AddShape(&sd);
			m_world->Create(&bd);
		}

		{
			b2PolygonDef sd;
			sd.SetAsBox(2.0f, 0.1f);
			sd.density = 1.0f;
			sd.restitution = 0.0f;

			b2BodyDef bd;
			bd.isBullet = true;
			bd.position.Set(0.0f, 20.0f);
			bd.linearVelocity.Set(0.0f, -200.0f);
			bd.AddShape(&sd);
			m_world->Create(&bd);

			sd.SetAsBox(1.0f, 0.1f);
			bd.position.Set(0.0f, 20.2f);
			bd.linearVelocity.Set(0.0f, -201.0f);
			m_world->Create(&bd);

			sd.SetAsBox(0.5f, 0.1f);
			bd.position.Set(0.0f, 20.4f);
			bd.linearVelocity.Set(0.0f, -202.0f);
			m_world->Create(&bd);
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
