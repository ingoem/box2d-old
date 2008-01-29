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

#ifndef DISTANCE_TEST_H
#define DISTANCE_TEST_H

class DistanceTest : public Test
{
public:
	DistanceTest()
	{
#if 0
		{
			b2PolygonDef sd;
			sd.SetAsBox(50.0f, 10.0f);
			sd.friction = 0.3f;
			m_shape1 = m_world->Create(&sd);

			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			body->Create(m_shape1);
			m_body1 = m_world->Create(&bd);
		}
#else
		{
			b2PolygonDef sd;
			sd.SetAsBox(1.0f, 1.0f);
			sd.density = 0.0f;
			m_shape1 = m_world->Create(&sd);

			b2BodyDef bd;
			bd.position.Set(0.0f, 10.0f);
			body->Create(m_shape1);
			m_body1 = m_world->Create(&bd);
		}
#endif

		{
#if 0
			b2PolygonDef sd;
			float32 a = 0.25f;
			sd.SetAsBox(a, a);
			sd.density = 1.0f;
#elif 0
			b2CircleDef sd;
			sd.radius = 0.5f;
			sd.density = 1.0f;
#else
			b2PolygonDef sd;
			sd.vertexCount = 3;
			sd.vertices[0].Set(-1.0f, 0.0f);
			sd.vertices[1].Set(1.0f, 0.0f);
			sd.vertices[2].Set(0.0f, 15.0f);
			sd.density = 1.0f;
#endif
			m_shape2 = m_world->Create(&sd);

			b2BodyDef bd;
#if 0
			bd.position.Set(-48.377853f, 0.49244255f);
			bd.rotation = 90.475891f;
#else
			bd.position.Set(0.0f, 10.0f);
#endif
			body->Create(m_shape2);
			m_body2 = m_world->Create(&bd);
		}

		m_world->m_gravity.Set(0.0f, 0.0f);
		b2World::s_enablePositionCorrection = false;
	}

	~DistanceTest()
	{
		b2World::s_enablePositionCorrection = true;
	}

	static Test* Create()
	{
		return new DistanceTest;
	}

	void Step(Settings* settings)
	{
		B2_NOT_USED(settings);

		uint32 flags = 0;
		flags += settings->drawShapes			* b2DebugDraw::e_shapeBit;
		flags += settings->drawJoints			* b2DebugDraw::e_jointBit;
		flags += settings->drawCoreShapes		* b2DebugDraw::e_coreShapeBit;
		flags += settings->drawAABBs			* b2DebugDraw::e_aabbBit;
		flags += settings->drawOBBs				* b2DebugDraw::e_obbBit;
		flags += settings->drawPairs			* b2DebugDraw::e_pairBit;
		flags += settings->drawContactPoints	* b2DebugDraw::e_contactPointBit;
		flags += settings->drawContactNormals	* b2DebugDraw::e_contactNormalBit;
		flags += settings->drawContactForces	* b2DebugDraw::e_contactForceBit;
		flags += settings->drawFrictionForces	* b2DebugDraw::e_frictionForceBit;
		flags += settings->drawCOMs				* b2DebugDraw::e_centerOfMassBit;
		m_debugDraw.SetFlags(flags);

		m_world->Step(0.0f, 0);

		b2Vec2 x1, x2;
		float32 distance = b2Distance(&x1, &x2, m_shape1, m_body1->GetXForm(), m_shape2, m_body2->GetXForm());

		DrawString(5, m_textLine, "distance = %g", distance);
		m_textLine += 15;

		extern int32 g_GJK_Iterations;

		DrawString(5, m_textLine, "iterations = %d", g_GJK_Iterations);
		m_textLine += 15;

		glPointSize(4.0f);
		glColor3f(1.0f, 0.0f, 0.0f);
		glBegin(GL_POINTS);
		glVertex2f(x1.x, x1.y);
		glVertex2f(x2.x, x2.y);
		glEnd();
		glPointSize(1.0f);

		glColor3f(1.0f, 1.0f, 0.0f);
		glBegin(GL_LINES);
		glVertex2f(x1.x, x1.y);
		glVertex2f(x2.x, x2.y);
		glEnd();
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'a':
			m_body2->m_xf.position.x -= 0.1f;
			m_body2->SynchronizeShapes();
			break;

		case 'd':
			m_body2->m_xf.position.x += 0.1f;
			m_body2->SynchronizeShapes();
			break;

		case 's':
			m_body2->m_xf.position.y -= 0.1f;
			m_body2->SynchronizeShapes();
			break;

		case 'w':
			m_body2->m_xf.position.y += 0.1f;
			m_body2->SynchronizeShapes();
			break;

		case 'q':
			m_body2->m_angle += 0.1f * b2_pi;
			m_body2->m_xf.R.Set(m_body2->m_angle);
			m_body2->SynchronizeShapes();
			break;

		case 'e':
			m_body2->m_angle -= 0.1f * b2_pi;
			m_body2->m_xf.R.Set(m_body2->m_angle);
			m_body2->SynchronizeShapes();
			break;
		}
	}

	b2Body* m_body1;
	b2Body* m_body2;
	b2Shape* m_shape1;
	b2Shape* m_shape2;
};

#endif
