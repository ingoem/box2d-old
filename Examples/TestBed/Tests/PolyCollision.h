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

#ifndef POLYCOLLISION_H
#define POLYCOLLISION_H


#include "../Framework/Render.h"
#include "freeglut/GL/glut.h"

class PolyCollision : public Test
{
public:
	PolyCollision()
	{
		{
			b2BoxDef sd;
			sd.type = e_boxShape;
			sd.extents.Set(0.375f, 0.125f);
			sd.extents *= 10.0f;
			sd.density = 0.0f;

			b2BodyDef bd;
			bd.position.Set(0.0f, 10.0f);
			bd.AddShape(&sd);
			m_body1 = m_world->CreateBody(&bd);
		}

		{
			b2BoxDef sd;
			float32 a = 0.5f;
			sd.type = e_boxShape;
			sd.extents.Set(a, a);
			sd.extents *= 10.0f;
			sd.density = 1.0f;

			b2BodyDef bd;
			bd.position.Set(0.0f, 10.0f);
			bd.AddShape(&sd);
			m_body2 = m_world->CreateBody(&bd);
		}

		m_world->m_gravity.Set(0.0f, 0.0f);
		b2World::s_enablePositionCorrection = false;
	}

	~PolyCollision()
	{
		b2World::s_enablePositionCorrection = true;
	}

	static Test* Create()
	{
		return new PolyCollision;
	}

	void Step(const Settings* settings)
	{
		m_world->Step(0.0f, 1);

		for (b2Body* b = m_world->m_bodyList; b; b = b->m_next)
		{
			for (b2Shape* s = b->m_shapeList; s; s = s->m_next)
			{
				if (b->m_invMass == 0.0f)
				{
					DrawShape(s, Color(0.5f, 0.9f, 0.5f));
				}
				else
				{
					DrawShape(s, Color(0.9f, 0.9f, 0.9f));
				}
			}
		}

		for (b2Contact* c = m_world->m_contactList; c; c = c->m_next)
		{
			b2Manifold* ms = c->GetManifolds();
			for (int32 i = 0; i < c->GetManifoldCount(); ++i)
			{
				b2Manifold* m = ms + i;
				glPointSize(4.0f);
				glColor3f(1.0f, 0.0f, 0.0f);
				glBegin(GL_POINTS);
				for (int j = 0; j < m->pointCount; ++j)
				{
					b2Vec2 v = m->points[j].position;
					glVertex2f(v.x, v.y);
				}
				glEnd();
				glPointSize(1.0f);

				glColor3f(1.0f, 1.0f, 0.0f);
				glBegin(GL_LINES);
				for (int32 j = 0; j < m->pointCount; ++j)
				{
					b2Vec2 v1 = m->points[j].position;
					b2Vec2 v2 = v1 + 1.0f * m->normal;
					glVertex2f(v1.x, v1.y);
					glVertex2f(v2.x, v2.y);
				}
				glEnd();
			}
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'a':
			m_body2->m_position.x -= 0.1f;
			m_body2->SynchronizeShapes();
			break;

		case 'd':
			m_body2->m_position.x += 0.1f;
			m_body2->SynchronizeShapes();
			break;

		case 's':
			m_body2->m_position.y -= 0.1f;
			m_body2->SynchronizeShapes();
			break;

		case 'w':
			m_body2->m_position.y += 0.1f;
			m_body2->SynchronizeShapes();
			break;

		case 'q':
			m_body2->m_rotation += 0.1f * b2_pi;
			m_body2->m_R.Set(m_body2->m_rotation);
			m_body2->SynchronizeShapes();
			break;

		case 'e':
			m_body2->m_rotation -= 0.1f * b2_pi;
			m_body2->m_R.Set(m_body2->m_rotation);
			m_body2->SynchronizeShapes();
			break;

		case 'p':
			b2World::s_enablePositionCorrection = !b2World::s_enablePositionCorrection;
			break;
		}
	}

	b2Body* m_body1;
	b2Body* m_body2;
};

#endif
