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

#include "Test.h"
#include "Render.h"
#include "Engine/Collision/b2Shape.h"
#include "Engine/Dynamics/b2Body.h"
#include "Engine/Dynamics/b2World.h"
#include "Engine/Dynamics/Contacts/b2Contact.h"
#include "Engine/Dynamics/Joints/b2Joint.h"
#include "Engine/Dynamics/Joints/b2MouseJoint.h"

#include "TestBed/Framework/freeglut/gl/glut.h"

#include <stdio.h>

Test::Test()
{
	b2AABB worldAABB;
	worldAABB.minVertex.Set(-100.0f, -100.0f);
	worldAABB.maxVertex.Set(100.0f, 100.0f);
	b2Vec2 gravity;
	gravity.Set(0.0f, -10.0f);
	bool doSleep = true;
	m_world = new b2World(worldAABB, gravity, doSleep);
	m_bomb = NULL;
	m_textLine = 30;
	m_mouseJoint = NULL;
}

Test::~Test()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	delete m_world;
}

void Test::MouseDown(const b2Vec2& p)
{
	b2Assert(m_mouseJoint == NULL);

	// Make a small box.
	b2AABB aabb;
	b2Vec2 d;
	d.Set(0.001f, 0.001f);
	aabb.minVertex = p - d;
	aabb.maxVertex = p + d;

	// Query the world for overlapping shapes.
	const int32 k_maxCount = 10;
	b2Shape* shapes[k_maxCount];
	int32 count = m_world->Query(aabb, shapes, k_maxCount);
	b2Body* body = NULL;
	for (int32 i = 0; i < count; ++i)
	{
		if (shapes[i]->m_body->IsStatic() == false)
		{
			bool inside = shapes[i]->TestPoint(p);
			if (inside)
			{
				body = shapes[i]->m_body;
				break;
			}
		}
	}

	if (body)
	{
		b2MouseDescription md;
		md.body1 = m_world->m_groundBody;
		md.body2 = body;
		md.target = p;
		md.motorForce = 400.0f * body->m_mass;
		m_mouseJoint = (b2MouseJoint*)m_world->CreateJoint(&md);
		body->WakeUp();
	}
}

void Test::MouseUp()
{
	if (m_mouseJoint)
	{
		m_world->DestroyJoint(m_mouseJoint);
		m_mouseJoint = NULL;
	}
}

void Test::MouseMove(const b2Vec2& p)
{
	if (m_mouseJoint)
	{
		m_mouseJoint->SetTarget(p);
	}
}

void Test::LaunchBomb()
{
	if (!m_bomb)
	{
		b2ShapeDescription sd;
		float32 a = 0.5f;
		sd.type = e_boxShape;
		sd.box.m_extents.Set(a, a);
		sd.density = 20.0f;

		b2BodyDescription bd;
		bd.AddShape(&sd);
		m_bomb = m_world->CreateBody(&bd);
	}

#if 1
	b2Vec2 position; position.Set(b2Random(-15.0f, 15.0f), 30.0f);
	float rotation = b2Random(-1.5f, 1.5f);
	m_bomb->SetRootPosition(position, rotation);
	m_bomb->m_linearVelocity = -1.0f * position;
	m_bomb->m_angularVelocity = b2Random(-20.0f, 20.0f);
#else
	b2Vec2 position; position.Set(0.25f, 10.0f);
	m_bomb->SetRootPosition(position, -0.5f * b2_pi);
	m_bomb->m_linearVelocity = -1.0f * position;
	m_bomb->m_angularVelocity = 0.0f;
#endif
	m_bomb->WakeUp();
}

typedef const char *(APIENTRY * WGLGETEXTENSIONSSTRINGEXT_T)( void );

void Test::Step(const Settings* settings)
{
	float timeStep = settings->hz > 0.0f ? 1.0f / settings->hz : 0.0f;

	b2World::s_enableWarmStarting = settings->enableWarmStarting;
	b2World::s_enablePositionCorrection = settings->enablePositionCorrection;

	m_world->Step(timeStep, settings->iterationCount);

	m_world->m_broadPhase->Validate();

	for (b2Body* b = m_world->m_bodyList; b; b = b->m_next)
	{
		for (b2Shape* s = b->m_shapeList; s; s = s->m_next)
		{
			if (b->m_invMass == 0.0f)
			{
				DrawShape(s, Color(0.5f, 0.9f, 0.5f));
			}
			else if (b->m_isSleeping)
			{
				DrawShape(s, Color(0.5f, 0.5f, 0.9f));
			}
			else if (b == m_bomb)
			{
				DrawShape(s, Color(0.9f, 0.9f, 0.4f));
			}
			else
			{
				DrawShape(s, Color(0.9f, 0.9f, 0.9f));
			}
		}
	}

	for (b2Joint* j = m_world->m_jointList; j; j = j->m_next)
	{
		if (j != m_mouseJoint)
		{
			DrawJoint(j);
		}
	}

	if (settings->drawContacts)
	{
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
			}
		}
	}

	if (settings->drawImpulses)
	{
		for (b2Contact* c = m_world->m_contactList; c; c = c->m_next)
		{
			b2Manifold* ms = c->GetManifolds();
			for (int32 i = 0; i < c->GetManifoldCount(); ++i)
			{
				b2Manifold* m = ms + i;

				glColor3f(0.9f, 0.9f, 0.3f);
				glBegin(GL_LINES);
				for (int32 j = 0; j < m->pointCount; ++j)
				{
					b2Vec2 v1 = m->points[j].position;
					b2Vec2 v2 = v1 + m->points[j].normalImpulse * m->normal;
					glVertex2f(v1.x, v1.y);
					glVertex2f(v2.x, v2.y);
				}
				glEnd();
			}
		}
	}

	if (settings->drawPairs)
	{
		b2BroadPhase* bp = m_world->m_broadPhase;
		b2Vec2 invQ;
		invQ.Set(1.0f / bp->m_quantizationFactor.x, 1.0f / bp->m_quantizationFactor.y);
		glColor3f(0.9f, 0.9f, 0.3f);
		glBegin(GL_LINES);
		for (int32 i = 0; i < bp->m_pairManager.m_pairCount; ++i)
		{
			b2Pair* pair = bp->m_pairManager.m_pairs + i;
			uint16 id1 = pair->proxyId1;
			uint16 id2 = pair->proxyId2;
			b2Proxy* p1 = bp->m_proxyPool + id1;
			b2Proxy* p2 = bp->m_proxyPool + id2;

			b2AABB b1, b2;
			b1.minVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p1->lowerBounds[0]].value;
			b1.minVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p1->lowerBounds[1]].value;
			b1.maxVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p1->upperBounds[0]].value;
			b1.maxVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p1->upperBounds[1]].value;
			b2.minVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p2->lowerBounds[0]].value;
			b2.minVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p2->lowerBounds[1]].value;
			b2.maxVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p2->upperBounds[0]].value;
			b2.maxVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p2->upperBounds[1]].value;

			b2Vec2 x1 = 0.5f * (b1.minVertex + b1.maxVertex);
			b2Vec2 x2 = 0.5f * (b2.minVertex + b2.maxVertex);

			glVertex2f(x1.x, x1.y);
			glVertex2f(x2.x, x2.y);
		}
		glEnd();
	}

	if (settings->drawAABBs)
	{
		b2BroadPhase* bp = m_world->m_broadPhase;
		b2Vec2 invQ;
		invQ.Set(1.0f / bp->m_quantizationFactor.x, 1.0f / bp->m_quantizationFactor.y);
		glColor3f(0.9f, 0.3f, 0.9f);
		for (int32 i = 0; i < b2_maxProxies; ++i)
		{
			b2Proxy* p = bp->m_proxyPool + i;
			if (p->IsValid() == false)
			{
				continue;
			}

			b2AABB b;
			b.minVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p->lowerBounds[0]].value;
			b.minVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p->lowerBounds[1]].value;
			b.maxVertex.x = bp->m_worldAABB.minVertex.x + invQ.x * bp->m_bounds[0][p->upperBounds[0]].value;
			b.maxVertex.y = bp->m_worldAABB.minVertex.y + invQ.y * bp->m_bounds[1][p->upperBounds[1]].value;

			glBegin(GL_LINE_LOOP);
			glVertex2f(b.minVertex.x, b.minVertex.y);
			glVertex2f(b.maxVertex.x, b.minVertex.y);
			glVertex2f(b.maxVertex.x, b.maxVertex.y);
			glVertex2f(b.minVertex.x, b.maxVertex.y);
			glEnd();
		}
	}

	if (settings->drawStats)
	{
		DrawString(5, m_textLine, "proxies(max) = %d(%d), pairs(max) = %d(%d)",
			m_world->m_broadPhase->m_proxyCount, b2_maxProxies,
			m_world->m_broadPhase->m_pairManager.m_pairCount, b2_maxPairs);

		m_textLine += 15;

		DrawString(5, m_textLine, "bodies/contacts/joints = %d/%d/%d",
			m_world->m_bodyCount, m_world->m_contactCount, m_world->m_jointCount);
	}

	if (m_mouseJoint)
	{
		b2Body* body = m_mouseJoint->m_body2;
		b2Vec2 p1 = body->m_position + b2Mul(body->m_R, m_mouseJoint->m_localAnchor);
		b2Vec2 p2 = m_mouseJoint->m_target;

		glPointSize(4.0f);
		glColor3f(0.0f, 1.0f, 0.0f);
		glBegin(GL_POINTS);
		glVertex2f(p1.x, p1.y);
		glVertex2f(p2.x, p2.y);
		glEnd();
		glPointSize(1.0f);

		glColor3f(0.8f, 0.8f, 0.8f);
		glBegin(GL_LINES);
		glVertex2f(p1.x, p1.y);
		glVertex2f(p2.x, p2.y);
		glEnd();
	}
}