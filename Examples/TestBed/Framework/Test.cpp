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

#ifdef __APPLE__
#include <GLUT/glut.h>
#define APIENTRY
#else
#include "freeglut/gl/glut.h"
#endif

#include <cstdio>

void DestructionListener::SayGoodbye(b2Joint* joint)
{
	if (test->m_mouseJoint == joint)
	{
		test->m_mouseJoint = NULL;
	}
	else
	{
		test->JointDestroyed(joint);
	}
}

b2BoundaryListener::Response BoundaryListener::Violation(b2Body* body)
{
	if (test->m_bomb == body)
	{
		test->m_bomb = NULL;
		return e_destroyBody;
	}

	return test->BoundaryViolated(body);
}

Test::Test()
{
	b2AABB worldAABB;
	worldAABB.minVertex.Set(-100.0f, -100.0f);
	worldAABB.maxVertex.Set(100.0f, 200.0f);
	b2Vec2 gravity;
	gravity.Set(0.0f, -10.0f);
	bool doSleep = true;
	m_world = new b2World(worldAABB, gravity, doSleep);
	m_bomb = NULL;
	m_textLine = 30;
	m_mouseJoint = NULL;
	
	m_destructionListener.test = this;
	m_boundaryListener.test = this;
	m_world->SetListener(&m_destructionListener);
	m_world->SetListener(&m_boundaryListener);
	m_world->SetDebugDraw(&m_debugDraw);
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
		b2Body* shapeBody = shapes[i]->GetBody();
		if (shapeBody->IsStatic() == false)
		{
			bool inside = shapes[i]->TestPoint(shapeBody->GetXForm(), p);
			if (inside)
			{
				body = shapes[i]->m_body;
				break;
			}
		}
	}

	if (body)
	{
		b2MouseJointDef md;
		md.body1 = m_world->m_groundBody;
		md.body2 = body;
		md.target = p;
		md.maxForce = 1000.0f * body->m_mass;
		m_mouseJoint = (b2MouseJoint*)m_world->Create(&md);
		body->WakeUp();
	}
}

void Test::MouseUp()
{
	if (m_mouseJoint)
	{
		m_world->Destroy(m_mouseJoint);
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
	if (m_bomb)
	{
		m_world->Destroy(m_bomb);
		m_bomb = NULL;
	}

	b2PolygonDef sd;
	b2Vec2 extents(0.25, 0.25f);
	sd.SetAsBox(extents, b2XForm::s_identity);
	sd.density = 10.0f;
	b2Shape* shape = m_world->Create(&sd);

	b2BodyDef bd;
	bd.AddShape(shape);
	bd.allowSleep = true;
	bd.position.Set(b2Random(-15.0f, 15.0f), 30.0f);
	bd.rotation = b2Random(-1.5f, 1.5f);
	bd.isBullet = true;

	m_bomb = m_world->Create(&bd);
	m_bomb->SetLinearVelocity(-5.0f * bd.position);
	m_bomb->SetAngularVelocity(b2Random(-20.0f, 20.0f));
}

typedef const char *(APIENTRY * WGLGETEXTENSIONSSTRINGEXT_T)( void );

void Test::Step(Settings* settings)
{
	float32 timeStep = settings->hz > 0.0f ? 1.0f / settings->hz : 0.0f;

	if (settings->pause)
	{
		if (settings->singleStep)
		{
			settings->singleStep = 0;
		}
		else
		{
			timeStep = 0.0f;
		}

		DrawString(5, m_textLine, "****PAUSED****");
		m_textLine += 15;
	}

	uint32 flags = 0;
	flags += settings->drawShapes			* b2DebugDraw::e_shapeBit;
	flags += settings->drawJoints			* b2DebugDraw::e_jointBit;
	flags += settings->drawCoreShapes		* b2DebugDraw::e_coreShapeBit;
	flags += settings->drawAABBs			* b2DebugDraw::e_aabbBit;
	flags += settings->drawOBBs				* b2DebugDraw::e_obbBit;
	flags += settings->drawPairs			* b2DebugDraw::e_pairBit;
	flags += settings->drawContactPoints	* b2DebugDraw::e_contactPointBit;
	flags += settings->drawContactNormals	* b2DebugDraw::e_contactNormalBit;
	flags += settings->drawContactImpulses	* b2DebugDraw::e_contactImpulseBit;
	flags += settings->drawFrictionImpulses	* b2DebugDraw::e_frictionImpulseBit;
	m_debugDraw.SetFlags(flags);

	b2World::s_enableWarmStarting = settings->enableWarmStarting;
	b2World::s_enablePositionCorrection = settings->enablePositionCorrection;

	m_world->Step(timeStep, settings->iterationCount);

	m_world->m_broadPhase->Validate();

	if (settings->drawStats)
	{
		DrawString(5, m_textLine, "proxies(max) = %d(%d), pairs(max) = %d(%d)",
			m_world->m_broadPhase->m_proxyCount, b2_maxProxies,
			m_world->m_broadPhase->m_pairManager.m_pairCount, b2_maxPairs);
		m_textLine += 15;

		DrawString(5, m_textLine, "bodies/contacts/joints = %d/%d/%d",
			m_world->m_bodyCount, m_world->m_contactCount, m_world->m_jointCount);
		m_textLine += 15;

		DrawString(5, m_textLine, "position iterations = %d", m_world->m_positionIterationCount);
		m_textLine += 15;

		DrawString(5, m_textLine, "heap bytes = %d", b2_byteCount);
		m_textLine += 15;
	}

	if (m_mouseJoint)
	{
		b2Body* body = m_mouseJoint->m_body2;
		b2Vec2 p1 = body->m_xf.position + b2Mul(body->m_xf.R, m_mouseJoint->m_localAnchor);
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
