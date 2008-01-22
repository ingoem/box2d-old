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

#ifndef TIME_OF_IMPACT_H
#define TIME_OF_IMPACT_H

class TimeOfImpact : public Test
{
public:
	TimeOfImpact()
	{
		{
			b2PolygonDef sd;
			sd.density = 0.0f;

			sd.SetAsBox(0.1f, 10.0f, b2Vec2(10.0f, 0.0f), 0.0f);
			m_shape1 = m_world->Create(&sd);

			b2BodyDef bd;
			bd.position.Set(0.0f, 20.0f);
			bd.angle = 0.0f;
			body->AddShape(m_shape1);
			m_world->Create(&bd);
		}

		{
			b2PolygonDef sd;
			sd.SetAsBox(0.25f, 0.25f);
			sd.density = 1.0f;
			m_shape2 = (b2PolygonShape*)m_world->Create(&sd);

			b2BodyDef bd;
			body->AddShape(m_shape2);
			bd.position.Set(9.6363468f, 28.050615f);
			bd.angle = 1.6408679f;
			m_world->Create(&bd);
		}
	}

	~TimeOfImpact()
	{
	}

	static Test* Create()
	{
		return new TimeOfImpact;
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

		b2Sweep sweep1;
		sweep1.position.Set(0.0f, 20.0f);
		sweep1.angle = 0.0f;
		sweep1.velocity = b2Vec2_zero;
		sweep1.omega = 0.0f;

		b2Sweep sweep2;
		sweep2.position.Set(9.6363468f, 28.050615f);
		sweep2.angle = 1.6408679f;
		sweep2.velocity.Set(-0.075121880f, 0.27358246f);
		sweep2.omega = -10.434675f;

		b2Vec2 x1, x2;
		float32 toi = b2TimeOfImpact(&x1, &x2, m_shape1, sweep1, m_shape2, sweep2, 1.0f);

		DrawString(5, m_textLine, "toi = %g", toi);
		m_textLine += 15;

		b2XForm xf2 = sweep2.GetXForm(toi);
		b2Vec2 vertices[b2_maxPolygonVertices];
		const b2Vec2* localVertices = m_shape2->GetVertices();
		int32 vertexCount = m_shape2->GetVertexCount();
		for (int32 i = 0; i < vertexCount; ++i)
		{
			vertices[i] = b2Mul(xf2, localVertices[i]);
		}
		m_debugDraw.DrawPolygon(vertices, vertexCount, b2Color(0.5f, 0.7f, 0.9f));

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

	b2Shape* m_shape1;
	b2PolygonShape* m_shape2;
};

#endif
