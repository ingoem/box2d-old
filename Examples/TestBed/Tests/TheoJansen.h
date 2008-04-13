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

// Contributed by roman_m

#ifndef THEO_JANSEN_H
#define THEO_JANSEN_H

#include "../Framework/Render.h"

static float32 g_x = 0.0f, g_y = 5.0f, g_o = 1.0f;

struct Line
{
	Line(b2World * world_, float32 x1, float32 y1, float32 x2, float32 y2, float32 w = 0.1f) 
		: world(world_), p1(g_x + g_o*x1, g_y + y1), p2(g_x + g_o*x2, g_y + y2)
	{
		b2Vec2 center((p1.x + p2.x)/2, (p1.y + p2.y)/2);
		float32 length = sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y));
		float32 a = atan2f(p2.y - p1.y, p2.x - p1.x);

		b2PolygonDef sd;
		sd.density = 10.0f;
		sd.friction = 0.6f;
		sd.SetAsBox(length*0.5f, w);
		sd.filter.groupIndex = -1;

		b2BodyDef bd;
		bd.position = center;
		bd.angle = a;

		body = world->CreateBody(&bd);
		body->CreateShape(&sd);
		body->SetMassFromShapes();
	}

	b2World * world;
	b2Vec2 p1, p2;
	b2Body * body;
};

b2Joint* CreateJoint(b2World * world, b2Body * b1, b2Body * b2, float32 x, float32 y)
{
	b2RevoluteJointDef jd;
	jd.Initialize(b1, b2, b2Vec2(x, y));
	jd.collideConnected = false;
	return world->CreateJoint(&jd);
}

void CreateJoint(Line &l1, Line&l2)
{
	if(l1.world != l2.world)
		return;

	float32 x, y;
	float32 z  = (l1.p2.y-l1.p1.y)*(l2.p1.x-l2.p2.x)-(l2.p1.y-l2.p2.y)*(l1.p2.x-l1.p1.x);
	if(fabsf(z) < 0.0001f)
	{
		float32 d11 = abs(l1.p1.x - l2.p1.x) + abs(l1.p1.y - l2.p1.y);
		float32 d12 = abs(l1.p1.x - l2.p2.x) + abs(l1.p1.y - l2.p2.y);
		float32 d21 = abs(l1.p2.x - l2.p1.x) + abs(l1.p2.y - l2.p1.y);
		float32 d22 = abs(l1.p2.x - l2.p2.x) + abs(l1.p2.y - l2.p2.y);
		if(d11 < d12 && d11 < d21 && d11 < d22)
		{
			x = (l1.p1.x + l2.p1.x)/2;
			y = (l1.p1.y + l2.p1.y)/2;
		}
		else if(d12 < d21 && d12 < d22)
		{
			x = (l1.p1.x + l2.p2.x)/2;
			y = (l1.p1.y + l2.p2.y)/2;
		}
		else if(d21  < d22)
		{
			x = (l1.p2.x + l2.p1.x)/2;
			y = (l1.p2.y + l2.p1.y)/2;
		}
		else
		{
			x = (l1.p2.x + l2.p2.x)/2;
			y = (l1.p2.y + l2.p2.y)/2;
		}
	}
	else
	{
		float32 t = ((l2.p1.y-l1.p1.y)*(l2.p1.x-l2.p2.x)-(l2.p1.y-l2.p2.y)*(l2.p1.x-l1.p1.x)) / z;
		x = l1.p1.x + (l1.p2.x - l1.p1.x) * t;
		y = l1.p1.y + (l1.p2.y - l1.p1.y) * t;
	}
	CreateJoint(l1.world, l1.body, l2.body, x, y);
}

class TheoJansen : public Test
{
public:

	void CreateLeg(float32 x, float32 o, b2Body * body, b2Body * m_wheel)
	{
		float32 tx = g_x, to = g_o;
		g_x = x;
		g_o = o;
		Line line1(m_world, 0, 0, -1, 4);
		Line line2(m_world, -1, 4, -3, 3.5f);
		Line line3(m_world, 0, 0, -3, 3.5f);
		Line line4(m_world, -1, 4, -1.5f, 6);
		Line line5(m_world, -3, 3.5f, -3.5f, 5.5f);
		Line line6(m_world, -1.5f, 6, -3.5f, 5.5f);
		Line line7(m_world, -2, 8, -1.5f, 6);
		Line line8(m_world, -2, 8, -3.5f, 5.5f);
		Line line9(m_world, -1, 4, 1, 6.5f);
		Line line10(m_world, -2, 8, 1, 6.5f);
		CreateJoint(line1, line2);
		CreateJoint(line1, line3);
		CreateJoint(line1, line4);
		CreateJoint(line1, line9);
		CreateJoint(line2, line3);
		CreateJoint(line2, line5);
		CreateJoint(line2, line9);
		CreateJoint(line4, line6);
		CreateJoint(line4, line7);
		CreateJoint(line5, line6);
		CreateJoint(line5, line8);
		CreateJoint(line7, line8);
		CreateJoint(line7, line10);
		CreateJoint(line9, line10);
		CreateJoint(m_world, line4.body, body, line4.p2.x, line4.p2.y);
		CreateJoint(m_world, line10.body, m_wheel, 0, g_y + 6.5f);
		CreateJoint(m_world, line9.body, m_wheel, 0, g_y + 6.5f);

		g_x = tx;
		g_o = to;
	}

	TheoJansen()
	{
		{
			b2PolygonDef sd;
			sd.SetAsBox(50.0f, 10.0f);
			sd.friction = 0.3f;
			sd.filter.categoryBits = 0x0001;

			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			m_fixed = m_world->CreateBody(&bd);
			m_fixed->CreateShape(&sd);

			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateShape(&sd);
		}

		{
			b2PolygonDef sd;
			sd.density = 2.0f;
			sd.SetAsBox(2.5f, 0.5f);
			sd.filter.groupIndex = -1;
			b2BodyDef bd;
			bd.position.Set(0, 6 + g_y);
			m_chassis = m_world->CreateBody(&bd);
			m_chassis->CreateShape(&sd);
			//m_chassis->SetMassFromShapes();
		}

		{
			b2PolygonDef sd;
			sd.density = 2.0f;
			sd.SetAsBox(1.0f, 1.0f);
			sd.filter.groupIndex = -1;
			b2BodyDef bd;
			bd.position.Set(0, 6 + g_y);
			m_wheel = m_world->CreateBody(&bd);
			m_wheel->CreateShape(&sd);
			m_wheel->SetMassFromShapes();
		}

		{
			b2RevoluteJointDef jd;
			jd.Initialize(m_wheel, m_chassis, b2Vec2(0.0f, 6.0f + g_y));
			jd.collideConnected = false;
			jd.motorSpeed = 1.0f;
			jd.maxMotorTorque = 100000.0f;
			//jd.enableMotor = true;
			m_motorJoint = m_world->CreateJoint(&jd);
		}

		//CreateJoint(m_world, m_fixed, m_chassis, 0, 0);
		//CreateJoint(m_world, m_fixed, m_chassis, 0, 10);

		m_stage = 0;
	}

	void Step(Settings* settings)
	{
		int r = ((int(m_wheel->GetAngle() * 180 / 3.1415f) % 360) + 360) % 360;
		DrawString(5, m_textLine, "Rotation = %d", r);

		if(m_stage == 0 && r == 0)
		{
			CreateLeg(-1, 1, m_chassis, m_wheel);
			CreateLeg(1, -1, m_chassis, m_wheel);
			++m_stage;
		}

		if (m_stage == 10 && r == 120)
		{
			CreateLeg(-1, 1, m_chassis, m_wheel);
			CreateLeg(1, -1, m_chassis, m_wheel);
			++m_stage;
		}

		if (m_stage == 20 && r == 240)
		{
			CreateLeg(-1, 1, m_chassis, m_wheel);
			CreateLeg(1, -1, m_chassis, m_wheel);
			++m_stage;
		}

		if (m_stage == 3 && m_fixed)
		{
			m_world->DestroyBody(m_fixed);
			m_fixed = NULL;
		}

		Test::Step(settings);
	}

	static Test* Create()
	{
		return new TheoJansen;
	}

	int32 m_stage;
	b2Body* m_chassis;
	b2Body* m_wheel;
	b2Body* m_fixed;
	b2Joint* m_motorJoint;
};

#endif // THEO_JANSEN_H
