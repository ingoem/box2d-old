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

#ifndef WEB_H
#define WEB_H

#include "TestBed/Framework/Test.h"
#include "Engine/Dynamics/b2Body.h"
#include "Engine/Dynamics/b2World.h"
#include "Engine/Collision/b2Shape.h"
#include "Engine/Dynamics/Joints/b2DistanceJoint.h"

class Web : public Test
{
public:
	Web()
	{
		b2Body* ground = NULL;
		{
			b2ShapeDescription sd;
			sd.type = e_boxShape;
			sd.box.m_extents.Set(50.0f, 10.0f);

			b2BodyDescription bd;
			bd.position.Set(0.0f, -10.0f);
			bd.AddShape(&sd);
			ground = m_world->CreateBody(&bd);
		}

		{
			b2ShapeDescription sd;
			sd.type = e_boxShape;
			sd.box.m_extents.Set(0.5f, 0.5f);
			sd.density = 5.0f;
			sd.friction = 0.2f;

			b2BodyDescription bd;
			bd.AddShape(&sd);

			bd.position.Set(-5.0f, 5.0f);
			b2Body* b1 = m_world->CreateBody(&bd);

			bd.position.Set(5.0f, 5.0f);
			b2Body* b2 = m_world->CreateBody(&bd);

			bd.position.Set(5.0f, 15.0f);
			b2Body* b3 = m_world->CreateBody(&bd);

			bd.position.Set(-5.0f, 15.0f);
			b2Body* b4 = m_world->CreateBody(&bd);

			b2DistanceJointDescription jd;

			jd.body1 = ground;
			jd.body2 = b1;
			jd.anchorPoint1.Set(-10.0f, 0.0f);
			jd.anchorPoint2 = b1->m_position + b2Vec2::Make(-0.5f, -0.5f);
			m_joints[0] = m_world->CreateJoint(&jd);

			jd.body1 = ground;
			jd.body2 = b2;
			jd.anchorPoint1.Set(10.0f, 0.0f);
			jd.anchorPoint2 = b2->m_position + b2Vec2::Make(0.5f, -0.5f);
			m_joints[1] = m_world->CreateJoint(&jd);

			jd.body1 = ground;
			jd.body2 = b3;
			jd.anchorPoint1.Set(10.0f, 20.0f);
			jd.anchorPoint2 = b3->m_position + b2Vec2::Make(0.5f, 0.5f);
			m_joints[2] = m_world->CreateJoint(&jd);

			jd.body1 = ground;
			jd.body2 = b4;
			jd.anchorPoint1.Set(-10.0f, 20.0f);
			jd.anchorPoint2 = b4->m_position + b2Vec2::Make(-0.5f, 0.5f);
			m_joints[3] = m_world->CreateJoint(&jd);

			jd.body1 = b1;
			jd.body2 = b2;
			jd.anchorPoint1 = b1->m_position + b2Vec2::Make(0.5f, 0.0f);
			jd.anchorPoint2 = b2->m_position + b2Vec2::Make(-0.5f, 0.0f);;
			m_joints[4] = m_world->CreateJoint(&jd);

			jd.body1 = b2;
			jd.body2 = b3;
			jd.anchorPoint1 = b2->m_position + b2Vec2::Make(0.0f, 0.5f);
			jd.anchorPoint2 = b3->m_position + b2Vec2::Make(0.0f, -0.5f);
			m_joints[5] = m_world->CreateJoint(&jd);

			jd.body1 = b3;
			jd.body2 = b4;
			jd.anchorPoint1 = b3->m_position + b2Vec2::Make(-0.5f, 0.0f);
			jd.anchorPoint2 = b4->m_position + b2Vec2::Make(0.5f, 0.0f);
			m_joints[6] = m_world->CreateJoint(&jd);

			jd.body1 = b4;
			jd.body2 = b1;
			jd.anchorPoint1 = b4->m_position + b2Vec2::Make(0.0f, -0.5f);
			jd.anchorPoint2 = b1->m_position + b2Vec2::Make(0.0f, 0.5f);
			m_joints[7] = m_world->CreateJoint(&jd);
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'b':
			for (int32 i = 0; i < 8; ++i)
			{
				if (m_joints[i])
				{
					m_world->DestroyJoint(m_joints[i]);
					m_joints[i] = NULL;
					break;
				}
			}
			break;
		}
	}

	void Step(const Settings* settings)
	{
		DrawString(5, m_textLine, "Press (b) to break constraints");
		m_textLine += 15;
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Web;
	}

	b2Joint* m_joints[8];
};

#endif