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

#ifndef PULLEYS_H
#define PULLEYS_H

class Pulleys : public Test
{
public:
	Pulleys()
	{
		b2Body* ground = NULL;
		{
			b2BoxDef sd;
			sd.type = e_boxShape;
			sd.extents.Set(50.0f, 10.0f);

			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			bd.AddShape(&sd);
			ground = m_world->CreateBody(&bd);
		}

		{
			float32 a = 2.0f;
			float32 y = 22.0f;
			float32 L = 8.0f;

			b2BoxDef sd;
			sd.type = e_boxShape;
			sd.extents.Set(a, a);
			sd.density = 5.0f;

			b2BodyDef bd;
			bd.AddShape(&sd);

			bd.position.Set(-10.0f, y);
			b2Body* body1 = m_world->CreateBody(&bd);

			bd.position.Set(10.0f, y);
			b2Body* body2 = m_world->CreateBody(&bd);

			b2PulleyJointDef pulleyDef;
			pulleyDef.body1 = body1;
			pulleyDef.body2 = body2;
			pulleyDef.anchorPoint1.Set(-10.0f, y + a);
			pulleyDef.anchorPoint2.Set(10.0f, y + a);
			pulleyDef.groundPoint1.Set(-10.0f, y + a + L);
			pulleyDef.groundPoint2.Set(10.0f, y + a + L);
			pulleyDef.ratio = 2.0f;
			pulleyDef.collideConnected = true;

			pulleyDef.maxLength1 = 16.0f;
			pulleyDef.maxLength2 = 10.0f;

			m_joint1 = (b2PulleyJoint*)m_world->CreateJoint(&pulleyDef);
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 0:
			break;
		}
	}

	void Step(const Settings* settings)
	{		
		float32 L = m_joint1->GetLength1() + m_joint1->GetRatio() * m_joint1->GetLength2();
		DrawString(5, m_textLine, "L1 + ratio*L2 = %4.2f", L);
		m_textLine += 15;

		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Pulleys;
	}

	b2PulleyJoint* m_joint1;
};

#endif
