/*
* Copyright (c) 2007 Erin Catto http://www.gphysics.com
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

#ifndef GEARS_H
#define GEARS_H

class Gears : public Test
{
public:
	Gears()
	{
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			bd.type = b2BodyDef::e_staticBody;
			bd.position.Set(0.0f, -10.0f);
			ground = m_world->Create(&bd);

			b2PolygonDef sd;
			sd.SetAsBox(50.0f, 10.0f);
			ground->Create(&sd);
		}

		{
			b2CircleDef circle1;
			circle1.radius = 1.0f;
			circle1.density = 5.0f;
			
			b2CircleDef circle2;
			circle2.radius = 2.0f;
			circle2.density = 5.0f;

			b2PolygonDef box;
			box.SetAsBox(0.5f, 5.0f);
			box.density = 5.0f;

			b2BodyDef bd1;
			bd1.type = b2BodyDef::e_dynamicBody;
			bd1.position.Set(-3.0f, 12.0f);
			b2Body* body1 = m_world->Create(&bd1);
			body1->Create(&circle1);
			body1->SetMassFromShapes();

			b2RevoluteJointDef jd1;
			jd1.body1 = ground;
			jd1.body2 = body1;
			jd1.localAnchor1 = ground->GetLocalPoint(bd1.position);
			jd1.localAnchor2 = body1->GetLocalPoint(bd1.position);
			jd1.referenceAngle = body1->GetAngle() - ground->GetAngle();
			m_joint1 = (b2RevoluteJoint*)m_world->Create(&jd1);

			b2BodyDef bd2;
			bd2.type = b2BodyDef::e_dynamicBody;
			bd2.position.Set(0.0f, 12.0f);
			b2Body* body2 = m_world->Create(&bd2);
			body2->Create(&circle2);
			body2->SetMassFromShapes();

			b2RevoluteJointDef jd2;
			jd2.body1 = ground;
			jd2.body2 = body2;
			jd2.localAnchor1 = ground->GetLocalPoint(bd2.position);
			jd2.localAnchor2 = body2->GetLocalPoint(bd2.position);
			jd2.referenceAngle = body2->GetAngle() - body1->GetAngle();
			m_joint2 = (b2RevoluteJoint*)m_world->Create(&jd2);

			b2BodyDef bd3;
			bd3.type = b2BodyDef::e_dynamicBody;
			bd3.position.Set(2.5f, 12.0f);
			b2Body* body3 = m_world->Create(&bd3);
			body3->Create(&box);
			body3->SetMassFromShapes();

			b2PrismaticJointDef jd3;
			jd3.body1 = ground;
			jd3.body2 = body3;
			jd3.localAnchor1 = ground->GetLocalPoint(bd3.position);
			jd3.localAnchor2 = body3->GetLocalPoint(bd3.position);
			jd3.referenceAngle = body3->GetAngle() - ground->GetAngle();
			jd3.localAxis1 = ground->GetLocalVector(b2Vec2(0.0f, 1.0f));
			jd3.lowerTranslation = -5.0f;
			jd3.upperTranslation = 5.0f;
			jd3.enableLimit = true;

			m_joint3 = (b2PrismaticJoint*)m_world->Create(&jd3);

			b2GearJointDef jd4;
			jd4.body1 = body1;
			jd4.body2 = body2;
			jd4.joint1 = m_joint1;
			jd4.joint2 = m_joint2;
			jd4.ratio = circle2.radius / circle1.radius;
			m_joint4 = (b2GearJoint*)m_world->Create(&jd4);

			b2GearJointDef jd5;
			jd5.body1 = body2;
			jd5.body2 = body3;
			jd5.joint1 = m_joint2;
			jd5.joint2 = m_joint3;
			jd5.ratio = -1.0f / circle2.radius;
			m_joint5 = (b2GearJoint*)m_world->Create(&jd5);
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

	void Step(Settings* settings)
	{
		Test::Step(settings);

		float32 ratio, value;
		
		ratio = m_joint4->GetRatio();
		value = m_joint1->GetJointAngle() + ratio * m_joint2->GetJointAngle();
		DrawString(5, m_textLine, "theta1 + %4.2f * theta2 = %4.2f", ratio, value);
		m_textLine += 15;

		ratio = m_joint5->GetRatio();
		value = m_joint2->GetJointAngle() + ratio * m_joint3->GetJointTranslation();
		DrawString(5, m_textLine, "theta2 + %4.2f * delta = %4.2f", ratio, value);
		m_textLine += 15;
	}

	static Test* Create()
	{
		return new Gears;
	}

	b2RevoluteJoint* m_joint1;
	b2RevoluteJoint* m_joint2;
	b2PrismaticJoint* m_joint3;
	b2GearJoint* m_joint4;
	b2GearJoint* m_joint5;
};

#endif
