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

#ifndef SLIDER_CRANK_H
#define SLIDER_CRANK_H

// A motor driven slider crank with joint friction.

class SliderCrank : public Test
{
public:
	SliderCrank()
	{
		b2Body* ground = NULL;
		{
			b2PolygonDef sd;
			sd.SetAsBox(50.0f, 10.0f);

			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			ground = m_world->Create(&bd);
			ground->Create(&sd);
		}

		{
			// Define crank.
			b2PolygonDef sd;
			sd.SetAsBox(0.5f, 2.0f);
			sd.density = 1.0f;

			b2RevoluteJointDef rjd;

			b2Body* prevBody = ground;

			b2BodyDef bd;
			bd.type = b2BodyDef::e_dynamicBody;
			bd.position.Set(0.0f, 7.0f);
			b2Body* body = m_world->Create(&bd);
			body->Create(&sd);
			body->SetMassFromShapes();

			b2Vec2 anchor;
			anchor.Set(0.0f, 5.0f);
			rjd.body1 = prevBody;
			rjd.body2 = body;
			rjd.localAnchor1 = prevBody->GetLocalPoint(anchor);
			rjd.localAnchor2 = body->GetLocalPoint(anchor);
			rjd.referenceAngle = body->GetAngle() - prevBody->GetAngle();
			rjd.motorSpeed = 1.0f * b2_pi;
			rjd.maxMotorTorque = 10000.0f;
			rjd.enableMotor = true;
			m_joint1 = (b2RevoluteJoint*)m_world->Create(&rjd);

			prevBody = body;

			// Define follower.
			sd.SetAsBox(0.5f, 4.0f);
			bd.position.Set(0.0f, 13.0f);
			body = m_world->Create(&bd);
			body->Create(&sd);
			body->SetMassFromShapes();

			anchor.Set(0.0f, 9.0f);
			rjd.body1 = prevBody;
			rjd.body2 = body;
			rjd.localAnchor1 = prevBody->GetLocalPoint(anchor);
			rjd.localAnchor2 = body->GetLocalPoint(anchor);
			rjd.referenceAngle = body->GetAngle() - prevBody->GetAngle();
			rjd.enableMotor = false;
			m_world->Create(&rjd);

			prevBody = body;

			// Define piston
			sd.SetAsBox(1.5f, 1.5f);
			bd.position.Set(0.0f, 17.0f);
			body = m_world->Create(&bd);
			body->Create(&sd);
			body->SetMassFromShapes();

			anchor.Set(0.0f, 17.0f);
			rjd.body1 = prevBody;
			rjd.body2 = body;
			rjd.localAnchor1 = prevBody->GetLocalPoint(anchor);
			rjd.localAnchor2 = body->GetLocalPoint(anchor);
			rjd.referenceAngle = body->GetAngle() - prevBody->GetAngle();
			m_world->Create(&rjd);

			b2PrismaticJointDef pjd;
			anchor.Set(0.0f, 17.0f);
			pjd.body1 = ground;
			pjd.body2 = body;
			pjd.localAnchor1 = ground->GetLocalPoint(anchor);
			pjd.localAnchor2 = body->GetLocalPoint(anchor);
			pjd.referenceAngle = body->GetAngle() - ground->GetAngle();

			b2Vec2 axis;
			axis.Set(0.0f, 1.0f);
			pjd.localAxis1 = ground->GetLocalVector(axis);
			pjd.motorSpeed = 0.0f;		// joint friction
			pjd.maxMotorForce = 1000.0f;
			pjd.enableMotor = true;

			m_joint2 = (b2PrismaticJoint*)m_world->Create(&pjd);

			// Create a payload
			sd.density = 2.0f;
			bd.position.Set(0.0f, 23.0f);
			body = m_world->Create(&bd);
			body->Create(&sd);
			body->SetMassFromShapes();
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'f':
			m_joint2->m_enableMotor = !m_joint2->m_enableMotor;
			m_joint2->GetBody2()->WakeUp();
			break;

		case 'm':
			m_joint1->m_enableMotor = !m_joint1->m_enableMotor;
			m_joint1->GetBody2()->WakeUp();
			break;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		DrawString(5, m_textLine, "Keys: (f) toggle friction, (m) toggle motor");
		m_textLine += 15;
		float32 torque = m_joint1->GetMotorTorque();
		DrawString(5, m_textLine, "Motor Torque = %5.0f", torque);
		m_textLine += 15;
	}

	static Test* Create()
	{
		return new SliderCrank;
	}

	b2RevoluteJoint* m_joint1;
	b2PrismaticJoint* m_joint2;
};

#endif
