/*
* Copyright (c) 2008 Erin Catto http://www.gphysics.com
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

#ifndef CAR_H
#define CAR_H

// Adapted from SpiritWalkers by darkzerox
class Car : public Test
{
public:
	Car()
	{
		{	// car body
			b2PolygonDef poly1, poly2;
			b2BodyDef bd;

			// bottom half
			poly1.vertexCount = 5;
			poly1.vertices[4].Set(-2.2f,-0.74f);
			poly1.vertices[3].Set(-2.2f,0);
			poly1.vertices[2].Set(1.0f,0);
			poly1.vertices[1].Set(2.2f,-0.2f);
			poly1.vertices[0].Set(2.2f,-0.74f);
			poly1.groupIndex = -1;

			poly1.density		= 20.0f;
			poly1.friction		= 0.68f;
			poly1.groupIndex	= -1;

			// top half
			poly2.vertexCount = 4;
			poly2.vertices[3].Set(-1.7f,0);
			poly2.vertices[2].Set(-1.3f,0.7f);
			poly2.vertices[1].Set(0.5f,0.74f);
			poly2.vertices[0].Set(1.0f,0);
			poly2.groupIndex = -1;

			poly2.density		= 5.0f;
			poly2.friction		= 0.68f;
			poly2.groupIndex	= -1;

			bd.type = b2BodyDef::e_dynamicBody;
			bd.position.Set(-35.0f, 2.8f);

			m_vehicle = m_world->CreateBody(&bd);
			m_vehicle->CreateShape(&poly1);
			m_vehicle->CreateShape(&poly2);
			m_vehicle->SetMassFromShapes();
		}

		{	// ground
			b2PolygonDef box;
			box.SetAsBox(19.5f, 0.5f);
			box.friction = 0.62f;

			b2BodyDef bd;
			bd.position.Set(-25.0f, 1.0f);

			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateShape(&box);
		}

		{	// vehicle wheels
			b2CircleDef	circ;
			circ.density = 40.0f;
			circ.radius = 0.38608f;
			circ.friction = 0.8f;
			circ.groupIndex = -1;

			b2BodyDef bd;
			bd.type = b2BodyDef::e_dynamicBody;
			bd.allowSleep = false;
			bd.position.Set(-33.8f, 2.0f);

			m_rightWheel = m_world->CreateBody(&bd);
			m_rightWheel->CreateShape(&circ);
			m_rightWheel->SetMassFromShapes();

			bd.position.Set(-36.2f, 2.0f);
			m_leftWheel = m_world->CreateBody(&bd);
			m_leftWheel->CreateShape(&circ);
			m_leftWheel->SetMassFromShapes();
		}

		{	// join wheels to chassis
			b2Vec2 anchor;
			b2RevoluteJointDef jd;
			jd.body1 = m_vehicle;
			jd.body2 = m_leftWheel;
			jd.collideConnected = false;
			anchor = m_leftWheel->GetWorldCenter();
			jd.localAnchor1 = m_vehicle->GetLocalPoint(anchor);
			jd.localAnchor2 = m_leftWheel->GetLocalPoint(anchor);
			jd.referenceAngle = m_leftWheel->GetAngle() - m_vehicle->GetAngle();
			jd.enableMotor = true;
			jd.maxMotorTorque = 10.0f;
			jd.motorSpeed = 0.0f;
			m_leftJoint = (b2RevoluteJoint*)m_world->CreateJoint(&jd);

			jd.body1 = m_vehicle;
			jd.body2 = m_rightWheel;
			jd.collideConnected = false;
			anchor = m_rightWheel->GetWorldCenter();
			jd.localAnchor1 = m_vehicle->GetLocalPoint(anchor);
			jd.localAnchor2 = m_rightWheel->GetLocalPoint(anchor);
			jd.referenceAngle = m_rightWheel->GetAngle() - m_vehicle->GetAngle();
			m_rightJoint = (b2RevoluteJoint*)m_world->CreateJoint(&jd);
		}

		{	// more ground
			b2PolygonDef box;
			b2BodyDef bd;

			box.SetAsBox(9.5f, 0.5f, b2Vec2_zero, 0.1f * b2_pi);
			box.friction = 0.62f;
			bd.position.Set(27.0f - 30.0f, 3.1f);

			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateShape(&box);
		}

		{	// more ground
			b2PolygonDef box;
			b2BodyDef bd;

			box.SetAsBox(9.5f, 0.5f, b2Vec2_zero, -0.1f * b2_pi);
			box.friction = 0.62f;
			bd.position.Set(55.0f - 30.0f, 3.1f);

			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateShape(&box);
		}

		{	// more ground
			b2PolygonDef box;
			b2BodyDef bd;

			box.SetAsBox(9.5f, 0.5f, b2Vec2_zero, 0.03f * b2_pi);
			box.friction = 0.62f;
			bd.position.Set(41.0f, 2.0f);

			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateShape(&box);
		}

		{	// more ground
			b2PolygonDef box;
			b2BodyDef bd;

			box.SetAsBox(5.0f, 0.5f, b2Vec2_zero, 0.15f * b2_pi);
			box.friction = 0.62f;
			bd.position.Set(50.0f, 4.0f);

			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateShape(&box);
		}

		{	// more ground
			b2PolygonDef box;
			b2BodyDef bd;

			box.SetAsBox(20.0f, 0.5f);
			box.friction = 0.62f;
			bd.position.Set(85.0f, 2.0f);

			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateShape(&box);
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case ',':
			m_leftJoint->SetMaxMotorTorque(800.0f);
			m_leftJoint->SetMotorSpeed(12.0f);
			break;

		case '.':
			m_leftJoint->SetMaxMotorTorque(1200.0f);
			m_leftJoint->SetMotorSpeed(-36.0f);
			break;

		case '/':
			m_leftJoint->SetMaxMotorTorque(100.0f);
			m_leftJoint->SetMotorSpeed(0.0f);
			break;
		}
	}

	static Test* Create()
	{
		return new Car;
	}

	b2Body* m_leftWheel;
	b2Body* m_rightWheel;
	b2Body* m_vehicle;
	b2RevoluteJoint* m_rightJoint;
	b2RevoluteJoint* m_leftJoint;
};

#endif
