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



#ifndef CAR_H
#define CAR_H

// Adapted from SpiritWalkers by darkzerox
class Car : public Test
{
public:
	Car()
	{
		// initialize world
		b2Vec2 gravity;
		b2AABB aabb;

		aabb.minVertex.Set(-50,-50);
		aabb.maxVertex.Set(200,100);
		gravity.Set(0, -9.80665f);

		m_world = new b2World(aabb, gravity, true);
		/*
		{	// car body
		b2PolygonDef	box;
		b2BodyDef	body;

		box.SetAsBox(2.2225f,0.73787f);
		box.type		= e_boxShape;
		box.density		= 227.6f;
		box.friction	= 0.68f;
		box.groupIndex	= -1;

		body.position.Set(10,2.8f);
		body.AddShape(&box);

		m_vehicle = m_world->Create(&body);
		}
		*/
		{	// car body
			b2PolygonDef	poly1, poly2;
			b2BodyDef	body;

			// bottom half
			poly1.type = e_polygonShape;
			poly1.vertexCount = 5;
			poly1.vertices[4].Set(-2.2f,-0.74f);
			poly1.vertices[3].Set(-2.2f,0);
			poly1.vertices[2].Set(1.0f,0);
			poly1.vertices[1].Set(2.2f,-0.2f);
			poly1.vertices[0].Set(2.2f,-0.74f);
			poly1.groupIndex = -1;

			poly1.density		= 300;
			poly1.friction		= 0.68f;
			poly1.groupIndex	= -1;

			// top half
			poly2.type = e_polygonShape;
			poly2.vertexCount = 4;
			poly2.vertices[3].Set(-1.7f,0);
			poly2.vertices[2].Set(-1.3f,0.7f);
			poly2.vertices[1].Set(0.5f,0.74f);
			poly2.vertices[0].Set(1.0f,0);
			poly2.groupIndex = -1;

			poly2.density		= 300;
			poly2.friction		= 0.68f;
			poly2.groupIndex	= -1;

			body.AddShape(&poly1);
			body.AddShape(&poly2);
			body.position.Set(10,2.8f);

			m_vehicle = m_world->Create(&body);
		}

		{	// ground
			b2PolygonDef	box;
			b2BodyDef	body;

			box.SetAsBox(9.5,0.5);
			box.density		= 0;
			box.friction	= 0.62f;

			body.position.Set(10,1);
			body.AddShape(&box);

			m_world->Create(&body);
		}


		{	// vehicle wheels
			b2CircleDef	circ;
			b2BodyDef	body;

			circ.density		= 945;
			circ.radius			= 0.38608f;
			circ.friction		= 0.8f;
			circ.restitution	= 0.3f;
			circ.groupIndex		= -1;

			body.angularDamping = 0.02f;
			body.allowSleep = false;
			body.position.Set(11.2f,2);
			body.AddShape(&circ);

			m_rightWheel = m_world->Create(&body);
			body.position.Set(8.8f,2);
			m_leftWheel = m_world->Create(&body);
		}

		{	// axles
			b2CircleDef circ;
			b2BodyDef	body;
			b2Body		*leftAxle, *rightAxle;

			circ.radius		= 0.10f;
			circ.density	= 235;
			circ.groupIndex	= -1;

			body.position = m_leftWheel->GetCenterPosition();
			body.AddShape(&circ);

			leftAxle = m_world->Create(&body);
			body.position = m_rightWheel->GetCenterPosition();
			rightAxle = m_world->Create(&body);

			{	// join wheels to axles
				b2RevoluteJointDef	joint;
				joint.body1 = leftAxle;
				joint.body2 = m_leftWheel;
				joint.collideConnected = false;
				joint.anchorPoint = leftAxle->GetCenterPosition();
				m_world->CreateJoint(&joint);
				joint.body1 = rightAxle;
				joint.body2 = m_rightWheel;
				joint.anchorPoint = rightAxle->GetCenterPosition();
				m_world->CreateJoint(&joint);
			}
			{	// join axles to car
				b2PrismaticJointDef	joint;
				joint.body1 = leftAxle;
				joint.body2 = m_vehicle;
				joint.collideConnected = false;
				joint.lowerTranslation = -0.2f;
				joint.upperTranslation = 0.2f;
				joint.enableLimit = true;
				joint.motorForce = 20000;
				joint.enableMotor = true;
				joint.motorSpeed = 0;
				joint.axis.Set(0,1);
				joint.anchorPoint = leftAxle->GetCenterPosition();
				m_world->CreateJoint(&joint);
				joint.body1 = rightAxle;
				joint.anchorPoint = rightAxle->GetCenterPosition();
				m_world->CreateJoint(&joint);
			}
		}
		{	// falling person
			b2PolygonDef	box;
			b2BodyDef	body;

			box.SetAsBox(0.5f,1.7f);
			box.type		= e_boxShape;
			box.density		= 1000;
			box.friction	= 0.1f;

			body.position.Set(10,40);
			body.AddShape(&box);

			//m_world->Create(&body);
		}
		{	// more ground
			b2PolygonDef	box;
			b2BodyDef	body;

			box.SetAsBox(9.5,0.5);
			box.density		= 0;
			box.friction	= 0.62f;
			box.localRotation	= 0.1f * b2_pi;
			body.position.Set(27,3.1f);
			body.AddShape(&box);

			m_world->Create(&body);
		}
		{	// more ground
			b2PolygonDef	box;
			b2BodyDef	body;

			box.SetAsBox(9.5,0.5);
			box.density		= 0;
			box.friction	= 0.62f;
			box.localRotation	= -0.1f * b2_pi;
			body.position.Set(55,3.1f);
			body.AddShape(&box);

			m_world->Create(&body);
		}
		{	// more ground
			b2PolygonDef	box;
			b2BodyDef	body;

			box.SetAsBox(9.5,0.5);
			box.density		= 0;
			box.friction	= 0.62f;
			box.localRotation	= 0.03f * b2_pi;
			body.position.Set(71,2);
			body.AddShape(&box);

			m_world->Create(&body);
		}
		{	// more ground
			b2PolygonDef	box;
			b2BodyDef	body;

			box.SetAsBox(5,0.5);
			box.density		= 0;
			box.friction	= 0.62f;
			box.localRotation	= 0.15f * b2_pi;
			body.position.Set(80,4);
			body.AddShape(&box);

			m_world->Create(&body);
		}
		{	// more ground
			b2PolygonDef	box;
			b2BodyDef	body;

			box.SetAsBox(30,0.5);
			box.density		= 0;
			box.friction	= 0.62f;
			box.localRotation	= 0;
			body.position.Set(140,2);
			body.AddShape(&box);

			m_world->Create(&body);
		}
	}

	static Test* Create()
	{
		return new Car;
	}

	b2Body *m_leftWheel, *m_rightWheel, *m_vehicle;
};

#endif
