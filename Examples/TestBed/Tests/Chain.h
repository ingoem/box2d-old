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

#ifndef CHAIN_H
#define CHAIN_H

class Chain : public Test
{
public:
	Chain()
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
			b2PolygonDef sd;
			sd.SetAsBox(0.6f, 0.125f);
			sd.density = 20.0f;
			sd.friction = 0.2f;

			b2RevoluteJointDef jd;
			jd.collideConnected = false;

			const float32 y = 25.0f;
			b2Body* prevBody = ground;
			for (int32 i = 0; i < 30; ++i)
			{
				b2BodyDef bd;
				bd.type = b2BodyDef::e_dynamicBody;
				bd.position.Set(0.5f + i, y);
				b2Body* body = m_world->Create(&bd);
				body->Create(&sd);
				body->SetMassFromShapes();

				b2Vec2 anchor(float32(i), y);
				jd.localAnchor1 = prevBody->GetLocalPoint(anchor);
				jd.localAnchor2 = body->GetLocalPoint(anchor);
				jd.referenceAngle = body->GetAngle() - prevBody->GetAngle();
				jd.body1 = prevBody;
				jd.body2 = body;
				m_world->Create(&jd);

				prevBody = body;
			}
		}
	}

	static Test* Create()
	{
		return new Chain;
	}
};

#endif
