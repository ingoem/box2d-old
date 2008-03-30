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

#ifndef ELASTIC_BODY_H
#define ELASTIC_BODY_H

class ElasticBody : public Test

{
public:
	ElasticBody()
	{
		/// Bottom static body
		{ 
			b2PolygonDef sd;
			sd.SetAsBox(50.0f, 5.0f);
			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			b2Body* ground = m_world->CreateStaticBody(&bd);
			ground->CreateShape(&sd);
		}

		/// Upper static 
		{
            b2PolygonDef sd;
			sd.SetAsBox(20.0f, 0.50f);
			sd.friction = 0.01f;
			sd.restitution = 0.001f;  
			b2BodyDef bd;
			bd.position.Set(-20.0f, 93.0f);
			bd.angle = 0.047f*b2_pi;
			b2Body* g = m_world->CreateStaticBody(&bd);
			g->CreateShape(&sd);
        }

		/// Left vertical static  
		{
            b2PolygonDef sd;
			sd.SetAsBox(0.50f, 60.0f);
			sd.friction = 0.05f;
			sd.restitution = 0.01f;  
			b2BodyDef bd;
			bd.position.Set(-50.0f, 52.0f);
			b2Body* g = m_world->CreateStaticBody(&bd);
			g->CreateShape(&sd);
        }

		/// Left static - right channel upper wall  
		{
            b2PolygonDef sd;
			sd.SetAsBox(0.5f, 20.0f);
			sd.friction = 0.05f;
			sd.restitution = 0.01f;  
			b2BodyDef bd;
			bd.position.Set(-42.0f, 70.0f);
		    bd.angle = -0.03f*b2_pi;
			b2Body* g = m_world->CreateStaticBody(&bd);
			g->CreateShape(&sd);
        }

		/// Left static - right channel lower wall
		{
            b2PolygonDef sd;
			sd.SetAsBox(0.50f, 22.0f);
			sd.friction = 0.05f;
			sd.restitution = 0.01f;  
			b2BodyDef bd;
			bd.position.Set(-44.0f, 31.0f);
			b2Body* g = m_world->CreateStaticBody(&bd);
			g->CreateShape(&sd);

        /// Some motors
		    b2CircleDef cd;
			cd.radius = 3.0f;
			cd.density = 15.0f;
			cd.friction = 1.f;
        /// 1. 
			bd.position.Set(-40.0f,3.0f);
			b2Body* body = m_world->CreateDynamicBody(&bd);
			body->CreateShape(&cd);
            body->SetMassFromShapes(); 

            b2RevoluteJointDef jr; 
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
            jr.maxMotorTorque = 30000.f;
            jr.enableMotor = true; 
            jr.motorSpeed = 20.f;
			m_world->CreateJoint(&jr);

        /// 2.
			bd.position.Set(-32.0f,2.5f);
			body = m_world->CreateDynamicBody(&bd);
			body->CreateShape(&cd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
			m_world->CreateJoint(&jr);
        /// 3.
			bd.position.Set(-24.0f,1.5f);
			body = m_world->CreateDynamicBody(&bd);
			body->CreateShape(&cd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
			m_world->CreateJoint(&jr);
        /// 4.
			bd.position.Set(-16.0f,1.0f);
			body = m_world->CreateDynamicBody(&bd);
			body->CreateShape(&cd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
			m_world->CreateJoint(&jr);
        /// 5.
			bd.position.Set(-8.0f,0.5f);
			body = m_world->CreateDynamicBody(&bd);
			body->CreateShape(&cd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
			m_world->CreateJoint(&jr);
        /// 6.
			bd.position.Set(0.0f,0.1f);
			body = m_world->CreateDynamicBody(&bd);
			body->CreateShape(&cd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
			m_world->CreateJoint(&jr);
        /// 7.
			bd.position.Set(8.0f,0.0f);
			body = m_world->CreateDynamicBody(&bd);
			body->CreateShape(&cd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
			m_world->CreateJoint(&jr);
        /// 8.
            sd.SetAsBox(6.0f, 0.5f);
            sd.density = 2.0f;
			bd.position.Set(18.0f,2.f);
			body = m_world->CreateDynamicBody(&bd);
			body->CreateShape(&sd);
			sd.SetAsBox(6.0f, 0.5f, b2Vec2(0.f,0.f),b2_pi*.5f);
			body->CreateShape(&sd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter());
            jr.maxMotorTorque = 50000.f;
            jr.motorSpeed = -1.f;
            m_world->CreateJoint(&jr);
		}

        /// Middle static
		{
            b2PolygonDef sd;
			sd.SetAsBox(0.50f, 5.0f);
			sd.friction = 0.05f;
			sd.restitution = 0.5f;  
			b2BodyDef bd;
			bd.position.Set(16.0f, 13.f);
            bd.angle = 0.0;
			b2Body* g = m_world->CreateStaticBody(&bd);
			g->CreateShape(&sd);
        }

        /// Above circles
		{
            b2PolygonDef sd;
			sd.SetAsBox(20.0f, 0.5f);
			sd.friction = 0.05f;
			sd.restitution = 0.01f;  
			b2BodyDef bd;
			bd.position.Set(-6.0f, 12.f);
            bd.angle = 0.0;
			b2Body* g = m_world->CreateStaticBody(&bd);
			g->CreateShape(&sd);
        }

		/// 64 bodies - something like a lin. elastic 'compound'
		/// connected via dynamic forces (springs) - see Step() function
		{
			b2PolygonDef sd;
			sd.SetAsBox(0.55f, 0.55f);
			sd.density = 1.5f;
			sd.friction = 0.01f;
			sd.groupIndex = -1;
			b2Vec2 startpoint(-10.f,100.f);
			b2BodyDef bd;
			bd.isBullet = false;
  	 	    bd.allowSleep = false;	
			for (int i = 0; i < 8; ++i)
			{
				for (int j = 0; j < 8; ++j)
				{
					bd.position.Set(j*1.02f, 2.51f + 1.02f * i);
					bd.position  += startpoint;
					b2Body* body  = m_world->CreateDynamicBody(&bd);
					m_bodies[8*i+j] = body;
					body->CreateShape(&sd);
					body->SetMassFromShapes();
				}
			}
		}
	}

	/// Overwrite Test::Step() function to apply dynamic forces (springs)
	void Step(Settings* settings)
	{
		Test::Step(settings);
		for (int i=0; i<8; ++i)
		{
			for (int j=0; j<8; ++j)
			{
				b2Vec2 zero(0.0f,0.0f);
				b2Vec2 down(0.0f, -0.5f);
				b2Vec2 up(0.0f, 0.5f);
				b2Vec2 right(0.5f, 0.0f);
				b2Vec2 left(-0.5f, 0.0f);
				int ind = i*8+j;
				int indr = ind+1;
				int indd = ind+8;
				float32 spring = 500.0f;
				float32 damp = 5.0f;
				if (j<7)
				{
					AddSpringForce(*(m_bodies[ind]),zero,*(m_bodies[indr]),zero,spring, damp, 1.0f);
					AddSpringForce(*(m_bodies[ind]),right,*(m_bodies[indr]),left,0.5f*spring, damp, 0.0f);
				}
				if (i<7)
				{
					AddSpringForce(*(m_bodies[ind]),zero,*(m_bodies[indd]),zero,spring, damp, 1.0f);
					AddSpringForce(*(m_bodies[ind]),up,*(m_bodies[indd]),down,0.5f*spring,damp,0.0f);
				}
				int inddr = indd + 1;
				int inddl = indd - 1;
				float32 drdist = sqrtf(2.0f);
				if (i < 7 && j < 7)
				{
					AddSpringForce(*(m_bodies[ind]),zero,*(m_bodies[inddr]),zero,spring, damp, drdist);
				}
				if (i < 7 && j > 0)
				{
					AddSpringForce(*(m_bodies[ind]),zero,*(m_bodies[inddl]),zero,spring, damp, drdist);
				}

				indr = ind+2;
				indd = ind+8*2;
				if (j<6)
				{
					AddSpringForce(*(m_bodies[ind]),zero,*(m_bodies[indr]),zero,spring, damp, 2.0f);
				}
				if (i<6)
				{
					AddSpringForce(*(m_bodies[ind]),zero,*(m_bodies[indd]),zero,spring,damp,2.0f);
				}

				inddr = indd + 2;
				inddl = indd - 2;
				drdist = sqrtf(2.0f)*2.0f;
				if (i < 6 && j < 6)
				{
					AddSpringForce(*(m_bodies[ind]),zero,*(m_bodies[inddr]),zero,spring, damp, drdist);
				}
				if (i < 6 && j > 1)
				{
					AddSpringForce(*(m_bodies[ind]),zero,*(m_bodies[inddl]),zero,spring, damp, drdist);
				}
			}
		}
	}

	/// Add a spring force
	void AddSpringForce(b2Body& bA, b2Vec2& localA, b2Body& bB, b2Vec2& localB, float32 k, float32 friction, float32 desiredDist)
	{
        b2Vec2 pA = bA.GetWorldPoint(localA);
        b2Vec2 pB = bB.GetWorldPoint(localB);
        b2Vec2 diff = pB - pA;
        //Find velocities of attach points
        b2Vec2 vA = bA.GetLinearVelocity() - b2Cross(bA.GetWorldVector(localA), bA.GetAngularVelocity());
        b2Vec2 vB = bB.GetLinearVelocity() - b2Cross(bB.GetWorldVector(localB), bB.GetAngularVelocity());
        b2Vec2 vdiff = vB-vA;
        float32 dx = diff.Normalize(); //normalizes diff and puts length into dx
        float32 vrel = vdiff.x*diff.x + vdiff.y*diff.y;
        float32 forceMag = -k*(dx-desiredDist) - friction*vrel;
        diff *= forceMag; // diff *= forceMag
        bB.ApplyForce(diff, bA.GetWorldPoint(localA));
		diff *= -1.0f;
        bA.ApplyForce(diff, bB.GetWorldPoint(localB));
    }

	static Test* Create()
	{
		return new ElasticBody;
	}

	b2Body* m_bodies[64];
};

#endif
