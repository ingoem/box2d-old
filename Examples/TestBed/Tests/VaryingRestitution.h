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

#ifndef VARYING_RESTITUTION_H
#define VARYING_RESTITUTION_H

class VaryingRestitution : public Test
{
public:

	VaryingRestitution()
	{
#if 1
		{
			b2PolygonDef sd;
			sd.SetAsBox(50.0f, 10.0f);
			
			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			body->AddShape(m_world->Create(&sd));
			m_world->Create(&bd);
		}

		{
			b2CircleDef sd;
			sd.radius = 1.0f;
			sd.density = 1.0f;

			float32 restitution[7] = {0.0f, 0.1f, 0.3f, 0.5f, 0.75f, 0.9f, 1.0f};

			for (int32 i = 0; i < 7; ++i)
			{
				sd.restitution = restitution[i];
				b2BodyDef bd;
				body->AddShape(m_world->Create(&sd));
				bd.position.Set(-10.0f + 3.0f * i, 20.0f);
				m_world->Create(&bd);
			}
		}
#elif 1
		// Test bug in poly collision
		{
			b2PolygonDef sd;
			sd.SetAsBox(50.0f, 10.0f);

			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			body->AddShape(&sd);
			m_world->Create(&bd);
		}

		{
			b2PolygonDef sd_bottom;
			sd_bottom.SetAsBox( 1.5f, 0.15f );

			b2PolygonDef sd_left;
			sd_left.SetAsBox( 0.15f, 2.5f );
			sd_left.localPosition.Set( -1.45f, 2.35f );

			b2PolygonDef sd_right;
			sd_right.SetAsBox( 0.15f, 2.5f );
			sd_right.localPosition.Set( 1.45f, 2.35f );

			b2BodyDef bd;
			bd.position.Set( 0.0f, 3.0f );
			body->AddShape( &sd_bottom );
			body->AddShape( &sd_left );
			body->AddShape( &sd_right );

			m_world->Create(&bd);
		}

		// debug thing
		{
			b2PolygonDef sd;
			sd.SetAsBox(3.0f, 0.5f);
			sd.localRotation = 3.141596f * 0.25f;
			sd.friction = 0.01f;

			b2BodyDef bd;
			bd.position.Set( 3.5f, 10.0f );
			body->AddShape( &sd );

			m_world->Create( &bd );


			sd.SetAsBox( 0.5f, 0.5f );
			sd.density = 5.0f;

			b2BodyDef penetrating_box;
			penetrating_box.position.Set( 4.5f, 20.0f );
			penetrating_box.AddShape( &sd );
			m_world->Create( &penetrating_box );
		}
#elif 1
		// Test bug in poly collision

		m_world->m_gravity.Set(0.0f, 0.005f);
		m_world->m_allowSleep = false;

		{
			b2PolygonDef box;
			box.vertexCount = 4;
			box.vertices[0].Set(2.0f, 1.2f);
			box.vertices[1].Set(-2.0f, 1.2f);
			box.vertices[2].Set(-2.0f, -1.2f);
			box.vertices[3].Set(2.0f, -1.2f);

			box.density = 1.0f;
			box.friction = 0.15f;
			box.restitution = 0.6f;

			b2BodyDef body;
			body.position.Set(45.0f, 27.65f);
			body.position.Set(45.0f, 27.729946f);
			body.allowSleep = false;
			body.angle = 330.0f / 180.0f * b2_pi;
			body.angle = 5.7595868f;
			body.AddShape(&box);

			m_world->Create(&body);
		}

		{
			b2PolygonDef box;
			box.vertexCount = 4;
			box.vertices[0].Set(20.1f, 0.22f);
			box.vertices[1].Set(-20.03f, 0.23f);
			box.vertices[2].Set(-20.05f, -0.21f);
			box.vertices[3].Set(20.01f, -0.23f);

			box.density = 0.0f;
			box.friction = 0.5f;
			box.restitution = 0.5f;
			b2BodyDef body;
			body.position.Set(32.0f, 30.0f);
			body.AddShape(&box);

			m_world->Create(&body);
		}
#else
		{
			b2PolygonDef sd;
			sd.type = e_boxShape;
			sd.SetAsBox(50.0f, 10.0f);

			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			body->AddShape(&sd);
			m_world->Create(&bd);
		}

		{
			b2PolygonDef sd_bottom;
			sd_bottom.type = e_boxShape;
			sd_bottom.SetAsBox( 1.5f, 0.15f );

			b2PolygonDef sd_left;
			sd_left.type = e_boxShape;
			sd_left.SetAsBox( 0.15f, 2.7f );
			sd_left.localPosition.Set( -1.45f, 2.35f );

			b2PolygonDef sd_right;
			sd_right.type = e_boxShape;
			sd_right.SetAsBox( 0.15f, 2.7f );
			sd_right.localPosition.Set( 1.45f, 2.35f );

			b2BodyDef bd;
			bd.position.Set( 0.0f, 3.0f );
			body->AddShape( &sd_bottom );
			body->AddShape( &sd_left );
			body->AddShape( &sd_right );

			m_world->Create(&bd);
		}

		// debug thing
		{
			b2PolygonDef sd;
			sd.SetAsBox( 0.5f, 0.5f );
			sd.density = 5.0f;

			b2BodyDef penetrating_box;
			penetrating_box.position.Set( -0.80f, 20.0f );
			penetrating_box.AddShape( &sd );
			m_world->Create( &penetrating_box );

		}
#endif
	}

	static Test* Create()
	{
		return new VaryingRestitution;
	}
};

#endif
