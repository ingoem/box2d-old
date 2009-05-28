/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
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

#ifndef POLYCOLLISION_H
#define POLYCOLLISION_H

class PolyCollision : public Test
{
public:
	PolyCollision()
	{
		{
			b2Vec2 vertices[4];
			vertices[0].Set(-9.0f, -1.1f);
			vertices[1].Set(7.0f, -1.1f);
			vertices[2].Set(5.0f, -0.9f);
			vertices[3].Set(-11.0f, -0.9f);
			m_polygonA.Set(vertices, 4);
			m_transformA.Set(b2Vec2(0.0f, 10.0f), 0.0f);
		}

		{
			m_polygonB.SetAsBox(0.5f, 0.5f);
			m_positionB.SetZero();
			m_angleB = 0.0f;
			m_transformB.Set(m_positionB, m_angleB);
		}
	}

	static Test* Create()
	{
		return new PolyCollision;
	}

	void Step(Settings* settings)
	{
		B2_NOT_USED(settings);

		b2CollidePolygons(&m_manifold, &m_polygonA, m_transformA, &m_polygonB, m_transformB);

		m_debugDraw.DrawString(5, m_textLine, "point count = %d", m_manifold.pointCount);
		m_textLine += 15;

		{
			b2Color color(0.9f, 0.9f, 0.9f);
			b2Vec2 v[b2_maxPolygonVertices];
			for (int32 i = 0; i < m_polygonA.m_vertexCount; ++i)
			{
				v[i] = b2Mul(m_transformA, m_polygonA.m_vertices[i]);
			}
			m_debugDraw.DrawPolygon(v, m_polygonA.m_vertexCount, color);

			for (int32 i = 0; i < m_polygonB.m_vertexCount; ++i)
			{
				v[i] = b2Mul(m_transformB, m_polygonB.m_vertices[i]);
			}
			m_debugDraw.DrawPolygon(v, m_polygonB.m_vertexCount, color);
		}

		for (int32 i = 0; i < m_manifold.pointCount; ++i)
		{
			m_debugDraw.DrawString(5, m_textLine, "separation %d = %g", i, m_manifold.points[i].separation);
			m_textLine += 15;

			b2Vec2 point = b2Mul(m_transformA, m_manifold.points[i].localPointA);
			m_debugDraw.DrawPoint(point, 4.0f, b2Color(0.9f, 0.3f, 0.3f));
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'a':
			m_positionB.x -= 0.1f;
			break;

		case 'd':
			m_positionB.x += 0.1f;
			break;

		case 's':
			m_positionB.y -= 0.1f;
			break;

		case 'w':
			m_positionB.y += 0.1f;
			break;

		case 'q':
			m_angleB += 0.1f * b2_pi;
			break;

		case 'e':
			m_angleB -= 0.1f * b2_pi;
			break;
		}

		m_transformB.Set(m_positionB, m_angleB);
	}

	b2Manifold m_manifold;

	b2PolygonShape m_polygonA;
	b2PolygonShape m_polygonB;

	b2XForm m_transformA;
	b2XForm m_transformB;

	b2Vec2 m_positionB;
	float32 m_angleB;
};

#endif