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

#ifndef COLLISION_FILTERING_H
#define COLLISION_FILTERING_H

#include <string.h>

// This is a test of collision filtering.
// There is a triangle, a box, and an octogon.
// There are 6 shapes. 3 large and 3 small.
// The 3 small ones always collide.
// The 3 large ones never collide.
// The boxes don't collide with triangles (except if both are small).
const int16	k_smallGroup = 1;
const int16 k_largeGroup = -1;

const uint16 k_defaultCategory = 0x0001;
const uint16 k_triangleCategory = 0x0002;
const uint16 k_boxCategory = 0x0004;
const uint16 k_octogonCategory = 0x0008;

const uint16 k_triangleMask = 0xFFFF;
const uint16 k_boxMask = 0xFFFF ^ k_triangleCategory;
const uint16 k_octogonMask = 0xFFFF;

class CollisionFiltering : public Test
{
public:
	CollisionFiltering()
	{
		// Ground body
		{
			b2BoxDef sd;
			sd.type = e_boxShape;
			sd.extents.Set(50.0f, 10.0f);
			sd.friction = 0.3f;

			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			bd.AddShape(&sd);
			m_world->CreateBody(&bd);
		}

		// Small triangle
		b2PolyDef triangleShapeDef;
		triangleShapeDef.vertexCount = 3;
		triangleShapeDef.vertices[0].Set(-1.0f, 0.0f);
		triangleShapeDef.vertices[1].Set(1.0f, 0.0f);
		triangleShapeDef.vertices[2].Set(0.0f, 2.0f);
		triangleShapeDef.density = 1.0f;

		triangleShapeDef.groupIndex = k_smallGroup;
		triangleShapeDef.categoryBits = k_triangleCategory;
		triangleShapeDef.maskBits = k_triangleMask;

		b2BodyDef triangleBodyDef;
		triangleBodyDef.AddShape(&triangleShapeDef);
		triangleBodyDef.position.Set(-5.0f, 2.0f);

		m_world->CreateBody(&triangleBodyDef);

		// Large triangle (recycle definitions)
		triangleShapeDef.vertices[0] *= 2.0f;
		triangleShapeDef.vertices[1] *= 2.0f;
		triangleShapeDef.vertices[2] *= 2.0f;
		triangleShapeDef.groupIndex = k_largeGroup;
		triangleBodyDef.position.Set(-5.0f, 6.0f);
		m_world->CreateBody(&triangleBodyDef);

		// Small box
		b2BoxDef boxShapeDef;
		boxShapeDef.extents.Set(1.0f, 0.5f);
		boxShapeDef.density = 1.0f;

		boxShapeDef.groupIndex = k_smallGroup;
		boxShapeDef.categoryBits = k_boxCategory;
		boxShapeDef.maskBits = k_boxMask;

		b2BodyDef boxBodyDef;
		boxBodyDef.position.Set(0.0f, 2.0f);
		boxBodyDef.AddShape(&boxShapeDef);

		m_world->CreateBody(&boxBodyDef);

		// Large box (recycle definitions)
		boxShapeDef.extents *= 2.0f;
		boxShapeDef.groupIndex = k_largeGroup;
		boxBodyDef.position.Set(0.0f, 6.0f);
		m_world->CreateBody(&boxBodyDef);

		// Small octogon
		b2PolyDef octogonShapeDef;
		octogonShapeDef.vertexCount = 8;
		float32 w = 2.0f;
		float32 b = w / (2.0f + sqrtf(2.0f));
		float32 s = sqrtf(2.0f) * b;
		octogonShapeDef.vertices[0].Set(0.5f * s, 0.0f);
		octogonShapeDef.vertices[1].Set(0.5f * w, b);
		octogonShapeDef.vertices[2].Set(0.5f * w, b + s);
		octogonShapeDef.vertices[3].Set(0.5f * s, w);
		octogonShapeDef.vertices[4].Set(-0.5f * s, w);
		octogonShapeDef.vertices[5].Set(-0.5f * w, b + s);
		octogonShapeDef.vertices[6].Set(-0.5f * w, b);
		octogonShapeDef.vertices[7].Set(-0.5f * s, 0.0f);
		octogonShapeDef.density = 1.0f;

		octogonShapeDef.groupIndex = k_smallGroup;
		octogonShapeDef.categoryBits = k_octogonCategory;
		octogonShapeDef.maskBits = k_octogonMask;

		b2BodyDef octogonBodyDef;
		octogonBodyDef.AddShape(&octogonShapeDef);
		octogonBodyDef.position.Set(5.0f, 2.0f);
		
		m_world->CreateBody(&octogonBodyDef);

		// Large octogon
		octogonShapeDef.vertices[0] *= 2.0f;
		octogonShapeDef.vertices[1]	*= 2.0f;
		octogonShapeDef.vertices[2]	*= 2.0f;
		octogonShapeDef.vertices[3]	*= 2.0f;
		octogonShapeDef.vertices[4]	*= 2.0f;
		octogonShapeDef.vertices[5]	*= 2.0f;
		octogonShapeDef.vertices[6]	*= 2.0f;
		octogonShapeDef.vertices[7]	*= 2.0f;
		octogonShapeDef.groupIndex = k_largeGroup;
		octogonBodyDef.position.Set(5.0f, 6.0f);
		m_world->CreateBody(&octogonBodyDef);
	}
	static Test* Create()
	{
		return new CollisionFiltering;
	}
};

#endif
