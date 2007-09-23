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

#include "BroadPhaseTest.h"
#include "TestBed/Framework/Render.h"

#include <stdio.h>
#include <string.h>

const float32 k_width = 1.0f;

inline void GetRandomAABB(b2AABB* aabb)
{
	b2Vec2 w; w.Set(k_width, k_width);
	aabb->minVertex.x = b2Random(-k_extent, k_extent);
	aabb->minVertex.y = b2Random(0.0f, 2.0f * k_extent);
	aabb->maxVertex = aabb->minVertex + w;
}

inline void MoveAABB(b2AABB* aabb)
{
	b2Vec2 d;
	d.x = b2Random(-0.5f, 0.5f);
	d.y = b2Random(-0.5f, 0.5f);
	//d.x = 2.0f;
	//d.y = 0.0f;
	aabb->minVertex += d;
	aabb->maxVertex += d;

	b2Vec2 c0 = 0.5f * (aabb->minVertex + aabb->maxVertex);
	b2Vec2 min; min.Set(-k_extent, 0.0f);
	b2Vec2 max; max.Set(k_extent, 2.0f * k_extent);
	b2Vec2 c = b2Clamp(c0, min, max);

	aabb->minVertex += c - c0;
	aabb->maxVertex += c - c0;
}

void* Callback::PairAdded(void* proxyUserData1, void* proxyUserData2)
{
	Actor* actor1 = (Actor*)proxyUserData1;
	Actor* actor2 = (Actor*)proxyUserData2;

	//bool overlap = TestOverlap(actor1->aabb, actor2->aabb);
	//b2Assert(overlap == true);

	uint16 id1 = (uint16)(actor1 - m_test->m_actors);
	uint16 id2 = (uint16)(actor2 - m_test->m_actors);
	b2Assert(id1 < k_actorCount);
	b2Assert(id2 < k_actorCount);

	b2Assert(m_test->m_overlaps[id1][id2] == false);
	m_test->m_overlaps[id1][id2] = true;
	m_test->m_overlaps[id2][id1] = true;
	++m_test->m_overlapCount;

	++actor1->overlapCount;
	++actor2->overlapCount;

	uint32 id = (id2 << 16) | id1;
	return (void*)id;
}

void Callback::PairRemoved(void* pairUserData)
{
	uint32 id = (uint32)pairUserData;
	uint16 id1 = (uint16)(id & 0xFFFF);
	uint16 id2 = (uint16)(id >> 16);

	b2Assert(id1 < k_actorCount && id2 < k_actorCount);

	Actor* actor1 = m_test->m_actors + id1;
	Actor* actor2 = m_test->m_actors + id2;

	// The pair may have been removed by destroying a proxy.
	m_test->m_overlaps[id1][id2] = false;
	m_test->m_overlaps[id2][id1] = false;
	--m_test->m_overlapCount;

	--actor1->overlapCount;
	--actor2->overlapCount;
}

Test* BroadPhaseTest::Create()
{
	return new BroadPhaseTest;
}

BroadPhaseTest::BroadPhaseTest()
{
	srand(888);

	b2AABB worldAABB;
	worldAABB.minVertex.Set(-5.0f * k_extent, -5.0f * k_extent);
	worldAABB.maxVertex.Set(5.0f * k_extent, 5.0f * k_extent);

	m_overlapCount = 0;
	m_overlapCountExact = 0;
	m_callback.m_test = this;

	m_broadPhase = new b2BroadPhase(worldAABB, &m_callback);

	memset(m_overlaps, 0, sizeof(m_overlaps));

	for (int32 i = 0; i < k_actorCount; ++i)
	{
		Actor* actor = m_actors + i;
		GetRandomAABB(&actor->aabb);
		//actor->aabb.minVertex.Set(0.0f, 0.0f);
		//actor->aabb.maxVertex.Set(k_width, k_width);
		actor->proxyId = m_broadPhase->CreateProxy(actor->aabb, 0, 0x0001, 0xFFFF, actor);
		actor->overlapCount = 0;
		m_broadPhase->Validate();
	}

	m_automated = true;
	m_stepCount = 0;
}

BroadPhaseTest::~BroadPhaseTest()
{
	delete m_broadPhase;
}

void BroadPhaseTest::CreateProxy()
{
	for (int32 i = 0; i < k_actorCount; ++i)
	{
		int32 j = rand() % k_actorCount;
		Actor* actor = m_actors + j;
		if (actor->proxyId == b2_nullProxy)
		{
			actor->overlapCount = 0;
			GetRandomAABB(&actor->aabb);
			actor->proxyId = m_broadPhase->CreateProxy(actor->aabb, 0, 0x0001, 0xFFFF, actor);
			return;
		}
	}
}

void BroadPhaseTest::DestroyProxy()
{
	for (int32 i = 0; i < k_actorCount; ++i)
	{
		int32 j = rand() % k_actorCount;
		Actor* actor = m_actors + j;
		if (actor->proxyId != b2_nullProxy)
		{
			m_broadPhase->DestroyProxy(actor->proxyId);
			actor->proxyId = b2_nullProxy;
			actor->overlapCount = 0;
			return;
		}
	}
}

void BroadPhaseTest::MoveProxy()
{
	for (int32 i = 0; i < k_actorCount; ++i)
	{
		int32 j = rand() % k_actorCount;
		//int32 j = 1;
		Actor* actor = m_actors + j;
		if (actor->proxyId == b2_nullProxy)
		{
			continue;
		}

		MoveAABB(&actor->aabb);
		m_broadPhase->MoveProxy(actor->proxyId, actor->aabb);
		return;
	}
}

void BroadPhaseTest::Action()
{
	int32 choice = rand() % 20;

	switch (choice)
	{
	case 0:
		CreateProxy();
		break;

	case 1:
		DestroyProxy();
		break;

	default:
		MoveProxy();
	}
}

void BroadPhaseTest::Step(const Settings* settings)
{
	if (m_stepCount == 1348)
	{
		int32 test = 1;
	}

	if (m_automated == true)
	{
		int32 actionCount = b2Max(1, k_actorCount >> 2);

		for (int32 i = 0; i < actionCount; ++i)
		{
			Action();
		}
	}

	m_broadPhase->Flush();

	for (int32 i = 0; i < k_actorCount; ++i)
	{
		Actor* actor = m_actors + i;
		if (actor->proxyId == b2_nullProxy)
			continue;

		Color c;
		switch (actor->overlapCount)
		{
		case 0:
			c.cx = 0.9f; c.cy = 0.9f; c.cz = 0.9f;
			break;

		case 1:
			c.cx = 0.6f; c.cy = 0.9f; c.cz = 0.6f;
			break;

		default:
			c.cx = 0.9f; c.cy = 0.6f; c.cz = 0.6f;
			break;
		}

		DrawAABB(&actor->aabb, c);
	}

	char buffer[64];
	sprintf(buffer, "overlaps = %d, exact = %d, diff = %d", m_overlapCount, m_overlapCountExact, m_overlapCount - m_overlapCountExact);
	DrawString(5, 30, buffer);
	Validate();

	++m_stepCount;
}

void BroadPhaseTest::Keyboard(unsigned char key)
{
	switch (key)
	{
	case 'a':
		m_automated = !m_automated;
		break;

	case 'c':
		CreateProxy();
		break;

	case 'd':
		DestroyProxy();
		break;

	case 'm':
		MoveProxy();
		break;
	}
}

void BroadPhaseTest::Validate()
{
	m_broadPhase->Validate();

	m_overlapCountExact = 0;

	for (int32 i = 0; i < k_actorCount; ++i)
	{
		Actor* actor1 = m_actors + i;
		if (actor1->proxyId == b2_nullProxy)
			continue;

		for (int32 j = i + 1; j < k_actorCount; ++j)
		{
			Actor* actor2 = m_actors + j;
			if (actor2->proxyId == b2_nullProxy)
				continue;

			bool overlap = b2TestOverlap(actor1->aabb, actor2->aabb);
			if (overlap) ++m_overlapCountExact;

			if (overlap)
			{
				b2Assert(m_overlaps[actor1-m_actors][actor2-m_actors] == true);
			}
		}
	}

	b2Assert(m_overlapCount >= m_overlapCountExact);
}
