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

#include <Box2D/Collision/b2BroadPhase2.h>

#include <Box2D/Collision/b2PairManager.h>

b2BroadPhase2::b2BroadPhase2(b2PairCallback* callback)
{
	m_pairCallback = callback;

	// Build initial proxy pool and free list.
	m_proxyCount = b2_maxProxies;
	m_proxyPool = (b2Proxy2*)b2Alloc(m_proxyCount * sizeof(b2Proxy2));

	m_freeProxy = 0;
	for (int32 i = 0; i < m_proxyCount - 1; ++i)
	{
		m_proxyPool[i].next = i + 1;
	}
	m_proxyPool[m_proxyCount-1].next = e_nullProxy;

	// Build initial pair pool and free list.
	m_pairCount = b2_maxPairs;
	m_pairPool = (b2Pair2*)b2Alloc(m_pairCount * sizeof(b2Pair2));
	
	m_freePair = 0;
	for (int32 i = 0; i < m_pairCount - 1; ++i)
	{
		m_pairPool[i].next = i + 1;
	}
	m_pairPool[m_pairCount-1].next = e_nullPair;

	m_hashTable = (uint16*)b2Alloc(m_pairCount * sizeof(uint16));
	for (int32 i = 0; i < m_pairCount; ++i)
	{
		m_hashTable[i] = e_nullPair;
	}
}


b2BroadPhase2::~b2BroadPhase2()
{
	b2Free(m_proxyPool);
	b2Free(m_pairPool);
	b2Free(m_hashTable);
}

uint16 b2BroadPhase2::CreateProxy(const b2AABB& aabb, void* userData)
{

}

void b2BroadPhase2::DestroyProxy(uint16 proxyId)
{

}

void b2BroadPhase2::MoveProxy(uint16 proxyId, const b2AABB& aabb)
{

}

void* b2BroadPhase2::GetUserData(uint16 proxyId)
{

}

void b2BroadPhase2::Commit()
{

}

void b2BroadPhase2::Validate()
{

}
