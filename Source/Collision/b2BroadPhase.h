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

#ifndef B2_BROAD_PHASE_H
#define B2_BROAD_PHASE_H

/*
This broad phase uses the Sweep and Prune algorithm as described in:
Collision Detection in Interactive 3D Environments by Gino van den Bergen
Also, some ideas, such as using integral values for fast compares comes from
Bullet (http:/www.bulletphysics.com).
*/

#include "../Common/b2Settings.h"
#include "b2Collision.h"
#include "b2PairManager.h"
#include <limits.h>

const uint16 b2_invalid = USHRT_MAX;
const uint16 b2_nullEdge = USHRT_MAX;


struct b2Bound
{
	bool IsLower() const { return (value & 1) == 0; }
	bool IsUpper() const { return (value & 1) == 1; }

	uint16 value;
	uint16 proxyId;
	uint16 stabbingCount;
};

struct b2Proxy
{
	uint16 GetNext() const { return lowerBounds[0]; }
	void SetNext(uint16 next) { lowerBounds[0] = next; }
	bool IsValid() const { return overlapCount != b2_invalid; }

	uint16 lowerBounds[2], upperBounds[2];
	uint16 overlapCount;
	uint16 timeStamp;
	void* userData;
	uint16 categoryBits;
	uint16 maskBits;
	int16 groupIndex;
};

struct b2BufferedPair
{
	uint16 proxyId1;
	uint16 proxyId2;
};

struct b2PairCallback
{
	virtual ~b2PairCallback() {}

	// This returns the new pair user data.
	virtual void* PairAdded(void* proxyUserData1, void* proxyUserData2) = 0;

	// This should free the pair's user data.
	virtual void PairRemoved(void* pairUserData) = 0;
};

class b2BroadPhase
{
public:
	b2BroadPhase(const b2AABB& worldAABB, b2PairCallback* callback);
	~b2BroadPhase();

	// Create and destroy proxies. These call Flush first.
	uint16 CreateProxy(const b2AABB& aabb, int16 groupIndex, uint16 categoryBits, uint16 maskBits, void* userData);
	void DestroyProxy(uint16 proxyId);

	// Call MoveProxy as many times as you like, then when you are done
	// call Flush to finalized the proxy pairs (for your time step).
	void MoveProxy(uint16 proxyId, const b2AABB& aabb);
	void Flush();

	// Query an AABB for overlapping proxies, returns the user data and
	// the count, up to the supplied maximum count.
	int32 Query(const b2AABB& aabb, void** userData, int32 maxCount);

	void Validate();
	void ValidatePairs();

private:
	void ComputeBounds(uint16* lowerValues, uint16* upperValues, const b2AABB& aabb);

	void AddPair(uint16 proxyId1, uint16 proxyId2);
	void RemovePair(uint16 proxyId1, uint16 proxyId2);

	bool TestOverlap(b2Proxy* p1, b2Proxy* p2);

	void Query(uint16* lowerIndex, uint16* upperIndex, uint16 lowerValue, uint16 upperValue,
				b2Bound* edges, uint16 edgeCount, int32 axis);
	void IncrementOverlapCount(uint16 proxyId);
	void IncrementTimeStamp();

	bool InRange(const b2AABB& aabb);

	bool ShouldCollide(uint16 id1, uint16 id2);

public:
	b2PairManager m_pairManager;

	b2Proxy m_proxyPool[b2_maxProxies];
	uint16 m_freeProxy;

	b2BufferedPair m_pairBuffer[b2_maxPairs];
	int32 m_pairBufferCount;

	b2Bound m_bounds[2][2*b2_maxProxies];

	b2PairCallback* m_pairCallback;
	uint16 m_queryResults[b2_maxProxies];
	int32 m_queryResultCount;

	b2AABB m_worldAABB;
	b2Vec2 m_quantizationFactor;
	int32 m_proxyCount;
	uint16 m_timeStamp;

	static bool s_validate;
};

#endif