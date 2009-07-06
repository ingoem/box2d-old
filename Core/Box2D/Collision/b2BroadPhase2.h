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

#ifndef B2_BROADPHASE2_H
#define B2_BROADPHASE2_H

#include "../Common/b2Settings.h"
#include "b2Collision.h"
#include "b2DynamicTree.h"

class b2PairCallback;

struct b2Proxy2
{
	b2AABB aabb;
	uint16 next;
	void* userData;
};

struct b2Pair2
{
	enum
	{
		e_pairBuffered	= 0x0001,
		e_pairRemoved	= 0x0002,
		e_pairFinal		= 0x0004,
	};

	void SetBuffered()		{ status |= e_pairBuffered; }
	void ClearBuffered()	{ status &= ~e_pairBuffered; }
	bool IsBuffered()		{ return (status & e_pairBuffered) == e_pairBuffered; }

	void SetRemoved()		{ status |= e_pairRemoved; }
	void ClearRemoved()		{ status &= ~e_pairRemoved; }
	bool IsRemoved()		{ return (status & e_pairRemoved) == e_pairRemoved; }

	void SetFinal()		{ status |= e_pairFinal; }
	bool IsFinal()		{ return (status & e_pairFinal) == e_pairFinal; }

	void* userData;
	uint16 proxyId1;
	uint16 proxyId2;
	uint16 next;
	uint16 status;
};

/// The broad-phase is used for managing pairs and performing volume queries and ray casts.
/// The size of the world is limited only by single precision floating point. You can
/// only have USHRT_MAX - 1 proxies. There is no limit to the number of pairs, however
/// creating all the proxies at the origin can lead to
class b2BroadPhase2
{
public:

	enum
	{
		e_nullProxy = USHRT_MAX,
		e_nullPair = USHRT_MAX
	};

	b2BroadPhase2(b2PairCallback* callback);
	~b2BroadPhase2();

	/// Create and destroy proxies. These result in pair creation/destruction.
	uint16 CreateProxy(const b2AABB& aabb, void* userData);
	void DestroyProxy(uint16 proxyId);

	/// Call MoveProxy as many times as you like, then when you are done
	/// call UpdatePairs to finalized the proxy pairs (for your time step).
	void MoveProxy(uint16 proxyId, const b2AABB& aabb);

	/// Get user data from a proxy. Returns NULL if the id is invalid.
	void* GetUserData(uint16 proxyId);

	/// Update the pairs. This results in pair callbacks.
	void Commit();

	/// Validate the current pairs.
	void Validate();

private:

	b2DynamicTree m_tree;

	b2Proxy2* m_proxyPool;
	int32 m_proxyCount;
	uint16 m_freeProxy;

	// TODO_ERIN need to fix move list on destroy.
	uint16 m_moveList;

	b2Pair2* m_pairPool;
	int32 m_pairCount;
	uint16 m_freePair;

	uint16* m_hashTable;

	b2PairCallback* m_pairCallback;

	uint16 m_timeStamp;
};


inline void* b2BroadPhase2::GetUserData(uint16 proxyId)
{
	if (proxyId == e_nullProxy)
	{
		return NULL;
	}

	return m_proxyPool[proxyId].userData;
}

#endif
