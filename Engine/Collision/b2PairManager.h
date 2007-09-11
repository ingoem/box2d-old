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

// The pair manager is used by the broad-phase to quickly add/remove/find pairs
// of overlapping proxies. It is based closely on code provided by Pierre Terdiman.
// http://www.codercorner.com/IncrementalSAP.txt

#ifndef B2_PAIR_MANAGER_H
#define B2_PAIR_MANAGER_H

#include "Engine/Common/b2Settings.h"
#include "Engine/Common/b2Math.h"

const uint16 b2_nullPair = USHRT_MAX;
const uint16 b2_nullProxy = USHRT_MAX;
const int32 b2_tableCapacity = b2_maxPairs;	// must be a power of two
const int32 b2_tableMask = b2_tableCapacity - 1;

struct b2Pair
{
	enum
	{
		e_bufferedPair = 0x0001,
		e_removePair = 0x0002,
	};

	void SetBuffered()	{ status |= e_bufferedPair; }
	void ClearBuffered()	{ status &= ~e_bufferedPair; }
	bool IsBuffered()		{ return status & e_bufferedPair; }

	void SetAdded()		{ status &= ~e_removePair; }
	void SetRemoved()	{ status |= e_removePair; }
	bool IsRemoved()	{ return (status & e_removePair) == e_removePair; }

	void* userData;
	uint16 proxyId1;
	uint16 proxyId2;
	uint16 status;
};

class b2PairManager
{
public:
	b2PairManager();
	~b2PairManager();

	// Add a pair and return the new pair. If the pair already exists,
	// no new pair is created and the old one is returned.
	b2Pair* Add(uint16 proxyId1, uint16 proxyId2);

	// Remove a pair, return the pair's userData.
	void* Remove(uint16 proxyId1, uint16 proxyId2);

	b2Pair* Find(uint16 proxyId1, uint16 proxyId2);

	int32 GetCount() const { return m_pairCount; }
	b2Pair* GetPairs() { return m_pairs; }

private:
	b2Pair* Find(uint16 proxyId1, uint16 proxyId2, uint32 hashValue);

public:
	b2Pair m_pairs[b2_maxPairs];
	int32 m_pairCount;

	uint16 m_hashTable[b2_tableCapacity];
	uint16 m_next[b2_maxPairs];
};

#endif