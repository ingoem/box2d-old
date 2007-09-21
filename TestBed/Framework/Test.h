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

#ifndef TEST_H
#define TEST_H

#include "Engine/Common/b2Settings.h"
#include "Engine/Dynamics/b2WorldCallbacks.h"

struct b2World;
struct b2Body;
struct b2MouseJoint;
struct b2Vec2;
class Test;
struct Settings;

typedef Test* TestCreateFcn();

struct Settings
{
	Settings() :
		hz(60.0f),
		iterationCount(10),
		drawStats(0),
		drawContacts(0),
		drawImpulses(0),
		drawAABBs(0),
		drawPairs(0),
		enableWarmStarting(1),
		enablePositionCorrection(1)
		{}

	float hz;
	int iterationCount;
	int drawStats;
	int drawContacts;
	int drawImpulses;
	int drawAABBs;
	int drawPairs;
	int enableWarmStarting;
	int enablePositionCorrection;
};

struct TestEntry
{
	const char *name;
	TestCreateFcn *createFcn;
};

extern TestEntry g_testEntries[];

// This is called when a joint in the world is implicitly destroyed
// because an attached body is destroyed. This gives us a chance to
// nullify the mouse joint.
class JointCallback : public b2JointDestroyedCallback
{
public:
	void Notify(b2Joint* joint);

	Test* test;
};

class Test
{
public:

	Test();
	virtual ~Test();

	void SetTextLine(int32 line) { m_textLine = line; }
	virtual void Step(const Settings* settings);
	virtual void Keyboard(unsigned char key) {}
	void MouseDown(const b2Vec2& p);
	void MouseUp();
	void MouseMove(const b2Vec2& p);
	void LaunchBomb();

	// Let derived tests know that a joint was destroyed.
	virtual void JointDestroyed(b2Joint* joint) {}

protected:
	friend JointCallback;

	JointCallback m_jointCallback;
	int32 m_textLine;
	b2World* m_world;
	b2Body* m_bomb;
	b2MouseJoint* m_mouseJoint;
};

#endif