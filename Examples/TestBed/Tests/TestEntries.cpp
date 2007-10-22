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

#include "../Framework/Test.h"

#include "VerticalStack.h"
#include "VaryingFriction.h"
#include "VaryingRestitution.h"
#include "CompoundShapes.h"
#include "PolyShapes.h"
#include "Pendulum.h"
#include "Bridge.h"
#include "PolyCollision.h"
#include "BroadPhaseTest.h"
#include "MotorsAndLimits.h"
#include "Pyramid.h"
#include "Web.h"
#include "CollisionFiltering.h"
#include "CollisionProcessing.h"
#include "SliderCrank.h"
#include "Pulleys.h"
#include "Gears.h"

TestEntry g_testEntries[] =
{
	{"Compound Shapes", CompoundShapes::Create},
	{"Collision Processing", CollisionProcessing::Create},
	{"Vertical Stack", VerticalStack::Create},
	{"Pyramid", Pyramid::Create},
	{"Web", Web::Create},
	{"Gears", Gears::Create},
	{"Pulleys", Pulleys::Create},
	{"Bridge", Bridge::Create},
	{"Slider Crank", SliderCrank::Create},
	{"Pendulum", Pendulum::Create},
	{"Polygon Shapes", PolyShapes::Create},
	{"Collision Filtering", CollisionFiltering::Create},
	{"Varying Restitution", VaryingRestitution::Create},
	{"Motors and Limits", MotorsAndLimits::Create},
	{"Varying Friction", VaryingFriction::Create},
	{"PolyCollision", PolyCollision::Create},
	{"Broad Phase", BroadPhaseTest::Create},
	{NULL, NULL}
};
