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
#include "../Framework/Render.h"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "freeglut/gl/glut.h"
#endif

//#include "ApplyForce.h"
#include "BipedTest.h"
#include "Bridge.h"
//#include "BroadPhaseTest.h"
#include "CCDTest.h"
#include "Chain.h"
#include "CollisionFiltering.h"
#include "CollisionProcessing.h"
#include "CompoundShapes.h"
//#include "DistanceTest.h"
#include "Gears.h"
//#include "MotorsAndLimits.h"
//#include "PolyCollision.h"
//#include "PolyShapes.h"
//#include "Pulleys.h"
#include "Pyramid.h"
//#include "SliderCrank.h"
//#include "TimeOfImpact.h"
#include "VaryingFriction.h"
#include "VaryingRestitution.h"
#include "VerticalStack.h"
//#include "Web.h"

TestEntry g_testEntries[] =
{
	{"Biped Test", BipedTest::Create},
	{"Collision Processing", CollisionProcessing::Create},
	{"CCD Test", CCDTest::Create},
	{"Vertical Stack", VerticalStack::Create},
	{"Collision Filtering", CollisionFiltering::Create},
	{"Varying Friction", VaryingFriction::Create},
	{"Varying Restitution", VaryingRestitution::Create},
	{"Compound Shapes", CompoundShapes::Create},
	{"Chain", Chain::Create},
	{"Gears", Gears::Create},
	{"Pyramid", Pyramid::Create},
	{"Bridge", Bridge::Create},
	//{"Time of Impact", TimeOfImpact::Create},
	//{"Polygon Shapes", PolyShapes::Create},
	//{"Distance Test", DistanceTest::Create},
	//{"Apply Force", ApplyForce::Create},
	//{"PolyCollision", PolyCollision::Create},
	//{"Broad Phase", BroadPhaseTest::Create},
	//{"Web", Web::Create},
	//{"Pulleys", Pulleys::Create},
	//{"Slider Crank", SliderCrank::Create},
	//{"Motors and Limits", MotorsAndLimits::Create},
	{NULL, NULL}
};
