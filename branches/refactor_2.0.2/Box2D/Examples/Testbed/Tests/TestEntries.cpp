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

#include "../Framework/Test.h"
#include "../Framework/Render.h"
#include "../Framework/freeglut/GL/glut.h"
#include <cstring>

#include "ApplyForce.h"
#include "Bridge.h"
#include "CCDTest.h"
#include "Chain.h"
#include "CollisionFiltering.h"
#include "CollisionProcessing.h"
#include "CompoundShapes.h"
#include "DistanceTest.h"
#include "Dominos.h"
#include "DynamicTreeTest.h"
#include "Gears.h"
#include "LineJoint.h"
#include "PolyCollision.h"
#include "PolyShapes.h"
#include "Prismatic.h"
#include "Pulleys.h"
#include "Pyramid.h"
#include "Revolute.h"
#include "SensorTest.h"
#include "ShapeEditing.h"
#include "SliderCrank.h"
#include "SphereStack.h"
#include "TheoJansen.h"
#include "TimeOfImpact.h"
#include "VaryingFriction.h"
#include "VaryingRestitution.h"
#include "VerticalStack.h"
#include "Web.h"

TestEntry g_testEntries[] =
{
	{"CCD Test", CCDTest::Create},
	{"Pyramid", Pyramid::Create},
	{"SphereStack", SphereStack::Create},
	{"Sensor Test", SensorTest::Create},
	{"Vertical Stack", VerticalStack::Create},
	{"Time of Impact", TimeOfImpact::Create},
	{"Distance Test", DistanceTest::Create},
	{"PolyCollision", PolyCollision::Create},
	{"Dynamic Tree", DynamicTreeTest::Create},
	{"Line Joint", LineJoint::Create},
	{"Prismatic", Prismatic::Create},
	{"Revolute", Revolute::Create},
	{"Bridge", Bridge::Create},
	{"Polygon Shapes", PolyShapes::Create},
	{"Theo Jansen's Walker", TheoJansen::Create},
	{"Web", Web::Create},
	{"Varying Friction", VaryingFriction::Create},
	{"Varying Restitution", VaryingRestitution::Create},
	{"Dominos", Dominos::Create},
	{"Gears", Gears::Create},
	{"Slider Crank", SliderCrank::Create},
	{"Compound Shapes", CompoundShapes::Create},
	{"Chain", Chain::Create},
	{"Collision Processing", CollisionProcessing::Create},
	{"Collision Filtering", CollisionFiltering::Create},
	{"Apply Force", ApplyForce::Create},
	{"Pulleys", Pulleys::Create},
	{"Shape Editing", ShapeEditing::Create},
	{NULL, NULL}
};
