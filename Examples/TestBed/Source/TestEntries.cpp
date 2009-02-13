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

#include "TestEntries.h"

#include "Tests/ApplyForce.h"
#include "Tests/BipedTest.h"
#include "Tests/BreakableBody.h"
#include "Tests/Bridge.h"
#include "Tests/BroadPhaseTest.h"
#include "Tests/Buoyancy.h"
#include "Tests/Car.h"
#include "Tests/CCDTest.h"
#include "Tests/Chain.h"
#include "Tests/ContactCallbackTest.h"
#include "Tests/CollisionFiltering.h"
#include "Tests/CollisionProcessing.h"
#include "Tests/CompoundShapes.h"
#include "Tests/DistanceTest.h"
#include "Tests/Dominos.h"
#include "Tests/DynamicEdges.h"
#include "Tests/ElasticBody.h"
#include "Tests/Gears.h"
#include "Tests/LineJoint.h"
#include "Tests/PolyCollision.h"
#include "Tests/PolyShapes.h"
#include "Tests/Prismatic.h"
#include "Tests/Pulleys.h"
#include "Tests/Pyramid.h"
#include "Tests/PyramidStaticEdges.h"
#include "Tests/RaycastTest.h"
#include "Tests/Revolute.h"
#include "Tests/SensorTest.h"
#include "Tests/ShapeEditing.h"
#include "Tests/SliderCrank.h"
#include "Tests/SphereStack.h"
#include "Tests/StaticEdges.h"
#include "Tests/TheoJansen.h"
#include "Tests/TimeOfImpact.h"
#include "Tests/VaryingFriction.h"
#include "Tests/VaryingRestitution.h"
#include "Tests/VerticalStack.h"
#include "Tests/Web.h"

TestEntry g_testEntries[] =
{
	{"Static Edges", StaticEdges::Create},
	{"Pyramid And Static Edges", PyramidStaticEdges::Create},
	{"Dynamic Edges", DynamicEdges::Create},
	{"Line Joint", LineJoint::Create},
	{"SphereStack", SphereStack::Create},
	{"Pyramid", Pyramid::Create},
	{"Prismatic", Prismatic::Create},
	{"Revolute", Revolute::Create},
	{"Bridge", Bridge::Create},
	{"Sensor Test", SensorTest::Create},
	{"Breakable Body", BreakableBody::Create},
	{"Vertical Stack", VerticalStack::Create},
	{"Polygon Shapes", PolyShapes::Create},
	{"Theo Jansen's Walker", TheoJansen::Create},
	{"Contact Callback Test", ContactCB::Create},
	{"Web", Web::Create},
	{"Varying Friction", VaryingFriction::Create},
	{"Varying Restitution", VaryingRestitution::Create},
	{"Dominos", Dominos::Create},
	{"CCD Test", CCDTest::Create},
	{"Biped Test", BipedTest::Create},
	{"Car", Car::Create},
	{"Gears", Gears::Create},
	{"Slider Crank", SliderCrank::Create},
	{"Compound Shapes", CompoundShapes::Create},
	{"Chain", Chain::Create},
	{"Collision Processing", CollisionProcessing::Create},
	{"Collision Filtering", CollisionFiltering::Create},
	{"Apply Force", ApplyForce::Create},
	{"Pulleys", Pulleys::Create},
	{"Shape Editing", ShapeEditing::Create},
	{"Time of Impact", TimeOfImpact::Create},
	{"Distance Test", DistanceTest::Create},
	{"Broad Phase", BroadPhaseTest::Create},
	{"PolyCollision", PolyCollision::Create},
	{"Elastic Body", ElasticBody::Create},
	{"Raycast Test", RaycastTest::Create},
	{"Buoyancy", Buoyancy::Create},
	{NULL, NULL}
};
