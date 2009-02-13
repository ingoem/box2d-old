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

#ifndef BOX2D_H
#define BOX2D_H

/**
\mainpage Box2D API Documentation

\section intro_sec Getting Started

For tutorials please see http://www.box2d.org/manual.html

For discussion please visit http://www.box2d.org/forum
*/

// These include files constitute the main Box2D API

#include "box2d/Common/b2Settings.h"

#include "box2d/Collision/Shapes/b2CircleShape.h"
#include "box2d/Collision/Shapes/b2PolygonShape.h"
#include "box2d/Collision/Shapes/b2EdgeShape.h"
#include "box2d/Collision/b2BroadPhase.h"
#include "box2d/Dynamics/b2WorldCallbacks.h"
#include "box2d/Dynamics/b2World.h"
#include "box2d/Dynamics/b2Body.h"

#include "box2d/Dynamics/Contacts/b2Contact.h"

#include "box2d/Dynamics/Joints/b2DistanceJoint.h"
#include "box2d/Dynamics/Joints/b2GearJoint.h"
#include "box2d/Dynamics/Joints/b2LineJoint.h"
#include "box2d/Dynamics/Joints/b2MouseJoint.h"
#include "box2d/Dynamics/Joints/b2PrismaticJoint.h"
#include "box2d/Dynamics/Joints/b2PulleyJoint.h"
#include "box2d/Dynamics/Joints/b2RevoluteJoint.h"

#include "box2d/Dynamics/Controllers/b2BuoyancyController.h"
#include "box2d/Dynamics/Controllers/b2ConstantForceController.h"
#include "box2d/Dynamics/Controllers/b2ConstantAccelController.h"
#include "box2d/Dynamics/Controllers/b2GravityController.h"
#include "box2d/Dynamics/Controllers/b2TensorDampingController.h"

#endif
