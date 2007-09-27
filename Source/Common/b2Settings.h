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

#ifndef B2_SETTINGS_H
#define B2_SETTINGS_H

#define NOT_USED(x) x

typedef signed char	int8;
typedef signed short int16;
typedef signed int int32;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef float float32;

const float32 b2_pi = 3.14159265359f;

// Global tuning constants based on MKS units.

// Collision
const int32 b2_maxManifoldPoints = 2;
const int32 b2_maxShapesPerBody = 64;
const int32 b2_maxPolyVertices = 8;
const int32 b2_maxProxies = 512;				// this must be a power of two
const int32 b2_maxPairs = 8 * b2_maxProxies;	// this must be a power of two

// Dynamics
const float32 b2_linearSlop = 0.01f;
const float32 b2_angularSlop = 2.0f / 180.0f * b2_pi;
const float32 b2_velocityThreshold = 1.0f;
const float32 b2_maxLinearCorrection = 0.2f;
const float32 b2_maxAngularCorrection = 8.0f / 180.0f * b2_pi;
const float32 b2_contactBaumgarte = 0.2f;

// Sleep
const float32 b2_timeToSleep = 0.5f;
const float32 b2_linearSleepTolerance = 0.01f;
const float32 b2_angularSleepTolerance = 0.01f;

void b2Assert(bool condition);

#endif