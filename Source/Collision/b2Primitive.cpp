/*
* Copyright (c) 2007 Erin Catto http://www.gphysics.com
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

#include "b2Primitive.h"

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.4.1
// x = mu1 * p1 + mu2 * p2
// mu1 + mu2 = 1 && mu1 >= 0 && mu2 >= 0
// mu1 = 1 - mu2;
// x = (1 - mu2) * p1 + mu2 * p2
//   = p1 + mu2 * (p2 - p1)
// x = s + a * r (s := start, r := end - start)
// s + a * r = p1 + mu2 * d (d := p2 - p1)
// -a * r + mu2 * d = b (b := s - p1)
// [-r d] * [a; mu2] = b
// Cramer's rule:
// denom = det[-r d]
// a = det[b d] / denom
// mu2 = det[-r b] / denom
bool b2SegmentPrimitive::TestSegment(float32* lambda, b2Vec2* normal, const b2SegmentPrimitive& segment, float32 maxLambda)
{
	b2Vec2 s = segment.m_p1;
	b2Vec2 r = segment.m_p2 - s;
	b2Vec2 d = m_p2 - m_p1;
	b2Vec2 n = b2Cross(d, 1.0f);

	const float32 k_slop = 100.0f * FLT_EPSILON;
	float32 denom = -b2Dot(r, n);

	// Cull back facing collision and ignore parallel segments.
	if (denom > k_slop)
	{
		// Does the segment intersect the infinite line associated with this segment?
		b2Vec2 b = s - m_p1;
		float32 a = b2Dot(b, n);

		if (0.0f <= a && a <= maxLambda * denom)
		{
			float32 mu2 = -r.x * b.y + r.y * b.x;

			// Does the segment intersect this segment?
			if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0f + k_slop))
			{
				a /= denom;
				n.Normalize();
				*lambda = a;
				*normal = n;
				return true;
			}
		}
	}

	return false;
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
bool b2CirclePrimitive::TestSegment(float32* lambda, b2Vec2* normal, const b2SegmentPrimitive& segment, float32 maxLambda)
{
	b2Vec2 s = segment.m_p1 - m_position;
	float32 b = b2Dot(s, s) - m_radius * m_radius;

	// Does the segment start inside the circle?
	if (b < 0.0f)
	{
		return false;
	}

	// Solve quadratic equation.
	b2Vec2 r = segment.m_p2 - segment.m_p1;
	float32 c =  b2Dot(s, r);
	float32 rr = b2Dot(r, r);
	float32 sigma = c * c - rr * b;

	// Check for negative discriminant and short segment.
	if (sigma < 0.0f || rr < FLT_EPSILON)
	{
		return false;
	}

	// Find the point of intersection of the line with the circle.
	float32 a = -(c + sqrtf(sigma)) / rr;

	// Is the intersection point on the segment?
	if (0.0f <= a && a <= maxLambda)
	{
		*lambda = a;
		*normal = s + a * r;
		normal->Normalize();
		return true;
	}

	return false;

}

static b2Vec2 PolyCentroid(const b2Vec2* vs, int32 count)
{
	b2Assert(count >= 3);

	b2Vec2 c; c.Set(0.0f, 0.0f);
	float32 area = 0.0f;

	// pRef is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	b2Vec2 pRef(0.0f, 0.0f);
	const float32 inv3 = 1.0f / 3.0f;

	for (int32 i = 0; i < count; ++i)
	{
		// Triangle vertices.
		b2Vec2 p1 = pRef;
		b2Vec2 p2 = vs[i];
		b2Vec2 p3 = i + 1 < count ? vs[i+1] : vs[0];

		b2Vec2 e1 = p2 - p1;
		b2Vec2 e2 = p3 - p1;

		float32 D = b2Cross(e1, e2);

		float32 triangleArea = 0.5f * D;
		area += triangleArea;

		// Area weighted centroid
		c += triangleArea * inv3 * (p1 + p2 + p3);
	}

	// Centroid
	b2Assert(area > FLT_EPSILON);
	c *= 1.0f / area;
	return c;
}

void b2PolygonPrimitive::Initialize(const b2Vec2& position, const b2Mat22& R, const b2Vec2* vertices, int32 vertexCount)
{
	m_R = R;
	m_position = position;

	b2Assert(3 <= vertexCount && vertexCount <= b2_maxPolygonVertices);
	m_vertexCount = vertexCount;
	for (int32 i = 0; i < m_vertexCount; ++i)
	{
		m_vertices[i] = vertices[i];
	}

	m_centroid = PolyCentroid(m_vertices, m_vertexCount);

	// Compute the edge normals and next index map.
	for (int32 i = 0; i < m_vertexCount; ++i)
	{
		int32 i1 = i;
		int32 i2 = i + 1 < m_vertexCount ? i + 1 : 0;
		b2Vec2 edge = m_vertices[i2] - m_vertices[i1];
		m_normals[i] = b2Cross(edge, 1.0f);
		m_normals[i].Normalize();
	}

	// Create core polygon shape by shifting edges inward.
	m_minRadius = FLT_MAX;
	m_maxRadius = -FLT_MAX;
	for (int32 i = 0; i < m_vertexCount; ++i)
	{
		int32 i1 = i - 1 >= 0 ? i - 1 : m_vertexCount - 1;
		int32 i2 = i;

		b2Vec2 n1 = m_normals[i1];
		b2Vec2 n2 = m_normals[i2];
		b2Vec2 v = m_vertices[i] - m_centroid;

		// dot(n1, vc) = d.x
		// dot(n2, vc) = d.y
		b2Vec2 d;
		d.x = b2Dot(n1, v) - b2_toiSlop;
		d.y = b2Dot(n2, v) - b2_toiSlop;

		// Shifting the edge inward by b2_toiSlop should
		// not cause the plane to pass the centroid.
		b2Assert(d.x >= 0.0f);
		b2Assert(d.y >= 0.0f);
		b2Mat22 A;
		A.col1.x = n1.x; A.col2.x = n1.y;
		A.col1.y = n2.x; A.col2.y = n2.y;
		m_coreVertices[i] = A.Solve(d) + m_centroid;

		m_minRadius = b2Min(m_minRadius, b2Min(d.x, d.y));
		m_maxRadius = b2Max(m_maxRadius, m_coreVertices[i].Length());
	}

#ifdef _DEBUG
	// Ensure the polygon in convex.
	for (int32 i = 0; i < m_vertexCount; ++i)
	{
		for (int32 j = 0; j < m_vertexCount; ++j)
		{
			// Don't check vertices on the current edge.
			if (j == i || j == (i + 1) % m_vertexCount)
			{
				continue;
			}

			float32 s = b2Dot(m_normals[i], m_vertices[j] - m_vertices[i]);
			b2Assert(s < -b2_linearSlop);
		}
	}
#endif
}

b2Vec2 b2PolygonPrimitive::Support(const b2Vec2& d) const
{
	b2Vec2 dLocal = b2MulT(m_R, d);

	int32 bestIndex = 0;
	float32 bestValue = b2Dot(m_coreVertices[0], dLocal);
	for (int32 i = 1; i < m_vertexCount; ++i)
	{
		float32 value = b2Dot(m_coreVertices[i], dLocal);
		if (value > bestValue)
		{
			bestIndex = i;
			bestValue = value;
		}
	}

	return m_position + b2Mul(m_R, m_coreVertices[bestIndex]);
}

bool b2PolygonPrimitive::TestPoint(const b2Vec2& p) const
{
	b2Vec2 pLocal = b2MulT(m_R, p - m_position);

	for (int32 i = 0; i < m_vertexCount; ++i)
	{
		float32 dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
		if (dot > 0.0f)
		{
			return false;
		}
	}

	return true;
}

bool b2PolygonPrimitive::TestSegment(float32* lambda, b2Vec2* normal, const b2SegmentPrimitive& segment, float32 maxLambda)
{
	float32 lower = 0.0f, upper = maxLambda;

	b2Vec2 p1 = b2MulT(m_R, segment.m_p1 - m_position);
	b2Vec2 p2 = b2MulT(m_R, segment.m_p2 - m_position);
	b2Vec2 d = p2 - p1;
	int32 index = -1;

	for (int32 i = 0; i < m_vertexCount; ++i)
	{
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		float32 numerator = b2Dot(m_normals[i], m_vertices[i] - p1);
		float32 denominator = b2Dot(m_normals[i], d);

		if (denominator < 0.0f && numerator > lower * denominator)
		{
			// The segment enters this half-space.
			lower = numerator / denominator;
			index = i;
		}
		else if (denominator > 0.0f && numerator < upper * denominator)
		{
			// The segment exits this half-space.
			upper = numerator / denominator;
		}

		if (upper < lower)
		{
			return false;
		}
	}

	b2Assert(0.0f <= lower && lower <= maxLambda);

	if (index >= 0)
	{
		*lambda = lower;
		*normal = b2Mul(m_R, m_normals[index]);
		return true;
	}

	return false;
}

void b2PolygonPrimitive::ComputeAABB(b2AABB* aabb) const
{
	b2Vec2 minVertex(FLT_MAX, FLT_MAX);
	b2Vec2 maxVertex(-FLT_MAX, -FLT_MAX);
	for (int32 i = 0; i < m_vertexCount; ++i)
	{
		b2Vec2 v = b2Mul(m_R, m_vertices[i]);
		minVertex = b2Min(minVertex, v);
		maxVertex = b2Max(maxVertex, v);
	}

	minVertex += m_position;
	maxVertex += m_position;
	aabb->minVertex = minVertex;
	aabb->maxVertex = maxVertex;
}
