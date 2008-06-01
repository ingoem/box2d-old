/* Header file for TriangleMesh.cpp
 */
#ifndef __TRIANGLE_MESH_H
#define __TRIANGLE_MESH_H

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <float.h>
#include <math.h>
#include <assert.h>

#ifndef TRIANGLEMESH_STANDALONE
#include "box2d.h"
#else
   typedef float      float32;
   typedef signed int int32;
#endif

#define tmAssert(condition) assert((condition)) 

// errors and warnings - TODO
#define tmE_OK    0
#define tmE_MEM   1
#define tmE_HOLES 2

// constants 
#define tmC_PI         3.14159265359f
#define tmC_PIx2       6.28318530718f
#define tmC_PI_3       1.04719755119f
#define tmC_SQRT2      1.41421356237f
#define tmC_BIGNUMBER       1.0e10f
#define tmC_MAXVERTEXCOUNT  500

// options
#define tmO_SEGMENTBOUNDARY  2
#define tmO_CONVEXHULL       4
#define tmO_MINIMALGRID      8
#define tmO_BASICMESH       16
// bit to mark if maxVertexCount was enough
#define tmO_ENOUGHVERTICES 128

typedef struct
{
  float32 x,y;
} tmVertex;

typedef struct
{
  tmVertex *v[2];
} tmSegment;

typedef struct
{
  int32 i1,i2;
} tmSegmentId;

typedef struct
{
 tmVertex        *v[2];
 struct Triangle *t[2];
 bool   locked;
} tmEdge ;

typedef struct Triangle
{
 tmVertex *v[3];
 tmEdge   *e[3];
 float32 minAngle, angle;
 bool    inside;
 // hold attributes for the triangles, internally not used
 void    *userData;
} tmTriangle;

class TriangleMesh
{
  public:

   TriangleMesh(int32 aMaxVertexCount=tmC_MAXVERTEXCOUNT,
                float32 aMinAngle=30.0f,
                int32 aOptions=tmO_MINIMALGRID|tmO_CONVEXHULL);

   int32  Mesh(tmVertex *input, int32 n_input,
               tmSegmentId *segment, int32 n_segment,
               tmVertex *hole, int32 n_holes);

   void AutoSegment(int32 startNode, int32 endNode, bool doclose);

   void SetOptions(int32 aOptions)     { options = aOptions;  }

   void AddOption(int32 aOptions)      { options |= aOptions; }

   void SetMaxVertexCount(int32 count) {
       if ( count>3 )
       {
           maxVertexCount = count;
           options &= ~tmO_MINIMALGRID;
       }
   }
   // angle in degree !
   void SetAngle(float32 angle)     { minAngle = angle; }

   int32  GetVertexCount()          { return vertexCount;        }

   int32  GetInputVertexCount()     { return inputVertexCount;   }

   int32  GetEdgeCount()            { return edgeCount;          }

   int32  GetTriangleCount()        { return triangleCount;      }

   int32  GetInsideTriangleCount()  { return insideTriangleCount;}

   tmVertex*     GetVertices()  { return Vertices;  }

   tmEdge*       GetEdges()     { return Edges;     }

   tmTriangle*   GetTriangles() { return Triangles;  }

   void PrintData(FILE* f = stdout);

   void FreeMemory();

  private:

   /* Data */
   tmVertex   *Vertices;
   tmEdge     *Edges;
   tmTriangle* Triangles;
   tmSegment  *Segments;
   tmVertex   *Holes;

   int32       maxVertexCount, maxEdgeCount,
               maxTriangleCount,maxSegmentCount;
   int32       vertexCount, inputVertexCount;
   int32       edgeCount, triangleCount, segmentCount, holeCount;
   int32       insideTriangleCount;

   float32     minAngle ;
   int32       options;

   tmTriangle* lastTriangle;

   /* Functions */

   void        Triangulate();
   int32       MarkInsideTriangles(bool holes);
   void        InsertSegments();
   void        DeleteBadTriangles();
   void        DeleteTriangle(tmTriangle* t);

   tmVertex*   AddVertex();
   tmVertex*   GetClosestVertex(float32 x, float32 y);
   tmTriangle* FindVertex(tmVertex* v);
   bool        ContainsVertex(tmVertex* v0, tmVertex* v1, tmVertex* v);
   float32     GetVertexPosition(tmVertex* a, tmVertex* b, tmVertex* c);
   void        InsertVertexAt(tmVertex* v, tmEdge* e);
   void        InsertVertex(tmVertex* v);
   tmVertex*   GetOppositeVertex(tmEdge* e, tmTriangle* t);

   tmEdge*     AddEdge();
   void        SetEdge(tmEdge* e, tmVertex* v0, tmVertex* v1, tmTriangle* t0, tmTriangle* t1);
   void        FixEdge(tmEdge* e, tmTriangle* t0, tmTriangle* t1);
   tmEdge*     GetEdge(tmVertex* v0, tmVertex* v1);
   bool        CheckEdge(tmEdge* e);
   tmSegment*  AddSegment();
   tmSegment*  GetSegment(tmVertex* v0, tmVertex* v1);

   tmTriangle* AddTriangle();
   void        SetTriangle(tmTriangle* t,
                           tmVertex* v0, tmVertex* v1, tmVertex* v2,
                           tmEdge* e0, tmEdge* e1, tmEdge* e2);
   void        SetTriangleData(tmVertex* v0,tmVertex* v1,tmVertex* v2,
                               float32 *minAngle, float32 *angle);

   void        GetAdjacentEdges(tmEdge* e, tmTriangle* t,
                                tmEdge** e0, tmEdge** e1, tmVertex** v);
   bool        IsOppositeVertex(tmVertex* v0, tmVertex* v1, tmVertex* v2);
   bool        HasBoundingVertices(tmVertex* v0,tmVertex* v1,tmVertex* v2);
   void        CircumCenter(tmVertex* c, tmTriangle* t);
   void        GetSplitPosition(tmVertex* v, tmVertex* v0, tmVertex* v1);
   void        SplitSegment(tmSegment* s);
   void        ConvexHull();

   void        Reset();
   void        CheckNumber(float32 x);
   float32     ArcTan2(float32 x, float32 y);
   float32     GetAngle(float32 a1, float32 a0);
};

#endif
