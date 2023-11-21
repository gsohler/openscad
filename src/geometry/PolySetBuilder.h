#pragma once

#include <Reindexer.h>
#include "Polygon2d.h"
#include <memory>
#include "boost-utils.h"
class PolySet;

class PolySetBuilder
{
  PolySet *ps;
  Reindexer<Vector3d> allVertices;
public:
  PolySetBuilder(int vertices_count=0, int indices_count=0, int dim=3, boost::tribool convex=unknown);
  PolySetBuilder(const Polygon2d pol);
  void reset(void);
  void setConvexity(int n);
  int vertexIndex(const Vector3d &coord);
  int numVertices(void);
  void append_poly(int nvertices);
  void append(const PolySet *ps);
  void append_poly(const std::vector<int> &inds);
  void append_geometry(const shared_ptr<const Geometry>& geom);
  void append_poly(const std::vector<Vector3d> &v);
  void append_vertex(int n);
  void prepend_vertex(int n);
  PolySet *result(void);
};
