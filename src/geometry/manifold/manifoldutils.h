#pragma once

#include "Geometry.h"
#include "enums.h"
#include "ManifoldGeometry.h"
#include "manifold.h"
#include "Material.h"

class PolySet;

namespace manifold {
  class Manifold;
  struct Mesh;
};

namespace ManifoldUtils {

  const char* statusToString(manifold::Manifold::Error status);

  /*! If the PolySet isn't trusted, use createManifoldFromPolySet which will triangulate and reorient it. */
  std::shared_ptr<manifold::Manifold> trustedPolySetToManifold(const PolySet& ps);

  std::shared_ptr<const ManifoldGeometry> createManifoldFromPolySet(std::vector<Material> &mat, std::vector<unsigned int> &matind_org, const PolySet& ps);
  std::shared_ptr<const ManifoldGeometry> createManifoldFromGeometry(std::vector<Material> &mat, std::vector<unsigned int> &matind_org, const std::shared_ptr<const Geometry>& geom);

  template <class TriangleMesh>
  std::shared_ptr<const ManifoldGeometry> createManifoldFromSurfaceMesh(const TriangleMesh& mesh);

  std::shared_ptr<const ManifoldGeometry> applyOperator3DManifold(const Geometry::Geometries& children, OpenSCADOperator op);

#ifdef ENABLE_CGAL
  std::shared_ptr<const Geometry> applyMinkowskiManifold(const Geometry::Geometries& children);
#endif
};
