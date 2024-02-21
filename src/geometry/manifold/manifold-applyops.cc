// this file is split into many separate cgalutils* files
// in order to workaround gcc 4.9.1 crashing on systems with only 2GB of RAM

#ifdef ENABLE_MANIFOLD

#include "manifoldutils.h"
#include "ManifoldGeometry.h"
#include "node.h"
#include "progress.h"
#include "printutils.h"

namespace ManifoldUtils {

Location getLocation(const std::shared_ptr<const AbstractNode>& node)
{
  return node && node->modinst ? node->modinst->location() : Location::NONE;
}

/*!
   Applies op to all children and returns the result.
   The child list should be guaranteed to contain non-NULL 3D or empty Geometry objects
 */
std::shared_ptr<const ManifoldGeometry> applyOperator3DManifold(const Geometry::Geometries& children, OpenSCADOperator op)
{
  auto N = std::make_shared<ManifoldGeometry>();

  bool foundFirst = false;

  std::vector<Material> matnew;
  std::vector<std::vector<unsigned int>> matinds_org;
  for (const auto& item : children) {
    std::vector<unsigned int> matind_org;	  
    std::shared_ptr<ManifoldGeometry> chN = item.second ? createMutableManifoldFromGeometry(matnew, matind_org, item.second) : nullptr;
    matinds_org.push_back(matind_org);

    // Intersecting something with nothing results in nothing
    if (!chN || chN->isEmpty()) {
      if (op == OpenSCADOperator::INTERSECTION) {
        N = nullptr;
        break;
      }
      if (op == OpenSCADOperator::DIFFERENCE && !foundFirst) {
        N = nullptr;
        break;
      }
      continue;
    }
    // Initialize N with first expected geometric object
    if (!foundFirst) {
      N = chN;
      foundFirst = true;
      continue;
    }

    switch (op) {
    case OpenSCADOperator::UNION:
      *N += *chN;
      break;
    case OpenSCADOperator::INTERSECTION:
      *N *= *chN;
      break;
    case OpenSCADOperator::DIFFERENCE:
      *N -= *chN;
      break;
    case OpenSCADOperator::MINKOWSKI:
      N->minkowski(*chN);
      break;
    default:
      LOG(message_group::Error, "Unsupported CGAL operator: %1$d", static_cast<int>(op));
    }
    if (item.first) item.first->progress_report();
  }
  N->mat=matnew;
  N->matinds_org = matinds_org;

  return N;
}

};  // namespace ManifoldUtils

#endif // ENABLE_MANIFOLD
