// this file is split into many separate cgalutils* files
// in order to workaround gcc 4.9.1 crashing on systems with only 2GB of RAM

#ifdef ENABLE_MANIFOLD

#include "manifoldutils.h"
#include "ManifoldGeometry.h"
#include "node.h"
#include "progress.h"
#include "printutils.h"
#include "PolySet.h"
#include "PolySetBuilder.h"

namespace ManifoldUtils {

Location getLocation(const std::shared_ptr<const AbstractNode>& node)
{
  return node && node->modinst ? node->modinst->location() : Location::NONE;
}

/*!
   Applies op to all children and returns the result.
   The child list should be guaranteed to contain non-NULL 3D or empty Geometry objects
 */
std::shared_ptr<ManifoldGeometry> applyOperator3DManifold(const Geometry::Geometries& children, OpenSCADOperator op)
{
	PolySetBuilder builder;
      	std::shared_ptr<ManifoldGeometry> geom;

  bool foundFirst = false;
  std::vector<Material> matnew;
  std::vector<std::vector<unsigned int>> matinds_org;
  int cnt=0;
  std::vector<int> runOriginalMap;
  for (const auto& item : children) {
    cnt++;	  
    std::vector<unsigned int> matind_org;	  
    std::shared_ptr<const ManifoldGeometry> chN = item.second ? createManifoldFromGeometry(item.second) : nullptr;
    manifold::MeshGL meshgl = chN->getManifold().GetMeshGL();
    runOriginalMap.push_back(meshgl.runOriginalID[0]); // TODO 0???						      
    for(auto ind: chN->matind) {
      int found=-1;	    
      for(int j=0;j<matnew.size();j++) {
        if(matnew[j].color == chN->mat[ind].color) // TODO fix
          found=j;
      }
      if(found == -1) {
        found=matnew.size();
        matnew.push_back(chN->mat[ind]); 	 
      }
      matind_org.push_back(found);	   
    }
    matinds_org.push_back(matind_org);

    // Intersecting something with nothing results in nothing
    if (!chN || chN->isEmpty()) {
      if (op == OpenSCADOperator::INTERSECTION) {
        geom = nullptr;
        break;
      }
      if (op == OpenSCADOperator::DIFFERENCE && !foundFirst) {
        geom = nullptr;
        break;
      }
      continue;
    }

    // Initialize geom with first expected geometric object
    if (!foundFirst) {
      geom = std::make_shared<ManifoldGeometry>(*chN);
      foundFirst = true;
      continue;
    }

    switch (op) {
    case OpenSCADOperator::UNION:
      *geom = *geom + *chN;
      break;
    case OpenSCADOperator::INTERSECTION:
      *geom = *geom * *chN;
      break;
    case OpenSCADOperator::DIFFERENCE:
      *geom = *geom - *chN;
      break;
    case OpenSCADOperator::MINKOWSKI:
      *geom = geom->minkowski(*chN);
      break;
    default:
      LOG(message_group::Error, "Unsupported CGAL operator: %1$d", static_cast<int>(op));
    }
    if (item.first) item.first->progress_report();
  }
  geom->mat=matnew;

  std::vector<unsigned int> matind;

	const auto & mesh = geom->getManifold().GetMeshGL();
	assert(mesh.runIndex.size() >= 2);
	const auto meshNumVerts = mesh.vertProperties.size() / mesh.numProp;
	const auto meshNumTris = mesh.triVerts.size();


	auto id = mesh.runOriginalID[0];
	auto start = mesh.runIndex[0];

	for (int run = 0, numRun = mesh.runIndex.size() - 1; run < numRun; ++run) {
		const auto nextID = mesh.runOriginalID[run + 1];
		if (nextID != id) {
			int ind=-1;
			for(int i=0;i<runOriginalMap.size();i++) {
                          if(runOriginalMap[i] == id) ind=i;				
			}
			if(ind == -1) { printf("Program error!\n"); ind=0; }

			const auto end = mesh.runIndex[run + 1];
			const size_t numTri = (end - start) / 3;

			for (int i = start; i < end; i += 3) {
				builder.beginPolygon(3);
				for (int j = 0; j < 3; ++j) {
					auto iVert = mesh.triVerts[i + j];
					auto propOffset = iVert * mesh.numProp;
					builder.addVertex({
						mesh.vertProperties[propOffset],
						mesh.vertProperties[propOffset + 1],
						mesh.vertProperties[propOffset + 2]
					});
				}
				int orgfaceid=mesh.faceID[i/3];
				matind.push_back(matinds_org[ind][orgfaceid]);
				builder.endPolygon();
			}
			id = nextID;
			start = end;
		}
	}
	auto result = builder.build();
	result->mat = matnew;
	result->matind = matind;
	return createManifoldFromPolySet(*result);
}

};  // namespace ManifoldUtils

#endif // ENABLE_MANIFOLD
