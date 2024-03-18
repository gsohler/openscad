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
std::shared_ptr<ManifoldGeometry> applyOperator3DManifold(const Geometry::Geometries& children, OpenSCADOperator op)
{
  std::shared_ptr<ManifoldGeometry> geom;

  bool foundFirst = false;
  printf("start csg %d\n",children.size());
  std::vector<Material> matnew;
  std::vector<std::vector<unsigned int>> matinds_org;
  int cnt=0;
  for (const auto& item : children) {
    std::vector<unsigned int> matind_org;	  
    std::shared_ptr<const ManifoldGeometry> chN = item.second ? createManifoldFromGeometry(item.second) : nullptr;
    printf("op mat=%d matind=%d\n",chN->mat.size(), chN->matind.size());
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
      printf("%d ",found);
      matind_org.push_back(found);	   
    }
    printf("\n");
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
    int sum=0;
    for(int add : chN->runWeights)
      sum += add;
    if (!foundFirst) {
      geom = std::make_shared<ManifoldGeometry>(*chN);
      geom->runWeights.clear();
      geom->runWeights.push_back(sum);
      foundFirst = true;
      continue;
    }
    geom->runWeights.push_back(sum);

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
  geom->matind.clear();
  int oldind=0;
  int i=0;
  manifold::MeshGL mesh = geom->getManifold().GetMeshGL();
  printf("ind is ");
  for(int i=0;i<mesh.runIndex.size();i++) printf("%d ",mesh.runIndex[i]);
  printf("\n");
  for(int n=0;n<geom->runWeights.size();n++) {
    int step=geom->runWeights[i];	  
    int newind = mesh.runIndex[i+step];	  
    printf("Step %d process from %d to %d, data len is %d\n", n, oldind, newind, matinds_org[n].size());
    for(int j=oldind;j<newind;j+=3) {
      int ind=mesh.faceID[j/3];    
      printf("%d ",ind);
      geom->matind.push_back(matinds_org[n][ind]); 
    }
    printf("\n");
    oldind=newind;
    i+= step;
  }
  printf("matind %d mat %d\n", geom->matind.size(),geom->mat.size());
  printf("end csg\n");

  return geom;
}

};  // namespace ManifoldUtils

#endif // ENABLE_MANIFOLD
