// this file is split into many separate cgalutils* files
// in order to workaround gcc 4.9.1 crashing on systems with only 2GB of RAM

#ifdef ENABLE_MANIFOLD

#include "manifoldutils.h"
#include "ManifoldGeometry.h"
#include "node.h"
#include "progress.h"
#include "printutils.h"
#include "PolySet.h"

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
    cnt++;	  
    std::vector<unsigned int> matind_org;	  
    std::shared_ptr<const ManifoldGeometry> chN = item.second ? createManifoldFromGeometry(item.second) : nullptr;
    printf("op %d matsize=%d matindsize=%d\n",cnt, chN->mat.size(), chN->matind.size());
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
  manifold::MeshGL meshgl = geom->getManifold().GetMeshGL();

  printf("runIndex is ");
  for(int i=0;i<meshgl.runIndex.size();i++) printf("%d ",meshgl.runIndex[i]);
  printf("\n");
  int cntx=0;
  for(int i=1;i<meshgl.runIndex.size();i++) {
	  while(cntx < meshgl.runIndex[i]/3)
		  printf("%d ",meshgl.faceID[cntx++]);
	  printf("\n");
  }

  printf("runOriginalID is ");
  for(int i=0;i<meshgl.runOriginalID.size();i++) printf("%d ",meshgl.runOriginalID[i]);
  printf("\n");

  const std::shared_ptr<const PolySet> psxs = geom->toPolySet(true);


  std::vector<int> runMapping ; // insane mapping from runIndex to matinds
  for(int i=0;i<meshgl.runIndex.size()-1;i++) {
    int findLen=(meshgl.runIndex[i+1]-meshgl.runIndex[i])/3;
    int found=-1;
    printf("findlen %d\n",findLen);
    for(int j=0;found == -1 &&j<matinds_org.size();j++) {
	    printf("Compare against %d\n",matinds_org[j].size());
      if(matinds_org[j].size() == findLen){
        bool valid=true;	      
        for(int k=0;valid && k<runMapping.size();k++)
          if(runMapping[k] == j) valid=false;		
        if(valid) found=j;	 
      }
    }   
    if(found == -1) {
	    printf("No mapping found\n");
    }
    runMapping.push_back(found);
  }	  

  printf("mapping %d %d\n",runMapping.size(), matinds_org.size());
  for(int i=0;i<runMapping.size();i++)
	  printf("m %d %d\n",i,runMapping[i]);
  for(int i=0;i<meshgl.runIndex.size()-1;i++) {
    int oldind = meshgl.runIndex[i];	  
    int newind = meshgl.runIndex[i+1];	  
    printf("Step %d process from %d to %d, data len is %d\n", i, oldind, newind, matinds_org[i].size());
    if(newind-oldind != 3*matinds_org[i].size()){
	    printf("wrong\n");
	    exit(1);
    }
    printf("faceID ");
    for(int j=oldind;j<newind;j+=3) {
      int ind=meshgl.faceID[j/3];    
      printf("%d ",ind);
      matind.push_back(matinds_org[runMapping[i]][ind]); 
    }
    printf("\n");
  }

  geom->mat = matnew;
  geom->matind = matind;


//  auto mani = std::make_shared<manifold::Manifold>(std::move(meshgl));
//  geom  = std::make_shared<ManifoldGeometry>(mani);

  const std::shared_ptr<const PolySet> ps = geom->toPolySet(true);

  geom = createManifoldFromPolySet(*ps);
  printf("finally\n");
  meshgl = geom->getManifold().GetMeshGL();
   printf("mat:\n");
   for(int i=0;i<geom->mat.size();i++)
	   printf("%d %g/%g/%g\n",i, geom->mat[i].color[0],geom->mat[i].color[1],geom->mat[i].color[2]); 
   printf("tri:\n");
   for(int i=0;i<meshgl.triVerts.size();i+=3)
	   printf("%d %d/%d/%d - %d\n",i/3,
	static_cast<int>(meshgl.triVerts[i]),
        static_cast<int>(meshgl.triVerts[i + 1]),
        static_cast<int>(meshgl.triVerts[i + 2]),
	geom->matind[meshgl.faceID[i/3]]);
   printf("Vert:\n");
   for (int i = 0; i < meshgl.vertProperties.size(); i += meshgl.numProp)
      printf("%d %g/%g/%g\n",i/3,	   
        meshgl.vertProperties[i],
        meshgl.vertProperties[i + 1],
        meshgl.vertProperties[i + 2]);

  printf("end csg\n");

  return geom;
}

};  // namespace ManifoldUtils

#endif // ENABLE_MANIFOLD
