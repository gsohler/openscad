#include "io/import.h"
#include "geometry/PolySet.h"
#include "geometry/PolySetBuilder.h"

// https://stepcode.github.io/docs/p21_cpp_example/
//#include <sc_cf.h>
extern void SchemaInit( class Registry & );
#include "StepKernel.h"

// https://github.com/slugdev/stltostp/blob/master/StepKernel.cpp
//
Vector4d calcTriangleNormal(const std::vector<Vector3d> &vertices,const IndexedFace &pol);
std::unique_ptr<PolySet> import_step(const std::string& filename, const Location& loc) {
  PolySetBuilder builder;
  printf("Importing %s\n",filename.c_str());
  StepKernel sk;
  sk.read_step(filename);
  for(auto  *e : sk.entities) {
    const StepKernel::ManifoldSolid *mani = dynamic_cast<StepKernel::ManifoldSolid *>(e);
    if (mani == nullptr) continue;
    StepKernel::Shell *shell = mani->shell;
    if(shell == nullptr) continue;
    for(int i=0;i<shell->faces.size();i++) {
      StepKernel::Face *face = shell->faces[i];
      if(face == nullptr) continue;
      StepKernel::Plane *plane = face->plane;
      if(plane == nullptr) continue;
      StepKernel::Csys3D *sys = plane->csys;
      if(sys == nullptr) continue;
      StepKernel::Direction *dir1=sys->dir1;
      StepKernel::Direction *dir2=sys->dir2;
      if(dir1 == nullptr || dir2 == nullptr) continue;
      Vector3d d1(dir1->x, dir1->y, dir2->z);
      Vector3d d2(dir2->x, dir2->y, dir2->z);
      Vector3d dn=d1.cross(d2).normalized();

      for(int j=0;j<face->faceBounds.size();j++) {
        StepKernel::FaceBound *bound = face->faceBounds[j];
	std::vector<IndexedFace> stubs;
	if(bound == nullptr) continue;
	StepKernel::EdgeLoop *loop = bound->edgeLoop;
	for(int k=0;k<loop->faces.size();k++) {
	  StepKernel::OrientedEdge *edge=loop->faces[k];
	  if(edge == nullptr) continue;
	  StepKernel::EdgeCurve *edgecurv = edge->edge;
	  StepKernel::Point *pt1 = edgecurv->vert1->point;
	  StepKernel::Point *pt2 = edgecurv->vert2->point;
	  IndexedFace stub;
	  Vector3d p1(pt1->x, pt1->y, pt1->z);
	  stub.push_back(builder.vertexIndex(p1));
	  Vector3d p2(pt2->x, pt2->y, pt2->z);
	  stub.push_back(builder.vertexIndex(p2));
	  stubs.push_back(stub);

	}
	// now combine the stub
	if(stubs.size() == 0) continue;
	IndexedFace combined = stubs[0];
	stubs.erase(stubs.begin());
	while(stubs.size() > 0) {
         int done=0;		
         int conn = combined[combined.size()-1];
	 if(conn == combined[0]) { printf("complete break\n"); break; }

	 for(int i=0;i<stubs.size();i++)
	 {
	   if(stubs[i][0] == conn) {
             for(int j=1;j<stubs[i].size();j++) {
               combined.push_back(stubs[i][j]);		     
	     }		   
	     stubs.erase(stubs.begin()+i);
	     done=1;
	     break; 
	   }		   
	   if(stubs[i][stubs[i].size()-1] == conn) {
             for(int j=stubs[i].size()-2;j>=0;j--) {
               combined.push_back(stubs[i][j]);		     
	     }		   
	     stubs.erase(stubs.begin()+i);
	     done=1;
	     break; 
	   }		   
	 }
	}
	combined.erase(combined.begin());

	std::vector<Vector3d> vertices;
	builder.copyVertices(vertices);
	Vector4d tn=calcTriangleNormal(vertices,combined);
	if(tn.head<3>().dot(dn) < 0)
	  std::reverse(combined.begin(), combined.end());

	builder.beginPolygon(combined.size());
	// TODO thrown
	// TODO export, TODO Vector3d
        for(int i=0;i<combined.size();i++) builder.addVertex(combined[i]);			
       	
      }
    }
  }  
  // https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/Part/App/ImportStep.cpp
/*

*/
  return builder.build();
} 
