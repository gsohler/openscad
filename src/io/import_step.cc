#include "io/import.h"
#include "geometry/PolySet.h"
#include "geometry/PolySetBuilder.h"

// https://stepcode.github.io/docs/p21_cpp_example/
//#include <sc_cf.h>
extern void SchemaInit( class Registry & );
#include "StepKernel.h"

// https://github.com/slugdev/stltostp/blob/master/StepKernel.cpp
//
typedef std::vector<int> intList;
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
      for(int j=0;j<face->faceBounds.size();j++) {
        StepKernel::FaceBound *bound = face->faceBounds[j];
	std::vector<intList> stubs;
	if(bound == nullptr) continue;
	StepKernel::EdgeLoop *loop = bound->edgeLoop;
	for(int k=0;k<loop->faces.size();k++) {
	  StepKernel::OrientedEdge *edge=loop->faces[k];
	  if(edge == nullptr) continue;
	  StepKernel::EdgeCurve *edgecurv = edge->edge;
	  StepKernel::Point *pt1 = edgecurv->vert1->point;
	  StepKernel::Point *pt2 = edgecurv->vert2->point;
	  intList stub;
	  Vector3d p1(pt1->x, pt1->y, pt1->z);
	  stub.push_back(builder.vertexIndex(p1));
	  Vector3d p2(pt2->x, pt2->y, pt2->z);
	  stub.push_back(builder.vertexIndex(p2));
	  stubs.push_back(stub);

	}
	// now combine the stub
	if(stubs.size() == 0) continue;
	intList combined = stubs[0];
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
	builder.beginPolygon(combined.size());
	// TODO thrown, debugging weg
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
