#include "io/import.h"
#include "geometry/PolySet.h"
#include "geometry/PolySetBuilder.h"

// https://stepcode.github.io/docs/p21_cpp_example/
//#include <sc_cf.h>
extern void SchemaInit( class Registry & );
#include "StepKernel.h"

// https://github.com/slugdev/stltostp/blob/master/StepKernel.cpp
std::unique_ptr<PolySet> import_step(const std::string& filename, const Location& loc) {
  PolySetBuilder builder;
  printf("Importing %s\n",filename.c_str());
  StepKernel sk;
  sk.read_step(filename);
  for(auto  *e : sk.entities) {
    const StepKernel::ManifoldSolid *mani = dynamic_cast<StepKernel::ManifoldSolid *>(e);
    if (mani == nullptr) continue;
    printf("Found Manifold\n");
    StepKernel::Shell *shell = mani->shell;
    printf("shell node is %p\n",shell);
    if(shell == nullptr) continue;
    printf("t %d\n",shell->faces.size());		   
    for(int i=0;i<shell->faces.size();i++) {
      printf("Face %d\n",i);
      StepKernel::Face *face = shell->faces[i];
      if(face == nullptr) continue;
      printf("%d bounds\n",face->faceBounds.size());
      for(int j=0;j<face->faceBounds.size();j++) {
        printf("Bound %d\n",j);	     
        StepKernel::FaceBound *bound = face->faceBounds[j];
	if(bound == nullptr) continue;
	StepKernel::EdgeLoop *loop = bound->edgeLoop;
	for(int k=0;k<loop->faces.size();k++) {
	  StepKernel::OrientedEdge *edge=loop->faces[k];
	  if(edge == nullptr) continue;
	  StepKernel::EdgeCurve *edgecurv = edge->edge;
	  StepKernel::Point *pt1 = edgecurv->vert1->point;
	  StepKernel::Point *pt2 = edgecurv->vert2->point;
	  printf("(%g/%g/%g) - (%g/%g/%g)\n",pt1->x, pt1->y, pt1->z, pt2->x, pt2->y, pt2->z);
	}
       	
      }
    }
  }  
  // https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/Part/App/ImportStep.cpp
/*

*/
  return builder.build();
} 
