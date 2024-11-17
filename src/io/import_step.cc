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
void import_shell(PolySetBuilder &builder, StepKernel &sk, StepKernel::Shell *shell)
{
  if(shell == nullptr) return;
  for(int i=0;i<shell->faces.size();i++) {
    StepKernel::Face *face = shell->faces[i];
    if(face == nullptr) return;
    StepKernel::Plane *plane = face->plane;
    if(plane == nullptr) return;
    StepKernel::Csys3D *sys = plane->csys;
    if(sys == nullptr) return;
    StepKernel::Direction *dir1=sys->dir1;
    StepKernel::Direction *dir2=sys->dir2;
    if(dir1 == nullptr || dir2 == nullptr) return;
    Vector3d d1=dir1->pt;
    Vector3d d2=dir2->pt;
    Vector3d dn=d1.cross(d2).normalized();

    for(int j=0;j<face->faceBounds.size();j++) {
      StepKernel::FaceBound *bound = face->faceBounds[j];
      std::vector<IndexedFace> stubs;
      if(bound == nullptr) return;
      StepKernel::EdgeLoop *loop = bound->edgeLoop;
      for(int k=0;k<loop->faces.size();k++) {
        StepKernel::OrientedEdge *edge=loop->faces[k];
        if(edge == nullptr) return;
        StepKernel::EdgeCurve *edgecurv = edge->edge; 
        StepKernel::Point *pt1 = edgecurv->vert1->point;
        StepKernel::Point *pt2 = edgecurv->vert2->point;
        IndexedFace stub;
	if(edge->dir) {
          stub.push_back(builder.vertexIndex(pt1->pt));
          stub.push_back(builder.vertexIndex(pt2->pt));
	} else {
          stub.push_back(builder.vertexIndex(pt2->pt));
          stub.push_back(builder.vertexIndex(pt1->pt));
	}
        stubs.push_back(stub);

      }
      // now combine the stub
      if(stubs.size() == 0) return;
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
      for(int i=0;i<combined.size();i++) builder.addVertex(combined[i]);			
     	
    }
  }
};
std::unique_ptr<PolySet> import_step(const std::string& filename, const Location& loc) {
  printf("Importing %s\n",filename.c_str());
  PolySetBuilder builder;
  StepKernel sk;
  sk.read_step(filename);

 
/*
 * ManifoldShape
 * ShellModel
 * Shell
 */
  for(auto  *e : sk.entities) {
    const StepKernel::ManifoldSolid *mani_solid = dynamic_cast<StepKernel::ManifoldSolid *>(e);
    if(mani_solid != nullptr) {
      StepKernel::Shell *shell = mani_solid->shell;
      import_shell(builder, sk, shell);
      continue;
    }  
    const StepKernel::ManifoldShape *mani_shape = dynamic_cast<StepKernel::ManifoldShape *>(e);
    if(mani_shape != nullptr) {
      StepKernel::ShellModel *shell_model = mani_shape->shellModel;
      if(shell_model == nullptr) continue;
      for(auto &shell: shell_model->shells) {
         if(shell == nullptr) continue;
         import_shell(builder, sk, shell);
      }	 
      continue;
    }  
  }
  // https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/Part/App/ImportStep.cpp
/*

*/
  return builder.build();
} 
