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
int linsystem( Vector3d v1,Vector3d v2,Vector3d v3,Vector3d pt,Vector3d &res,double *detptr=NULL);
void import_shell(PolySetBuilder &builder, StepKernel &sk, StepKernel::Shell *shell)
{
  if(shell == nullptr) return; // Shell , CLOSED_SHELL
  for(int i=0;i<shell->faces.size();i++) {
    StepKernel::Face *face = shell->faces[i]; // Face, ADVANCED_FACE
    if(face == nullptr) continue;
    StepKernel::Plane *plane = face->plane;
    StepKernel::Csys3D *sys = nullptr;
    if(plane != nullptr) sys = plane->csys;

    for(int j=0;j<face->faceBounds.size();j++) {
      StepKernel::FaceBound *bound = face->faceBounds[j]; //FaceBound, FACE_OUTER_BOUND
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
	if(edgecurv->circle != nullptr) {
	  int fn=20;
	  auto csys = edgecurv->circle->csys;
	  Vector3d xdir=csys->dir2->pt;
	  Vector3d zdir=csys->dir1->pt;
	  Vector3d ydir=xdir.cross(zdir).normalized();
	  // calc start angle
	  Vector3d res;
	  if(linsystem( xdir, ydir, zdir, pt1->pt - csys->point->pt,res)) continue;
	  double startang=atan2(res[1], res[0]);

	  if(linsystem( xdir, ydir, zdir, pt2->pt - csys->point->pt,res)) continue;
	  double endang=atan2(res[1], res[0]);

	  if(endang < startang) endang += 2*M_PI;
	  //
	  double r=edgecurv->circle->r;
	  Vector3d cent=edgecurv->circle->csys->point->pt;
          stub.push_back(builder.vertexIndex(pt1->pt));
	  for(int i=1;i<fn-1;i++) {
            double ang=startang + (endang-startang)*i/(double) fn;		 
	    Vector3d pt=cent+xdir*r*cos(ang)+ydir*r*sin(ang);
            stub.push_back(builder.vertexIndex(pt));
	  }
          stub.push_back(builder.vertexIndex(pt2->pt));
	}
	else {
          stub.push_back(builder.vertexIndex(pt1->pt));
          stub.push_back(builder.vertexIndex(pt2->pt));
	}  
        stubs.push_back(stub);

      }
      // now combine the stub
      std::vector<Vector3d> vertices;
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

      builder.copyVertices(vertices);
      Vector4d tn=calcTriangleNormal(vertices,combined);
      if(sys != nullptr) {
        Vector3d dn =sys->dir1->pt;
        if(!face->dir) dn=-dn;
        if(tn.head<3>().dot(dn) < 0)
          std::reverse(combined.begin(), combined.end());
      }	

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
