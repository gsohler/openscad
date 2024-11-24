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
bool GeometryUtils::tessellatePolygonWithHoles(const std::vector<Vector3f>& vertices,
                                               const std::vector<IndexedFace>& faces,
                                               std::vector<IndexedTriangle>& triangles,
                                               const Vector3f *normal);
void import_shell(PolySetBuilder &builder, StepKernel &sk, StepKernel::Shell *shell)
{
  if(shell == nullptr) return; // Shell , CLOSED_SHELL
  for(int i=0;i<shell->faces.size();i++) {
    StepKernel::Face *face = shell->faces[i]; // Face, ADVANCED_FACE
    if(face == nullptr) continue;
    StepKernel::Plane *plane = dynamic_cast<StepKernel::Plane *>(face->surface);
    StepKernel::Axis2Placement *sys = nullptr;
    if(plane != nullptr) sys = plane->axis;

    std::vector<IndexedFace> faceLoopInd;
    std::vector<Vector3d> vertices;
    for(int j=0;j<face->faceBounds.size();j++) {
      StepKernel::FaceBound *bound = face->faceBounds[j]; //FaceBound, FACE_OUTER_BOUND
      std::vector<IndexedFace> stubs;
      if(bound == nullptr) continue;
      StepKernel::EdgeLoop *loop = bound->edgeLoop; 
      for(int k=0;k<loop->faces.size();k++) {
        StepKernel::OrientedEdge *edge=loop->faces[k];
        if(edge == nullptr) continue;
        StepKernel::EdgeCurve *edgecurv = edge->edge; 
//        printf("%d %d %d %d\n",face->dir,bound->dir,edge->dir,edgecurv->dir);
        StepKernel::Point *pt1 = edgecurv->vert1->point;
        StepKernel::Point *pt2 = edgecurv->vert2->point;
        IndexedFace stub;
	StepKernel::Circle *circ = dynamic_cast<StepKernel::Circle *>(edgecurv->round);
	StepKernel::SurfaceCurve *scurve = dynamic_cast<StepKernel::SurfaceCurve *>(edgecurv->round);
	StepKernel::Line *line = dynamic_cast<StepKernel::Line *>(edgecurv->round);

	if(circ != nullptr) {
	  int fn=20;
	  auto axis = circ->axis;
	  Vector3d xdir=axis->dir2->pt;
	  Vector3d zdir=axis->dir1->pt;
	  Vector3d ydir=zdir.cross(xdir).normalized();
	  // calc start angle
	  Vector3d res;
	  if(linsystem( xdir, ydir, zdir, pt1->pt - axis->point->pt,res)) continue;
	  double startang=atan2(res[1], res[0]);

	  if(linsystem( xdir, ydir, zdir, pt2->pt - axis->point->pt,res)) continue;
	  double endang=atan2(res[1], res[0]);
	  int closed = (startang == endang)?1:0;

	  if(endang <= startang) endang += 2*M_PI;
//	  printf("startang=%g endang=%g\n",startang, endang);
	  //
	  double r=circ->r;
	  Vector3d cent=circ->axis->point->pt;
          stub.push_back(builder.vertexIndex(pt1->pt));
	  for(int i=1;i<fn;i++) {
            double ang=startang + (endang-startang)*i/(double) fn;		 
	    Vector3d pt=cent+xdir*r*cos(ang)+ydir*r*sin(ang);
            stub.push_back(builder.vertexIndex(pt));
	  }
          stub.push_back(builder.vertexIndex(pt2->pt));
	}
	else if(scurve != nullptr) {
          stub.push_back(builder.vertexIndex(pt1->pt));
          stub.push_back(builder.vertexIndex(pt2->pt));
        } else if(line != nullptr) {
          stub.push_back(builder.vertexIndex(pt1->pt));
          stub.push_back(builder.vertexIndex(pt2->pt));
	} else {
          printf("Unimplemented csurfacecurve for id %d\n",edgecurv->id);
	}
        int totaldir = (edge->dir+edgecurv->dir)&1;
	if(totaldir) {
          std::reverse(stub.begin(), stub.end());
	}

        stubs.push_back(stub);

      }
      // now combine the stub
      if(stubs.size() == 0) continue;
      IndexedFace combined = stubs[0];
      stubs.erase(stubs.begin());
      int conn = combined[combined.size()-1];
      bool done=true;
      while(stubs.size() > 0 && done) {
       done=false;
       for(int i=0;i<stubs.size();i++)
       {
         if(stubs[i][0] == conn) {
           for(int j=1;j<stubs[i].size();j++) {
             combined.push_back(stubs[i][j]);		     
           }		   
           stubs.erase(stubs.begin()+i);
	   done=true;
           break; 
         }		   
       }
       conn = combined[combined.size()-1];
      }

      if(stubs.size() != 0) {
      	printf("Warning: Cannot detect full loop for facebound %d\n",bound->id);
      }
      combined.erase(combined.begin());
      builder.copyVertices(vertices);
      Vector4d tn=calcTriangleNormal(vertices,combined);
      int totaldir = (bound->dir)&1;
      if(!totaldir)
      {
        std::reverse(combined.begin(), combined.end());
      }

      faceLoopInd.push_back(combined);
    }
    int n=face->faceBounds.size();

    StepKernel::CylindricalSurface *cyl_surf = dynamic_cast<StepKernel::CylindricalSurface *>(face->surface);
    // TODO help cylindrical surfaces to tessellate correctly, maybe by using cylindrical surface ?
    //if(cyl_surf != nullptr) continue;
    if(face->id == 211) continue; 

    if(n == 1) {
      builder.beginPolygon(faceLoopInd[0].size());
      for(int i=0;i<faceLoopInd[0].size();i++) builder.addVertex(faceLoopInd[0][i]);			
    } else {
      std::vector<IndexedTriangle> triangles;
      if(sys == nullptr) continue;
      Vector3d dn =sys->dir1->pt;
      Vector3f df(dn[0], dn[1], dn[2]);
      std::vector<Vector3f> verticesf;
      for(auto v : vertices)
	verticesf.push_back(Vector3f(v[0], v[1], v[2]));	      // TODO better

      GeometryUtils::tessellatePolygonWithHoles(verticesf,faceLoopInd, triangles,&df);
      for(auto tri: triangles) {
        builder.beginPolygon(3);
        for(int i=0;i<3;i++) builder.addVertex(tri[i]);			
      }
    }  
  }
}
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
