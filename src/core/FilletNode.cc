/*
 *  OpenSCAD (www.openscad.org)
 *  Copyright (C) 2009-2011 Clifford Wolf <clifford@clifford.at> and
 *                          Marius Kintel <marius@kintel.net>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  As a special exception, you have permission to link this program
 *  with the CGAL library and distribute executables, as long as you
 *  follow the requirements of the GNU GPL in regard to all of the
 *  software in the executable aside from CGAL.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "FilletNode.h"
#include "module.h"
#include "ModuleInstantiation.h"
#include "Children.h"
#include "Parameters.h"
#include "printutils.h"
#include "io/fileutils.h"
#include "Builtins.h"
#include "handle_dep.h"
#include "PolySetBuilder.h"

#include <cmath>
#include <sstream>

#include <PolySetUtils.h>
#include <Tree.h>
#include <GeometryEvaluator.h>
#include <boost/functional/hash.hpp>
#include <hash.h>
#include <PolySetUtils.h>

class EdgeKey
{
  public:
  int ind1, ind2 ;
  int operator==(const EdgeKey ref)
  {
    if(this->ind1 == ref.ind1 && this->ind2 == ref.ind2) return 1;
    return 0;
  }
};

struct EdgeVal {
  int sel;
  int face1, face2;  
  int pos1, pos2;
  std::vector<int> bez1;
  std::vector<int> bez2;
};

struct SearchReplace {
  int pol;
  int search;
  std::vector<int> replace;
};

unsigned int hash_value(const EdgeKey& r) {
        unsigned int i;
        i=r.ind1 |(r.ind2<<16) ;
        return i;
}

int operator==(const EdgeKey &t1, const EdgeKey &t2) 
{
        if(t1.ind1 == t2.ind1 && t1.ind2 == t2.ind2) return 1;
        return 0;
}

Vector4d offset3D_normal(const std::vector<Vector3d> &vertices,const IndexedFace &pol);
std::vector<Vector4d> offset3D_normals(const std::vector<Vector3d> &vertices, const std::vector<IndexedFace> &indices);


typedef std::vector<int> intList;

bool list_included(const std::vector<int> &list,int item) {
  if(std::find(list.begin(), list.end(),item) != 	list.end()) return true;
  return false;
}
std::shared_ptr<const PolySet> childToPolySet( std::shared_ptr<AbstractNode> child)
{
  Tree tree(child, "");
  GeometryEvaluator geomevaluator(tree);
  std::shared_ptr<const Geometry> geom = geomevaluator.evaluateGeometry(*tree.root(), true);
  std::shared_ptr<const PolySet> ps;
  return PolySetUtils::getGeometryAsPolySet(geom);
}

int linsystem( Vector3d v1,Vector3d v2,Vector3d v3,Vector3d pt,Vector3d &res,double *detptr);


int point_in_polyhedron(const PolySet & ps, const Vector3d &pt) 
{
  // polygons are clockwise     
  int cuts=0;
  Vector3d vc(1,0,0);
  Vector3d res;
  for(int i=0;i<ps.indices.size();i++) {
    const IndexedFace &f=ps.indices[i];	  
    Vector3d va=ps.vertices[f[1]]-ps.vertices[f[0]];
    Vector3d vb=ps.vertices[f[2]]-ps.vertices[f[0]];
    if(linsystem(va, vb, vc,pt-ps.vertices[f[0]],res,nullptr)) continue;
    if(res[2] < 0) continue;
    if(res[0] >= -1e-6 && res[1] > -1e-6 && res[0]+res[1] <1+1e-6){
	    if(res[0] > 0 && res[1] > 0 && res[0]+res[1] <1 ) cuts += 2;
	    if(fabs(res[0]) < 1e-6 ) cuts++;
	    if(fabs(res[1]) < 1e-6 ) cuts++;
	    if(fabs(res[0]+res[1]-1) < 1e-6) cuts++;
    }
  }
  cuts /=2;
  return cuts&1;
}

// Credit: inphase Ryan Colyer
Vector3d Bezier(double t, Vector3d a, Vector3d b, Vector3d c)
{
	return (a*(1-t)+b*t)*(1-t)+ (b*(1-t)+c*t)*t; // TODO improve
}

void bezier_patch(PolySetBuilder &builder, Vector3d center, Vector3d dir1, Vector3d dir2, Vector3d dir3, int N) {
  if((dir2.cross(dir1)).dot(dir3) < 0) {
    Vector3d tmp=dir1;
    dir1=dir2;
    dir2=tmp;    
  }	  
  Vector3d xdir=dir1.normalized();	
  Vector3d ydir=dir2.normalized();	
  Vector3d zdir=dir3.normalized();

  // zdir shall look upwards
  		       

  Matrix3d mat;
  mat <<  xdir[0], ydir[0], zdir[0],
          xdir[1], ydir[1], zdir[1],
          xdir[2], ydir[2], zdir[2];



  xdir=Vector3d(1,0,0)*dir1.norm();
  ydir=Vector3d(0,1,0)*dir2.norm();
  zdir=Vector3d(0,0,1)*dir3.norm();


  // now use matrices to transform the vectors into std orientation
  //
  N = floor(N/2)*2 + 1;
  Vector3d pt;
  double s1 = 1.0 / (N-1);
  std::vector<Vector3d> points_xz;
  std::vector<Vector3d> points_yz;
  for(int i=0;i<N;i++) {
    double t=(double)i/(double)(N-1);
    points_xz.push_back(Bezier(t,  xdir, xdir+zdir, zdir));
    points_yz.push_back(Bezier(t,  ydir, ydir+zdir, zdir));
  }
 
  std::vector<int> points; 
  for(int i=0;i<N;i++){
    double t1=(double)i/(double)(N-1);
    if(i == N-1) {
      pt = zdir;
      pt = mat * pt;
      points.push_back(builder.vertexIndex(pt+center));
    } else {
      int M=N-i;
      for(int j=0;j<M;j++) {
        double t2=(double)j/(double)(M-1);
	pt = Bezier(t2, points_xz[i], Vector3d(points_xz[i][0], points_yz[i][1], points_xz[i][2]), points_yz[i]);
	pt = mat * pt;
        points.push_back(builder.vertexIndex(center + pt));
      }
    }
  }
  // total points = N*(N-1)/2
  int off=0;
  for(int i=0;i<N-1;i++) { // Zeile i, i-1
    int off_new=off+(N-i);
    for(int j=0;j<N-i-1;j++) {
     builder.appendPolygon({points[off+j], points[off+j+1], points[off_new+j]});
     if(j < N-i-2){builder.appendPolygon({points[off+j+1], points[off_new+j+1], points[off_new+j]});   }
    }
    off=off_new;
  }
}

std::vector<IndexedFace> mergetriangles(const std::vector<IndexedFace> polygons,const std::vector<Vector4d> normals,std::vector<Vector4d> &newNormals, std::vector<int> &faceParents, const std::vector<Vector3d> &vert); // TODO
																											  //

std::unique_ptr<const Geometry> createFilletInt(std::shared_ptr<const PolySet> ps,  std::vector<bool> corner_selected, double r)
{
  int bn=11; // bezier points  // odd
  std::vector<Vector4d> normals, newnormals;
  std::vector<int> faceParents;
  normals = offset3D_normals(ps->vertices, ps->indices);
  std::vector<IndexedFace> merged = mergetriangles(ps->indices, normals, newnormals, faceParents, ps->vertices);


  // Create vertex2face db
  std::vector<intList> polinds, polposs;
  intList empty;
  for(int i=0;i<ps->vertices.size();i++) {
    polinds.push_back(empty);	  
    polposs.push_back(empty);	  
  }
  for(int i=0;i<merged.size();i++) {
    for(int j=0;j<merged[i].size();j++) {
      int ind=merged[i][j];	    
      polinds[ind].push_back(i);
      polposs[ind].push_back(j);
    }	    
  }

  EdgeKey edge;                                                    
  std::unordered_map<EdgeKey, EdgeVal, boost::hash<EdgeKey> > edge_db;
  //
  // Create EDGE DB
  EdgeVal val;
  val.sel=0;
  val.face1=-1;
  val.face2=-1;
  val.pos1=-1;
  val.pos2=-1;
  int ind1, ind2;
  for(int i=0;i<merged.size();i++) {
    int  n=merged[i].size();
    for(int j=0;j<n;j++) {
      ind1=merged[i][j];	    
      ind2=merged[i][(j+1)%n];	    
      if(ind2 > ind1){
        edge.ind1=ind1;
        edge.ind2=ind2;	
	if(edge_db.count(edge) == 0) edge_db[edge]=val;
	edge_db[edge].face1=i;
	edge_db[edge].pos1=j;
      } else {
        edge.ind1=ind2;
        edge.ind2=ind1;	
	if(edge_db.count(edge) == 0) edge_db[edge]=val;
	edge_db[edge].face2=i;
	edge_db[edge].pos2=j;
      }
    }    
  }
//  printf("%d edges found\n",edge_db.size());


  std::vector<std::vector<int>> corner_rounds ; // which rounded edges in a corner
  for(int i=0;i<ps->vertices.size();i++) corner_rounds.push_back(empty);				  

  std::vector<SearchReplace> sp;
  // TODO select edges ( and by intersection , by number)
  for(auto &e: edge_db) {
    if(corner_selected[e.first.ind1] && corner_selected[e.first.ind2])
    {
      e.second.sel=1;
      corner_rounds[e.first.ind1].push_back(e.first.ind2);
      corner_rounds[e.first.ind2].push_back(e.first.ind1);
    }		      
  }

  // start builder with existing vertices to have VertexIndex available
  //
  PolySetBuilder builder;
  for(int i=0;i<ps->vertices.size();i++) {
    builder.vertexIndex(ps->vertices[i]); // allocate all vertices in the right order
  }

  SearchReplace s;

  // plan fillets of all edges now
  for(auto &e: edge_db) {
    if(e.second.sel == 1) {
//      printf("Create rnd for %d %d %d %d\n",e.first.ind1, e.first.ind2, e.second.face1, e.second.face2);	    
      Vector3d p1=ps->vertices[e.first.ind1];
      Vector3d p2=ps->vertices[e.first.ind2];
      Vector3d dir=(p2-p1).normalized();
      if(corner_rounds[e.first.ind1].size() >=  3) p1 += dir*r;
      if(corner_rounds[e.first.ind2].size() >=  3) p2 -= dir*r;
      auto &face1 =merged[e.second.face1];								  
      auto &face2 =merged[e.second.face2];								  
      Vector3d f1n=offset3D_normal(ps->vertices, face1).head<3>();
      Vector3d f2n=offset3D_normal(ps->vertices, face2).head<3>();

      int face1n=face1.size();
      int face2n=face2.size();
      int indo;
      indo = face1[(e.second.pos1+face1n-1)%face1n];
      Vector3d e_f1n = (ps->vertices[indo]-ps->vertices[face1[(e.second.pos1+0)%face1n]]).normalized(); // Face1 nahe  richtung


      Vector3d e_f1nr = f1n.cross(dir).normalized();
      double b=e_f1n.dot(e_f1nr);
      e_f1n *= r;

      if(corner_rounds[e.first.ind1].size() == 2 && list_included(corner_rounds[e.first.ind1],indo)) e_f1n += dir*r; 

      indo = face1[(e.second.pos1+2)%face1n];		
      Vector3d e_f1f = (ps->vertices[indo]-ps->vertices[face1[(e.second.pos1+1)%face1n]]).normalized()*r; // Face1 entfernte richtung
//      e_f1f = f1n.cross(dir).normalized()*r;
      if(corner_rounds[e.first.ind2].size() == 2 && list_included(corner_rounds[e.first.ind2],indo)) e_f1f -= dir*r;
     
      indo = face2[(e.second.pos2+2)%face2n];
      Vector3d e_f2n = (ps->vertices[indo]-ps->vertices[face2[(e.second.pos2+1)%face2n]]).normalized()*r; // Face2 nahe Richtung
 //     e_f2n = dir.cross(f2n).normalized()*r;
      if(corner_rounds[e.first.ind1].size() == 2 && list_included(corner_rounds[e.first.ind1],indo)) e_f2n += dir*r;

      indo = face2[(e.second.pos2+face2n-1)%face2n];
      Vector3d e_f2f = (ps->vertices[indo]-ps->vertices[face2[(e.second.pos2+0)%face2n]]).normalized()*r; // Face2 entfernte Rcithung
  //    e_f2f = dir.cross(f2n).normalized()*r;
      if(corner_rounds[e.first.ind2].size() == 2 && list_included(corner_rounds[e.first.ind2],indo)) e_f2f -= dir*r;
																	  

      for(int i=0;i<bn;i++) {
        double f=(double)i/(double)(bn-1);		
        e.second.bez1.push_back(builder.vertexIndex(p1 + e_f1n -2*f*e_f1n + f*f*(e_f1n+e_f2n)));
        e.second.bez2.push_back(builder.vertexIndex(p2 + e_f1f -2*f*e_f1f + f*f*(e_f1f+e_f2f)));
      }
      s.pol=e.second.face1; // laengsseite1
      s.search=e.first.ind1;
      s.replace={e.second.bez1[0]};
      sp.push_back(s);
      s.pol=e.second.face1; // laengsseite1
      s.search=e.first.ind2 ;
      s.replace={ e.second.bez2[0]};
      sp.push_back(s);

      s.pol=e.second.face2; // laengsseite2
      s.search=e.first.ind2 ;
      s.replace={e.second.bez2[bn-1]};
      sp.push_back(s);
      s.pol=e.second.face2; // laengsseite2
      s.search=e.first.ind1 ;
      s.replace={e.second.bez1[bn-1]};
      sp.push_back(s);

      // stirnseite 1
      if(corner_rounds[e.first.ind1].size() == 1) {
        for(int i=0;i<polinds[e.first.ind1].size();i++) { 
          int faceid=polinds[e.first.ind1][i];
          int facepos=polinds[e.first.ind1][i];
          if(faceid == e.second.face1) continue;       
          if(faceid == e.second.face2) continue;       
	  s.pol=faceid; // stirnseite2
          s.search=e.first.ind1 ;
          s.replace={e.second.bez1};
	  std::reverse(s.replace.begin(), s.replace.end());
          sp.push_back(s);
        }   
      }	
      
      //stirnseite2
      if(corner_rounds[e.first.ind2].size() == 1) {
        for(int i=0;i<polinds[e.first.ind2].size();i++) {
          int faceid=polinds[e.first.ind2][i];
          int facepos=polinds[e.first.ind2][i];
          if(faceid == e.second.face1) continue;       
          if(faceid == e.second.face2) continue;       
	  s.pol=faceid; // stirnseite2
          s.search=e.first.ind2 ;
          s.replace={e.second.bez2};
          sp.push_back(s);
        }   
     }	

    }	

  }
  // now dump all sp
//  for(int i=0;i<sp.size();i++) {
//    printf("Poly %d S: ",sp[i].pol);	  
//    printf("%d ",sp[i].search);
//    printf(" R:");
//    for(int j=0;j<sp[i].replace.size();j++) printf("%d ",sp[i].replace[j]);
//    printf("\n");
//  } 
  // copy modified faces
  for(int i=0;i<merged.size();i++)  {
    const IndexedFace &face = merged[i];
    std::vector<int> newface;
    for(int j=0;j<face.size();j++) {
      int ind=face[j];	    
      newface.push_back(ind);
    }      
    int fn=newface.size();
    // does newface need any mods ?
    for(int j=0;j<sp.size();j++){ // TODO effektiver, sp sortiren und 0 groesser machen
      if(sp[j].pol == i) {
        int needle=sp[j].search;
	for(int k=0;k<fn;k++) { // all possible shifts
          if(newface[k] == needle) {
	    // match bei shift k gefunden
	    std::vector<int> tmp=sp[j].replace;
	    for(int l=0;l<fn-1;l++) {
             tmp.push_back(newface[(k+1+l)%fn]);		    
	    }
	    newface=tmp;
	    fn=newface.size();
	    break;
	  }  
	}
      }	      
    }

    builder.appendPolygon(newface);    
  }

  // add Rounded edges 
  for(auto &e: edge_db) {
    if(e.second.sel == 1) {
      // now create the faces
      for(int i=0;i<bn-1;i++) {
        builder.appendPolygon({e.second.bez1[i], e.second.bez2[i], e.second.bez2[i+1], e.second.bez1[i+1]});
      }
    }	    
  }
  // add missing 3 corner patches
  //
  for(int i=0;i<ps->vertices.size();i++) {
    if(corner_rounds[i].size() > 3) {
      printf("corner %d not possible\n",i);	    
    }
    else if(corner_rounds[i].size() == 3) {
      Vector3d xdir=(ps->vertices[i] - ps->vertices[corner_rounds[i][0]]).normalized()*r;
      Vector3d ydir=(ps->vertices[i] - ps->vertices[corner_rounds[i][1]]).normalized()*r;
      Vector3d zdir=(ps->vertices[i] - ps->vertices[corner_rounds[i][2]]).normalized()*r;
      bezier_patch(builder, ps->vertices[i]-xdir-ydir-zdir, xdir, ydir, zdir,bn);
    }	    
  }
  //
  auto result = builder.build();

  return result;
}

std::unique_ptr<const Geometry> FilletNode::createGeometry() const
{
  std::shared_ptr<const PolySet> ps;
  std::vector<bool> corner_selected ;
  if(this->children.size() >= 1) {
    ps = childToPolySet(this->children[0]);
    if(ps == nullptr)
	return std::unique_ptr<PolySet>();
  } else return std::unique_ptr<PolySet>();

  if(this->children.size() >= 2) {
    std::shared_ptr<const PolySet> sel = childToPolySet(this->children[1]);
    if(sel != nullptr) {
      auto sel_tess=PolySetUtils::tessellate_faces(*sel);
      for(int i=0;i<ps->vertices.size();i++) {
        corner_selected.push_back(point_in_polyhedron(*sel_tess, ps->vertices[i]));
      }
    }

  } else {
      for(int i=0;i<ps->vertices.size();i++) 
        corner_selected.push_back(true);	      
  }
  return createFilletInt(ps, corner_selected, this->r);
}


