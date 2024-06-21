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
  int facea, posa;  // face a with edge ind1 -> ind2, posa = index of ind1 within facea
  int faceb, posb;  // face b with edge ind2 -> ind1, posb = index of ind2 within faceb
  IndexedFace bez1;
  IndexedFace bez2;
};

struct SearchReplace {
  int pol;
  int search;
  IndexedFace replace;
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



typedef std::vector<int> intList;

bool list_included(const std::vector<int> &list,int needle) {
  for(int i=0;i<list.size();i++){
    if(list[i] == needle) return true;
  }	  
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

void bezier_patch(PolySetBuilder &builder, Vector3d center, Vector3d dir1, Vector3d dir2, Vector3d dir3, int concave_1, int concave_2, int concave_3, int N) {
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
//  N = floor(N/2)*2 + 1;
  Vector3d pt;
  double s1 = 1.0 / (N-1);
  std::vector<Vector3d> points_xz;
  std::vector<Vector3d> points_yz;
  for(int i=0;i<N;i++) {
    double t=(double)i/(double)(N-1);
    points_xz.push_back(Bezier(t,  xdir, xdir+zdir, zdir+2*(concave_1+concave_2)*(xdir+ydir)));
    points_yz.push_back(Bezier(t,  ydir, ydir+zdir, zdir+2*(concave_1+concave_2)*(xdir+ydir)));
  }
 
  std::vector<int> points; 
  for(int i=0;i<N;i++){
    double t1=(double)i/(double)(N-1);
    if(i == N-1) {
      pt = zdir+2*(concave_1+concave_2)*(xdir+ydir);
      pt = mat * pt;
      points.push_back(builder.vertexIndex(pt+center));
    } else {
      int M=N-i;
      for(int j=0;j<M;j++) {
        int k;
	if(concave_1 == 1 || concave_3 == 1) k=j; else k=M-1-j;
        double t2=(double)k/(double)(M-1);
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

																											  //

std::unique_ptr<const Geometry> createFilletInt(std::shared_ptr<const PolySet> ps,  std::vector<bool> corner_selected, double r, int bn)
{
  std::vector<Vector4d> normals, newnormals;
  std::vector<int> faceParents;
  normals = calcTriangleNormals(ps->vertices, ps->indices);
  std::vector<IndexedFace> merged = mergeTriangles(ps->indices, normals, newnormals, faceParents, ps->vertices);


  if(bn < 2) bn=2;
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
  val.facea=-1;
  val.faceb=-1;
  val.posa=-1;
  val.posb=-1;
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
	edge_db[edge].facea=i;
	edge_db[edge].posa=j;
      } else {
        edge.ind1=ind2;
        edge.ind2=ind1;	
	if(edge_db.count(edge) == 0) edge_db[edge]=val;
	edge_db[edge].faceb=i;
	edge_db[edge].posb=j;
      }
    }    
  }
 // check if edge_db is complete
  
  for(auto &e: edge_db) {
    if(e.second.facea == -1 || e.second.faceb == -1 || e.second.posa == -1 || e.second.posb == -1 ) {
      printf("Edge %d - %d faca %d faceb %d posa %d posb %d\n", e.second.facea, e.second.faceb, e.second.posa, e.second.posb);
      printf("a %g/%g/%g\n", ps->vertices[e.first.ind1][0], ps->vertices[e.first.ind1][1], ps->vertices[e.first.ind1][2]);
      printf("b %g/%g/%g\n", ps->vertices[e.first.ind2][0], ps->vertices[e.first.ind2][1], ps->vertices[e.first.ind2][2]);
      exit(1);
    }
  }

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
      Vector3d p1=ps->vertices[e.first.ind1];
      Vector3d p2=ps->vertices[e.first.ind2];
      int debug=0;
      // if(debug) printf("====\nCreate rnd for %d %d %d %d\n",e.first.ind1, e.first.ind2, e.second.facea, e.second.faceb);	    
      if(debug) printf("p1 %g/%g/%g p2 %g/%g/%g\n",p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
      Vector3d dir=(p2-p1).normalized();
      if(corner_rounds[e.first.ind1].size() >=  3) p1 += dir*r;
      if(corner_rounds[e.first.ind2].size() >=  3) p2 -= dir*r;
      auto &facea =merged[e.second.facea];								  
      auto &faceb =merged[e.second.faceb];								  
      //

      int facean=facea.size();
      int facebn=faceb.size();
      double fanf=(faceParents[e.second.facea] != -1)?-1:1;
      double fbnf=(faceParents[e.second.faceb] != -1)?-1:1;
      Vector3d fan=calcTriangleNormal(ps->vertices, facea).head<3>();
      Vector3d fbn=calcTriangleNormal(ps->vertices, faceb).head<3>();

      int indposao, indposbo, indposai, indposbi;
      Vector3d unit;

      indposao = facea[(e.second.posa+facean-1)%facean];
      indposai = facea[(e.second.posa+1)%facean];

      indposbo = faceb[(e.second.posb+2)%facebn];
      indposbi = faceb[e.second.posb];
    
      Vector3d e_fa1  = (ps->vertices[indposao]-ps->vertices[facea[e.second.posa]]).normalized()*r*fanf; // Facea neben ind1
      Vector3d e_fa1p = (ps->vertices[indposai]-ps->vertices[facea[e.second.posa]])*fanf; // Face1 nahe  richtung
															   //
      Vector3d e_fb1 =  (ps->vertices[indposbo]-ps->vertices[faceb[(e.second.posb+1)%facebn]]).normalized()*r*fbnf; // Faceb neben ind1
      Vector3d e_fb1p = (ps->vertices[indposbi]-ps->vertices[faceb[(e.second.posb+1)%facebn]])*fbnf; 

      if(corner_rounds[e.first.ind1].size() == 2)
      {
        double a=(e_fb1.cross(e_fa1)).dot(dir);
	double b=(fan.cross(fbn)).dot(e_fa1p)*fanf*fbnf;
        if(list_included(corner_rounds[e.first.ind1],indposao)){
		e_fa1 += dir*r*fanf; 
		if(a*b < 0) e_fa1 = -e_fa1*fanf;
		else e_fa1 = e_fa1*fanf;
	}
        if(list_included(corner_rounds[e.first.ind1],indposbo)){
		e_fb1 += dir*r*fbnf;
		if(a*b < 0) e_fb1 = -e_fb1;
	}
      }

      if(corner_rounds[e.first.ind1].size() == 3)
      {
        if( (fbn.cross(fan)).dot(e_fa1p) < 0 || (fbn.cross(fan)).dot(e_fb1p) < 0) {
	  if((e_fa1p.cross(e_fa1)).dot(fan)*fanf < 0) {
	    e_fa1 = -e_fa1*fanf - 2*dir*r;
	  }
	  if((e_fb1p.cross(e_fb1)).dot(fbn)*fbnf > 0) {
	    e_fb1 = -e_fb1*fbnf - 2*dir*r;
	  }
	}
        if(  (fbn.cross(fan)).dot(e_fb1p) > 0 ) {
	  if((e_fa1p.cross(e_fa1)).dot(fan)*fanf > 0  && (e_fa1p.cross(e_fb1)).dot(fbn)*fbnf > 0) {
	    e_fb1 = -e_fb1*fbnf - 2*dir*r;
	  }
	  if((e_fb1p.cross(e_fb1)).dot(fbn)*fbnf < 0  && (e_fb1p.cross(e_fa1)).dot(fan)*fanf < 0 ) {
	    e_fa1 = -e_fa1*fanf - 2*dir*r;
	  }
	}
      }

      if(debug) printf("p1 fa %g/%g/%g fb  %g/%g/%g\n",e_fa1[0], e_fa1[1], e_fa1[2], e_fb1[0], e_fb1[1], e_fb1[2]);
      if(debug) printf("p1 fap %g/%g/%g fbp  %g/%g/%g\n",e_fa1p[0], e_fa1p[1], e_fa1p[2], e_fb1p[0], e_fb1p[1], e_fb1p[2]);

      indposao = facea[(e.second.posa+2)%facean];		
      indposai = facea[e.second.posa];		

      indposbo = faceb[(e.second.posb+facebn-1)%facebn];
      indposbi = faceb[(e.second.posb+1)%facebn];

      Vector3d e_fa2 = (ps->vertices[indposao]-ps->vertices[facea[(e.second.posa+1)%facean]]).normalized()*r*fanf; // Face1 entfernte richtung
      Vector3d e_fa2p = (ps->vertices[indposai]-ps->vertices[facea[(e.second.posa+1)%facean]])*fanf; // Face1 entfernte richtung
													       //
      Vector3d e_fb2 = (ps->vertices[indposbo]-ps->vertices[faceb[(e.second.posb+0)%facebn]]).normalized()*r*fbnf; // Face2 entfernte Rcithung
      Vector3d e_fb2p = (ps->vertices[indposbi]-ps->vertices[faceb[(e.second.posb+0)%facebn]])*fbnf; // Face2 entfernte Rcithung
        	

													   //
      if(corner_rounds[e.first.ind2].size() == 2) 
      {
        double a=(e_fb2.cross(e_fa2)).dot(dir);
	double b=(fan.cross(fbn)).dot(e_fa2p)*fanf*fbnf;
        if(list_included(corner_rounds[e.first.ind2],indposao)){
		e_fa2 -= dir*r*fanf;
		if(a*b > 0) e_fa2 = -e_fa2*fanf;
		else e_fa2 = e_fa2*fanf;
	}
        if(list_included(corner_rounds[e.first.ind2],indposbo)){
		e_fb2 -= dir*r*fbnf;
		if(a*b > 0)  e_fb2 = -e_fb2;
	}
      }	

      if(corner_rounds[e.first.ind2].size() == 3)
      {
//        if(debug) printf("e\n");
        if( -(fbn.cross(fan)).dot(e_fa2p) < 0 || -(fbn.cross(fan)).dot(e_fb2p) < 0 ) {
//          if(debug) printf("f\n");
	  if(-(e_fa2p.cross(e_fa2)).dot(fan)*fanf < 0){
//            if(debug) printf("g\n");
            e_fa2 = -e_fa2*fanf + 2*dir*r; 
	  }  
	  if(-(e_fb2p.cross(e_fb2)).dot(fbn)*fbnf > 0){
//            if(debug) printf("h\n");
            e_fb2 = -e_fb2*fbnf + 2*dir*r;
	  }  
	}
        if(/* -(fbn.cross(fan)).dot(e_fa2p) > 0 || */ -(fbn.cross(fan)).dot(e_fb2p) > 0  ) {
          if(debug) printf("y ind1=%d test=%g\n",e.first.ind2,(e_fa2p.cross(e_fb2)).dot(fbn));		
	  if(-(e_fb2p.cross(e_fb2)).dot(fbn)*fbnf < 0 && (e_fb2p.cross(e_fa2)).dot(fan)*fanf > 0){
            if(debug) printf("y1\n");
            e_fa2 = -e_fa2*fanf + 2*dir*r; // laengs links
	  }  
	  if(-(e_fa2p.cross(e_fa2)).dot(fan)*fanf > 0 && (e_fa2p.cross(e_fb2)).dot(fbn)*fbnf < 0 ){
            if(debug) printf("y2\n");
            e_fb2 = -e_fb2*fbnf + 2*dir*r; 
	  }  
	}
      }
      if(debug) printf("p2 fa %g/%g/%g fb  %g/%g/%g\n",e_fa2[0], e_fa2[1], e_fa2[2], e_fb2[0], e_fb2[1], e_fb2[2]);

      for(int i=0;i<bn;i++) {
        double f=(double)i/(double)(bn-1);		
        e.second.bez1.push_back(builder.vertexIndex(p1 + e_fa1 -2*f*e_fa1 + f*f*(e_fa1+e_fb1)));
        e.second.bez2.push_back(builder.vertexIndex(p2 + e_fa2 -2*f*e_fa2 + f*f*(e_fa2+e_fb2)));
      }
      s.pol=e.second.facea; // laengsseite1
      s.search=e.first.ind1;
      s.replace={e.second.bez1[0]};
      sp.push_back(s);
      s.pol=e.second.facea; // laengsseite1
      s.search=e.first.ind2 ;
      s.replace={ e.second.bez2[0]};
      sp.push_back(s);

      s.pol=e.second.faceb; // laengsseite2
      s.search=e.first.ind2 ;
      s.replace={e.second.bez2[bn-1]};
      sp.push_back(s);
      s.pol=e.second.faceb; // laengsseite2
      s.search=e.first.ind1 ;
      s.replace={e.second.bez1[bn-1]};
      sp.push_back(s);

      // stirnseite 1
      if(corner_rounds[e.first.ind1].size() == 1) {
        for(int i=0;i<polinds[e.first.ind1].size();i++) { 
          int faceid=polinds[e.first.ind1][i];
          int facepos=polinds[e.first.ind1][i];
          if(faceid == e.second.facea) continue;       
          if(faceid == e.second.faceb) continue;       
	  s.pol=faceid; // stirnseite1
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
          if(faceid == e.second.facea) continue;       
          if(faceid == e.second.faceb) continue;       
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
  std::vector<IndexedFace> newfaces;
  for(int i=0;i<merged.size();i++)  {
    const IndexedFace &face = merged[i];
    IndexedFace newface;
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
	    IndexedFace tmp=sp[j].replace;
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

    newfaces.push_back(newface);
  }
  std::vector<Vector3d> vertices;
  builder.copyVertices(vertices);
  std::vector<Vector3f> verticesFloat;
  for(const auto &v: vertices)
    verticesFloat.push_back(v.cast<float>());	  

  for(int i=0;i<newfaces.size();i++) {
    // tessellate first with holes // search all holes
    if(faceParents[i]  != -1) continue;
    std::vector<IndexedFace> faces;
    faces.push_back(newfaces[i]);
    for(int j=0;j<newfaces.size();j++) 
      if(faceParents[j] == i) faces.push_back(newfaces[j]);
//    if(faces.size() >1 ) continue;
    std::vector<IndexedTriangle> triangles;
    Vector3f norm(newnormals[i][0],newnormals[i][1],newnormals[i][2]);
    GeometryUtils::tessellatePolygonWithHoles(verticesFloat, faces, triangles, &norm);
    for (const auto& t : triangles) {
      builder.appendPolygon({t[0],t[1],t[2]});
    }
  }  

  // add Rounded edges 
  for(auto &e: edge_db) {
    if(e.second.sel == 1) {
      // now create the faces
      for(int i=0;i<bn-1;i++) {
        builder.appendPolygon({e.second.bez1[i],e.second.bez1[i+1],  e.second.bez2[i+1], e.second.bez2[i]});
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
      // now get the right ordering of corner_rounds[i]
      for(int j=0;j<3;j++) {
   	      
	IndexedFace &face =merged[polinds[i][j]];
	int pos=polposs[i][j];
	int len=face.size();
      }
      IndexedFace face[3];
      Vector3d facenorm[3];
      for(int j=0;j<3;j++) {
        face[j] =merged[polinds[i][j]];
        facenorm[j] = calcTriangleNormal(ps->vertices, face[j]).head<3>();
        if(faceParents[polinds[i][j]]  != -1) facenorm[j] = -facenorm[j];
      }

      int facebeg[3];
      int faceend[3];
      for(int j=0;j<3;j++){	     
        facebeg[j]=face[j][(polposs[i][j]+face[j].size()-1)%face[j].size()];
        faceend[j]=face[j][(polposs[i][j]+1)%face[j].size()];
      }

/*
      if(debug) {
        printf("Corner: ind=%d %f/%f/%f\n",i, ptx[0], ptx[1], ptx[2]);
	for(int j=0;j<3;j++) {
          printf("Bogen %d: \n",j);
	  for(int k=0;k<3;k++){
            int ind=face[j][(polposs[i][j]+face[j].size()+k-1)%face[j].size()];
	    printf("%d (%f/%f/%f)\t",ind, ps->vertices[ind][0],  ps->vertices[ind][1],ps->vertices[ind][2]);
	  }

	  printf("\n");
	}
	printf("Normals\n");
	for(int j=0;j<3;j++)
          printf("%d %f/%f/%f\n",j,facenorm[j][0], facenorm[j][1], facenorm[j][2]);
      }
*/
      std::vector<int> angle;
      std::vector<Vector3d> dir;
      if(faceend[0] == facebeg[1]) { // 0,1,2
//	if(debug) printf("Branch A\n");
        dir.push_back((ps->vertices[faceend[1]]-ps->vertices[i]).normalized()*r);
	angle.push_back((facenorm[1].cross(facenorm[2])).dot(ps->vertices[faceend[1]]-ps->vertices[i])>0?1:-1);
        dir.push_back((ps->vertices[faceend[2]]-ps->vertices[i]).normalized()*r);
	angle.push_back((facenorm[2].cross(facenorm[0])).dot(ps->vertices[faceend[2]]-ps->vertices[i])>0?1:-1);
        dir.push_back((ps->vertices[faceend[0]]-ps->vertices[i]).normalized()*r);
	angle.push_back((facenorm[0].cross(facenorm[1])).dot(ps->vertices[faceend[0]]-ps->vertices[i])>0?1:-1);
      } else if(faceend[0] == facebeg[2]) { 
//	if(debug) printf("Branch B\n");
        dir.push_back((ps->vertices[faceend[2]]-ps->vertices[i]).normalized()*r);
	angle.push_back((facenorm[2].cross(facenorm[1])).dot(ps->vertices[faceend[2]]-ps->vertices[i])>0?1:-1);
        dir.push_back((ps->vertices[faceend[0]]-ps->vertices[i]).normalized()*r);
	angle.push_back((facenorm[0].cross(facenorm[2])).dot(ps->vertices[faceend[0]]-ps->vertices[i])>0?1:-1);
        dir.push_back((ps->vertices[faceend[1]]-ps->vertices[i]).normalized()*r);
	angle.push_back((facenorm[1].cross(facenorm[0])).dot(ps->vertices[faceend[1]]-ps->vertices[i])>0?1:-1);
      } else assert(0);
/*
      if(debug) {
        for(int j=0;j<3;j++) {
          printf("Dir %d (%g/%g/%g) ang=%d\n",j, dir[j][0], dir[j][1], dir[j][2], angle[j]);		
	}  
		
	printf("---------\n");
        errcount++;
//	if(errcount == 2) exit(1);	
      }
*/
      if(angle[0] == -1 && angle[1] == -1 && angle[2] == -1)
	      bezier_patch(builder, ps->vertices[i]+dir[0]+dir[1]+dir[2], -dir[0], -dir[1], -dir[2],0, 0, 1, bn); // 0
      else if(angle[0] == -1 && angle[1] == -1 && angle[2] ==  1) 
	      bezier_patch(builder, ps->vertices[i]+dir[0]+dir[1]+dir[2], -dir[0], -dir[1], -dir[2],0, 1, 0, bn); // 1
      else if(angle[0] == -1 && angle[1] == 1 && angle[2]  == -1) 
	      bezier_patch(builder, ps->vertices[i]+dir[0]+dir[1]+dir[2], -dir[2], -dir[0], -dir[1],0, 1, 0, bn); // 2
      else if(angle[0] == -1 && angle[1] ==  1 && angle[2] ==  1) 
	      bezier_patch(builder, ps->vertices[i]+dir[0]+dir[1]+dir[2], -dir[1], -dir[2], -dir[0],1, 0, 0, bn); // 3
      else if(angle[0] ==  1 && angle[1] == -1 && angle[2] == -1) 
	      bezier_patch(builder, ps->vertices[i]+dir[0]+dir[1]+dir[2], -dir[1], -dir[2], -dir[0],0, 1, 0, bn); // 4
      else if(angle[0] ==  1 && angle[1] == -1 && angle[2] ==  1) 
	      bezier_patch(builder, ps->vertices[i]+dir[0]+dir[1]+dir[2], -dir[2], -dir[0], -dir[1],1, 0, 0, bn); // 5
      else if(angle[0] ==  1 && angle[1] ==  1 && angle[2] == -1) 
	      bezier_patch(builder, ps->vertices[i]+dir[0]+dir[1]+dir[1], -dir[0], -dir[1], -dir[2],1, 0, 0, bn); // 6
      else bezier_patch(builder, ps->vertices[i]+dir[0]+dir[1]+dir[2], -dir[0], -dir[1], -dir[2],0,  0, 0, bn);	  // 7
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
  return createFilletInt(ps, corner_selected, this->r, this->fn);
}


