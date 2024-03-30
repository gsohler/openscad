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
  std::vector<int> search; // TODO nur 1 ?
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

typedef std::vector<int> intList;

bool list_included(const std::vector<int> &list,int item) {
  if(std::find(list.begin(), list.end(),item) != 	list.end()) return true;
  return false;
}


std::unique_ptr<const Geometry> FilletNode::createGeometry() const
{
  if(this->children.size() == 0) {
	return std::unique_ptr<PolySet>();
  }
  std::shared_ptr<AbstractNode> child=this->children[0];
  Tree tree(child, "");
  GeometryEvaluator geomevaluator(tree);
  std::shared_ptr<const Geometry> geom = geomevaluator.evaluateGeometry(*tree.root(), true);
  int bn=11; // bezier points  // odd
  std::shared_ptr<const PolySet> ps = PolySetUtils::getGeometryAsPolySet(geom);
  if(ps == nullptr)
	return std::unique_ptr<PolySet>();

  // Create vertex2face db
  std::vector<intList> polinds, polposs;
  intList empty;
  for(int i=0;i<ps->vertices.size();i++) {
    polinds.push_back(empty);	  
    polposs.push_back(empty);	  
  }
  for(int i=0;i<ps->indices.size();i++) {
    for(int j=0;j<ps->indices[i].size();j++) {
      int ind=ps->indices[i][j];	    
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
  for(int i=0;i<ps->indices.size();i++) {
    int  n=ps->indices[i].size();
    for(int j=0;j<n;j++) {
      ind1=ps->indices[i][j];	    
      ind2=ps->indices[i][(j+1)%n];	    
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
  printf("%d edges found\n",edge_db.size());

  std::vector<std::vector<int>> corner_rounds ; // how many round edges in a corner
  for(int i=0;i<ps->vertices.size();i++) corner_rounds.push_back(empty);				  

  std::vector<SearchReplace> sp;
  // TODO select edges (by overlapping and by intersection , by number)
  int cnt=0;
  for(auto &e: edge_db) {
    if(cnt == 0 || cnt == 6 || cnt == 7 || cnt == 8 || cnt == 9){
      e.second.sel=1;
      corner_rounds[e.first.ind1].push_back(e.first.ind2);
      corner_rounds[e.first.ind2].push_back(e.first.ind1);
    }		      
    cnt++;
  }

//  for(int i=0;i<corner_rounds.size();i++){
//    printf("%d :",i);
//    for(int j=0;j<corner_rounds[i].size();j++)
//	    printf("%d ",corner_rounds[i][j]);
//    printf("\n");
//     
//  }

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
//      if(corner_rounds[e.first.ind1].size() >=  3) p1 += dir*this->r;
//      if(corner_rounds[e.first.ind2].size() >=  3) p2 -= dir*this->r;
      auto &face1 =ps->indices[e.second.face1];								  
      auto &face2 =ps->indices[e.second.face2];								  
      int face1n=face1.size();
      int face2n=face2.size();
      int indo;
      double f1=1.0;
      double f2=1.4142;
      indo = face1[(e.second.pos1+face1n-1)%face1n];
      Vector3d e_f1n = (ps->vertices[indo]-ps->vertices[face1[(e.second.pos1+0)%face1n]]).normalized()*this->r; // Face1 nahe  richtung
      if(corner_rounds[e.first.ind1].size() >= 2 && list_included(corner_rounds[e.first.ind1],indo)) e_f1n += dir*this->r;

      indo = face1[(e.second.pos1+2)%face1n];		
      Vector3d e_f1f = (ps->vertices[indo]-ps->vertices[face1[(e.second.pos1+1)%face1n]]).normalized()*this->r; // Face1 entfernte richtung
      if(corner_rounds[e.first.ind2].size() >= 2 && list_included(corner_rounds[e.first.ind2],indo)) e_f1f -= dir*this->r;
     
      indo = face2[(e.second.pos2+2)%face2n];
      Vector3d e_f2n = (ps->vertices[indo]-ps->vertices[face2[(e.second.pos2+1)%face2n]]).normalized()*this->r; // Face2 nahe Richtung
      if(corner_rounds[e.first.ind1].size() >= 2 && list_included(corner_rounds[e.first.ind1],indo)) e_f2n += dir*this->r;

      indo = face2[(e.second.pos2+face2n-1)%face2n];
      Vector3d e_f2f = (ps->vertices[indo]-ps->vertices[face2[(e.second.pos2+0)%face2n]]).normalized()*this->r; // Face2 entfernte Rcithung
      if(corner_rounds[e.first.ind2].size() >= 2 && list_included(corner_rounds[e.first.ind2],indo)) e_f2f -= dir*this->r;
																	  

      for(int i=0;i<bn;i++) {
        double f=(double)i/(double)(bn-1);		
        e.second.bez1.push_back(builder.vertexIndex(p1 + e_f1n -2*f*e_f1n + f*f*(e_f1n+e_f2n)));
        e.second.bez2.push_back(builder.vertexIndex(p2 + e_f1f -2*f*e_f1f + f*f*(e_f1f+e_f2f)));
      }
      s.pol=e.second.face1; // laengsseite1
      s.search={e.first.ind1};
      s.replace={e.second.bez1[0]};
      sp.push_back(s);
      s.pol=e.second.face1; // laengsseite1
      s.search={e.first.ind2 };
      s.replace={ e.second.bez2[0]};
      sp.push_back(s);

      s.pol=e.second.face2; // laengsseite2
      s.search={e.first.ind2 };
      s.replace={e.second.bez2[bn-1]};
      sp.push_back(s);
      s.pol=e.second.face2; // laengsseite2
      s.search={e.first.ind1 };
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
          s.search={e.first.ind1 };
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
          s.search={e.first.ind2 };
          s.replace={e.second.bez2};
          sp.push_back(s);
        }   
     }	

    }	

  }
  // now dump all sp
//  for(int i=0;i<sp.size();i++) {
//    printf("Poly %d S: ",sp[i].pol);	  
//    for(int j=0;j<sp[i].search.size();j++) printf("%d ",sp[i].search[j]);
//    printf(" R:");
//    for(int j=0;j<sp[i].replace.size();j++) printf("%d ",sp[i].replace[j]);
//    printf("\n");
//  } 
  // copy modified faces
  for(int i=0;i<ps->indices.size();i++)  {
    const IndexedFace &face = ps->indices[i];
    std::vector<int> newface;
    for(int j=0;j<face.size();j++) {
      int ind=face[j];	    
      newface.push_back(ind);
    }      
    int fn=newface.size();
    // does newface need any mods ?
    for(int j=0;j<sp.size();j++){ // TODO effektiver, sp sortiren und 0 groesser machen
      if(sp[j].pol == i) {
        auto needle=sp[j].search;
        int nn=needle.size();
	for(int k=0;k<fn;k++) { // all possible shifts
	  bool match=true;
	  for(int l=0;match && l<nn;l++) {
            if(newface[(k+l)%fn] != needle[l]) match=false;
	  }
	  if(match) {
	    // match bei shift k gefunden
	    std::vector<int> tmp=sp[j].replace;
	    for(int l=0;l<fn-nn;l++) {
             tmp.push_back(newface[(k+nn+l)%fn]);		    
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
      printf("Corner %d needs sphere\n",i);	    
      std::vector<std::vector<int>> conns;
      for(auto &e: edge_db) {
 	if( e.first.ind1 == i) conns.push_back(e.second.bez1);
 	if( e.first.ind2 == i){
		auto tmp = e.second.bez2;		
	  	std::reverse(tmp.begin(), tmp.end());
		conns.push_back(tmp);
	}
      }
      printf("%d sets found\n", conns.size());
      assert(conns.size() == 3);
//      auto tmp=conns[1];
//      conns[1]=conns[2];
//      conns[2]=tmp; // TODO fix
//      // 1und 2 tauschen
      for(int j=0;j<bn/2;j++) {
      	builder.appendPolygon({conns[0][j], conns[1][bn-1-j], conns[1][bn-2-j], conns[0][1+j]}); 
      	builder.appendPolygon({conns[1][j], conns[2][bn-1-j], conns[2][bn-2-j], conns[1][1+j]}); 
      	builder.appendPolygon({conns[2][j], conns[0][bn-1-j], conns[0][bn-2-j], conns[2][1+j]}); 
      }
      int j=bn/2;
      builder.appendPolygon({conns[0][j], conns[1][j],conns[2][j]});
    }	    
  }
  auto result = builder.build();

  printf("%d faces\n",result->indices.size());
  return result;
}

