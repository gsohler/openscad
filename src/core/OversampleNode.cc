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

#include "OversampleNode.h"
#include "module.h"
#include "ModuleInstantiation.h"
#include "Children.h"
#include "Parameters.h"
#include "printutils.h"
#include "io/fileutils.h"
#include "Builtins.h"
#include "handle_dep.h"

#include <cmath>
#include <sstream>

#include <PolySetUtils.h>
#include <Tree.h>
#include <GeometryEvaluator.h>
#include <boost/functional/hash.hpp>
#include <hash.h>
typedef std::vector<int> intList;

void ov_add_poly_round(Vector3d &p)
{
	p.normalize();
}
void ov_add_poly(PolySet *ps, Vector3d p1, Vector3d p2, Vector3d p3, int round)
{
  ps->append_poly();
  if(round)
  {
    ov_add_poly_round(p1);
    ov_add_poly_round(p2);
    ov_add_poly_round(p3);
  }
  ps->append_vertex(p1[0],p1[1],p1[2]);
  ps->append_vertex(p2[0],p2[1],p2[2]);
  ps->append_vertex(p3[0],p3[1],p3[2]);
}

const Geometry *OversampleNode::createGeometry() const
{
  PolySet *ps_tess = new PolySet(3,true);
  if(this->children.size() > 0) {
   std::shared_ptr<AbstractNode> child=this->children[0];
   Tree tree(child, "");
   GeometryEvaluator geomevaluator(tree);
   shared_ptr<const Geometry> geom = geomevaluator.evaluateGeometry(*tree.root(), true);
   std::shared_ptr<const PolySet> ps = dynamic_pointer_cast<const PolySet>(geom);
  // tesselate object
   PolySetUtils::tessellate_faces(*ps, *ps_tess);
  }
  std::vector<Vector3d> pt_dir;
  if(this->round == 1) {
    printf("round\n");	  
    // create indexed point list
    std::unordered_map<Vector3d, int, boost::hash<Vector3d> > pointIntMap;
    std::vector<Vector3d> pointList; // list of all the points in the object
    std::vector<Vector3d> pointListNew; // list of all the points in the object
    std::vector<intList> polygons; // list polygons represented by indexes
    std::vector<intList>  pointToFaceInds; //  mapping pt_ind -> list of polygon inds which use it
    std::vector<intList>  pointToFacePoss; //  mapping pt_ind -> list of polygon inds which use it
    intList emptyList;
    for(int i=0;i<ps_tess->polygons.size();i++) {
      Polygon pol = ps_tess->polygons[i];
      intList polygon;
      for(int j=0;j<pol.size(); j++) {
        int ptind=0;
        Vector3d  pt=pol[j];
        pt[0]=(int)(pt[0]*1000+0.5)/1000.0;
        pt[1]=(int)(pt[1]*1000+0.5)/1000.0;
        pt[2]=(int)(pt[2]*1000+0.5)/1000.0;
        if(!pointIntMap.count(pt)) {
          pointList.push_back(pt);
          pointToFaceInds.push_back(emptyList);
          pointToFacePoss.push_back(emptyList);
          ptind=pointList.size()-1;
          pointIntMap[pt]=ptind;
        } else ptind=pointIntMap[pt];
        polygon.push_back(ptind);
	pointToFaceInds[ptind].push_back(i);
	pointToFacePoss[ptind].push_back(j);
      }
      polygons.push_back(polygon);
    }
    // for each vertex, calculate the dir
    for(int i=0;i<pointList.size();i++) {
      printf("i=%d\n",i);
      Vector3d dir(0,0,0);
      for(int j=0;j<pointToFaceInds[i].size();j++) {
        int polind=pointToFaceInds[i][j];
        int polptind=pointToFacePoss[i][j];
        int n=polygons[polind].size();
        int othptind=polygons[polind][(polptind+1)%n];
        Vector3d diff=(pointList[i] - pointList[othptind]).normalized();
        dir=dir+diff;
      }
      dir.normalize();
      printf("x %f y %f z %f\n",dir[0],dir[1],dir[2]);
      pt_dir.push_back(dir);
    }

  }
  PolySet *ps_ov = new PolySet(3,true);
  for(int i=0;i<ps_tess->polygons.size();i++)
  {
    Polygon &pol = ps_tess->polygons[i];
    Vector3d p1=pol[0];
    Vector3d p2=pol[1];
    Vector3d p3=pol[2];
    Vector3d p21=(p2-p1)/this->n;
    Vector3d p31=(p3-p1)/this->n;
    Vector3d botlast,botcur, toplast, topcur;
    for(int j=0;j<this->n;j++) {
      botcur=p1 + p31*j;
      topcur=p1 + p31*(j+1);


      for(int k=0;k<this->n-j;k++) {
        if(k != 0) {
          toplast=topcur;
          topcur=topcur+p21;	
	  ov_add_poly(ps_ov,botcur, topcur, toplast,round);
	}
	botlast=botcur;
	botcur=botlast+p21;
	ov_add_poly(ps_ov,botlast, botcur, topcur,round);
      }	      
    }				 

  }
  return ps_ov;
}

