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

std::unordered_map<Vector3d, Vector3d, boost::hash<Vector3d> > weldMap; // TODO nicht global

void ov_add_poly_round(PolySet *ps, Vector3d p,const Vector3d & center,  double r, int round, int orgpt)
{
  if(round && !orgpt) {
    Vector3d diff=p-center;
    diff.normalize();
    Vector3d pnew=center+diff*r;
    if(weldMap.count(p) == 0) weldMap[p] = pnew;
    else weldMap[p]=(weldMap[p]+pnew)/2.0;
	
  }
  ps->append_vertex(p[0],p[1],p[2]);
}

double roundCoord(double c) {
	if(c > 0) return (int)(c*1000+0.5)/1000.0;
	else  return (int)(c*1000-0.5)/1000.0;
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
  std::unordered_map<Vector3d, int, boost::hash<Vector3d> > pointIntMap;
  if(this->round == 1) {
    // create indexed point list
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
        pt[0]=roundCoord(pt[0]);
        pt[1]=roundCoord(pt[1]);
        pt[2]=roundCoord(pt[2]);
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
    double r=1.0;
    Vector3d center(0,0,0);
    if(this->round ==1) {
      Vector3d gravity=(p1+p2+p3)/3.0; // schwerpunkt im dreieck ausrechnen // TODO umkreismittelpunkt
      Vector3d cutmean(0,0,0);				 
      int results=0;
      for(int j=0;j<3;j++) { // for all 3 edges
        Vector3d vec=pol[j]-gravity; // ebene vec, pol[j]
        Vector3d cut;
        int ptind=pointIntMap[pol[j]]; 
        if(!cut_face_line(gravity, vec, pol[j],pt_dir[ptind],cut,NULL)) {
         cutmean = cutmean + cut;	      
         results++;

        }
      }	    
      cutmean = cutmean *(1.0/results);
      Vector3d facen=(p2-p1).cross(p3-p1).normalized();
      double dist=(gravity-cutmean).dot(facen);
      center = gravity - dist*facen*3; // TODO 5 weg
      r=(center-p1).norm();
      // distance to face
//      center=cutmean;
    }  
  // mitteln
  // TODO alle fehlerfaelle finden
  // TODO naehte veswchweissen, 
  // TODO overround
    for(int j=0;j<this->n;j++) {
      botcur=p1 + p31*j;
      topcur=p1 + p31*(j+1);


      for(int k=0;k<this->n-j;k++) {
        if(k != 0) {
          toplast=topcur;
          topcur=topcur+p21;	
          ps_ov->append_poly();
          ov_add_poly_round(ps_ov, botcur,center, r, round, 0 );
          ov_add_poly_round(ps_ov, topcur,center, r, round , 0);
          ov_add_poly_round(ps_ov, toplast,center, r, round , 0);
	}
	botlast=botcur;
	botcur=botlast+p21;
        ps_ov->append_poly();
        ov_add_poly_round(ps_ov, botlast,center, r, round, j == 0 && k == 0 );
        ov_add_poly_round(ps_ov, botcur,center, r, round, j == 0 && k == this->n-1 );
        ov_add_poly_round(ps_ov, topcur,center, r, round, j == this->n-1 );
      }	      
    }				 

  }
  for(int i=0;i<ps_ov->polygons.size();i++) {
    for(int j=0;j<ps_ov->polygons[i].size();j++)
    {
       Vector3d  pt=ps_ov->polygons[i][j];	    
       if(weldMap.count(pt) > 0) {
	       ps_ov->polygons[i][j]=weldMap[pt];
       }
    }
  }
  return ps_ov;
}

