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

#include "DebugNode.h"
#include "module.h"
#include "ModuleInstantiation.h"
#include "Children.h"
#include "Parameters.h"
#include "printutils.h"
#include "io/fileutils.h"
#include "Builtins.h"
#include "handle_dep.h"
#include "PolySetBuilder.h"
#include "ManifoldGeometry.h"

#include <cmath>
#include <sstream>

#include <PolySetUtils.h>
#include <Tree.h>
#include <GeometryEvaluator.h>
#include <boost/functional/hash.hpp>
#include <hash.h>
#include "Renderer.h"

std::unique_ptr<const Geometry> DebugNode::createGeometry() const
{
  printf("a\n");	
  if(this->children.size() == 0) {
	printf("ee\n");
	return std::unique_ptr<PolySet>();
  }

  std::shared_ptr<AbstractNode> child=this->children[0];
  Tree tree(child, "");
  GeometryEvaluator geomevaluator(tree);
  std::shared_ptr<const Geometry> geom = geomevaluator.evaluateGeometry(*tree.root(), true);
  std::shared_ptr<const PolySet> ps=nullptr;
  auto mani = std::dynamic_pointer_cast<const ManifoldGeometry>(geom);
  if(mani != nullptr) ps=mani->toPolySet();
  else ps=std::dynamic_pointer_cast<const PolySet>(geom);
  if(ps != nullptr) {

    auto psx  = std::make_unique<PolySet>(ps->getDimension(), ps->convexValue());	  
    psx->vertices=ps->vertices;
    psx->indices=ps->indices;
    psx->mat=ps->mat;
    psx->matind=ps->matind;

    int matind;
    Material matcolor;
    if(psx->matind.size() > psx->indices.size()) {
      auto cs = ColorMap::inst()->defaultColorScheme();
      matcolor.color = ColorMap::getColor(cs, RenderColor::OPENCSG_FACE_FRONT_COLOR);
      matind = psx->mat.size();
      psx->mat.push_back(matcolor);    
      while(psx->matind.size() < psx->indices.size()) {
        psx->matind.push_back(matind);	  
      }
    }
    matcolor.color = Color4f(1,0,0,1);
    matind = psx->mat.size();
    psx->mat.push_back(matcolor);    
    for(int i=0;i<this->faces.size();i++) {
	    int ind=this->faces[i];
	    if(ind >= 0 && ind<psx->matind.size())
		    psx->matind[ind] = matind;
    }
    for(int i=0;i<psx->mat.size();i++)
	    printf("%g/%g/%g/%g\n",psx->mat[i].color[0],psx->mat[i].color[1],psx->mat[i].color[2],psx->mat[i].color[3]);
    for(int i=0;i<psx->matind.size();i++) printf("%d ",psx->matind[i]);
    printf("\n");

    return psx;
  }
  return std::unique_ptr<PolySet>();
}

