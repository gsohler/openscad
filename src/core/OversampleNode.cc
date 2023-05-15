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

const Geometry *OversampleNode::createGeometry() const
{
  // tesselate object
  PolySet *ps_tess = new PolySet(3,true);
  if(this->children.size() > 0) {
    std::shared_ptr<AbstractNode> child=this->children[0];
    LeafNode *node = (LeafNode *)   child.get();
    const Geometry *geom = node->createGeometry();
    if (const auto *ps = dynamic_cast<const PolySet *>(geom)) {
      PolySetUtils::tessellate_faces(*ps, *ps_tess);
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
          ps_ov->append_poly();
          ps_ov->append_vertex(botcur[0],botcur[1],botcur[2]);
          ps_ov->append_vertex(topcur[0],topcur[1],topcur[2]);
          ps_ov->append_vertex(toplast[0],toplast[1],toplast[2]);
	}
	botlast=botcur;
	botcur=botlast+p21;
        ps_ov->append_poly();
        ps_ov->append_vertex(botlast[0],botlast[1],botlast[2]);
        ps_ov->append_vertex(botcur[0],botcur[1],botcur[2]);
        ps_ov->append_vertex(topcur[0],topcur[1],topcur[2]);
      }	      
    }				 

  }
  return ps_ov;
}

