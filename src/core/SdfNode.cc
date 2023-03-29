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

#include "SdfNode.h"
#include "pylibfive.h"

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

#include "libfive.h"


using namespace libfive;

const Geometry *SdfNode::createGeometry() const
{
	auto p = new PolySet(3, true);
	PyObject *exp = this->expression;
	if(exp == NULL || exp->ob_type != &PyLibFiveType) 
		return p;
	libfive_tree tree = PyLibFiveObjectToTree(exp);
	printf("tree: %s\n",libfive_tree_print(tree)); // TODO free all tree

	libfive_region3 reg;
	reg.X.lower = -10; // TODO parameter
	reg.X.upper =  10;
	reg.Y.lower = -10;
	reg.Y.upper =  10;
	reg.Z.lower = -10;
	reg.Z.upper =  10;

	libfive_mesh *mesh = libfive_tree_render_mesh(tree,  reg, 2); // TODO understand 10
	printf("t %d %d\n",mesh->tri_count, mesh->vert_count);
	libfive_tri t;
	for(int i=0;i<mesh->tri_count;i++) 
	{
		t = mesh->tris[i];
		p->append_poly(); 
		p->append_vertex(mesh->verts[t.a].x, mesh->verts[t.a].y, mesh->verts[t.a].z );
		p->append_vertex(mesh->verts[t.b].x, mesh->verts[t.b].y, mesh->verts[t.b].z );
		p->append_vertex(mesh->verts[t.c].x, mesh->verts[t.c].y, mesh->verts[t.c].z );
	}
	libfive_mesh_delete(mesh);

/*
	libfive_tree_delete(out);
*/

	return p;
}
