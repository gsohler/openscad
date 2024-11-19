/*Copyright(c) 2018, slugdev
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met :
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
must display the following acknowledgement :
This product includes software developed by slugdev.
4. Neither the name of the slugdev nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY SLUGDEV ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED.IN NO EVENT SHALL SLUGDEV BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

#include "StepKernel.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <sstream>
#include <map>
#include <iomanip> // put_time
StepKernel::StepKernel()
{

}

StepKernel::~StepKernel()
{
}

StepKernel::EdgeCurve* StepKernel::create_edge_curve(StepKernel::Vertex * vert1, StepKernel::Vertex * vert2,bool dir)
{
	// curve 1
	auto line_point1 = new Point(entities, vert1->point->pt);
	Vector3d v = (vert2->point->pt - vert1->point->pt).normalized();

	auto line_dir1 = new Direction(entities, v);
	auto line_vector1 = new Vector(entities, line_dir1, 1.0);
	auto line1 = new Line(entities, line_point1, line_vector1);
	auto surf_curve1 = new SurfaceCurve(entities, line1);
	return new EdgeCurve(entities, vert1, vert2, surf_curve1, dir);
}

void StepKernel::build_tri_body(std::vector<Vector3d> vertices, std::vector<IndexedFace> faces, double tol)
{
	auto point = new Point(entities, Vector3d(0.0, 0.0, 0.0));
	auto dir_1 = new Direction(entities, Vector3d(0.0, 0.0, 1.0));
	auto dir_2 = new Direction(entities, Vector3d(1.0, 0.0, 0.0));

	auto base_csys = new Csys3D(entities, dir_1, dir_2, point);
	std::vector<Face*> sfaces;
	std::map<std::tuple<double, double, double, double, double, double>, EdgeCurve*> edge_map;
	for (std::size_t i = 0; i < faces.size() ; i++)
	{
		Vector3d p0 = vertices[faces[i][0]];
		Vector3d p1 = vertices[faces[i][1]];
		Vector3d p2 = vertices[faces[i][2]];

		Vector3d d0=(p1-p0).normalized();
		Vector3d d1=(p2-p0).normalized();
		Vector3d d2=d0.cross(d1).normalized();
		double dist2 = sqrt(d2[0] * d2[0] + d2[1] * d2[1] + d2[2] * d2[2]);
		Vector3d d1_cor=d0.cross(d2).normalized();

		int merged_edge_cnt;
		// build the face
		int n=faces[i].size();
		std::vector<StepKernel::Vertex *> vert;
		for(int j=0;j<n;j++){
			int ind=faces[i][j];
			auto point = new Point(entities, vertices[ind]);
			auto v = new Vertex(entities, point);
			vert.push_back(v);
		}

		std::vector<OrientedEdge*> oriented_edges;
		for(int j=0;j<n;j++)
		{
			int ind=faces[i][j];
			int indn=faces[i][(j+1)%n];
			EdgeCurve* edge_curve;
			bool edge_dir = true; 
			get_edge_from_map(vertices[ind], vertices[indn], edge_map, vert[j], vert[(j+1)%n], edge_curve, edge_dir, merged_edge_cnt);
			oriented_edges.push_back(new OrientedEdge(entities, edge_curve, edge_dir));
		}

//https://www.kleinanzeigen.de/s-anzeige/schlittschuhe-kinder-groesse-25-29/2925176078-230-6425
		// create the plane
		auto plane_point = new Point(entities, p0);
		auto plane_dir_1 = new Direction(entities, d2);
		auto plane_dir_2 = new Direction(entities, d0);
		auto plane_csys = new Csys3D(entities, plane_dir_1, plane_dir_2, plane_point);
		auto plane = new Plane(entities, plane_csys);

		// build the faces

		auto edge_loop = new EdgeLoop(entities, oriented_edges);
		std::vector<FaceBound*> face_bounds;
		face_bounds.push_back(new FaceBound(entities, edge_loop, true));
		sfaces.push_back(new Face(entities, face_bounds, plane, true));
	}

	// build the model
	auto open_shell = new Shell(entities, sfaces);
	std::vector<Shell*> shells;
	shells.push_back(open_shell);
	auto shell_model = new ShellModel(entities, shells);
	auto manifold_shape = new ManifoldShape(entities, base_csys, shell_model);
}

void StepKernel::get_edge_from_map(
	Vector3d p0,
	Vector3d p1,
	std::map<std::tuple<double, double, double, double, double, double>, StepKernel::EdgeCurve *> &edge_map,
	StepKernel::Vertex * vert1,
	StepKernel::Vertex * vert2,
	EdgeCurve *& edge_curve,
	bool &edge_dir,
	int &merge_cnt)
{
	edge_curve = 0;
	edge_dir = true;
	auto edge_tuple1_f = std::make_tuple(p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]);
	auto edge_tuple1_r = std::make_tuple(p1[0], p1[1], p1[2], p0[0], p0[1], p0[2]);
	if (edge_map.count(edge_tuple1_f))
	{
		edge_curve = edge_map[edge_tuple1_f];
		edge_dir = true;
		merge_cnt++;
	}
	else if (edge_map.count(edge_tuple1_r))
	{
		edge_curve = edge_map[edge_tuple1_r];
		edge_dir = false;
		merge_cnt++;
	}
	if (!edge_curve)
	{
		edge_curve = create_edge_curve(vert1, vert2, true);
		edge_map[edge_tuple1_f] = edge_curve;
	}
}

std::string StepKernel::read_line(std::ifstream &stp_file, bool skip_all_space)
{
	std::string line_str;
	bool leading_space = true;
	while (stp_file)
	{
		char get_char = ' ';
		stp_file.get(get_char);
		if (get_char == ';')
			break;

		if (get_char == '\n' || get_char == '\r' || get_char == '\t')
			continue;

		if (leading_space && (get_char == ' ' || get_char == '\t'))
			continue;
		if (!skip_all_space)
			leading_space = false;
		line_str.push_back(get_char);
	}
	return line_str;
}

void StepKernel::read_step(std::string file_name)
{
	std::ifstream stp_file;
	stp_file.open(file_name);
	if (!stp_file)
		return;

	// read the first line to get the iso stuff
	std::string iso_line = read_line(stp_file, true);

	bool data_section = false;
	std::vector<Entity*> ents;
	std::map<int,Entity*> ent_map;
	std::vector<std::string> args;
	while (stp_file)
	{
		std::string cur_str = read_line(stp_file, false);
		if (cur_str == "DATA")
		{
			data_section = true;
			continue;
		}
		if (!data_section)
			continue;

		if (cur_str == "ENDSEC")
		{
			data_section = false;
			break;
		}
		// parse the id
		int id = -1;
		if (cur_str.size() > 0 && cur_str[0] == '#' && cur_str.find('='))
		{
			auto equal_pos = cur_str.find('=');
			auto paren_pos = cur_str.find('(');
			auto id_str = cur_str.substr(1, equal_pos - 1);
			id = std::atoi(id_str.c_str());
			auto func_start = cur_str.find_first_not_of("\t ", equal_pos+1);
			auto func_end = cur_str.find_first_of("\t (", func_start + 1);
			auto func_name = cur_str.substr(func_start, func_end - func_start);
			bool unimplemented = false;

			// now parse the args
			auto arg_end = cur_str.find_last_of(')');
			auto arg_start = cur_str.find_first_not_of("\t (",func_end+1);
			auto arg_str = cur_str.substr(arg_start, arg_end - arg_start);
			Entity* ent = 0;
			if (func_name == "CARTESIAN_POINT")
				ent = new Point(entities);
			else if (func_name == "DIRECTION")
				ent = new Direction(entities);
			else if (func_name == "AXIS2_PLACEMENT_3D")
				ent = new Csys3D(entities);
			else if (func_name == "PLANE")
				ent = new Plane(entities);
			else if (func_name == "EDGE_LOOP")
				ent = new EdgeLoop(entities);
			else if (func_name == "FACE_BOUND")
				ent = new FaceBound(entities);
			else if (func_name == "FACE_OUTER_BOUND")
				ent = new FaceBound(entities);
			else if (func_name == "ADVANCED_FACE")
				ent = new Face(entities);
			else if (func_name == "FACE_SURFACE")
				ent = new Face(entities);
			else if (func_name == "OPEN_SHELL")
				ent = new Shell(entities);
			else if (func_name == "CLOSED_SHELL")
				ent = new Shell(entities);
			else if (func_name == "SHELL_BASED_SURFACE_MODEL")
				ent = new ShellModel(entities);
			else if (func_name == "MANIFOLD_SURFACE_SHAPE_REPRESENTATION")
				ent = new ManifoldShape(entities);
			else if (func_name == "MANIFOLD_SOLID_BREP")
				ent = new ManifoldSolid(entities);
			else if (func_name == "VERTEX_POINT")
				ent = new Vertex(entities);
			else if (func_name == "SURFACE_CURVE")
				ent = new SurfaceCurve(entities);
			else if (func_name == "EDGE_CURVE")
				ent = new EdgeCurve(entities);
			else if (func_name == "ORIENTED_EDGE")
				ent = new OrientedEdge(entities);
			else if (func_name == "VECTOR")
				 ent = new Vector(entities);
			else if (func_name == "LINE")
				 ent = new Line(entities);
			else if (func_name == "CIRCLE")
				 ent = new Circle(entities);
			else if (func_name == "CYLINDRICAL_SURFACE")
				 ent = new CylindricalSurface(entities);
			else if (func_name == "PCURVE") unimplemented=true;
			else if (func_name == "DEFINITIONAL_REPRESENTATION") unimplemented=true;
			else if (func_name == "UNCERTAINTY_MEASURE_WITH_UNIT") unimplemented=true;
			else if (func_name == "PRODUCT_TYPE") unimplemented=true;
			else if (func_name == "APPLICATION_PROTOCOL_DEFINITION") unimplemented=true;
			else if (func_name == "APPLICATION_CONTEXT") unimplemented=true;
			else if (func_name == "SHAPE_DEFINITION_REPRESENTATION") unimplemented=true;
			else if (func_name == "PRODUCT") unimplemented=true;
			else if (func_name == "PRODUCT_DEFINITION_SHAPE") unimplemented=true;
			else if (func_name == "PRODUCT_DEFINITION") unimplemented=true;
			else if (func_name == "PRODUCT_DEFINITION_FORMATION") unimplemented=true;
			else if (func_name == "MECHANICAL_CONTEXT") unimplemented=true;
			else if (func_name == "PRODUCT_DEFINITION_CONTEXT") unimplemented=true;
			else if (func_name == "ADVANCED_BREP_SHAPE_REPRESENTATION") unimplemented=true;
			else if (func_name == "PERSON") unimplemented=true;
			else if (func_name == "DATE_TIME_ROLE") unimplemented=true;
			else if (func_name == "LOCAL_TIME") unimplemented=true;
			else if (func_name == "APPROVAL_ROLE") unimplemented=true;
			else if (func_name == "APPROVAL") unimplemented=true;
			else if (func_name == "COORDINATED_UNIVERSAL_TIME_OFFSET") unimplemented=true;
			else if (func_name == "CC_DESIGN_PERSON_AND_ORGANIZATION_ASSIGNMENT") unimplemented=true;
			else if (func_name == "DATE_AND_TIME") unimplemented=true;
			else if (func_name == "APPROVAL_DATE_TIME") unimplemented=true;
			else if (func_name == "SECURITY_CLASSIFICATION_LEVEL") unimplemented=true;
			else if (func_name == "APPROVAL_STATUS") unimplemented=true;
			else if (func_name == "CC_DESIGN_APPROVAL") unimplemented=true;
			else if (func_name == "ORGANIZATION") unimplemented=true;
			else if (func_name == "PERSON_AND_ORGANIZATION") unimplemented=true;
			else if (func_name == "CALENDAR_DATE") unimplemented=true;
			else if (func_name == "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE") unimplemented=true;
			else if (func_name == "PERSON_AND_ORGANIZATION_ROLE") unimplemented=true;
			else if (func_name == "PRODUCT_RELATED_PRODUCT_CATEGORY") unimplemented=true;
			else if (func_name == "CC_DESIGN_DATE_AND_TIME_ASSIGNMENT") unimplemented=true;
			else if (func_name == "SECURITY_CLASSIFICATION") unimplemented=true;
			else if (func_name == "APPROVAL_PERSON_ORGANIZATION") unimplemented=true;
			else if (func_name == "DESIGN_CONTEXT") unimplemented=true;
			else if (func_name == "CC_DESIGN_SECURITY_CLASSIFICATION") unimplemented=true;
			else if (func_name == "(") unimplemented=true;
/*
 * * 
#149 = CLOSED_SHELL ( 'NONE', ( #155, #141, #100, #147 ) ) ;
 #147 = ADVANCED_FACE ( 'NONE', ( #143 ), #43, .F. ) ;
  #143 = FACE_OUTER_BOUND ( 'NONE', #55, .T. ) ;
   #55 = EDGE_LOOP ( 'NONE', ( #156, #13 ) ) ;
    #13 = ORIENTED_EDGE ( 'NONE', *, *, #31, .F. ) ;
    #14 = ORIENTED_EDGE ( 'NONE', *, *, #31, .T. ) ;
     #31 = EDGE_CURVE ( 'NONE', #71, #129, #28, .T. ) ;
      #71 = VERTEX_POINT ( 'NONE', #82 ) ;
       #82 = CARTESIAN_POINT ( 'NONE',  ( 6.123233995736766085E-15, 0.000000000000000000, 50.00000000000000000 ) ) ;
      #129 = VERTEX_POINT ( 'NONE', #101 ) ;
        CARTESIAN_POINT ( 'NONE',  ( 0.000000000000000000, 0.000000000000000000, -50.00000000000000000 ) ) ;
     
      #28 = CIRCLE ( 'NONE', #113, 50.00000000000000000 ) ;
       #113 = AXIS2_PLACEMENT_3D ( 'NONE', #107, #167, #96 ) ;
        #107 = CARTESIAN_POINT ( 'NONE',  ( 0.000000000000000000, 0.000000000000000000, 0.000000000000000000 ) ) ;
        #167 = DIRECTION ( 'NONE',  ( 0.000000000000000000, 1.000000000000000000, 0.000000000000000000 ) ) ;
        #96 = DIRECTION ( 'NONE',  ( 0.000000000000000000, 0.000000000000000000, 1.000000000000000000 ) ) ;
    


#148 = CYLINDRICAL_SURFACE ( 'NONE', #18, 50.00000000000000000 ) ;	
 #18 = AXIS2_PLACEMENT_3D ( 'NONE', #49, #19, #163 ) ;
* 
 */


			if(!ent) {
				if(unimplemented) {
				  ent = new Line(entities); // TODO fix
				} else {
				  printf("Unknown Type %s\n",func_name.c_str());
				  ent = new Line(entities);
				}  
			}

			if (ent)
			{
				ent->id = id;
				ent_map[id] = ent;
				ents.push_back(ent);
				args.push_back(arg_str);
			}
		}
//		std::cout << cur_str << "\n";
	}
	// processes all the arguments
	for (int i = 0; i < ents.size(); i++)
	{
		ents[i]->parse_args(ent_map,args[i]);
	}
	stp_file.close();
	this->entities=ents;
}
