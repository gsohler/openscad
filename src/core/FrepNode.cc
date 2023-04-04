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

#include "FrepNode.h"
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

#include "libfive/render/brep/dc/dc_mesher.hpp"
#include "libfive/render/brep/dc/dc_worker_pool.hpp"
#include "libfive/render/brep/region.hpp"
#include "libfive/render/brep/mesh.hpp"

#include <hash.h>
#include <unordered_set>


using namespace libfive;

const Geometry *FrepNode::createGeometry() const
{
	auto p = new PolySet(3, true);
	PyObject *exp = this->expression;
	std::unique_ptr<Mesh> mesh;
	if(exp == NULL ) return p;
	libfive::Region<3> reg(
			{this->x1, this->y1, this->z1}, 
			{this->x2, this->y2, this->z2});
	libfive::BRepSettings settings;
	settings.workers = 1;
	settings.min_feature = 1.0 / this->res;

	if(exp->ob_type == &PyLibFiveType) {
		libfive::Tree *tree = PyLibFiveObjectToTree(exp);
//		printf("tree: %s\n",libfive_tree_print(tree)); 
                mesh = Mesh::render(*tree, reg ,settings);
	} else if(exp->ob_type == &PyFunction_Type) {
		printf("Python Function!\n");
		mesh = NULL;
	} else { printf("xxx\n"); }
	libfive_tri t;
	// TODO libfive trees mergen
	for (const auto& t : mesh->branes)
	{
		p->append_poly(); 
		p->append_vertex(mesh->verts[t[0]].x(), mesh->verts[t[0]].y(), mesh->verts[t[0]].z() );
		p->append_vertex(mesh->verts[t[1]].x(), mesh->verts[t[1]].y(), mesh->verts[t[1]].z() );
		p->append_vertex(mesh->verts[t[2]].x(), mesh->verts[t[2]].y(), mesh->verts[t[2]].z() );
	}

	return p;
}


// polyset to SDF converter

void convertToIndex(const PolySet *ps, std::vector<Vector3d> &pointList,  std::vector<intList> &polygons,std::vector<intList>  &pointToFaceInds)
{
  std::unordered_map<Vector3d, int, boost::hash<Vector3d> > pointIntMap;
  intList emptyList;
//  printf("polygons\n");
  for(int i=0;i<ps->polygons.size();i++) {
    Polygon pol = ps->polygons[i];
    intList polygon;
    for(int j=0;j<pol.size(); j++) {
      int ptind=0;
      Vector3d  pt=pol[j];
      if(!pointIntMap.count(pt))
      {
        pointList.push_back(pt);
        pointToFaceInds.push_back(emptyList);
        ptind=pointList.size()-1;
        pointIntMap[pt]=ptind;
      } else ptind=pointIntMap[pt];
//      printf("%d ",ptind);
      polygon.push_back(ptind);
      pointToFaceInds[ptind].push_back(i);
    }
//    printf("\n");
    polygons.push_back(polygon);
  }

}

int operator==(const CutFace &a, const CutFace &b)
{
        if(a.a != b.a) return 0;
        if(a.b != b.b) return 0;
        if(a.c != b.c) return 0;
        if(a.d != b.d) return 0;
        return 1;
}

unsigned int hash_value(const CutFace& r) {
	double x=r.a+10*r.b+100*r.c+1000*r.d;
        unsigned int i;
	i=*((int *) &x);
        return i;
}

std::vector<CutFace> calculateEdgeFaces( std::vector<Vector3d> &pointList,std::vector<intList> &polygons, std::vector<intList>  &pointToFaceInds, std::vector<CutFace> &normfaces)
{
  std::vector<CutFace> edgeFaces;
  std::unordered_set<CutFace, boost::hash<CutFace> > edgeFacePresent;
  for(int i=0;i<polygons.size();i++)
  { 
    intList &poly = polygons[i];
    int n=poly.size();
    if(n < 3) continue;
    // normalvektor
    Vector3d p1, p2, p3;
    p1=pointList[poly[0]];
    p2=pointList[poly[1]];
    p3=pointList[poly[2]];
    Vector3d nb=(p2-p1).cross(p3-p1).normalized();
    CutFace cf;
    cf.a=nb[0];
    cf.b=nb[1];
    cf.c=nb[2];
    cf.d=-(cf.a*p1[0]+cf.b*p1[1]+cf.c*p1[2]);
    normfaces.push_back(cf);
//    printf("nb %.2g/%.2g/%.2g\n",nb[0],nb[1],nb[2]);
    for(int j=0;j<n;j++)
    {
      // find adajacent face
      int ind1=poly[j];
      int ind2=poly[(j+1)%n];
      int faceindfound=-1;
      for(int k=0;k<pointToFaceInds[ind1].size();k++) {
        int faceind=pointToFaceInds[ind1][k];
	if(faceind == i) continue;
	for(int l=0;l<polygons[faceind].size();l++) {
	  if(polygons[faceind][l] == ind2) faceindfound=faceind;
	}
      }
      if(faceindfound == -1) continue;
      intList &opoly = polygons[faceindfound];
      p1=pointList[opoly[0]];
      p2=pointList[opoly[1]];
      p3=pointList[opoly[2]];
      Vector3d no=(p2-p1).cross(p3-p1).normalized();
      
      // create edge face
      p1=nb.cross(no);
      p2=no+nb;
      p3=p2.cross(p1).normalized();
      p1=pointList[ind1];
      cf.a=p3[0];
      cf.b=p3[1];
      cf.c=p3[2];
      cf.d=-(cf.a*p1[0]+cf.b*p1[1]+cf.c*p1[2]);
      int swap=0;
     if(cf.a < 0) swap=1;
     if(cf.a == 0) {
	if(cf.b < 0) swap=1;
	if(cf.b == 0) {
          if(cf.c < 0) swap=1;
	}
     }
     if(swap) { cf.a =-cf.a; cf.b =-cf.b; cf.c =-cf.c; cf.d =-cf.d; }
//      printf("nf\t%.2g\t%.2g\t%.2g\t%.2g\n",cf.a, cf.b, cf.c, cf.d);
      if(!edgeFacePresent.count(cf)) { 
      	edgeFaces.push_back(cf);
	edgeFacePresent.insert(cf);
      } 
    }
  }
  return edgeFaces;
}
struct ProgramState
{
	intList validFaces;
	int state=0;
	int resultind;
};

std::vector<ProgramState> programStack;


int generateProgram(intList &table, std::vector<CutProgram> &program,std::vector<CutFace> &edgeFaces, int faces, intList &validFaces) 
{
	std::vector<int> validPos, validNeg;
	int i,j,v;
	CutProgram cp;
	// find out , which row has most equal balance between + and -
	int rate,ratebest=-1,edgebest=-1;
	int edgefaces = edgeFaces.size();
	int valid_size = validFaces.size();
//	printf("generateProgram round=%d\n",generateRound++);
	for(i=0;i<edgefaces;i++) {
		int poscount=0;
		int negcount=0;
		for(j=0;j<valid_size;j++)
		{
			v=table[i*faces+validFaces[j]];

			if(v == 1) poscount++;
			if(v == -1) negcount++;
		}
		rate=poscount<negcount?poscount:negcount;
		if(rate > ratebest) {
			ratebest=rate;
			edgebest=i;
		}

	}
	if(edgebest == -1) {
		printf("Program Error!\n");
		exit(1);
	}
	cp.a=edgeFaces[edgebest].a;
	cp.b=edgeFaces[edgebest].b;
	cp.c=edgeFaces[edgebest].c;
	cp.d=edgeFaces[edgebest].d;
	cp.posbranch=~1;
	cp.negbranch=~1; // TODO fix

	// split into positive and negative branch
	for(int i=0;i<validFaces.size();i++)
	{
		switch(table[edgebest*faces+validFaces[i]])
		{
			case 1:
				validPos.push_back(validFaces[i]);
				break;
			case 0:
				validPos.push_back(validFaces[i]);
				validNeg.push_back(validFaces[i]);
				break;
			case -1:
				validNeg.push_back(validFaces[i]);
				break;
		}
	}

	int startind=program.size();
	program.push_back(cp);

	if(validPos.size() == validFaces.size())
	{
		program[startind].posbranch = ~(validPos[0]);
		program[startind].negbranch = ~(validPos[0]);
		return startind;
	}

	if(validNeg.size() == validFaces.size())
	{
		program[startind].posbranch = ~(validNeg[0]);
		program[startind].negbranch = ~(validNeg[0]);
		return startind;
	}


	if(validNeg.size() > 1) {
  		ProgramState state;
		state.validFaces=validNeg;	
		state.resultind=~startind;
		programStack.push_back(state);
	} else {
		program[startind].negbranch = ~(validNeg[0]);
	}

	if(validPos.size() > 1) {
  		ProgramState state;
		state.validFaces=validPos;
		state.resultind=startind;
		programStack.push_back(state);
	} else {
		program[startind].posbranch = ~(validPos[0]);
	}
	return startind;
}

int generateProgramFlat(intList &table, std::vector<CutProgram> &program, std::vector<CutFace> &edgeFaces, int faces, std::vector<ProgramState> &stack) 
{
	printf("fglat\n");
	while(1)
	{
		if(stack.size() == 0) return 0;
		ProgramState state = stack[stack.size()-1];
		stack.pop_back();

		int result=generateProgram(table ,program, edgeFaces, faces,state.validFaces); // create recursive program
		if(state.resultind == 0x80000000) continue;
		if(state.resultind >= 0) program[state.resultind].posbranch = result;		
		else program[~(state.resultind)].negbranch = result;

	}
	return 0;
}

double evaluateProgram(std::vector<CutProgram> &program,int ind,std::vector<CutFace> &normFaces, double x,double y, double z)
{
	double e;
	int nextind;
	while(1) {
		CutProgram &prg = program[ind];
		e=prg.a*x+prg.b*y+prg.c*z+prg.d;
		if(e >= 0) nextind=prg.posbranch; else nextind=prg.negbranch;
		if(nextind < 0) {
			CutFace cf = normFaces[~nextind];
			double d=cf.a*x+cf.b*y+cf.c*z+cf.d;
			return d;
		}
		ind=nextind;
	}

	return 0;
}



// Libfive Oracle interface
OpenSCADOracle::OpenSCADOracle( const std::vector<CutProgram> &program, const std::vector<CutFace> &normFaces):program(program), normFaces(normFaces)
{
        // Nothing to do here
}

void OpenSCADOracle::evalInterval(libfive::Interval& out) {
	// Just pick a big ambiguous value.
	out = {-10000.0, 10000.0};
}

int evalCalled=0;
void OpenSCADOracle::evalPoint(float& out, size_t index) {
        const auto pt = points.col(index);
	evalCalled++;
        out = evaluateProgram(this->program,0,this->normFaces, pt.x(), pt.y(), pt.z());
}

void OpenSCADOracle::checkAmbiguous( Eigen::Block<Eigen::Array<bool, 1, LIBFIVE_EVAL_ARRAY_SIZE>, 1, Eigen::Dynamic> /* out */)
{
}

void OpenSCADOracle::evalFeatures(boost::container::small_vector<libfive::Feature, 4>& out) {
        const float EPSILON = 1e-6;
        float center, dx, dy, dz;

        Eigen::Vector3f before = points.col(0);
        evalPoint(center);

        points.col(0) = before + Eigen::Vector3f(EPSILON, 0.0, 0.0);
        evalPoint(dx);

        points.col(0) = before + Eigen::Vector3f(0.0, EPSILON, 0.0);
        evalPoint(dy);

        points.col(0) = before + Eigen::Vector3f(0.0, 0.0, EPSILON);
        evalPoint(dz);

        points.col(0) = before;

        out.push_back(Eigen::Vector3f(
            (dx - center) / EPSILON,
            (dy - center) / EPSILON,
            (dz - center) / EPSILON));
}


PyObject *ifrep(const PolySet *ps)
{
  printf("ifrep\n");
  std::vector<Vector3d> pointList; // list of all the points in the object
  std::vector<intList> polygons; // list polygons represented by indexes
  std::vector<intList>  pointToFaceInds; //  mapping pt_ind -> list of polygon inds which use it

  convertToIndex(ps,pointList, polygons,pointToFaceInds); // index umwandeln
  printf("%d Faces, %d Points\n",polygons.size(),pointList.size());

  std::vector<CutFace> edgeFaces;
  std::vector<CutFace> normFaces;
  edgeFaces = calculateEdgeFaces(pointList, polygons,pointToFaceInds,normFaces);
  printf("%d EdgeFaces generated\n",edgeFaces.size());

  std::vector<int> table; // x(0) dimenstion faces y(1) dimenions edgefas
  for(int i=0;i<edgeFaces.size();i++) // create table
  {
    CutFace &ef = edgeFaces[i];
    for(int j=0;j<polygons.size();j++)
    {
      intList &poly=polygons[j];
      int poscount=0, negcount=0;
      for(int k=0;k<poly.size(); k++)
      {
	      Vector3d pt=pointList[poly[k]];
	      double e=ef.a*pt[0]+ef.b*pt[1]+ef.c*pt[2]+ef.d;
	      if(e > 0.00001) poscount++;
	      if(e <  -0.00001) negcount++;
      }      
      if(poscount > 0 && negcount == 0) table.push_back(1);
      else if(poscount == 0 && negcount > 0) table.push_back(-1);
      else table.push_back(0); 
    }	    
  }	  
  printf("Table generated\n");


  ProgramState state;
  for(int i=0;i<polygons.size();i++) state.validFaces.push_back(i); 
  state.state=0;
  state.resultind=0x80000000;
  programStack.push_back(state); // initially all the work
  std::vector<CutProgram> program;
  int startind=generateProgramFlat(table ,program,edgeFaces, polygons.size(),programStack); // create recursive program
  for(int i=0;i<program.size();i++) {
	printf("%d\t%.3f\t%.3f\t%.3f\t%.3f\tP:%d\tN:%d\n",i,program[i].a,program[i].b,program[i].c,program[i].d,program[i].posbranch, program[i].negbranch);
  }
  /*
  printf("dist=%f\n",evaluateProgram(program,startind,normFaces, 0.5,0.5,1.5));
  printf("dist=%f\n",evaluateProgram(program,startind,normFaces, 0.5,0.5,-0.5));
  printf("dist=%f\n",evaluateProgram(program,startind,normFaces, 1.5,0.5,0.5));
  printf("dist=%f\n",evaluateProgram(program,startind,normFaces, -0.5,0.5,0.5));
  printf("dist=%f\n",evaluateProgram(program,startind,normFaces, 0.5,1.5,0.5));
  printf("dist=%f\n",evaluateProgram(program,startind,normFaces, 0.5,-0.5,0.5));
  printf("dist=%f\n",evaluateProgram(program,startind,normFaces, 0.5,0.5,0.9));
*/ 
  libfive::Tree oc = libfive::Tree(std::unique_ptr<OracleClause>(new OpenSCADOracleClause(program, normFaces)));
  printf("eval Called is %d\n",evalCalled);
  evalCalled=0;
  return PyLibFiveObjectFromTree(&PyLibFiveType,new Tree(oc));		  
}

