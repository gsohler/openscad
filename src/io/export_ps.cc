/*
 *  OpenSCAD (www.openscad.org)
 *  Copyright (C) 2021      Konstantin Podsvirov <konstantin@podsvirov.pro>
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

#include "export.h"

#include "PolySet.h"
#include "ManifoldGeometry.h"
#include "CGALHybridPolyhedron.h"
#include "CGAL_Nef_polyhedron.h"
#include "cgalutils.h"
#include "PolySetUtils.h"
#include <unordered_map>
#include "boost-utils.h"
#include <hash.h>


typedef struct
{
 Vector2d pt;
 double rot;
 char text[10];
} labelS;


typedef struct
{
  std::vector<Vector2d> pt;
  int done;
} plateS;

typedef struct
{
  int p1, f1, p2, f2;		
  int done;
} connS;


Vector2d pointrecht(Vector2d x)
{
 Vector2d y;
 y[0]=x[1];
 y[1]=-x[0];
 return y;
}

int plot_try(int destplate,Vector2d px,Vector2d py,Vector2d pm,std::vector<plateS> &plate,std::vector<Vector3d> vertices, std::vector<IndexedFace> faces, std::vector<Vector2d> &lines,double &xofs,double &yofs,double a4b,double a4h,double rand,int destedge,int bestend)
{
  int i;
  double xmax,xmin,ymax,ymin;
  Vector2d p1;
  int success=1;

  int n=faces[destplate].size();
  int i0=faces[destplate][(destedge+n-1)%n];
  int i1=faces[destplate][(destedge+0)%n];
  int i2=faces[destplate][(destedge+1)%n];
  Vector3d pt=vertices[i1];
  Vector3d xdir=(vertices[i2]-pt).normalized();
  Vector3d zdir=(xdir.cross(vertices[i0]-pt)).normalized();
  Vector3d ydir=(zdir.cross(xdir)).normalized();
  pt=(vertices[i1]+vertices[i2])/2;
  Matrix4d mat;
  mat <<  xdir[0], ydir[0], zdir[0], pt[0],
	  xdir[1], ydir[1], zdir[1], pt[1],
	  xdir[2], ydir[2], zdir[2], pt[2],
	  0      , 0      , 0      , 1;

  Matrix4d invmat = mat.inverse();	  

  plate[destplate].pt.clear();
  for(int i=0;i<n;i++) {
    Vector3d pt = vertices[faces[destplate][i]];
    Vector4d pt4(pt[0], pt[1], pt[2], 1);
    pt4 = invmat * pt4 ;
    plate[destplate].pt.push_back(px*pt4[0] + py*pt4[1]+pm);
  }
  for(i=0;i<n;i++) {
    lines.push_back(plate[destplate].pt[i]);
    lines.push_back(plate[destplate].pt[(i+1)%n]);
  }
  xmin=lines[0][0];
  xmax=lines[0][0];
  ymin=lines[0][1];
  ymax=lines[0][1];
  for(i=0;i<lines.size();i++)
  {
    if(lines[i][0] < xmin) xmin=lines[i][0];
    if(lines[i][0] > xmax) xmax=lines[i][0];
    if(lines[i][1] < ymin) ymin=lines[i][1];
    if(lines[i][1] > ymax) ymax=lines[i][1];
  }
  xofs=(a4b-xmax+xmin)/2.0-xmin;
  yofs=(a4h-ymax+ymin)/2.0-ymin;
  if(xofs+xmin < rand || yofs+ymin < rand) success=0 ;
  return success;
}

std::vector<IndexedFace> mergetriangles(const std::vector<IndexedFace> polygons,const std
::vector<Vector4d> normals,std::vector<Vector4d> &newNormals, const std::vector<Vector3d>
 &vert);

std::vector<Vector4d> offset3D_normals(const std::vector<Vector3d> &vertices, const std::vector<IndexedFace> &faces);


class EdgeDbStub
{
        public:
                int ind1, ind2, ind3 ;
                int operator==(const EdgeDbStub ref)
                {
                        if(this->ind1 == ref.ind1 && this->ind2 == ref.ind2) return 1;
                        return 0;
                }
};

unsigned int hash_value(const EdgeDbStub& r) {
        unsigned int i;
        i=r.ind1 |(r.ind2<<16) ;
        return i;
}

int operator==(const EdgeDbStub &t1, const EdgeDbStub &t2) 
{
        if(t1.ind1 == t2.ind1 && t1.ind2 == t2.ind2) return 1;
        return 0;
}

void export_ps(const std::shared_ptr<const Geometry>& geom, std::ostream& output)
{
  std::shared_ptr<const PolySet> ps=nullptr;
  std::shared_ptr<const ManifoldGeometry> mani = std::dynamic_pointer_cast<const ManifoldGeometry>(geom);
  std::shared_ptr<const CGALHybridPolyhedron> cgal = std::dynamic_pointer_cast<const CGALHybridPolyhedron>(geom);
  std::shared_ptr<const CGAL_Nef_polyhedron> nef = std::dynamic_pointer_cast<const CGAL_Nef_polyhedron>(geom);
  if(mani != nullptr) ps=mani->toPolySet();	  
  else if(cgal != nullptr) printf("cgal\n");
  else if(nef != nullptr){
         auto root_N = *nef; 	  
	 ps = CGALUtils::createPolySetFromNefPolyhedron3(*(root_N.p3));
  }
  else ps = std::dynamic_pointer_cast<const PolySet>(geom);
  if(ps == nullptr) {
    printf("Dont have PolySet\n");	  
    return;
  }
  std::vector<Vector4d> normals=offset3D_normals(ps->vertices, ps->indices);
  std::vector<Vector4d> newNormals;
  std::vector<IndexedFace> faces = mergetriangles(ps->indices, normals,newNormals, ps->vertices);

  int i,j,k, glue,num,cont,success,drawn,other;
  double lasche=5.0;
  double rand=5.0;
  const char *paperformat="A4";
  double factor=72.0/25.4;
  int pages=0;
  int polsdone=0,polstodo;
  //
  // create edge database
  
  EdgeDbStub stub;                                                    
//  std::unordered_map<EdgeDbStub, int, boost::hash<EdgeDbStub> > edge_db;  
  //std::unordered_map<EdgeDbStub, int> edge_db;  
  //

  for(i=0;i<faces.size();i++)
  {
    IndexedFace &face=faces[i];
    int n=face.size();
    for(j=0;j<n;j++){
      stub.ind1=face[j];
      stub.ind2=face[(j+1)%n];
//      edge_db[stub]=i;
    }
  }
  std::vector<connS> con;
  connS cx;

  for(i=0;i<faces.size();i++)
  {
    IndexedFace &face=faces[i];
    int n=face.size();
    for(j=0;j<n;j++){
      int i1=face[j];
      int i2=face[(j+1)%n];
      if(i2 < i1) continue;

      // TODO stupid workaround
      int oppface=-1;
      int opppos;
      for(int k=0;oppface == -1 && k<faces.size();k++)
      {
        IndexedFace &face1=faces[k];
        int n1=face1.size();
        for(int l=0;oppface == -1 && l<n1;l++){
          int I1=face1[l];
          int I2=face1[(l+1)%n1];
	  if(i1 == I2 && i2 == I1) {
		  oppface=k;
		  opppos=l;
	  }
	}
      }	
      printf("%d/%d -> %d/%d\n",i,j,oppface,opppos);
      cx.p1=i; cx.f1=j; cx.p2=oppface; cx.f2=opppos;  
      con.push_back(cx);

      //i,j, was ist gegnueber
    }

  }


 

  // kannten -> connections

  std::vector<Vector2d> lines,linesorg; // final postscript lines
  std::vector<labelS> label; // final postscript labels

  double xofs,yofs;
  double xofsorg,yofsorg;
  Vector2d px,py,p1,p2,pm;
  int bestend=0; // TODO besser
  std::vector<plateS> plate; // faces in final placement
  int polybesttouch;

/*

  bestend=reorder_edges(eder);
*/ 
  double paperwidth,paperheight;
  if(strcmp(paperformat,"A4") == 0) { paperheight=298; paperwidth=210; }
  else {paperheight=420; paperwidth=298;}

  polstodo=faces.size(); 

  plateS newplate;
  newplate.done=0;

  for(i=0;i<faces.size();i++)
  {
    plate.push_back(newplate);
  }
  for(i=0;i<con.size();i++) con[i].done=0;	 

  output << "%!PS-Adobe-2.0\n";
  output << "%%Orientation: Portrait\n";
  output << "%%DocumentMedia: " << paperformat << " " << paperwidth*factor << " " << paperheight * factor << "\n";
  output << "/Times-Roman findfont " << lasche*2 << " scalefont setfont\n";
  while(polsdone < polstodo)
  {
    printf("one round\n");	 
    // ein blatt designen
    polybesttouch=0;
    cont=1;
    while(cont == 1)
    {
      cont=0;
      drawn=0;
      if(lines.size()  ==  0)
      {
        for(i=0;i<faces.size()  && drawn == 0;i++)
        {
          if(plate[i].done == 0 )
          {
            success=0;
            linesorg=lines;
            xofsorg=xofs;
            yofsorg=yofs;

            pm[0]=0; pm[1]=0;  px=pm; px[0]=1; py=pointrecht(px);

            success =plot_try(i,px,py,pm,plate,ps->vertices, faces, lines, xofs,yofs,paperwidth,paperheight,rand+lasche,0,bestend);


            if(success  ==  1)  {  plate[i].done=1;  cont=1; drawn=1; printf("Set done %d\n",i); }  
            else
            {
              lines=linesorg;
              xofs=xofsorg;
              yofs=yofsorg;
            }  
          }
	}
      } else
      {
        for(i=0;i<con.size() && drawn == 0;i++)
        {
          if(con[i].done) continue;	    
          int p1=-1, p2=-1, f1=-1, f2=-1;
	  if(con[i].p1 >= faces.size()) continue;
	  if(con[i].p2 >= faces.size()) continue;
          if(plate[con[i].p1].done == 1 && plate[con[i].p2].done == 0)
          {
            p1=con[i].p1; f1=con[i].f1;
            p2=con[i].p2; f2=con[i].f2;				    
          }
          if(plate[con[i].p1].done == 0 && plate[con[i].p2].done == 1)
          {
            p2=con[i].p1; f2=con[i].f1;
            p1=con[i].p2; f1=con[i].f2;				    
          }
          if(p1 != -1) 
          {
          // draw p1:f1 -> p2:f2
            int n1=plate[p1].pt.size();
            Vector2d pt1=plate[p1].pt[f1];
            Vector2d pt2=plate[p1].pt[(f1+1)%n1];
            Vector2d px=(pt1-pt2).normalized();
            Vector2d py=pointrecht(px);
            Vector2d pm=(pt1+pt2)/2;
            success =plot_try(p2,px,py,pm,plate,ps->vertices, faces, lines, xofs,yofs,paperwidth,paperheight,rand+lasche,f2,bestend);
//                if(b == bestend)
//                {
//                  if(polybesttouch == 0) polybesttouch=1; else success=0;
//                }
            if(success == 1 )
            {
              plate[p2].done=1; drawn=1;
              con[i].done=1;
              cont=1;
            } 
            else
            {
              lines=linesorg;
              xofs=xofsorg;
              yofs=yofsorg;
            }		  
          }
        }
      }
    }
//----------------------------------- Alles fertigstellen

    // nachtraegliche laschenzeichnung
    if(lines.size() == 0)
    {
      printf("Was not able to fit something onto the page!\n");
      exit(1);
    }
    for(i=0;i<faces.size();i++) //polygone
    {
      if(plate[i].done == 1)
      {
        int n=plate[i].pt.size();	      
        for(j=0;j<n;j++) // faces
        {
          glue=0;
          for(k=0;k<con.size();k++) // connections
          {
            if(con[k].p1 == i && con[k].f1 == j && con[k].done == 0 ) { num=k; glue=1; other=con[k].p2; }
            if(con[k].p2 == i && con[k].f2 == j && con[k].done == 0 ) { num=k; glue=-1; other=con[k].p1; }
          }
          if(glue != 0)
          {
            py=plate[i].pt[(j+1)%n]-plate[i].pt[j];
            py.normalize();
            px=pointrecht(py);
            px=px*glue;
            p1=plate[i].pt[j]+(px+py)*lasche;
            py=-py;
            p2=plate[i].pt[(j+1)%n]+(px+py)*lasche;

            lines.push_back(plate[i].pt[j]);
            lines.push_back(p1);

            lines.push_back(p1);
            lines.push_back(p2);

            lines.push_back(p2);
            lines.push_back(plate[i].pt[(j+1)%n]);

            if(plate[other ].done != 1) //if the connection is not to the same page
            {
              p1=(plate[i].pt[j]+ plate[i].pt[(j+1)%n])*0.5;
              p1=p1+px*lasche*(0.5-0.27*glue);
              p1=p1+py*lasche*-0.35;
              labelS lnew;
              lnew.pt=p1;
              sprintf(lnew.text,"%d",num/2+1);
              lnew.rot=atan2(py[1],py[0])*180.0/3.1415;
              label.push_back(lnew);
            } 
          }
        }
      }
    }
    for(i=0;i<faces.size();i++)
    {
      if(plate[i].done == 1)
      {
        plate[i].done =2;
        polsdone++;  
      }
    }
    // doppelte in lines loeschen
    output << "%%Page: " << pages+1 << "\n";

    printf("line size is %d\n",lines.size());	
    output << "0 0 0 setrgbcolor\n";
    for(i=0;i<lines.size();i+=2)
    {
      output << "newpath\n";
      output << (xofs+lines[i+0][0])*factor << " " << (yofs+lines[i+0][1])*factor << " moveto\n";
      output << (xofs+lines[i+1][0])*factor << " " << (yofs+lines[i+1][1])*factor << " lineto\n";
      output << "stroke\n";
    }
    for(i=0;i<label.size();i++)
    {
      output << "%f %f moveto\n",(xofs+label[i].pt[0])*factor,(yofs+label[i].pt[1])*factor;
      output << "gsave " << label[i].rot << " rotate ( " << label[i].text << " ) show grestore \n";
    }
    output << "10 10 moveto\n";
    std::string filename="a.ps"; // TODO weg
    output << "( " << filename << "/" << pages+1 << ") show \n";
    output << "showpage\n";
    pages++;
    lines.clear();
    label.clear();
    break;
  }
  return;
}

