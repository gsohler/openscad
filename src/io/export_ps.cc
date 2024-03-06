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
#include "cgalutils.h"
#include "PolySetUtils.h"
#include <unordered_map>
#include "boost-utils.h"
#include <hash.h>


typedef struct
{
 Vector2d p1;
 Vector2d p2;
 int dashed=0;
} lineS;

typedef struct
{
 Vector2d pt;
 double rot;
 char text[10];
} labelS;


typedef struct
{
  std::vector<Vector2d> pt;
  std::vector<Vector2d> pt_l1;
  std::vector<Vector2d> pt_l2;
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

int point_in_polygon(const std::vector<Vector2d> &pol, const Vector2d &pt) 
{
  // polygons are clockwise	
  int cuts=0;
  int n=pol.size();
  for(int i=0;i<n;i++) {
    Vector2d p1=pol[i];
    Vector2d p2=pol[(i+1)%n];
    if(fabs(p1[1]-p2[1]) > 1e-9){
      if(pt[1] < p1[1] && pt[1] > p2[1]) {
        double x=p1[0]+(p2[0]-p1[0])*(pt[1]-p1[1])/(p2[1]-p1[1]);
        if(x > pt[0]) cuts++;
      }
      if(pt[1] < p2[1] && pt[1] > p1[1]) {
        double x=p1[0]+(p2[0]-p1[0])*(pt[1]-p1[1])/(p2[1]-p1[1]);
        if(x > pt[0]) cuts++;
      }
    }      
  }
  return cuts&1;
}

int plot_try(int refplate, int destplate,Vector2d px,Vector2d py,Vector2d pm,std::vector<plateS> &plate,std::vector<Vector3d> vertices, std::vector<IndexedFace> faces, std::vector<lineS> &lines,double &xofs,double &yofs,double a4b,double a4h,double rand,int destedge,int bestend, double lasche)
{
  int i;
  double xmax,xmin,ymax,ymin;
  Vector2d p1;
  int success=1;

  int n=faces[destplate].size();
  Vector3d totalnorm(0,0,0);
  for(int i=0;i<n;i++) {
    int i0=faces[destplate][(i+n-1)%n];
    int i1=faces[destplate][(i+0)%n];
    int i2=faces[destplate][(i+1)%n];
    Vector3d dir1=(vertices[i2]-vertices[i1]).normalized();
    Vector3d dir2=(vertices[i0]-vertices[i1]).normalized();
    totalnorm += dir1.cross(dir2);
  }
  int i0=faces[destplate][(destedge+n-1)%n];
  int i1=faces[destplate][(destedge+0)%n];
  int i2=faces[destplate][(destedge+1)%n];
  Vector3d pt=vertices[i1];
  Vector3d xdir=(vertices[i2]-pt).normalized();
  Vector3d zdir=(xdir.cross(vertices[i0]-pt)).normalized(); 
  if(totalnorm.dot(zdir) < 0) zdir=-zdir; // fix concave edge							    
  Vector3d ydir=(zdir.cross(xdir)).normalized();
  pt=(vertices[i1]+vertices[i2])/2;
  Matrix4d mat;
  mat <<  xdir[0], ydir[0], zdir[0], pt[0],
	  xdir[1], ydir[1], zdir[1], pt[1],
	  xdir[2], ydir[2], zdir[2], pt[2],
	  0      , 0      , 0      , 1;

  Matrix4d invmat = mat.inverse();	  

  plate[destplate].pt.clear();
  plate[destplate].pt_l1.clear();
  plate[destplate].pt_l2.clear();
  for(int i=0;i<n;i++) {
    Vector3d pt = vertices[faces[destplate][i]];
    Vector4d pt4(pt[0], pt[1], pt[2], 1);
    pt4 = invmat * pt4 ;
    plate[destplate].pt.push_back(px*pt4[0] + py*pt4[1]+pm);
  }
  // nun laschen planen
  for(int i=0;i<n;i++) {
  	  
    py=plate[destplate].pt[(i+1)%n]-plate[destplate].pt[i];
    double maxl=py.norm();
    double lasche_eff=lasche;
    if(lasche_eff > py.norm()/2.0) lasche_eff=py.norm()/2.0;
    py.normalize();
    px=pointrecht(py);
    px=-px;
    plate[destplate].pt_l1.push_back(plate[destplate].pt[i]+(px+py)*lasche_eff);
    py=-py;
    plate[destplate].pt_l2.push_back(plate[destplate].pt[(i+1)%n]+(px+py)*lasche_eff);
  }
  //
  for(int j=0;j<plate.size();j++) { // kollision mit anderen platten
    if(j == destplate) continue; // nicht mit sich selbst
    if(plate[j].done != 1) continue; // und nicht wenn sie nicht existiert
    for(int i=0;i<n;i++) { 
      if(point_in_polygon(plate[j].pt,plate[destplate].pt[i])) success=0;
      if(j == refplate && i == destedge) {} // joker
      else
      {
	// jede neue lasche
        if(point_in_polygon(plate[j].pt,plate[destplate].pt_l1[i])) success=0; 
        if(point_in_polygon(plate[j].pt,plate[destplate].pt_l2[i])) success=0;
      }  
    }
  }
  for(i=0;i<n;i++) {
    lineS line;
    line.p1=plate[destplate].pt[i]; 
    line.p2=plate[destplate].pt[(i+1)%n];
    line.dashed=0;
    lines.push_back(line);
  }
  xmin=lines[0].p1[0];
  xmax=lines[0].p1[0];
  ymin=lines[0].p1[1];
  ymax=lines[0].p1[1];
  for(i=0;i<lines.size();i++)
  {
    if(lines[i].p1[0] < xmin) xmin=lines[i].p1[0];
    if(lines[i].p1[0] > xmax) xmax=lines[i].p1[0];
    if(lines[i].p1[1] < ymin) ymin=lines[i].p1[1];
    if(lines[i].p1[1] > ymax) ymax=lines[i].p1[1];
    if(lines[i].p2[0] < xmin) xmin=lines[i].p2[0];
    if(lines[i].p2[0] > xmax) xmax=lines[i].p2[0];
    if(lines[i].p2[1] < ymin) ymin=lines[i].p2[1];
    if(lines[i].p2[1] > ymax) ymax=lines[i].p2[1];
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
  std::shared_ptr<const PolySet> ps= PolySetUtils::getGeometryAsPolySet(geom);
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
//      printf("%d/%d -> %d/%d\n",i,j,oppface,opppos);
      cx.p1=i; cx.f1=j; cx.p2=oppface; cx.f2=opppos;  
      con.push_back(cx);

      //i,j, was ist gegnueber
    }

  }

  std::sort(con.begin(), con.end(), [ps, faces](const connS &a, const connS &b ) {
    int na=faces[a.p1].size();		  
    int nb=faces[a.p1].size();		  
    double da= (ps->vertices[faces[a.p1][a.f1]] - ps->vertices[faces[a.p1][(a.f1+1)%na]]).norm();
    double db= (ps->vertices[faces[b.p1][b.f1]] - ps->vertices[faces[b.p1][(b.f1+1)%nb]]).norm();
    return(da>db)?1:0;
		  });

  std::vector<lineS> lines,linesorg; // final postscript lines
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
  output << "/Times-Roman findfont " << lasche*2 << " scalefont setfont 0.1 setlinewidth\n";
  while(polsdone < polstodo)
  {
//    printf("one round\n");	 
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

            success =plot_try(-1, i,px,py,pm,plate,ps->vertices, faces, lines, xofs,yofs,paperwidth,paperheight,rand+lasche,0,bestend, lasche);


            if(success  ==  1)
	    {
              printf("Successfully placed plate %d px=%g/%g, py=%g/%g\n",i,px[0], px[1], py[0], py[1]);		    
              plate[i].done=1;
	      cont=1; 
	      drawn=1;
	    }  
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
	    linesorg=lines;
	    xofsorg=xofs;
	    yofsorg=yofs;

            success =plot_try(p1, p2,px,py,pm,plate,ps->vertices, faces, lines, xofs,yofs,paperwidth,paperheight,rand+lasche,f2,bestend,lasche);
//                if(b == bestend)
//                {
//                  if(polybesttouch == 0) polybesttouch=1; else success=0;
//                }
            if(success == 1 )
            {
              printf("Successfully placed plate %d px=%g/%g, py=%g/%g\n",p2,px[0], px[1], py[0], py[1]);		    
              plate[p2].done=1;
	      drawn=1;
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
    labelS lnew;
    for(i=0;i<plate.size();i++)  // plates
    {
      if(plate[i].done == 1)
      {
        printf("Lasche Plate %d\n",i);	      
        int n=plate[i].pt.size();	      
	Vector2d mean(0,0);
        for(j=0;j<n;j++) // pts
        {
          mean += plate[i].pt[j];		
          glue=0;
          for(k=0;k<con.size();k++) // connections
          {
            if(con[k].p1 == i && con[k].f1 == j && con[k].done == 0 ) { num=k; glue=1; other=con[k].p2; }
            if(con[k].p2 == i && con[k].f2 == j && con[k].done == 0 ) { num=k; glue=-1; other=con[k].p1; }
          }
          if(glue != 0)
          {
            p1=plate[i].pt_l1[j];
            p2=plate[i].pt_l2[j];

	    if(glue < 0) { // nur aussenkannte
              lineS line;
	      line.dashed=0;
	      line.p1=plate[i].pt[j]; line.p2=p1; lines.push_back(line);
	      line.p1=p1; line.p2=p2; lines.push_back(line);
	      line.p1=p2; line.p2=plate[i].pt[(j+1)%n]; lines.push_back(line);
	      line.p1=plate[i].pt[(j+1)%n]; line.p2=plate[i].pt[j]; lines.push_back(line);
	    }

            if(plate[other ].done != 1) //if the connection is not to the same page
            {
              p1=(plate[i].pt[j]+ plate[i].pt[(j+1)%n])*0.5;

              py=plate[i].pt[(j+1)%n]-plate[i].pt[j]; // entlang der kante
              py.normalize();
              px=pointrecht(py);

//              p1=p1+px*lasche*(0.5-0.27*glue);
              p1=p1+py*lasche*-0.35;
              lnew.pt=p1;
              sprintf(lnew.text,"%d",num);
              lnew.rot=atan2(py[1],py[0])*180.0/3.1415;
              label.push_back(lnew);
            } 
          }
        }
        lnew.pt=p1;
        sprintf(lnew.text,"%d",i);
        lnew.pt = mean / n;
        lnew.rot=0;
        label.push_back(lnew);
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
    for(int i=0;i<lines.size()-1;i++) {
      auto &line1 = lines[i];	     
      for(int j=i+1;j<lines.size();j++) {
        auto &line2 = lines[j];
        if((line1.p1 - line2.p2).norm() < 1e-3 && (line1.p2 - line2.p1).norm() < 1e-3 ) {
          line1.dashed=1;
          lines.erase(lines.begin()+j);
          break;	  
	}
      }
    }
    // doppelte in lines loesche//n
    output << "%%Page: " << pages+1 << "\n";

//    printf("line size is %d\n",lines.size());	
    output << "0 0 0 setrgbcolor\n";
    for(i=0;i<lines.size();i++)
    {
      if(lines[i].dashed) output << "[2.5 2] 0 setdash\n";
      else output << "[3 0 ] 0 setdash\n";
      output << "newpath\n";
      output << (xofs+lines[i].p1[0])*factor << " " << (yofs+lines[i].p1[1])*factor << " moveto\n";
      output << (xofs+lines[i].p2[0])*factor << " " << (yofs+lines[i].p2[1])*factor << " lineto\n";
      output << "stroke\n";
    }
    for(i=0;i<label.size();i++)
    {
      output << (xofs+label[i].pt[0])*factor << " " << (yofs+label[i].pt[1])*factor << " moveto\n";
      output << "gsave " << label[i].rot << " rotate ( " << label[i].text << " ) show grestore \n";
    }
    output << "10 10 moveto\n";
    std::string filename="a.ps"; // TODO weg
    output << "( " << filename << "/" << pages+1 << ") show \n";
    output << "showpage\n";
    pages++;
    lines.clear();
    label.clear();
  }
  return;
}

