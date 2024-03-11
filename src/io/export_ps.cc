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
#include <cairo.h>
#include <cairo-pdf.h>



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
  double xofs, yofs;	
  std::vector<lineS> lines;	
  std::vector<labelS> label;
} sheetS;

typedef struct
{
  double lasche;	
  double rand;
  const char *paperformat;
  double paperwidth,paperheight;
  int bestend;
} plotSettingsS;

typedef struct
{
  std::vector<Vector2d> pt; // actual points
  std::vector<Vector2d> pt_l1; // leap starts
  std::vector<Vector2d> pt_l2; // leap ends
  std::vector<Vector2d> bnd; // Complete boundary representing points and leps
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
int edge_outwards(std::vector<connS> &con, int plate, int face) {
  int outwards=0;
  for(int k=0;k<con.size();k++)	{
  if(con[k].p2 == plate && con[k].f2 == face) outwards=1;		
  }
  return outwards;
}

int plot_try(int refplate, int destplate,Vector2d px,Vector2d py,Vector2d pm,std::vector<plateS> &plate,std::vector<connS> &con, std::vector<Vector3d> vertices, std::vector<IndexedFace> faces, sheetS &sheet,int destedge, plotSettingsS plot_s)
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
    double lasche_eff=plot_s.lasche;
    if(lasche_eff > py.norm()/2.0) lasche_eff=py.norm()/2.0;
    py.normalize();
    px=pointrecht(py);
    px=-px;
    plate[destplate].pt_l1.push_back(plate[destplate].pt[i]+(px+py)*lasche_eff);
    py=-py;
    plate[destplate].pt_l2.push_back(plate[destplate].pt[(i+1)%n]+(px+py)*lasche_eff);
  }
  // create complete view of bnd
  plate[destplate].bnd.clear();
  for(int i=0;i<n;i++) {
    plate[destplate].bnd.push_back(plate[destplate].pt[i]);
    if(edge_outwards(con, destplate, i)) {
      plate[destplate].bnd.push_back(plate[destplate].pt_l1[i]);
      plate[destplate].bnd.push_back(plate[destplate].pt_l2[i]);
    }
  }
  //
  // check if new points collide with some existing boundaries

  for(int j=0;j<plate.size();j++) { // kollision mit anderen platten
    if(j == destplate) continue; // nicht mit sich selbst
    if(plate[j].done != 1) continue; // und nicht wenn sie nicht existiert
    for(int i=0;i<n;i++) { 
      if(point_in_polygon(plate[j].bnd,plate[destplate].pt[i])) success=0; // alle neue punkte
      if(j == refplate && i == destedge) {} // joker
      else
      {
	// jede neue lasche wenn sie auswaerts geht
	// plate destplate, pt i
//        if(edge_outwards(con, destplate, i) ) {
        if(point_in_polygon(plate[j].pt,plate[destplate].pt_l1[i])) success=0; // alle neue laschen
        if(point_in_polygon(plate[j].pt,plate[destplate].pt_l2[i])) success=0;
//	}  
      }  
    }
  }
  //
  // check if any existing  points collide new boudnary
  for(int j=0;j<plate.size();j++) { // kollision mit anderen platten
    if(j == destplate) continue; // nicht mit sich selbst
    if(plate[j].done != 1) continue; // und nicht wenn sie nicht existiert
    int n=plate[j].pt.size();
    for(int i=0;i<n;i++) {
      if(point_in_polygon(plate[destplate].pt,plate[j].pt[i])) {
//        success=0; TODO activate
//	if(success) {
//rintf("playe %d pt %d hit		
//	}
      }
    }
  }

  for(i=0;i<n;i++) {
    lineS line;
    line.p1=plate[destplate].pt[i]; 
    line.p2=plate[destplate].pt[(i+1)%n];
    line.dashed=0;
    sheet.lines.push_back(line);
  }
  xmin=sheet.lines[0].p1[0];
  xmax=sheet.lines[0].p1[0];
  ymin=sheet.lines[0].p1[1];
  ymax=sheet.lines[0].p1[1];
  for(i=0;i<sheet.lines.size();i++)
  {
    if(sheet.lines[i].p1[0] < xmin) xmin=sheet.lines[i].p1[0];
    if(sheet.lines[i].p1[0] > xmax) xmax=sheet.lines[i].p1[0];
    if(sheet.lines[i].p1[1] < ymin) ymin=sheet.lines[i].p1[1];
    if(sheet.lines[i].p1[1] > ymax) ymax=sheet.lines[i].p1[1];
    if(sheet.lines[i].p2[0] < xmin) xmin=sheet.lines[i].p2[0];
    if(sheet.lines[i].p2[0] > xmax) xmax=sheet.lines[i].p2[0];
    if(sheet.lines[i].p2[1] < ymin) ymin=sheet.lines[i].p2[1];
    if(sheet.lines[i].p2[1] > ymax) ymax=sheet.lines[i].p2[1];
  }
  sheet.xofs=(plot_s.paperwidth-xmax+xmin)/2.0-xmin;
  sheet.yofs=(plot_s.paperheight-ymax+ymin)/2.0-ymin;
  if(sheet.xofs+xmin < plot_s.rand || sheet.yofs+ymin < plot_s.rand) success=0 ;
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

std::vector<sheetS> sheets_combine(std::vector<sheetS> sheets)
{
  return sheets;	// TODO impl
}

std::vector<sheetS> fold_3d(std::shared_ptr<const PolySet> ps, const plotSettingsS &plot_s)
{
  std::vector<Vector4d> normals=offset3D_normals(ps->vertices, ps->indices);
  std::vector<Vector4d> newNormals;
  std::vector<IndexedFace> faces = mergetriangles(ps->indices, normals,newNormals, ps->vertices);

  int i,j,k, glue,num,cont,success,drawn,other;
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
    int nb=faces[b.p1].size();		  
    double da= (ps->vertices[faces[a.p1][a.f1]] - ps->vertices[faces[a.p1][(a.f1+1)%na]]).norm();
    double db= (ps->vertices[faces[b.p1][b.f1]] - ps->vertices[faces[b.p1][(b.f1+1)%nb]]).norm();
    return(da>db)?1:0;
		  });

//  std::vector<lineS> lines,linesorg; // final postscript lines

  Vector2d px,py,p1,p2,pm;
  std::vector<plateS> plate; // faces in final placement
  int polybesttouch;

/*

  bestend=reorder_edges(eder);
*/ 

  polstodo=faces.size(); 

  plateS newplate;
  newplate.done=0;

  for(i=0;i<faces.size();i++)
  {
    plate.push_back(newplate);
  }
  for(i=0;i<con.size();i++) con[i].done=0;	 

  std::vector<sheetS> sheets;
  while(polsdone < polstodo)
  {
//    printf("one round\n");	 
    // ein blatt designen
    polybesttouch=0;
    cont=1;
    sheetS sheet, sheetorg;
    while(cont == 1)
    {
      cont=0;
      drawn=0;
      if(sheet.lines.size()  ==  0)
      {
        for(i=0;i<faces.size()  && drawn == 0;i++)
        {
          if(plate[i].done == 0 )
          {
            success=0;
            sheetorg=sheet;
            pm[0]=0; pm[1]=0;  px=pm; px[0]=1; py=pointrecht(px);

            success =plot_try(-1, i,px,py,pm,plate,con, ps->vertices, faces, sheet, 0, plot_s);


            if(success  ==  1)
	    {
//              printf("Successfully placed plate %d px=%g/%g, py=%g/%g\n",i,px[0], px[1], py[0], py[1]);		    
              plate[i].done=1;
	      cont=1; 
	      drawn=1;
	    }  
            else
            {
              sheet=sheetorg;
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
	    sheetorg=sheet;

            success =plot_try(p1, p2,px,py,pm,plate,con, ps->vertices, faces, sheet, f2, plot_s);
//                if(b == bestend)
//                {
//                  if(polybesttouch == 0) polybesttouch=1; else success=0;
//                }
            if(success == 1 )
            {
//              printf("Successfully placed plate %d px=%g/%g, py=%g/%g\n",p2,px[0], px[1], py[0], py[1]);		    
              plate[p2].done=1;
	      drawn=1;
              con[i].done=1;
              cont=1;
            } 
            else
            {
              sheet=sheetorg;
            }		  
          }
        }
      }
    }
//----------------------------------- Alles fertigstellen

    // nachtraegliche laschenzeichnung
    if(sheet.lines.size() == 0)
    {
      printf("Was not able to fit something onto the page!\n");
      exit(1);
    }
    labelS lnew;
    for(i=0;i<plate.size();i++)  // plates
    {
      if(plate[i].done == 1)
      {
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
	      line.p1=plate[i].pt[j]; line.p2=p1; sheet.lines.push_back(line);
	      line.p1=p1; line.p2=p2; sheet.lines.push_back(line);
	      line.p1=p2; line.p2=plate[i].pt[(j+1)%n]; sheet.lines.push_back(line);
	      line.p1=plate[i].pt[(j+1)%n]; line.p2=plate[i].pt[j]; sheet.lines.push_back(line);
	    }

            if(plate[other ].done != 1) //if the connection is not to the same page
            {
              p1=(plate[i].pt[j]+ plate[i].pt[(j+1)%n])*0.5;

              py=plate[i].pt[(j+1)%n]-plate[i].pt[j]; // entlang der kante
              py.normalize();
              px=pointrecht(py);

              p1=p1+px*plot_s.lasche*(0.5+0.27*glue);
              p1=p1+py*plot_s.lasche*-0.35;
              lnew.pt=p1;
              sprintf(lnew.text,"%d",num);
              lnew.rot=atan2(py[1],py[0])*180.0/3.1415;
              sheet.label.push_back(lnew);
            } 
          }
        }
//        lnew.pt=p1; // for DEBUG only
//        sprintf(lnew.text,"%d",i);
//        lnew.pt = mean / n;
//        lnew.rot=0;
//        label.push_back(lnew);
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
    for(int i=0;i<sheet.lines.size()-1;i++) {
      auto &line1 = sheet.lines[i];	     
      for(int j=i+1;j<sheet.lines.size();j++) {
        auto &line2 = sheet.lines[j];
        if((line1.p1 - line2.p2).norm() < 1e-3 && (line1.p2 - line2.p1).norm() < 1e-3 ) {
          line1.dashed=1;
          sheet.lines.erase(sheet.lines.begin()+j);
          break;	  
	}
      }
    }
    sheets.push_back(sheet);
  }
  return sheets_combine(sheets);
}


void output_ps(std::ostream &output, std::vector<sheetS> &sheets, const plotSettingsS & plot_s)
{
  double factor=72.0/25.4;
  output << "%!PS-Adobe-2.0\n";
  output << "%%Orientation: Portrait\n";
  output << "%%DocumentMedia: " << plot_s.paperformat << " " << plot_s.paperwidth*factor << " " << plot_s.paperheight * factor << "\n";
  output << "/Times-Roman findfont " << plot_s.lasche*2 << " scalefont setfont 0.1 setlinewidth\n";

  int pages=0;
  for(auto &sheet : sheets)
  {  
    // doppelte in lines loesche//n
    output << "%%Page: " << pages+1 << "\n";

//    printf("line size is %d\n",sheet.lines.size());	
    output << "0 0 0 setrgbcolor\n";
    for(int i=0;i<sheet.lines.size();i++)
    {
      if(sheet.lines[i].dashed) output << "[2.5 2] 0 setdash\n";
      else output << "[3 0 ] 0 setdash\n";
      output << "newpath\n";
      output << (sheet.xofs+sheet.lines[i].p1[0])*factor << " " << (sheet.yofs+sheet.lines[i].p1[1])*factor << " moveto\n";
      output << (sheet.xofs+sheet.lines[i].p2[0])*factor << " " << (sheet.yofs+sheet.lines[i].p2[1])*factor << " lineto\n";
      output << "stroke\n";
    }
    for(int i=0;i<sheet.label.size();i++)
    {
      output << (sheet.xofs+sheet.label[i].pt[0])*factor << " " << (sheet.yofs+sheet.label[i].pt[1])*factor << " moveto\n";
      output << "gsave " << sheet.label[i].rot << " rotate ( " << sheet.label[i].text << " ) show grestore \n";
    }
    output << "10 10 moveto\n";
    std::string filename="a.ps"; // TODO weg
    output << "( " << filename << "/" << pages+1 << ") show \n";
    output << "showpage\n";
    pages++;
  }
  return;
}


void export_ps(const std::shared_ptr<const Geometry>& geom, std::ostream& output)
{
  std::shared_ptr<const PolySet> ps= PolySetUtils::getGeometryAsPolySet(geom);
  if(ps == nullptr) {
    printf("Dont have PolySet\n");	  
    return;
  }
  plotSettingsS plot_s;
  plot_s.lasche=10.0;
  plot_s.rand=5.0;
  plot_s.paperformat="A4";
  plot_s.bestend=0; // TODO besser
  if(strcmp(plot_s.paperformat, "A4") == 0) { plot_s.paperheight=298; plot_s.paperwidth=210; }
  else {plot_s.paperheight=420; plot_s.paperwidth=298;}

  std::vector<sheetS> sheets = fold_3d(ps, plot_s);
  output_ps(output, sheets,plot_s);
}


void output_pdf(cairo_t *cr, std::vector<sheetS> &sheets, const plotSettingsS & plot_s) {
  double factor=72.0/25.4;
//  output << "%!PS-Adobe-2.0\n";
//  output << "%%Orientation: Portrait\n";
//  output << "%%DocumentMedia: " << plot_s.paperformat << " " << plot_s.paperwidth*factor << " " << plot_s.paperheight * factor << "\n";
//  output << "/Times-Roman findfont " << plot_s.lasche*2 << " scalefont setfont 0.1 setlinewidth\n";

  int pages=0;
  for(auto &sheet : sheets)
  {  
  //  output << "%%Page: " << pages+1 << "\n";

    cairo_set_source_rgba(cr, 0,0,0,1);
    for(int i=0;i<sheet.lines.size();i++)
    {
//      if(sheet.lines[i].dashed) output << "[2.5 2] 0 setdash\n";
//      else output << "[3 0 ] 0 setdash\n";
      cairo_move_to(cr, (sheet.xofs+sheet.lines[i].p1[0])*factor, (sheet.yofs+sheet.lines[i].p1[1])*factor);
      cairo_line_to(cr, (sheet.xofs+sheet.lines[i].p2[0])*factor, (sheet.yofs+sheet.lines[i].p2[1]));
      cairo_stroke(cr);
    }
    for(int i=0;i<sheet.label.size();i++)
    {
       cairo_move_to( cr, (sheet.xofs+sheet.label[i].pt[0])*factor  , (sheet.yofs+sheet.label[i].pt[1])*factor );
       cairo_show_text(cr, sheet.label[i].text);
//      output << (sheet.xofs+sheet.label[i].pt[0])*factor << " " << (sheet.yofs+sheet.label[i].pt[1])*factor << " moveto\n";
//      output << "gsave " << sheet.label[i].rot << " rotate ( " << sheet.label[i].text << " ) show grestore \n";
    }
//    output << "10 10 moveto\n"; // TODO move as label, aber kenne seitenzahlnicht
//    std::string filename="a.ps"; // TODO weg
//    output << "( " << filename << "/" << pages+1 << ") show \n";
    cairo_show_page(cr);
    pages++;
  }
  return;
}

void draw_fold_geom(std::shared_ptr<const PolySet> & ps, cairo_t *cr ){
  plotSettingsS plot_s;
  plot_s.lasche=10.0;
  plot_s.rand=5.0;
  plot_s.paperformat="A4";
  plot_s.bestend=0; // TODO besser
  if(strcmp(plot_s.paperformat, "A4") == 0) { plot_s.paperheight=298; plot_s.paperwidth=210; }
  else {plot_s.paperheight=420; plot_s.paperwidth=298;}
  std::vector<sheetS> sheets = fold_3d(ps, plot_s);
  output_pdf(cr, sheets,plot_s);

}
