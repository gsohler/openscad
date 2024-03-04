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
#include "PolySetUtils.h"

typedef struct
{
 Vector2d pt;
 double rot;
 char text[10];
} labelS;


typedef struct
{
  std::vector<Vector2d> pt;
  int color;
} fillS;

typedef struct
{
        int c,n;
} polyplate;

typedef struct
{
  std::vector<Vector2d> pt;
  Vector2d m;
  int done;
} plateS;

typedef struct
{
  int p1, f1, p2, f2;		
  int done;
} connS;

typedef struct
{
  std::vector<polyplate> pol;
  std::vector<connS> con;
} polyeder;

double COS(double x) { return cos(x*3.14159265359/180.0); }
double SIN(double x) { return sin(x*3.14159265359/180.0); }
double TAN(double x) { return tan(x*3.14159265359/180.0); }
double ATAN(double x, double y) { return atan2(y,x)*180.0/3.14159265359; }

Vector2d pointrecht(Vector2d x)
{
 Vector2d y;
 y[0]=x[1];
 y[1]=-x[0];
 return y;
}



int plot_try(std::vector<Vector2d> &lines,std::vector<fillS> &fills,int n,int a,double edgelen,Vector2d px,Vector2d py,Vector2d pm,std::vector<plateS> &plate,std::vector<Vector3d> vertices, std::vector<IndexedFace> indices, double *xofs,double *yofs,double a4b,double a4h,double rand,int destport,int bestend)
{
  int i;
  double xmax,xmin,ymax,ymin;
  Vector2d p1;
  int success=1;

// TODO indexedFace[a]


  for(i=0;i<n;i++) // TODO wird obsolet
  {
    p1=px*COS(360.0*((double)i-destport-0.5)/(double)n)+py*SIN(360.0*((double)i-destport-0.5)/(double)n);
    p1=p1*edgelen/2.0/SIN(180.0/n);
    p1=p1+pm;
    plate[a].pt[i]=p1;
  }
  plate[a].m=pm;
  fillS newfill;
  for(i=0;i<n;i++)
  {
    newfill.pt.push_back(plate[a].pt[i]);
    lines.push_back(plate[a].pt[i]);
    lines.push_back(plate[a].pt[(i+1)%n]);
  }
  newfill.color=n;
  fills.push_back(newfill);
  if(a == bestend)
  {
    for(i=0;i<n;i++)
    {
//      lines.push_back(plate[a].pt[i]);
//      lines.push_back(plate[a].m);
    }
  }
  //  if ueberlapp success=0;
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
  *xofs=(a4b-xmax+xmin)/2.0-xmin;
  *yofs=(a4h-ymax+ymin)/2.0-ymin;
  if(*xofs+xmin < rand || *yofs+ymin < rand) success=0 ;
  return success;
}

std::vector<IndexedFace> mergetriangles(const std::vector<IndexedFace> polygons,const std
::vector<Vector4d> normals,std::vector<Vector4d> &newNormals, const std::vector<Vector3d>
 &vert);

std::vector<Vector4d> offset3D_normals(const std::vector<Vector3d> &vertices, const std::vector<IndexedFace> &indices);

void export_ps(const std::shared_ptr<const Geometry>& geom, std::ostream& output)
{
  std::shared_ptr<const PolySet> ps = std::dynamic_pointer_cast<const PolySet>(geom);
  if(ps == nullptr) {
    printf("Dont have PolySet\n");	  
    return;
  }
  printf("r %d %d\n",ps->vertices.size(), ps->indices.size());
  std::vector<Vector4d> normals=offset3D_normals(ps->vertices, ps->indices);
  std::vector<Vector4d> newNormals;
  std::vector<IndexedFace> faces = mergetriangles(ps->indices, normals,newNormals, ps->vertices);
  printf("r %d %d\n",faces.size());

//int plot(polyeder &eder,double edgelen,double lasche,double rand,const char *filename,const char *paperformat)
//{
  int i,j,k,n,n1,glue,num,cont,success,drawn,other;
  double lasche=5.0;
  double edgelen=30;
  double rand=5.0;
  const char *paperformat="A4";
  double factor=72.0/25.4;
  int pages=0;
  int polsdone=0,polstodo;

  polyplate platex;
  platex.c=0;

  polyeder eder; 

  platex.n=4;
  eder.pol.push_back(platex);
  platex.n=5;
  eder.pol.push_back(platex);

  connS cx; cx.p1=0; cx.f1=0; cx.p2=1; cx.f2=0;  
  eder.con.push_back(cx);
  // TODO eder->pol -> indices
  // kannten -> connections

  // TODO find 1st face
  // TODO project face to 2D
  std::vector<Vector2d> lines,linesorg; // final postscript lines
  std::vector<fillS> fills,fillsorg; // final postscript fills
  std::vector<labelS> label; // final postscript lavels

//  std::vector<int> constat; // keeps track of connections established

  double xofs,yofs;
  double xofsorg,yofsorg;
  Vector2d px,py,p1,p2,pm;
  int bestend=0; // TODO besser
  int a,b,c,d;

  std::vector<plateS> plate; // faces in final placement
  int polybesttouch;

/*

  bestend=reorder_edges(eder);
*/ 
  double paperwidth,paperheight;
  if(strcmp(paperformat,"A4") == 0) { paperheight=298; paperwidth=210; }
  else {paperheight=420; paperwidth=298;}

  polstodo=eder.pol.size(); 

  plateS newplate;
  newplate.done=0;

  for(i=0;i<eder.pol.size();i++)
  {
    plate.push_back(newplate);
    Vector2d  pt;
    for(j=0;j<eder.pol[i].n;j++)
	plate[i].pt.push_back(pt);
  }
  for(i=0;i<eder.pol.size();i++) eder.con[i].done=0;	 

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
      for(a=0;a<eder.pol.size()  && drawn == 0;a++)
      {
        if(plate[a].done == 0 )
        {
          success=0;
          linesorg=lines;
          fillsorg=fills;
          xofsorg=xofs;
          yofsorg=yofs;
          n=((eder.pol)[a]).n;

          if(lines.size()  ==  0)
          {
            pm[0]=0; pm[1]=0;  px=pm; py=pm; px[0]=1; py[1]=1;
	    // TODO muss wissen, wo platzieren
            success =plot_try(lines,fills,n,a,edgelen,px,py,pm,plate,ps->vertices, faces, &xofs,&yofs,paperwidth,paperheight,rand+lasche,0,bestend);


            if(success  ==  1)  {  plate[a].done=1;  cont=1;  }  
            else
            {
              lines=linesorg;
              fills=fillsorg;
              xofs=xofsorg;
              yofs=yofsorg;
             }  
          }
          else
          {
            for(b=0;b<eder.pol.size() && drawn == 0;b++)
            {

              if(plate[b].done  == 1)
              {
                n1=eder.pol[b].n;
                for(c=0;c<n1 && drawn == 0;c++)
                {
                  for(d=0;d<eder.con.size() && drawn == 0;d++)
                  {
                    if(eder.con[d].p1 == b && eder.con[d].f1 == c && eder.con[d].p2 == a)
                    {
                      // try putting a to b:c removecon=d
                      p1=plate[b].pt[c];
                      p2=plate[b].pt[(c+1)%n1];
                      py=p2-p1;
                      py.normalize();
                      px=pointrecht(py);
                      pm=(p1+p2)*0.5;
                      px=-px;
                      py=-py;
                      pm=pm+px*-edgelen/2/TAN(180.0/(double)n);
		                
                      success =plot_try(lines,fills,n,a,edgelen,px,py,pm,plate,ps->vertices, faces, &xofs,&yofs,paperwidth,paperheight,rand+lasche,eder.con[d].f2,bestend);
                      if(b == bestend)
                      {
                        if(polybesttouch == 0) polybesttouch=1; else success=0;
                      }
                      if(success == 1)
                      {
                        plate[a].done=1; drawn=1;
                        eder.con[d].done=1;
                        cont=1;
                      } 
                      else
                      {
                        lines=linesorg;
                        fills=fillsorg;
                        xofs=xofsorg;
                        yofs=yofsorg;
                      } 
                    }
                  }
                }
              }
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
    for(i=0;i<eder.pol.size();i++) //polygone
    {
      if(plate[i].done == 1)
      {
        n=eder.pol[i].n;
        for(j=0;j<n;j++) // faces
        {
          glue=0;
          for(k=0;k<eder.con.size();k++) // connections
          {
            if(eder.con[k].p1 == i && eder.con[k].f1 == j && eder.con[k].done == 0 ) { num=k; glue=1; other=eder.con[k].p2; }
            if(eder.con[k].p2 == i && eder.con[k].f2 == j && eder.con[k].done == 0 ) { num=k; glue=-1; other=eder.con[k].p1; }
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
              lnew.rot=ATAN(py[0],py[1]);
              label.push_back(lnew);
            } 
          }
        }
      }
    }
    for(i=0;i<eder.pol.size();i++)
    {
      if(plate[i].done == 1)
      {
        plate[i].done =2;
        polsdone++;  
      }
    }
    // doppelte in lines loeschen
    output << "%%Page: " << pages+1 << "\n";

    for(i=0;i<fills.size();i++)
    {
      double r,g,b;
      r=sin(fills[i].color*0.628+0)*0.5+0.5;
      g=sin(fills[i].color*0.628+2)*0.5+0.5;
      b=sin(fills[i].color*0.628+4)*0.5+0.5;
      output  << r << " " << g << " " << b <<" setrgbcolor\n";
      output << "newpath\n";
      output << (xofs+fills[i].pt[0][0])*factor << " " << (yofs+fills[i].pt[0][1])*factor << " moveto\n";
      for(j=1;j<fills[i].pt.size();j++)
      {
        output << (xofs+fills[i].pt[j][0])*factor << " " << (yofs+fills[i].pt[j][1])*factor << " lineto\n";
      }
     output << "closepath\n";
     output << "fill\n";
    }
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
    fills.clear();
  }
  return;
}

