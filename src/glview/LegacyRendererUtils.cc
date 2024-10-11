#include "glview/LegacyRendererUtils.h"
#include "geometry/PolySet.h"
#include "geometry/Polygon2d.h"
#include "glview/ColorMap.h"
#include "utils/printutils.h"
#include "platform/PlatformUtils.h"
#include "glview/system-gl.h"
#include "core/TextureNode.h"

#include <Eigen/LU>
#include <cstddef>


#ifdef ENABLE_OPENCSG
static void set_texture_coord(const Vector3d &pt,const Vector3d &norm, double tf)
{
      double xt, yt; 

      if(fabs(norm[0]) > 0.1)
      {
	      xt=pt[2]/norm[0];
	      yt=pt[1]/norm[0];
      } else
      if(fabs(norm[1]) > 0.1)
      {
	      xt=pt[0]/norm[1];
	      yt=pt[2]/norm[1];
      } else
      if(fabs(norm[2]) > 0.1)
      {
	      xt=pt[0]/norm[2];
	      yt=pt[1]/norm[2];
      } 
      glTexCoord2f(xt/tf, yt/tf);
}

static void draw_triangle(const Renderer::shaderinfo_t *shaderinfo, const Vector3d& p0, const Vector3d& p1, const Vector3d& p2,
                          bool e0, bool e1, bool e2, double z, bool mirror, int textureind)
{
  Renderer::shader_type_t type =
    (shaderinfo) ? shaderinfo->type : Renderer::NONE;
  double tf=10.0;
  if(textureind > 0 && textureind  <= textures.size()) tf=textures[textureind-1].uvscale;

  // e0,e1,e2 are used to disable some edges from display.
  // Edges are numbered to correspond with the vertex opposite of them.
  // The edge shader draws edges when the minimum component of barycentric coords is near 0
  // Disabled edges have their corresponding components set to 1.0 when they would otherwise be 0.0.
  double d0 = e0 ? 0.0 : 1.0;
  double d1 = e1 ? 0.0 : 1.0;
  double d2 = e2 ? 0.0 : 1.0;

      Vector3d norm=(p1-p0).cross(p2-p0).normalized();
  switch (type) {
  case Renderer::EDGE_RENDERING:
    if (mirror) {
      glVertexAttrib3f(shaderinfo->data.csg_rendering.barycentric, 1.0, d1, d2);
      set_texture_coord(p0,norm,tf);
      glVertex3f(p0[0], p0[1], p0[2] + z);
      glVertexAttrib3f(shaderinfo->data.csg_rendering.barycentric, d0, d1, 1.0);
      set_texture_coord(p2,norm,tf);
      glVertex3f(p2[0], p2[1], p2[2] + z);
      glVertexAttrib3f(shaderinfo->data.csg_rendering.barycentric, d0, 1.0, d2);
      set_texture_coord(p1,norm,tf);
      glVertex3f(p1[0], p1[1], p1[2] + z);
    } else {
      glVertexAttrib3f(shaderinfo->data.csg_rendering.barycentric, 1.0, d1, d2);
      set_texture_coord(p0,norm,tf);
      glVertex3f(p0[0], p0[1], p0[2] + z);
      glVertexAttrib3f(shaderinfo->data.csg_rendering.barycentric, d0, 1.0, d2);
      set_texture_coord(p1,norm,tf);
      glVertex3f(p1[0], p1[1], p1[2] + z);
      glVertexAttrib3f(shaderinfo->data.csg_rendering.barycentric, d0, d1, 1.0);
      set_texture_coord(p2,norm,tf);
      glVertex3f(p2[0], p2[1], p2[2] + z);
    }
    break;
  default:
  case Renderer::SELECT_RENDERING:
    glVertex3d(p0[0], p0[1], p0[2] + z);
    if (!mirror) {
      glVertex3d(p1[0], p1[1], p1[2] + z);
    }
    glVertex3d(p2[0], p2[1], p2[2] + z);
    if (mirror) {
      glVertex3d(p1[0], p1[1], p1[2] + z);
    }
  }
}
#endif // ifdef ENABLE_OPENCSG

static void draw_tri(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2, double z, bool mirror, int textureind)
{
  Vector3d norm=(p1-p0).cross(p2-p0).normalized();
  double tf=10.0;
  if(textureind > 0 && textureind  <= textures.size()) tf=textures[textureind-1].uvscale;
  set_texture_coord(p0,norm,tf);
  glVertex3d(p0[0], p0[1], p0[2] + z);
  if (!mirror) {
  	set_texture_coord(p1,norm,tf);
	  glVertex3d(p1[0], p1[1], p1[2] + z);
  }
  set_texture_coord(p2,norm,tf);
  glVertex3d(p2[0], p2[1], p2[2] + z);
  if (mirror){
  	set_texture_coord(p1,norm,tf);
	glVertex3d(p1[0], p1[1], p1[2] + z);
  }
}

static void gl_draw_triangle(const Renderer::shaderinfo_t *shaderinfo, const Vector3d& p0, const Vector3d& p1, const Vector3d& p2, bool e0, bool e1, bool e2, double z, bool mirrored, int textureind)
{
  double ax = p1[0] - p0[0], bx = p1[0] - p2[0];
  double ay = p1[1] - p0[1], by = p1[1] - p2[1];
  double az = p1[2] - p0[2], bz = p1[2] - p2[2];
  double nx = ay * bz - az * by;
  double ny = az * bx - ax * bz;
  double nz = ax * by - ay * bx;
  double nl = sqrt(nx * nx + ny * ny + nz * nz);
  glNormal3d(nx / nl, ny / nl, nz / nl);
#ifdef ENABLE_OPENCSG
  if (shaderinfo) {
    draw_triangle(shaderinfo, p0, p1, p2, e0, e1, e2, z, mirrored,textureind);
  } else
#endif
  {
    draw_tri(p0, p1, p2, z, mirrored, textureind);
  }
}

void render_surface(const PolySet& ps, const Transform3d& m, int textureind,  const Renderer::shaderinfo_t *shaderinfo)
{
  PRINTD("render_surface");
  bool mirrored = m.matrix().determinant() < 0;
  bool draw_edge=true;
  if(textureind > 0) draw_edge=0;
  for (const auto& poly : ps.indices) {
    glBegin(GL_TRIANGLES);
    if (poly.size() == 3) {
      gl_draw_triangle(shaderinfo, ps.vertices[poly.at(0)], ps.vertices[poly.at(1)], ps.vertices[poly.at(2)], draw_edge, draw_edge, draw_edge, 0, mirrored, textureind);
    } else if (poly.size() == 4) {
      gl_draw_triangle(shaderinfo, ps.vertices[poly.at(0)], ps.vertices[poly.at(1)], ps.vertices[poly.at(3)], false, draw_edge, draw_edge, 0, mirrored, textureind);
      gl_draw_triangle(shaderinfo, ps.vertices[poly.at(2)], ps.vertices[poly.at(3)], ps.vertices[poly.at(1)], false, draw_edge, draw_edge, 0, mirrored, textureind);
    } else {
      Vector3d center = Vector3d::Zero();
      for (const auto& point : poly) {
        center += ps.vertices[point];
      }
      center /= poly.size();
      for (size_t j = 1; j <= poly.size(); ++j) {
        gl_draw_triangle(shaderinfo, center, ps.vertices[poly.at(j - 1)], ps.vertices[poly.at(j % poly.size())], draw_edge, false, false, 0, mirrored, textureind);
      }
    }
    glEnd();
  }
}
