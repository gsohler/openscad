#include "rotextrude.h"

#include <queue>
#include <boost/logic/tribool.hpp>

#include "GeometryUtils.h"
#include "RotateExtrudeNode.h"
#include "PolySet.h"
#include "PolySetBuilder.h"
#include "PolySetUtils.h"
#include "calc.h"
#include "degree_trig.h"
#include "Feature.h"
#define _USE_MATH_DEFINES
#include <cmath>

#include "manifoldutils.h"

Outline2d alterprofile(Outline2d profile,double scalex, double scaley, double
		origin_x, double origin_y,double offset_x, double offset_y,
		double rot);

void  append_rotary_vertex(PolySetBuilder &builder,const Outline2d *face, int index, double ang)
{
	double a=ang*G_PI / 180.0;
	builder.addVertex(builder.vertexIndex(Vector3d(
			face->vertices[index][0]*cos(a),
			face->vertices[index][0]*sin(a),
			face->vertices[index][1])));
}



static void fill_ring(std::vector<Vector3d>& ring, const Outline2d& o, double a, bool flip)
{
  if (flip) {
    unsigned int l = o.vertices.size() - 1;
    for (unsigned int i = 0; i < o.vertices.size(); ++i) {
      ring[i][0] = o.vertices[l - i][0] * sin_degrees(a);
      ring[i][1] = o.vertices[l - i][0] * cos_degrees(a);
      ring[i][2] = o.vertices[l - i][1];
    }
  } else {
    for (unsigned int i = 0; i < o.vertices.size(); ++i) {
      ring[i][0] = o.vertices[i][0] * sin_degrees(a);
      ring[i][1] = o.vertices[i][0] * cos_degrees(a);
      ring[i][2] = o.vertices[i][1];
    }
  }
}

/*!
   Input to extrude should be clean. This means non-intersecting, correct winding order
   etc., the input coming from a library like Clipper.

   FIXME: We should handle some common corner cases better:
   o 2D polygon having an edge being on the Y axis:
    In this case, we don't need to generate geometry involving this edge as it
    will be an internal edge.
   o 2D polygon having a vertex touching the Y axis:
    This is more complex as the resulting geometry will (may?) be nonmanifold.
    In any case, the previous case is a specialization of this, so the following
    should be handled for both cases:
    Since the ring associated with this vertex will have a radius of zero, it will
    collapse to one vertex. Any quad using this ring will be collapsed to a triangle.

   Currently, we generate a lot of zero-area triangles

 */
std::shared_ptr<Geometry> rotatePolygon(const RotateExtrudeNode& node, const Polygon2d& poly)
{
  if (node.angle == 0) return nullptr;

  PolySetBuilder builder;
  builder.setConvexity(node.convexity);

  double min_x = 0;
  double max_x = 0;
  unsigned int fragments = 0;
  for (const auto& o : poly.outlines()) {
    for (const auto& v : o.vertices) {
      min_x = fmin(min_x, v[0]);
      max_x = fmax(max_x, v[0]);
    }
  }

  if ((max_x - min_x) > max_x && (max_x - min_x) > fabs(min_x)) {
    LOG(message_group::Error, "all points for rotate_extrude() must have the same X coordinate sign (range is %1$.2f -> %2$.2f)", min_x, max_x);
    return nullptr;
  }

  fragments = (unsigned int)std::ceil(fmax(Calc::get_fragments_from_r(max_x - min_x, 360.0, node.fn, node.fs, node.fa) * std::abs(node.angle) / 360, 1));

  bool flip_faces = (min_x >= 0 && node.angle > 0 && node.angle != 360) || (min_x < 0 && (node.angle < 0 || node.angle == 360));

#ifdef ENABLE_PYTHON  
  if(node.profile_func != NULL)
  {
	fragments=node.fn; // TODO fix
	Outline2d lastFace;
	Outline2d curFace;
	double last_ang=0, cur_ang=0;
	double last_twist=0.0, cur_twist=0.0;

#ifdef ENABLE_PYTHON
        if(node.twist_func != NULL) {
          last_twist = python_doublefunc(node.twist_func, 0);
        } else last_twist=0;
#endif	
	

		lastFace = alterprofile(python_getprofile(node.profile_func, node.fn, 0),1.0, 1.0,node.origin_x, node.origin_y, node.offset_x, node.offset_y, last_twist);
	if(node.angle != 360) {
		// Add initial closing
		Polygon2d lastface;
	        lastface.addOutline(lastFace);
	        std::unique_ptr<PolySet> ps_last = lastface.tessellate();

		Transform3d rot(angle_axis_degrees(90, Vector3d::UnitX()));
		ps_last->transform(rot);
		// Flip vertex ordering
		if (!flip_faces) {
		for (auto& p : ps_last->indices) {
		std::reverse(p.begin(), p.end());
		}
		}
		builder.appendPolySet(*ps_last);

	}
  	for (unsigned int i = 1; i <= fragments; i++) {
		cur_ang=i*node.angle/fragments;

		if(node.twist_func != NULL) {
		  cur_twist = python_doublefunc(node.twist_func, i/(double) fragments);
		} else
		cur_twist=i*node.twist /fragments;

		curFace = alterprofile(python_getprofile(node.profile_func, node.fn, cur_ang), 1.0, 1.0 , node.origin_x, node.origin_y, node.offset_x, node.offset_y , cur_twist);

		if(lastFace.vertices.size() == curFace.vertices.size()) {
			unsigned int n=lastFace.vertices.size();
			for(unsigned int j=0;j<n;j++) {
				builder.beginPolygon(3);
				append_rotary_vertex(builder,&lastFace,(j+0)%n, last_ang);
				append_rotary_vertex(builder,&lastFace,(j+1)%n, last_ang);
				append_rotary_vertex(builder,&curFace,(j+1)%n, cur_ang);
				builder.beginPolygon(3);
				append_rotary_vertex(builder,&lastFace,(j+0)%n, last_ang);
				append_rotary_vertex(builder,&curFace,(j+1)%n, cur_ang);
				append_rotary_vertex(builder,&curFace,(j+0)%n, cur_ang);
			}
		}

		lastFace = curFace;
		last_ang = cur_ang;
		last_twist = cur_twist;
	}
	if(node.angle != 360) {
		Polygon2d curface;
	        curface.addOutline(curFace);
		std::unique_ptr<PolySet> ps_cur = curface.tessellate();
		Transform3d rot2(angle_axis_degrees(cur_ang, Vector3d::UnitZ()) * angle_axis_degrees(90, Vector3d::UnitX()));
		ps_cur->transform(rot2);
		if (flip_faces) {
			for (auto& p : ps_cur->indices) {
				std::reverse(p.begin(), p.end());
			}
		}
		builder.appendPolySet(*ps_cur);
	}
	  
}
  else
#endif
  {	  
  if (node.angle != 360) {
    auto ps_start = poly.tessellate(); // starting face
    Transform3d rot(angle_axis_degrees(90, Vector3d::UnitX()));
    ps_start->transform(rot);
    // Flip vertex ordering
    if (!flip_faces) {
      for (auto& p : ps_start->indices) {
        std::reverse(p.begin(), p.end());
      }
    }
    builder.appendPolySet(*ps_start);

    auto ps_end = poly.tessellate();
    Transform3d rot2(angle_axis_degrees(node.angle, Vector3d::UnitZ()) * angle_axis_degrees(90, Vector3d::UnitX()));
    ps_end->transform(rot2);
    if (flip_faces) {
      for (auto& p : ps_end->indices) {
        std::reverse(p.begin(), p.end());
      }
    }
    builder.appendPolySet(*ps_end);
  }

  for (const auto& o : poly.outlines()) {
    std::vector<Vector3d> rings[2];
    rings[0].resize(o.vertices.size());
    rings[1].resize(o.vertices.size());

    fill_ring(rings[0], o, (node.angle == 360) ? -90 : 90, flip_faces); // first ring
    for (unsigned int j = 0; j < fragments; ++j) {
      double a;
      if (node.angle == 360) a = -90 + ((j + 1) % fragments) * 360.0 / fragments; // start on the -X axis, for legacy support
      else a = 90 - (j + 1) * node.angle / fragments; // start on the X axis
      fill_ring(rings[(j + 1) % 2], o, a, flip_faces);

      for (size_t i = 0; i < o.vertices.size(); ++i) {
        builder.appendPolygon({
                rings[j % 2][(i + 1) % o.vertices.size()],
                rings[(j + 1) % 2][(i + 1) % o.vertices.size()],
                rings[j % 2][i]
        });                

        builder.appendPolygon({
                rings[(j + 1) % 2][(i + 1) % o.vertices.size()],
                rings[(j + 1) % 2][i],
                rings[j % 2][i]
        });
      }
    }
  }
  }
  return builder.build();
}


