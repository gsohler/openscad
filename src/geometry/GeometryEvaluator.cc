#include "GeometryEvaluator.h"
#include "Tree.h"
#include "GeometryCache.h"
#include "Polygon2d.h"
#include "ModuleInstantiation.h"
#include "State.h"
#include "OffsetNode.h"
#include "TransformNode.h"
#include "LinearExtrudeNode.h"
#include "PathExtrudeNode.h"
#include "RoofNode.h"
#include "roof_ss.h"
#include "roof_vd.h"
#include "RotateExtrudeNode.h"
#include "PullNode.h"
#include "CgalAdvNode.h"
#include "ProjectionNode.h"
#include "CsgOpNode.h"
#include "TextNode.h"
#include "RenderNode.h"
#include "ClipperUtils.h"
#include "PolySetUtils.h"
#include "PolySet.h"
#include "PolySetBuilder.h"
#include "calc.h"
#include "printutils.h"
#include "calc.h"
#include "DxfData.h"
#include "degree_trig.h"
#include <ciso646> // C alternative tokens (xor)
#include <algorithm>
#include "boost-utils.h"
#include <hash.h>
#include "boolean_utils.h"
#include  "Selection.h"
#ifdef ENABLE_CGAL
#include "CGALCache.h"
#include "CGALHybridPolyhedron.h"
#include "cgalutils.h"
#include <CGAL/convex_hull_2.h>
#include <CGAL/Point_2.h>
#endif
#ifdef ENABLE_MANIFOLD
#include "ManifoldGeometry.h"
#include "manifoldutils.h"
#endif


#ifdef ENABLE_PYTHON
#include <src/python/python_public.h>
#endif
class Geometry;
class Polygon2d;
class Tree;

GeometryEvaluator::GeometryEvaluator(const Tree& tree) : tree(tree) { }

/*!
   Set allownef to false to force the result to _not_ be a Nef polyhedron

   There are some guarantees on the returned geometry:
   * 2D and 3D geometry cannot be mixed; we will return either _only_ 2D or _only_ 3D geometries
   * PolySet geometries are always 3D. 2D Polysets are only created for special-purpose rendering operations downstream from here.
   * Needs validation: Implementation-specific geometries shouldn't be mixed (Nef polyhedron, Manifold, CGAL Hybrid polyhedrons)
 */
std::shared_ptr<const Geometry> GeometryEvaluator::evaluateGeometry(const AbstractNode& node,
                                                               bool allownef)
{
  const std::string& key = this->tree.getIdString(node);
  if (!GeometryCache::instance()->contains(key)) {
    std::shared_ptr<const Geometry> N;
#ifdef ENABLE_CGAL
    if (CGALCache::instance()->contains(key)) {
      N = CGALCache::instance()->get(key);
    }
#endif

    // If not found in any caches, we need to evaluate the geometry
    if (N) {
      this->root = N;
    } else {
      this->traverse(node);
    }
#ifdef ENABLE_CGAL
    if (std::dynamic_pointer_cast<const CGALHybridPolyhedron>(this->root)) {
      this->root = PolySetUtils::getGeometryAsPolySet(this->root);
    }
#endif
#ifdef ENABLE_MANIFOLD
    if (std::dynamic_pointer_cast<const ManifoldGeometry>(this->root)) {
      this->root = PolySetUtils::getGeometryAsPolySet(this->root);
    }
#endif

    if (!allownef) {
      // We cannot render concave polygons, so tessellate any 3D PolySets
      auto ps = PolySetUtils::getGeometryAsPolySet(this->root);
      if (ps && !ps->isEmpty()) {
        // Since is_convex() doesn't handle non-planar faces, we need to tessellate
        // also in the indeterminate state so we cannot just use a boolean comparison. See #1061
        bool convex = bool(ps->convexValue()); // bool is true only if tribool is true, (not indeterminate and not false)
        if (!convex) {
          assert(ps->getDimension() == 3);
          this->root = PolySetUtils::tessellate_faces(*ps);
        }
      }
    }
    smartCacheInsert(node, this->root);
    return this->root;
  }
  return GeometryCache::instance()->get(key);
}

bool GeometryEvaluator::isValidDim(const Geometry::GeometryItem& item, unsigned int& dim) const {
  if (!item.first->modinst->isBackground() && item.second) {
    if (!dim) dim = item.second->getDimension();
    else if (dim != item.second->getDimension() && !item.second->isEmpty()) {
      LOG(message_group::Warning, item.first->modinst->location(), this->tree.getDocumentPath(), "Mixing 2D and 3D objects is not supported");
      return false;
    }
  }
  return true;
}

GeometryEvaluator::ResultObject GeometryEvaluator::applyToChildren(const AbstractNode& node, OpenSCADOperator op)
{
  unsigned int dim = 0;
  for (const auto& item : this->visitedchildren[node.index()]) {
    if (!isValidDim(item, dim)) break;
  }
  if (dim == 2) return {std::shared_ptr<Geometry>(applyToChildren2D(node, op))};
  else if (dim == 3) return applyToChildren3D(node, op);
  return {};
}

int linsystem( Vector3d v1,Vector3d v2,Vector3d v3,Vector3d pt,Vector3d &res,double *detptr)
{
        float det,ad11,ad12,ad13,ad21,ad22,ad23,ad31,ad32,ad33;
        det=v1[0]*(v2[1]*v3[2]-v3[1]*v2[2])-v1[1]*(v2[0]*v3[2]-v3[0]*v2[2])+v1[2]*(v2[0]*v3[1]-v3[0]*v2[1]);
        if(detptr != NULL) *detptr=det;
        ad11=v2[1]*v3[2]-v3[1]*v2[2];
        ad12=v3[0]*v2[2]-v2[0]*v3[2];
        ad13=v2[0]*v3[1]-v3[0]*v2[1];
        ad21=v3[1]*v1[2]-v1[1]*v3[2];
        ad22=v1[0]*v3[2]-v3[0]*v1[2];
        ad23=v3[0]*v1[1]-v1[0]*v3[1];
        ad31=v1[1]*v2[2]-v2[1]*v1[2];
        ad32=v2[0]*v1[2]-v1[0]*v2[2];
        ad33=v1[0]*v2[1]-v2[0]*v1[1];

        if(fabs(det) < 0.00001)
                return 1;
        
        res[0] = (ad11*pt[0]+ad12*pt[1]+ad13*pt[2])/det;
        res[1] = (ad21*pt[0]+ad22*pt[1]+ad23*pt[2])/det;
        res[2] = (ad31*pt[0]+ad32*pt[1]+ad33*pt[2])/det;
        return 0;
}

int cut_face_face_face(Vector3d p1, Vector3d n1, Vector3d p2,Vector3d n2, Vector3d p3, Vector3d n3, Vector3d &res,double *detptr)
{
        //   vec1     vec2     vec3
        // x*dirx + y*diry + z*dirz =( posx*dirx + posy*diry + posz*dirz )
        // x*dirx + y*diry + z*dirz =( posx*dirx + posy*diry + posz*dirz )
        // x*dirx + y*diry + z*dirz =( posx*dirx + posy*diry + posz*dirz )
        Vector3d vec1,vec2,vec3,sum;
        vec1[0]=n1[0]; vec1[1]= n2[0] ; vec1[2] = n3[0];
        vec2[0]=n1[1]; vec2[1]= n2[1] ; vec2[2] = n3[1];
        vec3[0]=n1[2]; vec3[1]= n2[2] ; vec3[2] = n3[2];
        sum[0]=p1.dot(n1);
        sum[1]=p2.dot(n2);
        sum[2]=p3.dot(n3);
        return linsystem( vec1,vec2,vec3,sum,res,detptr);
}

int cut_face_line(Vector3d fp, Vector3d fn, Vector3d lp, Vector3d ld, Vector3d &res, double *detptr)
{
	Vector3d c1 = fn.cross(ld);
	Vector3d c2 = fn.cross(c1);
	Vector3d diff=fp-lp;
        if(linsystem(ld, c1, c2, diff,res,detptr)) return 1;
	res=lp+ld*res[0];
	return 0;
}

typedef std::vector<int> intList;
typedef std::vector<intList> intListList;

class TriCombineStub
{
	public:
		int ind1, ind2, ind3 ;
		int operator==(const TriCombineStub ref)
		{
			if(this->ind1 == ref.ind1 && this->ind2 == ref.ind2) return 1;
			return 0;
		}
};

unsigned int hash_value(const TriCombineStub& r) {
	unsigned int i;
	i=r.ind1 |(r.ind2<<16) ;
        return i;
}

int operator==(const TriCombineStub &t1, const TriCombineStub &t2) 
{
	if(t1.ind1 == t2.ind1 && t1.ind2 == t2.ind2) return 1;
	return 0;
}
//
// combine all tringles into polygons where applicable
typedef std::vector<IndexedFace> indexedFaceList;
static indexedFaceList stl_tricombine(const std::vector<IndexedFace> &triangles)
{
	int i,j,n;
	int ind1, ind2;
	std::unordered_set<TriCombineStub, boost::hash<TriCombineStub> > stubs_pos;
	std::unordered_set<TriCombineStub, boost::hash<TriCombineStub> > stubs_neg;

	TriCombineStub e;
	for(i=0;i<triangles.size();i++)
	{
		n=triangles[i].size();
		for(j=0;j<n;j++)
		{
			ind1=triangles[i][j];
			ind2=triangles[i][(j+1)%n];
			if(ind2 > ind1) // positive edge
			{
				e.ind1=ind1;
				e.ind2=ind2;
				if(stubs_neg.find(e) != stubs_neg.end()) stubs_neg.erase(e);
				else if(stubs_pos.find(e) != stubs_pos.end()) printf("Duplicate Edge %d->%d \n",ind1,ind2);
				else stubs_pos.insert(e);
			}
			if(ind2 < ind1) // negative edge
			{
				e.ind1=ind2;
				e.ind2=ind1;
				if(stubs_pos.find(e) != stubs_pos.end() ) stubs_pos.erase(e);
				else if(stubs_neg.find(e) != stubs_neg.end() ) printf("Duplicate Edge %d->%d \n",ind2,ind1);
				else stubs_neg.insert(e);
			}
		}
	}	

	// now chain everything
	std::unordered_map<int,int> stubs_chain;
	std::vector<TriCombineStub> stubs;
	std::vector<TriCombineStub> stubs_bak;
	for( const auto& stubs : stubs_pos ) {
		if(stubs_chain.count(stubs.ind1) > 0)
		{
			stubs_bak.push_back(stubs);
		} else stubs_chain[stubs.ind1]=stubs.ind2;
	}

	for( const auto& stubs : stubs_neg ) {
		if(stubs_chain.count(stubs.ind2) > 0)
		{
			TriCombineStub ts;
			ts.ind1=stubs.ind2;
			ts.ind2=stubs.ind1;
			stubs_bak.push_back(ts);
		} else stubs_chain[stubs.ind2]=stubs.ind1;
	}
	std::vector<IndexedFace> result;

	while(stubs_chain.size() > 0)
	{
		int ind,ind_new;
		auto [ind_start,dummy]  = *(stubs_chain.begin());
		ind=ind_start;
		IndexedFace poly;
		while(1)
		{
			if(stubs_chain.count(ind) > 0)
			{
				ind_new=stubs_chain[ind];
				stubs_chain.erase(ind);
			}
			else
			{
				ind_new=-1;
				for(i=0;ind_new == -1 && i<stubs_bak.size();i++)
				{
					if(stubs_bak[i].ind1 == ind)
					{
						ind_new=stubs_bak[i].ind2;
						std::vector<TriCombineStub>::iterator it=stubs_bak.begin();
						std::advance(it,i);
						stubs_bak.erase(it);
						break;
					}
				}
				if(ind_new == -1) break;
			}
			poly.push_back(ind_new);
			if(ind_new == ind_start) break; // chain closed
			ind=ind_new;
		}

		// spitz-an-spitz loesen, in einzelketten trennen 
		int beg,end,dist,distbest,begbest,repeat;
		do
		{
			repeat=0;
			std::unordered_map<int,int> value_pos;
			distbest=-1;
			n=poly.size();
			for(i=0;i<n;i++)
			{
				ind=poly[i];
				if(value_pos.count(ind) > 0)
				{
					beg=value_pos[ind];
					end=i;
					dist=end-beg;
					std::unordered_map<int,int> value_pos2;
					int doubles=0;
					for(j=beg;j<end;j++)
					{
						if(value_pos2.count(poly[j]) > 0)
							doubles=1;
						value_pos2[poly[j]],j;
					}

					if(dist > distbest && doubles == 0) // es duerfen sich keine doppelten zahlen drinnen befinden
					{
						distbest=dist;
						begbest=beg;
					}
					if(end-beg == 2)
					{
						for(int i=0;i<2;i++) { // TODO make more efficient
							auto it=poly.begin();
							std::advance(it,beg);
							poly.erase(it);
						}
						repeat=1;
						distbest=-1;
						break;
					}
					if(beg+n-end == 2)
					{
						IndexedFace polynew;
						for(j=beg;j<end;j++)
							polynew.push_back(poly[j]);
						poly=polynew;
						repeat=1;
						distbest=-1;
						break;
					}
				}
				value_pos[ind]=i;
			}
			if(distbest  != -1)
			{
				IndexedFace polynew;
				for(i=begbest;i<begbest+distbest;i++)
				{
					polynew.push_back(poly[i]);
				}
				if(polynew.size() >= 3) result.push_back(polynew);
				for(int i=0;i<distbest;i++) { // TODO make more efficient
					auto  it=poly.begin();
					std::advance(it,begbest);
					poly.erase(it);
				}
				repeat=1;
			}
		}
		while(repeat);
		
		if(poly.size() > 2) result.push_back(poly);
	}
	return result;
}
bool offset3D_opposite(const IndexedFace & poly1, const IndexedFace &poly2)
{
	if(poly1.size() != poly2.size()) return false;
	int n=poly1.size();
	int off=-1;
	for(int i=0;off == -1 && i<n;i++)
		if(poly1[0] == poly2[i]) off=i;
	if(off == -1) return false;

	for(int i=1;i<n;i++)
		if(poly1[i] != poly2[(off+n-i)%n])
			return false;

	return true;
}


bool offset3D_pointInPolygon(const std::vector<Vector3d> &vert, const IndexedFace &bnd, int ptind)
{
	int i,n;
	double dist;
	n=bnd.size();
	int cuts=0;
	Vector3d fdir, fnorm, p1, p2;
	Vector3d pt=vert[ptind];
	if(n < 3) return false;
	Vector3d raydir=vert[bnd[1]]-vert[bnd[0]];
	Vector3d fn=raydir.cross(vert[bnd[1]]-vert[bnd[2]]).normalized();
	// check, how many times the ray crosses the 3D fence, classical algorithm in 3D
	for(i=1;i<n;i++) { // 0 is always parallel
		// build fence side
		const Vector3d &p1=vert[bnd[i]];
		const Vector3d &p2=vert[bnd[(i+1)%n]];
		fdir=p2-p1;
		fnorm=fdir.cross(fn);

		// make sure, fence is ahead
		if(fabs(fnorm.dot(raydir)) < 1e-5) continue;

		dist = (pt-p1).dot(fnorm);
		if(fnorm.dot(raydir) > 0) dist=-dist;
		if(dist < 0) continue;

		// make sure begin of fence is on the right
		dist= (pt-p1).dot(fdir);
		if(dist < 0) continue;

		// make sure end of fence is on the left
		dist= (pt-p2).dot(fdir);
		if(dist > 0) continue;
		cuts++;
	}
	return (cuts&1)?true:false;
}

using Eigen::Vector4d;

std::vector<IndexedFace> mergetriangles(const std::vector<IndexedFace> triangles,const std::vector<Vector4d> normals,std::vector<Vector4d> &newNormals, const std::vector<Vector3d> &vert) 
{
	int i,j,k;
	int n;
	indexedFaceList emptyList;
	std::vector<Vector4d> norm_list;
	std::vector<indexedFaceList>  triangles_sorted;
	// sort triangles into buckets of same orientation
	for(int i=0;i<triangles.size();i++) {
		Vector4d norm=normals[i];
		const IndexedFace &triangle = triangles[i]; 

		int norm_ind=-1;
		for(int j=0;norm_ind == -1 && j<norm_list.size();j++) {
			const auto &cur = norm_list[j];
			if(cur.head<3>().dot(norm.head<3>()) > 0.999 && fabs(cur[3] - norm[3]) < 0.001) {
				norm_ind=j;
			}
		}
		if(norm_ind == -1) {
			norm_ind=norm_list.size();
			norm_list.push_back(norm);
//			printf("new norm %g/%g/%g/%g for tri with %d pts\n",norm[0], norm[1], norm[2], norm[3],triangles[i].size());
			triangles_sorted.push_back(emptyList);
		}
		triangles_sorted[norm_ind].push_back(triangle);
	}

	//  now put back hole of polygons into the correct bucket
	printf("Buckets: %d\n",triangles_sorted.size());
	for(int i=0;i<triangles_sorted.size();i++ ) {
		// check if bucket has an opposite oriented bucket
		Vector4d n = norm_list[i];
		int i_=-1;
		Vector3d nref = norm_list[i].head<3>();
		for(int j=0;j<triangles_sorted.size();j++) {
			if(j == i) continue;
			if(norm_list[j].head<3>().dot(nref) < -0.999 && fabs(norm_list[j][3] + norm_list[i][3]) < 0.005) {
				i_=j;
				break;
			}
		}
		if(i_ == -1) continue;
		// assuming that i_ contains the holes, find, it there is a match

		for(int k=0;k< triangles_sorted[i].size();k++) {
			IndexedFace poly = triangles_sorted[i][k];
			for(int l=0;l<triangles_sorted[i_].size();l++) {
				IndexedFace hole = triangles_sorted[i_][l];
//				// holes dont intersect with the polygon, so its sufficent to check, if one point of the hole is inside the polygon
				if(offset3D_opposite(poly, hole)){
					triangles_sorted[i].erase(triangles_sorted[i].begin()+k);
					triangles_sorted[i_].erase(triangles_sorted[i_].begin()+l);
					k--;
					l--;
				}
				else if(offset3D_pointInPolygon(vert, poly, hole[0])){
					triangles_sorted[i].push_back(hole);
					triangles_sorted[i_].erase(triangles_sorted[i_].begin()+l);
					l--; // could have more holes
				}
			}
		}
		
		
	}

	// now merge the triangles in all buckets independly
	std::vector<IndexedFace> indices;
	newNormals.clear();
	for(int i=0;i<triangles_sorted.size();i++ ) {
		indexedFaceList indices_sub = stl_tricombine(triangles_sorted[i]);
		for(int j=0;j<indices_sub.size();j++) {
			indices.push_back(indices_sub[j]);
			newNormals.push_back(norm_list[i]);
		}
	}


	return indices;
}

void offset3D_RemoveColinear(const std::vector<Vector3d> &vertices, std::vector<IndexedFace> &indices, std::vector<intList> &pointToFaceInds, std::vector<intList> &pointToFacePos)
{
	
	// ----------------------------------------------------------
	// create a point database and use it. and form polygons
	// ----------------------------------------------------------
	pointToFaceInds.clear();
	pointToFacePos.clear();
	intList emptyList;
	for(int i=0;i<vertices.size();i++) {
		pointToFaceInds.push_back(emptyList);
		pointToFacePos.push_back(emptyList);
	}

	// -------------------------------
	// calculate point-to-polygon relation
	// -------------------------------
	for(int i=0;i<indices.size();i++) {
		IndexedFace pol = indices[i];
		for(int j=0;j<pol.size(); j++) {
			pointToFaceInds[pol[j]].push_back(i);
			pointToFacePos[pol[j]].push_back(j);
		}
	}
	
	for(int i=0;i<pointToFaceInds.size();i++) {
		const auto &index = pointToFaceInds[i];
		if(index.size() != 2) continue; // only works with 2 point uses
		int valid=1;
		for(int j=0;valid && j<2;j++) {
			const IndexedFace &face = indices[index[j]];
			int n=face.size();
			int pos = pointToFacePos[i][j];

			const Vector3d &prev =  vertices[face[(pos+n-1)%n]];
			const Vector3d &cur =   vertices[face[pos%n]];
			const Vector3d &next =  vertices[face[(pos+1)%n]];
			Vector3d d1=cur-prev;
			Vector3d d2=next-cur;
			if(d1.norm() < 0.001) continue; 
			if(d2.norm() < 0.001) continue;
			d1.normalize();
			d2.normalize();
			if(d1.dot(d2) < 0.999) valid=0;
		}
		if(valid){
			for(int j=0; j<2;j++) {
				int faceind=index[j];
				int pointpos = pointToFacePos[i][j];
				IndexedFace &face = indices[faceind];
				face.erase(face.begin()+pointpos);
				// db anpassen
				for(int k=0;k<pointToFaceInds.size();k++) {
					for(int l=0;l<pointToFaceInds[k].size();l++) {
						if(pointToFaceInds[k][l] != faceind) continue;
						if(pointToFacePos[k][l] > pointpos) pointToFacePos[k][l]--;
						else if(pointToFacePos[k][l] == pointpos) {
							pointToFaceInds[k].erase(pointToFaceInds[k].begin()+l);
							pointToFacePos[k].erase(pointToFacePos[k].begin()+l);
							l--;
						}

					}
				}

			}
		}
	}
}

void offset_3D_dump(const std::vector<Vector3d> &vertices, const std::vector<IndexedFace> &indices)
{
	printf("Vertices:%d Indices:%d\n",vertices.size(),indices.size());
	for(int i=0;i<vertices.size();i++)
	{
		printf("%d: \t%g/\t%g/\t%g\n",i,vertices[i][0], vertices[i][1] ,vertices[i][2]);
	}
	printf("===========\n");
	for(int i=0;i<indices.size();i++)
	{
		auto &face=indices[i];
		printf("%d :",i);
		for(int j=0;j<face.size();j++)
			printf("%d ",face[j]);
		printf("\n");
	}
}

Vector4d offset3D_normal(const std::vector<Vector3d> &vertices,const IndexedFace &pol)
{
	int n=pol.size();
	Vector3d norm(0,0,0);
	for(int j=0;j<n-2;j++) {
		// need to calculate all normals, as 1st 2 could be in a concave corner
		Vector3d diff1=(vertices[pol[0]] - vertices[pol[j+1]]);
		Vector3d diff2=(vertices[pol[j+1]] - vertices[pol[j+2]]);
		norm += diff1.cross(diff2);
	}
	norm.normalize();
	Vector3d pt=vertices[pol[0]];
	double off=norm.dot(pt);
	return Vector4d(norm[0],norm[1],norm[2],off);
}


std::vector<Vector4d> offset3D_normals(const std::vector<Vector3d> &vertices, const std::vector<IndexedFace> &indices)
{
	std::vector<Vector4d>  faceNormal;
	for(int i=0;i<indices.size();i++) {
		IndexedFace pol = indices[i];
		assert (pol.size() >= 3);
		faceNormal.push_back(offset3D_normal(vertices, pol));
	}
	return faceNormal;

}
double offset3D_angle(const Vector3d &refdir, const Vector3d &edgedir, const Vector3d &facenorm)
{
	double c,s;
	Vector3d tmp=refdir.cross(edgedir);
	c=refdir.dot(edgedir);
	s=tmp.norm();
	if(tmp.dot(facenorm) < 0) s=-s;
	double ang=atan2(s,c)*180/3.1415926;
	if(ang < 0) ang += 360;
	return ang;
}

void offset3D_calculateNefInteract(const std::vector<Vector4d> &faces, std::vector<IndexedFace> &faceinds,int selfind,  int newind) {
//	printf("Interact selfind is %d, newind=%d\n",selfind, newind);
	if(faces[selfind].head<3>().dot(faces[newind].head<3>()) < -0.99999) return;
	if(faceinds[selfind].size() == 0) {
		faceinds[selfind].push_back(newind); 
		return;
	}

	// calculate the angles of the cuts and find out position of newind
	// calculate refedge
	Vector3d facenorm=faces[selfind].head<3>();
	Vector3d refdir=facenorm.cross(faces[faceinds[selfind][0]].head<3>()).normalized();
//	printf("norm is %g/%g/%g\n",facenorm[0], facenorm[1], facenorm[2]);
//	printf("refdir is %g/%g/%g\n",refdir[0], refdir[1], refdir[2]);
	// TODO angles nicht neu berechnen, sondern uebernehmen
	Vector3d edgedir;
	double angle;
	std::vector<double> angles;
	for(int j=0;j<faceinds[selfind].size();j++) {
		edgedir=facenorm.cross(faces[faceinds[selfind][j]].head<3>()).normalized();
		angle=offset3D_angle(refdir, edgedir, facenorm);
		angles.push_back(angle);
//		printf("Edge %d angle=%g\n",faceinds[selfind][j],angle);				
	}
	edgedir=facenorm.cross(faces[newind].head<3>()).normalized();
	angle=offset3D_angle(refdir, edgedir, facenorm);
//	printf("New Ang is %g\n",angle);

	// now insert newind in the right place
	IndexedFace faceindsnew;
	for(int j=0;j<faceinds[selfind].size();j++) {
		if(fabs(angle-angles[j]) < 0.001) {
			printf("Angle equal selfind=%d \n",selfind);
			// wer schneidet mehr ein: faceinds[selfind][j]  oder newind
			// Testpunkt ist punkt auf selfind
			Vector3d testpt=faces[selfind].head<3>() * faces[selfind][3];
			int presind=faceinds[selfind][j];
			double pres_dist = testpt.dot(faces[presind].head<3>())-faces[presind][3];
			double new_dist = testpt.dot(faces[newind].head<3>())-faces[newind][3];
			if(pres_dist > new_dist) { printf("disregard new one\n"); angle=1e9; } // keep it, never insert it
			else { faceindsnew.push_back(newind);  printf("use new one\n"); angle=1e9; continue; } // insert new one instead							    
		} else if(angle < angles[j]) {
			faceindsnew.push_back(newind);
			angle=1e9;
		}
		faceindsnew.push_back(faceinds[selfind][j]);
	}
	if(angle < 1e9) faceindsnew.push_back(newind);
	faceinds[selfind] = faceindsnew;


	// TODO neues edge kann andere eliminiren(above) , eine reihe eliminieren
	// TODO new edge kann andere nur streifen, kann gaenzlich egal sein, wenn es zu weit weg ist
}
void offset3D_calculateNefPolyhedron(const std::vector<Vector4d> &faces,std::vector<Vector3d> &vertices, std::vector<IndexedFace> & indices)
{
	int i;
	std::vector<IndexedFace> nef_db;
	// a nef contains an entry for each face showing the correct order of the surrounding edges
	// each with everybody
	for(i=0;i<faces.size();i++) {
		// add each single face in  a sequence to the database;
		int orgsize=nef_db.size();
		IndexedFace new_face;
		nef_db.push_back(new_face);
		for(int j=0;j<orgsize;j++) {
			offset3D_calculateNefInteract(faces, nef_db,j, i);
			offset3D_calculateNefInteract(faces, nef_db, orgsize,j);
		}
	}
	// Now dump the NEF db
	printf("NEF DB\n");
	for(i=0;i<nef_db.size();i++)
	{
		printf("Face %d: ",i);
		for(int j=0;j<nef_db[i].size();j++)
			printf("%d ",nef_db[i][j]);
		printf("\n");
	}

	// synthesize the db and form polygons
  	Reindexer<Vector3d> vertices_;
	for(int i=0;i<nef_db.size();i++) {
		IndexedFace face;
		IndexedFace &db=nef_db[i];
		Vector3d d1=faces[i].head<3>();
		Vector3d p1=d1*faces[i][3];
		for(int j=0;j<db.size();j++)
		{
			int k=(j+1)%db.size();
			Vector3d d2=faces[db[j]].head<3>();
			Vector3d p2=d2*faces[db[j]][3];

			Vector3d d3=faces[db[k]].head<3>();
			Vector3d p3=d3*faces[db[k]][3];
			// cut face i, db[j], db[j+1]
			Vector3d newpt;
			if(cut_face_face_face( p1, d1, p2, d2, p3, d3, newpt)) printf("Problem during cut!\n");
//			printf("newpt is %g/%g/%g\n",newpt[0], newpt[1], newpt[2]);
      			face.push_back(vertices_.lookup(newpt));
		}
		indices.push_back(face);
	}
	vertices_.copy(std::back_inserter(vertices));
}

std::vector<std::shared_ptr<PolySet>>  offset3D_decompose(const std::vector<Vector3d> &vertices, std::vector<IndexedFace> indices)
{
	int count=0;
	std::vector<std::shared_ptr<PolySet>> results;
	if(indices.size() == 0) return results;
	std::vector<int> faces_done; // for one result
	std::vector<int> faces_convex; // for one result
	std::unordered_set<int> faces_included;

	std::vector<Vector3d> faces_refpt; // TODO nicht notwendig mit 4d norm
	std::vector<Vector3d> faces_norm;

	std::vector<int> faces_todo_face;
	std::vector<int> faces_todo_edge;
	//
	// edge db aufbauen
	std::unordered_map<TriCombineStub, int, boost::hash<TriCombineStub> > edge_db; // edge -> face
	TriCombineStub stub;							
	for(int i=0;i<indices.size();i++) {
		auto &face = indices[i];
		int n=face.size();
		for(int j=0;j<n;j++) {
			stub.ind1=face[j];
			stub.ind2=face[(j+1)%n];
			edge_db[stub]=i;
		}
	}

	int oppind;
	bool valid;
	while(1) {
		for(int i=0;i<indices.size();i++)
		{
			if(faces_included.count(i) == 0) {
//				printf("Chosen to start with %d\n",i);
				oppind=i;
				valid=true;
				faces_done.clear();
				faces_convex.clear();
				faces_refpt.clear();
				faces_norm.clear();
				goto v; // start with success
			}
		}
		return results;

		while(faces_todo_face.size() > 0) {
			{
				int faceind=faces_todo_face[0];
				int edgeind=faces_todo_edge[0];
//				printf("Doing Face %d/%d | %d in queue \n",faceind, edgeind,faces_todo_face.size());
				//
				//which is the opposite face
				int n = indices[faceind].size();
				stub.ind1=indices[faceind][(edgeind+1)%n];	 
				stub.ind2=indices[faceind][edgeind];	
			}
			faces_todo_face.erase(faces_todo_face.begin());
			faces_todo_edge.erase(faces_todo_edge.begin());
			if(edge_db.count(stub) > 0) {
				oppind=edge_db[stub];
				if(find(faces_done.begin(), faces_done.end(), oppind) != faces_done.end()){
					continue;
				}	
				// check if all pts of opp+_ind are below all planes
				{
					auto &oppface = indices[oppind];
					int no = oppface.size();
//					printf("Checking face %d :%d vertices against %d faces\n",oppind, no, faces_refpt.size());
					valid=true;
					for(int i=0;valid && i<no;i++) {
						Vector3d pt=vertices[oppface[i]];
						for(int j=0;valid && j<faces_refpt.size();j++) {
							double off=(pt-faces_refpt[j]).dot(faces_norm[j]);
							if(off > 1e-6){
							       	valid=false;
							}
						}
					}
				}
v:				faces_done.push_back(oppind);
				for(int i=0;i<indices[oppind].size();i++) {
					faces_todo_face.push_back(oppind);
					faces_todo_edge.push_back(i);
				}
				if(valid){
					faces_refpt.push_back(vertices[indices[oppind][0]]);
					faces_norm.push_back(offset3D_normal(vertices, indices[oppind]).head<3>());
					faces_convex.push_back(oppind);
					faces_included.insert(oppind);
				}
				count++;
	
			}
		}
	
		std::vector<IndexedFace> result;
		printf("Result ");
		for(int i=0;i<faces_convex.size();i++)
		{
			result.push_back(indices[faces_convex[i]]);
			printf("%d ",faces_done[i]);
		}
		printf("\n");
		// TODO calculate normals
		std::vector<Vector4d> normals;
		for(int i=0;i<faces_convex.size();i++)
		{
			Vector4d norm = offset3D_normal(vertices, indices[faces_convex[i]]);
			int j;
			for(j=0;j<normals.size();j++) {
				if(normals[j].head<3>().dot(norm.head<3>()) > 0.999 && fabs(normals[j][3] - norm[3]) < 0.001) break;
			}
			if(j == normals.size()) normals.push_back(norm);
		}
		printf("Normals\n");
		for(int i=0;i<normals.size();i++)
		{
			printf("%g/%g/%g/%g\n",normals[i][0],normals[i][1], normals[i][2], normals[i][3]);
		}
//		if(results.size() == 0) { // TODO fix
			std::vector<Vector3d> vertices1;
			std::vector<IndexedFace> indices1;
			offset3D_calculateNefPolyhedron(normals, vertices1, indices1);					

			PolySet *decomposed =  new PolySet(3,  true);
			decomposed->vertices = vertices1;
			decomposed->indices = indices1;
			results.push_back( std::shared_ptr<PolySet>(decomposed));
//		}

	}
	return results;
}

extern std::vector<SelectedObject> debug_pts;
std::vector<IndexedFace>  offset3D_removeOverPoints(const std::vector<Vector3d> &vertices, std::vector<IndexedFace> indices,int  round)
{
	// go  thourh all indicices and calculate area
	printf("Remove OverlapPoints\n");
//	debug_pts.clear();

	std::vector<IndexedFace> indicesNew;
	indicesNew.clear();
	std::vector<int> mapping;
	std::vector<int> uselist;
	for(int i=0;i<vertices.size();i++) mapping.push_back(i);
	for(int i=0;i<indices.size();i++) {
		auto &face=indices[i];
		IndexedFace facenew;
		int n=face.size();
		if(n < 3) continue;

		// ---------------------------------
		// skip overlapping vertices
		// ---------------------------------
		int curind=face[0];
		for(int j=0;j<n;j++) {
			int nextind=face[(j+1)%n];
			Vector3d d=vertices[nextind]-vertices[curind];
			if( d.norm() > 3e-3 ){
				curind=nextind;
				facenew.push_back(curind);
			} else {
				int newval=curind;
				while(newval != mapping[newval]) newval=mapping[newval];
				mapping[nextind]=newval;
			}
		}
	}

	for(int i=0;i<indices.size();i++) {
		auto &face=indices[i];
		IndexedFace facenew;
		int  indold=-1;
		for(int j=0;j<face.size();j++) {
			int ind=mapping[face[j]];
			if(ind == indold) continue;
			if(facenew.size() > 0 && ind == facenew[0]) continue; 
			facenew.push_back(ind);
			indold=	ind;
		}
		if(facenew.size() >= 3) indicesNew.push_back(facenew);
	}

	return indicesNew;
}


void offset3D_displayang(Vector3d dir){
	double pl=sqrt(dir[0]*dir[0]+dir[1]*dir[1]);
	printf("ang=%g elev=%g ",atan2(dir[1], dir[0])*180.0/3.14,atan2(dir[2],pl)*180.0/3.14);
}
double offset3D_area(Vector3d p1, Vector3d p2, Vector3d p3, Vector4d n) {
	double l1=(p2-p3).norm();
	double l2=(p2-p1).norm();
	double ang=acos((p2-p3).dot(p2-p1)/(l1*l2))*180/3.14;


	Vector3d c = (p2-p3).cross(p2-p1);
	double l=c.norm();
	if(c.dot(n.head<3>()) < 0) l=-l;
	return l;
}

std::vector<Vector3d> offset3D_offset(std::vector<Vector3d> vertices, std::vector<IndexedFace> &indices,const std::vector<Vector4d> &faceNormal, std::vector<intList> &pointToFaceInds, std::vector<intList> &pointToFacePos, double &off, int round) // off always contains the value, which is still to be sized
{														    
//	offset_3D_dump(vertices, indices);
	// -------------------------------
	// now calc length of all edges
	// -------------------------------
	std::vector<double> edge_len, edge_len_factor;
	double edge_len_min=0;
	if(off < 0) {
		for(int i=0;i<indices.size();i++)
		{
			auto &pol = indices[i];
			int n=pol.size();
			for(int j=0;j<n;j++){
				int a=pol[j];
				int b=pol[(j+1)%n];
				if(b > a) {
					double dist=(vertices[b]-vertices[a]).norm();
					edge_len.push_back(dist);
					if(edge_len_min == 0 || edge_len_min > dist) edge_len_min=dist;
				}
			}
		}
	}
	double off_act=off;
	if(off_act < 0) off_act=-edge_len_min/5.0; // TODO ist das sicher ?
	std::vector<Vector3d> verticesNew=vertices;
	std::map<int, intList> cornerFaces;
	for(int i=0;i<pointToFaceInds.size();i++) { // go through all vertices

		// -------------------------------
		// Find where several solid corners  share a single vertex
		// -------------------------------
		const intList &db=pointToFaceInds[i];		
		// now build chains
		//
		std::unordered_map<int, TriCombineStub> stubs;
		TriCombineStub s;
		// build corner edge database
//		printf("Vertex %d faces is %d\n",i,db.size());	
		for(int j=0;j<db.size();j++) {					   
			const IndexedFace &pol = indices[db[j]];
			int n=pol.size();
			int pos=pointToFacePos[i][j];
			int prev=pol[(pos+n-1)%n];
			int next=pol[(pos+1)%n];
			s.ind1=next; 
			s.ind2=db[j];
			s.ind3=pos;
			if(prev != next) stubs[prev]=s;
//			printf("%d -> %d\n",prev, next);
			// stubs are vertind -> nextvertind, faceind
		}
		std::vector<intList> polinds, polposs;
		std::unordered_set<int> vertex_visited;

		for(  auto it=stubs.begin();it != stubs.end();it++ ) {
			// using next available
			int ind=it->first;
			if(vertex_visited.count(ind) > 0) continue;
			int begind= ind;
			intList polind;
			intList polpos;
			do
			{
				polind.push_back(stubs[ind].ind2);
				polpos.push_back(stubs[ind].ind3);
				int indnew=stubs[ind].ind1;
				vertex_visited.insert(ind);
				if(ind == indnew) {
					printf("Programn error, loop! %d\n",ind); // TODO keep to catch a bug
					exit(1);
				}
				ind=indnew;
			} while(ind != begind);
			polinds.push_back(polind);
			polposs.push_back(polpos);
		}
//		printf("polinds size is: %d\n",polinds.size());
//		if(db.size() < 6) continue; // not possible  with less than 6 faces

		if(polinds.size() > 1) {
			for(int j=0;j<polinds.size();j++) { // 1st does not need treatment
				int vertind;
				if(j > 0) {
					vertind=vertices.size();
					vertices.push_back(vertices[i]);
					verticesNew.push_back(vertices[i]);
					pointToFaceInds.push_back(polinds[j]);
					pointToFacePos.push_back(polposs[j]);
					auto &polind =polinds[j];
					for(int k=0;k<polind.size();k++){
						IndexedFace &face = indices[polind[k]];
						for(int l=0;l<face.size();l++) {
							if(face[l] == i) face[l]=vertind;
						}
					}
				} else {
					vertind=i;
					pointToFaceInds[vertind]=polinds[j];
					pointToFacePos[vertind]=polposs[j];
				}
				cornerFaces[vertind] = polinds[j];
			}
		} 
		else if(polinds.size() == 1) cornerFaces[i] = polinds[0];
	} 
	
	std::vector<TriCombineStub> keile;
	for( int i = 0; i< pointToFaceInds.size();i++ ) {

		// ---------------------------------------
		// Calculate offset position to each point
		// ---------------------------------------

		Vector3d newpt;
		intList faceInds = pointToFaceInds[i];
		int valid;

		Vector3d oldpt=vertices[i];

		if(faceInds.size() >= 4) {
			if(cornerFaces.count(i) > 0) {
				auto &face_order = cornerFaces[i]; // alle faceinds rund um eine ecke
							   //
				int n=face_order.size();
				// create face_vpos
				std::vector<int> face_vpos; // TODO ist diese info schon verfuegbar ?
				for(int j=0;j<n;j++) {
					int pos=-1;
					auto &face= indices[face_order[j]];
					for(int k=0;k<face.size();k++)
						if(face[k] == i) pos=k;
					assert(pos != -1);
					face_vpos.push_back(pos);
				}				
	
				printf("Special alg for Vertex %d\n",i);
	
				IndexedFace mainfiller; // core of star

				int newind=verticesNew.size();
				
				for(int j=0;j<n;j++) {
					vertices.push_back(oldpt);
					verticesNew.push_back(oldpt);
				}
				std::vector<int> faces1best, faces2best;
				for(int j=0;j<n;j++) {
					int ind1, ind2, ind3;
					ind1=face_order[j];
					Vector3d xdir=faceNormal[ind1].head<3>();

					auto &face =indices[face_order[j]];  // alle punkte einer flaeche
					int nf=face.size();
					int pos=-1;
					for(int k=0;k<nf;k++)
						if(face[k] == i) pos=k;
					assert(pos != -1);

					double minarea=1e9;
					Vector3d bestpt;
					int face1best=-1, face2best=-1;
					for(int k=0;k<n;k++) {
						if(face_order[k] == ind1) continue;
						ind2=face_order[k]; 
						Vector3d ydir=faceNormal[ind2].head<3>();

						for(int l=0;l<n;l++) {

							if(face_order[l] == ind1) continue;
							if(face_order[l] == ind2) continue;
							ind3 = face_order[l];
							Vector3d zdir=faceNormal[ind3].head<3>();

							if(!cut_face_face_face( oldpt  +xdir*off_act  , xdir, oldpt  +ydir*off_act  , ydir, oldpt  +zdir*off_act  , zdir, newpt)){
								double area = offset3D_area(vertices[face[(pos+n-1)%nf]],newpt,vertices[face[(pos+1)%nf]],faceNormal[face_order[j]]); 
								if(area < minarea) {
									minarea = area;
									bestpt = newpt;
									face1best=face_order[k];
									face2best=face_order[l];
								}
							}
						}
					}
					printf("bestpt is %g/%g/%g\n",bestpt[0], bestpt[1], bestpt[2]);
					mainfiller.insert(mainfiller.begin(),newind+j); // insert in reversed order
					verticesNew[newind+j]=bestpt;
					faces1best.push_back(face1best);
					faces2best.push_back(face2best);
				}
				Vector3d area(0,0,0);
			 	for(int j=0;j<mainfiller.size()-2;j++) {
					Vector3d diff1=(verticesNew[mainfiller[0]] - verticesNew[mainfiller[j+1]]);
					Vector3d diff2=(verticesNew[mainfiller[j+1]] - verticesNew[mainfiller[j+2]]);
					area += diff1.cross(diff2);
				}
				printf("area is %g\n",area.norm());
				if(area.norm()  > 1e-6) { // only if points are actually diverging

					for(int j=0;j<face_order.size();j++) {
						auto &face= indices[face_order[j]];
						face[face_vpos[j]]=newind + j;
					}
	
					// insert missing triangles
					indices.push_back(mainfiller);
					for(int j=0;j<face_order.size();j++){
						auto &tmpface = indices[face_order[j]]; 
						int n1=tmpface.size();
			        		int commonpt= tmpface[(face_vpos[j]+1)%n1]; 
			
						IndexedFace filler;
						filler.push_back(newind+j);
						filler.push_back(newind+((j+1)%face_order.size()));
						filler.push_back(commonpt);
	
						TriCombineStub stub;
						stub.ind1=indices.size(); // index of stub
						stub.ind2=faces1best[j];
						stub.ind3=faces2best[j];;
						keile.push_back(stub);
						indices.push_back(filler);
					}
				} else 	verticesNew[i]=verticesNew[mainfiller[0]];
				continue;
			}
		} else if(faceInds.size() == 3) {
			Vector3d dir0=faceNormal[faceInds[0]].head<3>();
			Vector3d dir1=faceNormal[faceInds[1]].head<3>();
			Vector3d dir2=faceNormal[faceInds[2]].head<3>();
			if(cut_face_face_face( oldpt  +dir0*off_act  , dir0, oldpt  +dir1*off_act  , dir1, oldpt  +dir2*off_act  , dir2, newpt)) { printf("problem during cut\n");  break; }
			verticesNew[i] = newpt;
			continue;
		} else if(faceInds.size() == 2) {
			Vector3d dir= faceNormal[faceInds[0]].head<3>() +faceNormal[faceInds[1]].head<3>() ;
			verticesNew[i] = vertices[i] + off_act* dir;
			continue;
		} else if(faceInds.size() == 1) {
			Vector3d dir= faceNormal[faceInds[0]].head<3>();
			verticesNew[i] = vertices[i] + off_act* dir;
			continue;
		}
	}
	printf("%d Keile found\n",keile.size());
	for(int i=0;i<keile.size();i++) {
		int faceind=keile[i].ind1;
		printf("keil faceind is %d\n",faceind);
		Vector3d d1=( verticesNew[indices[faceind][1]]-verticesNew[indices[faceind][0]]).normalized();
		Vector3d d2=( verticesNew[indices[faceind][1]]-verticesNew[indices[faceind][2]]).normalized();
		if(d1.cross(d2).norm() < 1e-6) { // TODO jeden keil mergen, nur muss dann die normale passen
			printf("Keil %dZero area others are %d %d\n", i,keile[i].ind2, keile[i].ind3);
			std::vector<IndexedFace> output;
			{
				std::vector<IndexedFace> input;
				input.push_back(indices[keile[i].ind2]);
				input.push_back(indices[keile[i].ind1]);
				output = stl_tricombine(input);
			}
			if(output.size() == 1) {
				printf("Successful merge #1 %d -> %d\n",keile[i].ind1, keile[i].ind2);
				indices[keile[i].ind2]=output[0];
				indices[keile[i].ind1].clear();
				continue;
			}
			{
				std::vector<IndexedFace> input;
				input.push_back(indices[keile[i].ind3]);
				input.push_back(indices[keile[i].ind1]);
				output = stl_tricombine(input);
			}
			if(output.size() == 1) {
				printf("Successful merge #2 %d -> %d\n",keile[i].ind1, keile[i].ind3);
				indices[keile[i].ind3]=output[0];
				indices[keile[i].ind1].clear();
				continue;
			}
			// now tryu to merge 
		}
	}
	// -------------------------------
	// now calc new length of all new edges
	// -------------------------------
	if(off < 0) {
		double off_max=-1e9;
		int cnt=0;
		for(int i=0;i<indices.size();i++)
		{
			auto &pol = indices[i];
			int n=pol.size();
			for(int j=0;j<n;j++){
				int a=pol[j];
				int b=pol[(j+1)%n];
				if(b > a) {
					double dist=(verticesNew[b]-verticesNew[a]).norm();
					double fact = (dist-edge_len[cnt])/off_act;
//					printf("%d - %d: %g %g \n",a,b, dist, fact);
					if(fabs(fact) > 0.0001) {
						// find maximal downsize value
						double t_off_max=-edge_len[cnt]/fact;
						if(off_max < t_off_max && t_off_max < 0) off_max=t_off_max;
						edge_len_factor.push_back(fact);
					}
					cnt++;
				}
			}
		}

		double off_do=off;
		printf("off_max is %g\n",off_max);
		if(off_do <  off_max) off_do=off_max;
		printf("off_do  is %g\n",off_do);
		for(int i=0;i<verticesNew.size();i++) {
			Vector3d d=(verticesNew[i]-vertices[i])*off_do/off_act;
			verticesNew[i]=vertices[i]+d;
		}			
		off -= off_do;
	}
	return verticesNew;
}


void  offset3D_reindex(const std::vector<Vector3d> &vertices, std::vector<IndexedFace> & indices, std::vector<Vector3d> &verticesNew, std::vector<IndexedFace> &indicesNew)
{
  indicesNew.clear();
  verticesNew.clear();  
  Reindexer<Vector3d> vertices_;
  for(int i=0;i<indices.size();i++) {
    auto face= indices[i];		
    if(face.size() < 3) continue;
    Vector3d diff1=vertices[face[1]] - vertices[face[0]].normalized();
    Vector3d diff2=vertices[face[2]] - vertices[face[1]].normalized();
    Vector3d norm = diff1.cross(diff2);
    if(norm.norm() < 0.0001) continue;
    IndexedFace facenew;
    for(int j=0;j<face.size();j++) {
      int ind=vertices_.lookup(vertices[face[j]]);
      if(facenew.size() > 0){
        if(facenew[0] == ind) continue;
        if(facenew[facenew.size()-1] == ind) continue;
      }
      facenew.push_back(ind);
    }
    if(facenew.size() >= 3) indicesNew.push_back(facenew);
  }
  vertices_.copy(std::back_inserter(verticesNew));
}

std::shared_ptr<PolySet> offset3D_convex(const std::shared_ptr<const PolySet> &ps,double off) {
  printf("Running offset3D %d polygons\n",ps->indices.size());
//  if(off == 0) return ps;
  std::vector<Vector3d> verticesNew;
  std::vector<IndexedFace> indicesNew;
  std::vector<Vector4d> normals;
  std::vector<intList>  pointToFaceInds, pointToFacePos;
  if(off > 0) { // upsize
    // TODO decomposition and assemble	
    std::vector<IndexedFace> indicesNew;
//    std::vector<std::shared_ptr<PolySet>> decomposed =  offset3D_decompose(ps->vertices, ps->indices);
//    printf("decompose results is %d\n",decomposed.size());

    printf("Remove OverlapPoints\n");
    std::vector<IndexedFace> indicesX = offset3D_removeOverPoints(ps->vertices, ps->indices,0);

    printf("Calc Normals\n");
    std::vector<Vector4d>  faceNormal=offset3D_normals(ps->vertices, indicesX);

    printf("Merge Trtiangles %d %d\n",indicesX.size(), faceNormal.size());
    std::vector<Vector4d> newNormals;
    indicesNew  = mergetriangles(indicesX,faceNormal,newNormals, ps->vertices );

    printf("Remove Colinear Points\n");
    offset3D_RemoveColinear(ps->vertices, indicesNew,pointToFaceInds, pointToFacePos);

    printf("offset\n");
    verticesNew = offset3D_offset(ps->vertices, indicesNew, newNormals, pointToFaceInds, pointToFacePos, off, 0);
    // -------------------------------
    // Map all points and assemble
    // -------------------------------
    PolySet *offset_result =  new PolySet(3,  true);
    offset_result->vertices = verticesNew;
    offset_result->indices = indicesNew;
    return std::shared_ptr<PolySet>(offset_result);
  } else {
    printf("Downsize %g\n",off);	  

    std::vector<Vector3d> vertices = ps->vertices;
    std::vector<IndexedFace> indices = ps->indices;
    int round=0;
    std::vector<Vector4d> normals;
    std::vector<Vector4d> newNormals;
    do
    {
      printf("New Round %d Vertices %d Faces %d\n=======================\n",round,vertices.size(), indices.size());	    

      indices  = offset3D_removeOverPoints(vertices, indices,round);
      printf("Reindex OP\n");
      offset3D_reindex(vertices, indices, verticesNew, indicesNew);
      vertices = verticesNew;
      indices = indicesNew;

      printf("Normals OP\n");
      normals = offset3D_normals(vertices, indices);

      printf("Merge OP\n");
      indicesNew  = mergetriangles(indices,normals,newNormals, vertices ); 
      normals = newNormals;									   
      indices = indicesNew;									       

      printf("Remove Colinear Points\n");
      offset3D_RemoveColinear(vertices, indices,pointToFaceInds, pointToFacePos);

      if(indices.size() == 0 || fabs(off) <  0.0001) break;

      printf("Offset OP\n");
      verticesNew = offset3D_offset(vertices, indices, normals, pointToFaceInds, pointToFacePos, off,round);
      vertices = verticesNew;
      round++;
    }
    while(1);
    offset_3D_dump(vertices, indices);
    // -------------------------------
    // Map all points and assemble
    // -------------------------------
    PolySet *offset_result =  new PolySet(3, /* convex */ true);
    offset_result->vertices = vertices;
    offset_result->indices = indices;
    return std::shared_ptr<PolySet>(offset_result);
  }
}
std::shared_ptr<Geometry> offset3D(const std::shared_ptr<const PolySet> &ps,double off) {
  bool enabled=false; // geht mit 4faces
		     // geht nicht mit boxes, segfault
		     // sphere proigram error
		     // singlepoint prog error
  if(!enabled) return offset3D_convex(ps, off);

  std::vector<std::shared_ptr<PolySet>> decomposed =  offset3D_decompose(ps->vertices, ps->indices);
  printf("Decomposed into %d parts\n",decomposed.size());
  if(decomposed.size() == 0) {
    PolySet *offset_result =  new PolySet(3, /* convex */ true);
    return std::shared_ptr<PolySet>(offset_result);
  }
  if(decomposed.size() == 1) {
  	return offset3D_convex(decomposed[0], off);
  }
  auto N = std::make_shared<ManifoldGeometry>();
  for(int i=0;i<decomposed.size();i++)
  {
  	auto term = ManifoldUtils::createMutableManifoldFromGeometry(offset3D_convex(decomposed[i], off));
//  	auto term = ManifoldUtils::createMutableManifoldFromGeometry(decomposed[i]);
	if(i == 0) N = term;
	else *N += *term;	
  }
  return N;
}

/*!
   Applies the operator to all child nodes of the given node.

   May return nullptr or any 3D Geometry object
 */
GeometryEvaluator::ResultObject GeometryEvaluator::applyToChildren3D(const AbstractNode& node, OpenSCADOperator op)
{
  Geometry::Geometries children = collectChildren3D(node);
  if (children.empty()) return {};

  if (op == OpenSCADOperator::HULL) {
    return {std::shared_ptr<Geometry>(applyHull(children))};
  } else if (op == OpenSCADOperator::FILL) {
    for (const auto& item : children) {
      LOG(message_group::Warning, item.first->modinst->location(), this->tree.getDocumentPath(), "fill() not yet implemented for 3D");
    }
  }

  // Only one child -> this is a noop
  if (children.size() == 1 && op != OpenSCADOperator::OFFSET) return {children.front().second};

  switch (op) {
  case OpenSCADOperator::MINKOWSKI:
  {
    Geometry::Geometries actualchildren;
    for (const auto& item : children) {
      if (item.second && !item.second->isEmpty()) actualchildren.push_back(item);
    }
    if (actualchildren.empty()) return {};
    if (actualchildren.size() == 1) return {actualchildren.front().second};
    return {applyMinkowski(actualchildren)};
    break;
  }
  case OpenSCADOperator::UNION:
  {
    Geometry::Geometries actualchildren;
    for (const auto& item : children) {
      if (item.second && !item.second->isEmpty()) actualchildren.push_back(item);
    }
    if (actualchildren.empty()) return {};
    if (actualchildren.size() == 1) return {actualchildren.front().second};
#ifdef ENABLE_MANIFOLD
    if (Feature::ExperimentalManifold.is_enabled()) {
      return {ManifoldUtils::applyOperator3DManifold(actualchildren, op)};
    }
#endif
#ifdef ENABLE_CGAL
    else if (Feature::ExperimentalFastCsg.is_enabled()) {
      return {std::shared_ptr<Geometry>(CGALUtils::applyUnion3DHybrid(actualchildren.begin(), actualchildren.end()))};
    }
    return {CGALUtils::applyUnion3D(actualchildren.begin(), actualchildren.end())};
#else
    assert(false && "No boolean backend available");
#endif
    break;
  }
  case OpenSCADOperator::OFFSET:
  {
    Geometry::Geometries actualchildren;
    for (const auto& item : children) {
      if (item.second && !item.second->isEmpty()) actualchildren.push_back(item);
    }
    const OffsetNode *offNode = dynamic_cast<const OffsetNode *>(&node);
    std::shared_ptr<const Geometry> geom;
    switch(actualchildren.size())
    {
      case 0:
        return {};
        break;
      case 1:
        geom ={actualchildren.front().second};
        break;
      default:
        geom = {CGALUtils::applyUnion3D(actualchildren.begin(), actualchildren.end())};
	break;
    }
 
    if(std::shared_ptr<const PolySet> ps = std::dynamic_pointer_cast<const PolySet>(geom)) {
      auto ps_offset =  offset3D(ps,offNode->delta);

      geom = std::move(ps_offset);
      return geom;
    } else if (std::dynamic_pointer_cast<const ManifoldGeometry>(geom)) {
      auto ps = PolySetUtils::getGeometryAsPolySet(geom);
      auto ps_offset =  offset3D(ps,offNode->delta);
      geom = std::move(ps_offset);
      return geom;
    } else if(const auto geomlist = std::dynamic_pointer_cast<const GeometryList>(geom).get()) {
      for (const Geometry::GeometryItem& item : geomlist->getChildren()) { // TODO
      }
        
    } else if (std::shared_ptr<const CGAL_Nef_polyhedron> nef = std::dynamic_pointer_cast<const CGAL_Nef_polyhedron>(geom)) {
      const CGAL_Nef_polyhedron nefcont=*(nef.get());
      std::shared_ptr<PolySet> ps = CGALUtils::createPolySetFromNefPolyhedron3(*(nefcont.p3));
      std::shared_ptr<Geometry> ps_offset =  offset3D(ps,offNode->delta);
      geom = std::move(ps_offset);
      return geom;
    } else if (const auto hybrid = std::dynamic_pointer_cast<const CGALHybridPolyhedron>(geom)) { // TODO
    }
  }
  default:
  {
#ifdef ENABLE_MANIFOLD
    if (Feature::ExperimentalManifold.is_enabled()) {
      return {ManifoldUtils::applyOperator3DManifold(children, op)};
    }
#endif
#ifdef ENABLE_CGAL
    if (Feature::ExperimentalFastCsg.is_enabled()) {
      // FIXME: It's annoying to have to disambiguate here:
      return {std::shared_ptr<Geometry>(CGALUtils::applyOperator3DHybrid(children, op))};
    }
    return {CGALUtils::applyOperator3D(children, op)};
#else
    assert(false && "No boolean backend available");
    #endif
    break;
  }
  }
}



/*!
   Apply 2D hull.

   May return an empty geometry but will not return nullptr.
 */

std::unique_ptr<Polygon2d> GeometryEvaluator::applyHull2D(const AbstractNode& node)
{
  auto children = collectChildren2D(node);
  auto geometry = std::make_unique<Polygon2d>();

#ifdef ENABLE_CGAL
  using CGALPoint2 = CGAL::Point_2<CGAL::Cartesian<double>>;
  // Collect point cloud
  std::list<CGALPoint2> points;
  for (const auto& p : children) {
    if (p) {
      for (const auto& o : p->outlines()) {
        for (const auto& v : o.vertices) {
          points.emplace_back(v[0], v[1]);
        }
      }
    }
  }
  if (points.size() > 0) {
    // Apply hull
    std::list<CGALPoint2> result;
    try {
      CGAL::convex_hull_2(points.begin(), points.end(), std::back_inserter(result));
      // Construct Polygon2d
      Outline2d outline;
      for (const auto& p : result) {
        outline.vertices.emplace_back(p[0], p[1]);
      }
      geometry->addOutline(outline);
    } catch (const CGAL::Failure_exception& e) {
      LOG(message_group::Warning, "GeometryEvaluator::applyHull2D() during CGAL::convex_hull_2(): %1$s", e.what());
    }
  }
#endif
  return geometry;
}

std::unique_ptr<Polygon2d> GeometryEvaluator::applyFill2D(const AbstractNode& node)
{
  // Merge and sanitize input geometry
  auto geometry_in = ClipperUtils::apply(collectChildren2D(node), ClipperLib::ctUnion);

  std::vector<std::shared_ptr<const Polygon2d>> newchildren;
  // Keep only the 'positive' outlines, eg: the outside edges
  for (const auto& outline : geometry_in->outlines()) {
    if (outline.positive) {
      newchildren.push_back(std::make_shared<Polygon2d>(outline));
    }
  }

  // Re-merge geometry in case of nested outlines
  return ClipperUtils::apply(newchildren, ClipperLib::ctUnion);
}

std::unique_ptr<Geometry> GeometryEvaluator::applyHull3D(const AbstractNode& node)
{
  Geometry::Geometries children = collectChildren3D(node);

  auto P = std::make_unique<PolySet>(3);
  return applyHull(children);
}

std::unique_ptr<Polygon2d> GeometryEvaluator::applyMinkowski2D(const AbstractNode& node)
{
  auto children = collectChildren2D(node);
  if (!children.empty()) {
    return ClipperUtils::applyMinkowski(children);
  }
  return nullptr;
}

/*!
   Returns a list of Polygon2d children of the given node.
   May return empty Polygon2d object, but not nullptr objects
 */
std::vector<std::shared_ptr<const Polygon2d>> GeometryEvaluator::collectChildren2D(const AbstractNode& node)
{
  std::vector<std::shared_ptr<const Polygon2d>> children;
  for (const auto& item : this->visitedchildren[node.index()]) {
    auto& chnode = item.first;
    auto& chgeom = item.second;
    if (chnode->modinst->isBackground()) continue;

    // NB! We insert into the cache here to ensure that all children of
    // a node is a valid object. If we inserted as we created them, the
    // cache could have been modified before we reach this point due to a large
    // sibling object.
    smartCacheInsert(*chnode, chgeom);

    if (chgeom) {
      if (chgeom->getDimension() == 3) {
        LOG(message_group::Warning, item.first->modinst->location(), this->tree.getDocumentPath(), "Ignoring 3D child object for 2D operation");
        children.push_back(nullptr); // replace 3D geometry with empty geometry
      } else {
        if (chgeom->isEmpty()) {
          children.push_back(nullptr);
        } else {
          const auto polygon2d = std::dynamic_pointer_cast<const Polygon2d>(chgeom);
          assert(polygon2d);
          children.push_back(polygon2d);
        }
      }
    } else {
      children.push_back(nullptr);
    }
  }
  return children;
}

/*!
   Since we can generate both Nef and non-Nef geometry, we need to insert it into
   the appropriate cache.
   This method inserts the geometry into the appropriate cache if it's not already cached.
 */
void GeometryEvaluator::smartCacheInsert(const AbstractNode& node,
                                         const std::shared_ptr<const Geometry>& geom)
{
  const std::string& key = this->tree.getIdString(node);

#ifdef ENABLE_CGAL
  if (CGALCache::acceptsGeometry(geom)) {
    if (!CGALCache::instance()->contains(key)) CGALCache::instance()->insert(key, geom);
  } else {
#endif
    if (!GeometryCache::instance()->contains(key)) {
      if (!GeometryCache::instance()->insert(key, geom)) {
        LOG(message_group::Warning, "GeometryEvaluator: Node didn't fit into cache.");
      }
    }
#ifdef ENABLE_CGAL
  }
#endif
}

bool GeometryEvaluator::isSmartCached(const AbstractNode& node)
{
  const std::string& key = this->tree.getIdString(node);
  return (GeometryCache::instance()->contains(key)
#ifdef ENABLE_CGAL
	  || CGALCache::instance()->contains(key)
#endif
    );
}

std::shared_ptr<const Geometry> GeometryEvaluator::smartCacheGet(const AbstractNode& node, bool preferNef)
{
  const std::string& key = this->tree.getIdString(node);
  std::shared_ptr<const Geometry> geom;
  bool hasgeom = GeometryCache::instance()->contains(key);
#ifdef ENABLE_CGAL
  bool hascgal = CGALCache::instance()->contains(key);
  if (hascgal && (preferNef || !hasgeom)) geom = CGALCache::instance()->get(key);
  else
#endif
  if (hasgeom) geom = GeometryCache::instance()->get(key);
  return geom;
}

/*!
   Returns a list of 3D Geometry children of the given node.
   May return empty geometries, but not nullptr objects
 */
Geometry::Geometries GeometryEvaluator::collectChildren3D(const AbstractNode& node)
{
  Geometry::Geometries children;
  for (const auto& item : this->visitedchildren[node.index()]) {
    auto& chnode = item.first;
    const std::shared_ptr<const Geometry>& chgeom = item.second;
    if (chnode->modinst->isBackground()) continue;

    // NB! We insert into the cache here to ensure that all children of
    // a node is a valid object. If we inserted as we created them, the
    // cache could have been modified before we reach this point due to a large
    // sibling object.
    smartCacheInsert(*chnode, chgeom);

    if (chgeom && chgeom->getDimension() == 2) {
      LOG(message_group::Warning, item.first->modinst->location(), this->tree.getDocumentPath(), "Ignoring 2D child object for 3D operation");
      children.push_back(std::make_pair(item.first, nullptr)); // replace 2D geometry with empty geometry
    } else {
      // Add children if geometry is 3D OR null/empty
      children.push_back(item);
    }
  }
  return children;
}

/*!

 */
std::unique_ptr<Polygon2d> GeometryEvaluator::applyToChildren2D(const AbstractNode& node, OpenSCADOperator op)
{
  node.progress_report();
  if (op == OpenSCADOperator::MINKOWSKI) {
    return applyMinkowski2D(node);
  } else if (op == OpenSCADOperator::HULL) {
    return applyHull2D(node);
  } else if (op == OpenSCADOperator::FILL) {
    return applyFill2D(node);
  }

  auto children = collectChildren2D(node);

  if (children.empty()) {
    return nullptr;
  }

  if (children.size() == 1 && op != OpenSCADOperator::OFFSET ) {
    if (children[0]) {
      return std::make_unique<Polygon2d>(*children[0]); // Copy
    } else {
      return nullptr;
    }
  }

  ClipperLib::ClipType clipType;
  switch (op) {
  case OpenSCADOperator::UNION:
  case OpenSCADOperator::OFFSET:
    clipType = ClipperLib::ctUnion;
    break;
  case OpenSCADOperator::INTERSECTION:
    clipType = ClipperLib::ctIntersection;
    break;
  case OpenSCADOperator::DIFFERENCE:
    clipType = ClipperLib::ctDifference;
    break;
  default:
    LOG(message_group::Error, "Unknown boolean operation %1$d", int(op));
    return nullptr;
    break;
  }

  auto pol = ClipperUtils::apply(children, clipType);

  if (op == OpenSCADOperator::OFFSET) {
	const OffsetNode *offNode = dynamic_cast<const OffsetNode *>(&node);
        // ClipperLib documentation: The formula for the number of steps in a full
        // circular arc is ... Pi / acos(1 - arc_tolerance / abs(delta))
        double n = Calc::get_fragments_from_r(std::abs(offNode->delta), offNode->fn, offNode->fs, offNode->fa);
        double arc_tolerance = std::abs(offNode->delta) * (1 - cos_degrees(180 / n));
        return   ClipperUtils::applyOffset(*pol, offNode->delta, offNode->join_type, offNode->miter_limit, arc_tolerance);
  }
  return pol;
}

/*!
   Adds ourself to our parent's list of traversed children.
   Call this for _every_ node which affects output during traversal.
   Usually, this should be called from the postfix stage, but for some nodes,
   we defer traversal letting other components (e.g. CGAL) render the subgraph,
   and we'll then call this from prefix and prune further traversal.

   The added geometry can be nullptr if it wasn't possible to evaluate it.
 */
void GeometryEvaluator::addToParent(const State& state,
                                    const AbstractNode& node,
                                    const std::shared_ptr<const Geometry>& geom)
{
  this->visitedchildren.erase(node.index());
  if (state.parent()) {
    this->visitedchildren[state.parent()->index()].push_back(std::make_pair(node.shared_from_this(), geom));
  } else {
    // Root node
    this->root = geom;
    assert(this->visitedchildren.empty());
  }
}

/*!
   Custom nodes are handled here => implicit union
 */
Response GeometryEvaluator::visit(State& state, const AbstractNode& node)
{
  if (state.isPrefix()) {
    if (isSmartCached(node)) return Response::PruneTraversal;
    state.setPreferNef(true); // Improve quality of CSG by avoiding conversion loss
  }
  if (state.isPostfix()) {
    std::shared_ptr<const Geometry> geom;
    if (!isSmartCached(node)) {
      geom = applyToChildren(node, OpenSCADOperator::UNION).constptr();
    } else {
      geom = smartCacheGet(node, state.preferNef());
    }
    addToParent(state, node, geom);
    node.progress_report();
  }
  return Response::ContinueTraversal;
}

/*!
   Pass children to parent without touching them. Used by e.g. for loops
 */
Response GeometryEvaluator::visit(State& state, const ListNode& node)
{
  if (state.parent()) {
    if (state.isPrefix() && node.modinst->isBackground()) {
      if (node.modinst->isBackground()) state.setBackground(true);
      return Response::PruneTraversal;
    }
    if (state.isPostfix()) {
      unsigned int dim = 0;
      for (const auto& item : this->visitedchildren[node.index()]) {
        if (!isValidDim(item, dim)) break;
        auto& chnode = item.first;
        const std::shared_ptr<const Geometry>& chgeom = item.second;
        addToParent(state, *chnode, chgeom);
      }
      this->visitedchildren.erase(node.index());
    }
    return Response::ContinueTraversal;
  } else {
    // Handle when a ListNode is given root modifier
    return lazyEvaluateRootNode(state, node);
  }
}

/*!
 */
Response GeometryEvaluator::visit(State& state, const GroupNode& node)
{
  return visit(state, (const AbstractNode&)node);
}

Response GeometryEvaluator::lazyEvaluateRootNode(State& state, const AbstractNode& node) {
  if (state.isPrefix()) {
    if (node.modinst->isBackground()) {
      state.setBackground(true);
      return Response::PruneTraversal;
    }
    if (isSmartCached(node)) {
      return Response::PruneTraversal;
    }
  }
  if (state.isPostfix()) {
    std::shared_ptr<const Geometry> geom;

    unsigned int dim = 0;
    GeometryList::Geometries geometries;
    for (const auto& item : this->visitedchildren[node.index()]) {
      if (!isValidDim(item, dim)) break;
      auto& chnode = item.first;
      const std::shared_ptr<const Geometry>& chgeom = item.second;
      if (chnode->modinst->isBackground()) continue;
      // NB! We insert into the cache here to ensure that all children of
      // a node is a valid object. If we inserted as we created them, the
      // cache could have been modified before we reach this point due to a large
      // sibling object.
      smartCacheInsert(*chnode, chgeom);
      // Only use valid geometries
      if (chgeom && !chgeom->isEmpty()) geometries.push_back(item);
    }
    if (geometries.size() == 1) geom = geometries.front().second;
    else if (geometries.size() > 1) geom = std::make_shared<GeometryList>(geometries);

    this->root = geom;
  }
  return Response::ContinueTraversal;
}

/*!
   Root nodes are handled specially; they will flatten any child group
   nodes to avoid doing an implicit top-level union.

   NB! This is likely a temporary measure until a better implementation of
   group nodes is in place.
 */
Response GeometryEvaluator::visit(State& state, const RootNode& node)
{
  // If we didn't enable lazy unions, just union the top-level objects
  if (!Feature::ExperimentalLazyUnion.is_enabled()) {
    return visit(state, (const GroupNode&)node);
  }
  return lazyEvaluateRootNode(state, node);
}

Response GeometryEvaluator::visit(State& state, const OffsetNode& node)
{
  if (state.isPrefix() && isSmartCached(node)) return Response::PruneTraversal;
  if (state.isPostfix()) {
    std::shared_ptr<const Geometry> geom;
    if (!isSmartCached(node)) {
      ResultObject res = applyToChildren(node, OpenSCADOperator::OFFSET);
      auto mutableGeom = res.asMutableGeometry();
      if (mutableGeom) mutableGeom->setConvexity(1);
      geom = mutableGeom;
    } else {
      geom = smartCacheGet(node, false);
    }
    addToParent(state, node, geom);
    node.progress_report();
  }
  return Response::ContinueTraversal;
}

/*!
   RenderNodes just pass on convexity
 */
Response GeometryEvaluator::visit(State& state, const RenderNode& node)
{
  if (state.isPrefix()) {
    if (isSmartCached(node)) return Response::PruneTraversal;
    state.setPreferNef(true); // Improve quality of CSG by avoiding conversion loss
  }
  if (state.isPostfix()) {
    std::shared_ptr<const Geometry> geom;
    if (!isSmartCached(node)) {
      ResultObject res = applyToChildren(node, OpenSCADOperator::UNION);
      auto mutableGeom = res.asMutableGeometry();
      if (mutableGeom) mutableGeom->setConvexity(node.convexity);
      geom = mutableGeom;
    } else {
      geom = smartCacheGet(node, state.preferNef());
    }
    node.progress_report();
    addToParent(state, node, geom);
  }
  return Response::ContinueTraversal;
}

/*!
   Leaf nodes can create their own geometry, so let them do that

   input: None
   output: PolySet or Polygon2d
 */
Response GeometryEvaluator::visit(State& state, const LeafNode& node)
{
  if (state.isPrefix()) {
    std::shared_ptr<const Geometry> geom;
    if (!isSmartCached(node)) {
      geom = node.createGeometry();
      assert(geom);
      if (const auto polygon = std::dynamic_pointer_cast<const Polygon2d>(geom)) {
        if (!polygon->isSanitized()) {
          geom = ClipperUtils::sanitize(*polygon);
        }
      }
    } else {
      geom = smartCacheGet(node, state.preferNef());
    }
    addToParent(state, node, geom);
    node.progress_report();
  }
  return Response::PruneTraversal;
}

Response GeometryEvaluator::visit(State& state, const TextNode& node)
{
  if (state.isPrefix()) {
    std::shared_ptr<const Geometry> geom;
    if (!isSmartCached(node)) {
      auto geometrylist = node.createGeometryList();
      std::vector<std::shared_ptr<const Polygon2d>> polygonlist;
      for (const auto& geometry : geometrylist) {
        const auto polygon = std::dynamic_pointer_cast<const Polygon2d>(geometry);
        assert(polygon);
        polygonlist.push_back(polygon);
      }
      geom = ClipperUtils::apply(polygonlist, ClipperLib::ctUnion);
    } else {
      geom = GeometryCache::instance()->get(this->tree.getIdString(node));
    }
    addToParent(state, node, geom);
    node.progress_report();
  }
  return Response::PruneTraversal;
}


/*!
   input: List of 2D or 3D objects (not mixed)
   output: Polygon2d or 3D PolySet
   operation:
    o Perform csg op on children
 */
Response GeometryEvaluator::visit(State& state, const CsgOpNode& node)
{
  if (state.isPrefix()) {
    if (isSmartCached(node)) return Response::PruneTraversal;
    state.setPreferNef(true); // Improve quality of CSG by avoiding conversion loss
  }
  if (state.isPostfix()) {
    std::shared_ptr<const Geometry> geom;
    if (!isSmartCached(node)) {
      geom = applyToChildren(node, node.type).constptr();
    } else {
      geom = smartCacheGet(node, state.preferNef());
    }
    addToParent(state, node, geom);
    node.progress_report();
  }
  return Response::ContinueTraversal;
}

/*!
   input: List of 2D or 3D objects (not mixed)
   output: Polygon2d or 3D PolySet
   operation:
    o Union all children
    o Perform transform
 */
Response GeometryEvaluator::visit(State& state, const TransformNode& node)
{
  if (state.isPrefix() && isSmartCached(node)) return Response::PruneTraversal;
  if (state.isPostfix()) {
    std::shared_ptr<const Geometry> geom;
    if (!isSmartCached(node)) {
      if (matrix_contains_infinity(node.matrix) || matrix_contains_nan(node.matrix)) {
        // due to the way parse/eval works we can't currently distinguish between NaN and Inf
        LOG(message_group::Warning, node.modinst->location(), this->tree.getDocumentPath(), "Transformation matrix contains Not-a-Number and/or Infinity - removing object.");
      } else {
        // First union all children
        ResultObject res = applyToChildren(node, OpenSCADOperator::UNION);
        if ((geom = res.constptr())) {
          if (geom->getDimension() == 2) {
            std::shared_ptr<const Polygon2d> polygons = std::dynamic_pointer_cast<const Polygon2d>(geom);
            assert(polygons);

            // If we got a const object, make a copy
            std::shared_ptr<Polygon2d> newpoly;
            if (res.isConst()) {
              newpoly = std::make_shared<Polygon2d>(*polygons);
	    }
            else {
              newpoly = std::dynamic_pointer_cast<Polygon2d>(res.ptr());
	    }

            Transform2d mat2;
            mat2.matrix() <<
              node.matrix(0, 0), node.matrix(0, 1), node.matrix(0, 3),
              node.matrix(1, 0), node.matrix(1, 1), node.matrix(1, 3),
              node.matrix(3, 0), node.matrix(3, 1), node.matrix(3, 3);
            newpoly->transform(mat2);
	    // FIXME: We lose the transform if we copied a const geometry above. Probably similar issue in multiple places
            // A 2D transformation may flip the winding order of a polygon.
            // If that happens with a sanitized polygon, we need to reverse
            // the winding order for it to be correct.
            if (newpoly->isSanitized() && mat2.matrix().determinant() <= 0) {
              geom = ClipperUtils::sanitize(*newpoly);
            }
          } else if (geom->getDimension() == 3) {
            auto mutableGeom = res.asMutableGeometry();
            if (mutableGeom) mutableGeom->transform(node.matrix);
            geom = mutableGeom;
          }
        }
      }
    } else {
      geom = smartCacheGet(node, state.preferNef());
    }
    addToParent(state, node, geom);
    node.progress_report();
  }
  return Response::ContinueTraversal;
}

static void translate_PolySet(PolySet& ps, const Vector3d& translation) // TODO duplicate with CGALutils ?
{
  for (auto& v : ps.vertices) {
      v += translation;
  }
}

/*
   Compare Euclidean length of vectors
   Return:
    -1 : if v1  < v2
     0 : if v1 ~= v2 (approximation to compoensate for floating point precision)
     1 : if v1  > v2
 */
int sgn_vdiff(const Vector2d& v1, const Vector2d& v2) {
  constexpr double ratio_threshold = 1e5; // 10ppm difference
  double l1 = v1.norm();
  double l2 = v2.norm();
  // Compare the average and difference, to be independent of geometry scale.
  // If the difference is within ratio_threshold of the avg, treat as equal.
  double scale = (l1 + l2);
  double diff = 2 * std::fabs(l1 - l2) * ratio_threshold;
  return diff > scale ? (l1 < l2 ? -1 : 1) : 0;
}

/*
   Enable/Disable experimental 4-way split quads for linear_extrude, with added midpoint.
   These look very nice when(and only when) diagonals are near equal.
   This typically happens when an edge is colinear with the origin.
 */
//#define LINEXT_4WAY

/*
   Attempt to triangulate quads in an ideal way.
   Each quad is composed of two adjacent outline vertices: (prev1, curr1)
   and their corresponding transformed points one step up: (prev2, curr2).
   Quads are triangulated across the shorter of the two diagonals, which works well in most cases.
   However, when diagonals are equal length, decision may flip depending on other factors.
 */
static void add_slice(PolySetBuilder &builder, const Polygon2d& poly,
                      double rot1, double rot2,
                      double h1, double h2,
                      const Vector2d& scale1,
                      const Vector2d& scale2)
{
  Eigen::Affine2d trans1(Eigen::Scaling(scale1) * Eigen::Affine2d(rotate_degrees(-rot1)));
  Eigen::Affine2d trans2(Eigen::Scaling(scale2) * Eigen::Affine2d(rotate_degrees(-rot2)));
#ifdef LINEXT_4WAY
  Eigen::Affine2d trans_mid(Eigen::Scaling((scale1 + scale2) / 2) * Eigen::Affine2d(rotate_degrees(-(rot1 + rot2) / 2)));
  bool is_straight = rot1 == rot2 && scale1[0] == scale1[1] && scale2[0] == scale2[1];
#endif
  bool any_zero = scale2[0] == 0 || scale2[1] == 0;
  bool any_non_zero = scale2[0] != 0 || scale2[1] != 0;
  // Not likely to matter, but when no twist (rot2 == rot1),
  // setting back_twist true helps keep diagonals same as previous builds.
  bool back_twist = rot2 <= rot1;

  for (const auto& o : poly.outlines()) {
    Vector2d prev1 = trans1 * o.vertices[0];
    Vector2d prev2 = trans2 * o.vertices[0];

    // For equal length diagonals, flip selected choice depending on direction of twist and
    // whether the outline is negative (eg circle hole inside a larger circle).
    // This was tested against circles with a single point touching the origin,
    // and extruded with twist.  Diagonal choice determined by whichever option
    // matched the direction of diagonal for neighboring edges (which did not exhibit "equal" diagonals).
    bool flip = ((!o.positive) xor (back_twist));

    for (size_t i = 1; i <= o.vertices.size(); ++i) {
      Vector2d curr1 = trans1 * o.vertices[i % o.vertices.size()];
      Vector2d curr2 = trans2 * o.vertices[i % o.vertices.size()];

      int diff_sign = sgn_vdiff(prev1 - curr2, curr1 - prev2);
      bool splitfirst = diff_sign == -1 || (diff_sign == 0 && !flip);

#ifdef LINEXT_4WAY
      // Diagonals should be equal whenever an edge is co-linear with the origin (edge itself need not touch it)
      if (!is_straight && diff_sign == 0) {
        // Split into 4 triangles, with an added midpoint.
        //Vector2d mid_prev = trans3 * (prev1 +curr1+curr2)/4;
        Vector2d mid = trans_mid * (o.vertices[(i - 1) % o.vertices.size()] + o.vertices[i % o.vertices.size()]) / 2;
        double h_mid = (h1 + h2) / 2;
        builder.appendPoly(3);
        builder.insertVertex(prev1[0], prev1[1], h1);
        builder.insertVertex(mid[0],   mid[1], h_mid);
        builder.insertVertex(curr1[0], curr1[1], h1);
        builder.appendPoly(3);
        builder.insertVertex(curr1[0], curr1[1], h1);
        builder.insertVertex(mid[0],   mid[1], h_mid);
        builder.insertVertex(curr2[0], curr2[1], h2);
        builder.appendPoly(3);
        builder.insertVertex(curr2[0], curr2[1], h2);
        builder.insertVertex(mid[0],   mid[1], h_mid);
        builder.insertVertex(prev2[0], prev2[1], h2);
        builder.appendPoly(3);
        builder.insertVertex(prev2[0], prev2[1], h2);
        builder.insertVertex(mid[0],   mid[1], h_mid);
        builder.insertVertex(prev1[0], prev1[1], h1);
      } else
#endif // ifdef LINEXT_4WAY
      // Split along shortest diagonal,
      // unless at top for a 0-scaled axis (which can create 0 thickness "ears")
      if (splitfirst xor any_zero) {
        builder.appendPoly({
		Vector3d(curr1[0], curr1[1], h1),
		Vector3d(curr2[0], curr2[1], h2),
		Vector3d(prev1[0], prev1[1], h1)
		});
        if (!any_zero || (any_non_zero && prev2 != curr2)) {
          builder.appendPoly({
		Vector3d(prev2[0], prev2[1], h2),
		Vector3d(prev1[0], prev1[1], h1),
		Vector3d(curr2[0], curr2[1], h2)
	  });
        }
      } else {
        builder.appendPoly({
		Vector3d(curr1[0], curr1[1], h1),
		Vector3d(prev2[0], prev2[1], h2),
		Vector3d(prev1[0], prev1[1], h1)
	});
        if (!any_zero || (any_non_zero && prev2 != curr2)) {
          builder.appendPoly({
		Vector3d(curr1[0], curr1[1], h1),
		Vector3d(curr2[0], curr2[1], h2),
		Vector3d(prev2[0], prev2[1], h2)
	  });	
        }
      }
      prev1 = curr1;
      prev2 = curr2;
    }
  }
}

// Insert vertices for segments interpolated between v0 and v1.
// The last vertex (t==1) is not added here to avoid duplicate vertices,
// since it will be the first vertex of the *next* edge.
static void add_segmented_edge(Outline2d& o, const Vector2d& v0, const Vector2d& v1, unsigned int edge_segments) {
  for (unsigned int j = 0; j < edge_segments; ++j) {
    double t = static_cast<double>(j) / edge_segments;
    o.vertices.push_back((1 - t) * v0 + t * v1);
  }
}

// For each edge in original outline, find its max length over all slice transforms,
// and divide into segments no longer than fs.
static Outline2d splitOutlineByFs(
  const Outline2d& o,
  const double twist, const double scale_x, const double scale_y,
  const double fs, unsigned int slices)
{
  const auto sz = o.vertices.size();

  Vector2d v0 = o.vertices[0];
  Outline2d o2;
  o2.positive = o.positive;

  // non-uniform scaling requires iterating over each slice transform
  // to find maximum length of a given edge.
  if (scale_x != scale_y) {
    for (size_t i = 1; i <= sz; ++i) {
      Vector2d v1 = o.vertices[i % sz];
      double max_edgelen = 0.0; // max length for single edge over all transformed slices
      for (unsigned int j = 0; j <= slices; j++) {
        double t = static_cast<double>(j) / slices;
        Vector2d scale(Calc::lerp(1, scale_x, t), Calc::lerp(1, scale_y, t));
        double rot = twist * t;
        Eigen::Affine2d trans(Eigen::Scaling(scale) * Eigen::Affine2d(rotate_degrees(-rot)));
        double edgelen = (trans * v1 - trans * v0).norm();
        max_edgelen = std::max(max_edgelen, edgelen);
      }
      auto edge_segments = static_cast<unsigned int>(std::ceil(max_edgelen / fs));
      add_segmented_edge(o2, v0, v1, edge_segments);
      v0 = v1;
    }
  } else { // uniform scaling
    double max_scale = std::max(scale_x, 1.0);
    for (size_t i = 1; i <= sz; ++i) {
      Vector2d v1 = o.vertices[i % sz];
      unsigned int edge_segments = static_cast<unsigned int>(std::ceil((v1 - v0).norm() * max_scale / fs));
      add_segmented_edge(o2, v0, v1, edge_segments);
      v0 = v1;
    }
  }
  return o2;
}

// While total outline segments < fn, increment segment_count for edge with largest
// (max_edge_length / segment_count).
static Outline2d splitOutlineByFn(
  const Outline2d& o,
  const double twist, const double scale_x, const double scale_y,
  const double fn, unsigned int slices)
{

  struct segment_tracker {
    size_t edge_index;
    double max_edgelen;
    unsigned int segment_count{1u};
    segment_tracker(size_t i, double len) : edge_index(i), max_edgelen(len) { }
    // metric for comparison: average between (max segment length, and max segment length after split)
    [[nodiscard]] double metric() const { return max_edgelen / (segment_count + 0.5); }
    bool operator<(const segment_tracker& rhs) const { return this->metric() < rhs.metric();  }
    [[nodiscard]] bool close_match(const segment_tracker& other) const {
      // Edges are grouped when metrics match by at least 99.9%
      constexpr double APPROX_EQ_RATIO = 0.999;
      double l1 = this->metric(), l2 = other.metric();
      return std::min(l1, l2) / std::max(l1, l2) >= APPROX_EQ_RATIO;
    }
  };

  const auto sz = o.vertices.size();
  std::vector<unsigned int> segment_counts(sz, 1);
  std::priority_queue<segment_tracker, std::vector<segment_tracker>> q;

  Vector2d v0 = o.vertices[0];
  // non-uniform scaling requires iterating over each slice transform
  // to find maximum length of a given edge.
  if (scale_x != scale_y) {
    for (size_t i = 1; i <= sz; ++i) {
      Vector2d v1 = o.vertices[i % sz];
      double max_edgelen = 0.0; // max length for single edge over all transformed slices
      for (unsigned int j = 0; j <= slices; j++) {
        double t = static_cast<double>(j) / slices;
        Vector2d scale(Calc::lerp(1, scale_x, t), Calc::lerp(1, scale_y, t));
        double rot = twist * t;
        Eigen::Affine2d trans(Eigen::Scaling(scale) * Eigen::Affine2d(rotate_degrees(-rot)));
        double edgelen = (trans * v1 - trans * v0).norm();
        max_edgelen = std::max(max_edgelen, edgelen);
      }
      q.emplace(i - 1, max_edgelen);
      v0 = v1;
    }
  } else { // uniform scaling
    double max_scale = std::max(scale_x, 1.0);
    for (size_t i = 1; i <= sz; ++i) {
      Vector2d v1 = o.vertices[i % sz];
      double max_edgelen = (v1 - v0).norm() * max_scale;
      q.emplace(i - 1, max_edgelen);
      v0 = v1;
    }
  }

  std::vector<segment_tracker> tmp_q;
  // Process priority_queue until number of segments is reached.
  size_t seg_total = sz;
  while (seg_total < fn) {
    auto current = q.top();

    // Group similar length segmented edges to keep result roughly symmetrical.
    while (!q.empty() && (tmp_q.empty() || current.close_match(tmp_q.front()))) {
      q.pop();
      tmp_q.push_back(current);
      current = q.top();
    }

    if (seg_total + tmp_q.size() <= fn) {
      while (!tmp_q.empty()) {
        current = tmp_q.back();
        tmp_q.pop_back();
        ++current.segment_count;
        ++segment_counts[current.edge_index];
        ++seg_total;
        q.push(current);
      }
    } else {
      // fn too low to segment last group, push back onto queue without change.
      while (!tmp_q.empty()) {
        current = tmp_q.back();
        tmp_q.pop_back();
        q.push(current);
      }
      break;
    }
  }

  // Create final segmented edges.
  Outline2d o2;
  o2.positive = o.positive;
  v0 = o.vertices[0];
  for (size_t i = 1; i <= sz; ++i) {
    Vector2d v1 = o.vertices[i % sz];
    add_segmented_edge(o2, v0, v1, segment_counts[i - 1]);
    v0 = v1;
  }

  assert(o2.vertices.size() <= fn);
  return o2;
}

static Outline2d alterprofile(Outline2d profile,double scalex, double scaley, double origin_x, double origin_y,double offset_x, double offset_y, double rot)
{
	Outline2d result;
	double ang=rot*3.14/180.0;
	double c=cos(ang);
	double s=sin(ang);
	int n=profile.vertices.size();
	for(int i=0;i<n;i++) {
		double x=(profile.vertices[i][0]-origin_x)*scalex;
		double y=(profile.vertices[i][1]-origin_y)*scaley;
		double xr = (x*c - y*s)+origin_x + offset_x;
		double yr = (y*c + x*s)+origin_y + offset_y;
		result.vertices.push_back(Vector2d(xr,yr));
	}
	return result;
}

void  append_linear_vertex(PolySetBuilder &builder,const Outline2d *face, int index, double h)
{
	builder.appendVertex(builder.vertexIndex(Vector3d(
			face->vertices[index][0],
			face->vertices[index][1],
			h)));
}

void  append_rotary_vertex(PolySetBuilder &builder,const Outline2d *face, int index, double ang)
{
	double a=ang*G_PI / 180.0;
	builder.appendVertex(builder.vertexIndex(Vector3d(
			face->vertices[index][0]*cos(a),
			face->vertices[index][0]*sin(a),
			face->vertices[index][1])));
}

void calculate_path_dirs(Vector3d prevpt, Vector3d curpt,Vector3d nextpt,Vector3d vec_x_last, Vector3d vec_y_last, Vector3d *vec_x, Vector3d *vec_y) {
	Vector3d diff1,diff2;
	diff1 = curpt - prevpt;
	diff2 = nextpt - curpt;
	double xfac=1.0,yfac=1.0,beta, beta2;

	if(diff1.norm() > 0.001) diff1.normalize();
	if(diff2.norm() > 0.001) diff2.normalize();
	Vector3d diff=diff1+diff2;

	if(diff.norm() < 0.001) {
		printf("User Error!\n");
		return ;
	} 
	if(vec_y_last.norm() < 0.001)  { // Needed in first step only
		vec_y_last = diff2.cross(vec_x_last);
		if(vec_y_last.norm() < 0.001) { vec_x_last[0]=1; vec_x_last[1]=0; vec_x_last[2]=0; vec_y_last = diff.cross(vec_x_last); }
		if(vec_y_last.norm() < 0.001) { vec_x_last[0]=0; vec_x_last[1]=1; vec_x_last[2]=0; vec_y_last = diff.cross(vec_x_last); }
		if(vec_y_last.norm() < 0.001) { vec_x_last[0]=0; vec_x_last[1]=0; vec_x_last[2]=1; vec_y_last = diff.cross(vec_x_last); }
	} else {
		// make vec_last normal to diff1
		Vector3d xn= vec_y_last.cross(diff1).normalized();
		Vector3d yn= diff1.cross(vec_x_last).normalized();

		// now fix the angle between xn and yn
		Vector3d vec_xy_ = (xn + yn).normalized();
		Vector3d vec_xy = vec_xy_.cross(diff1).normalized();
		vec_x_last = (vec_xy_ + vec_xy).normalized();
		vec_y_last = diff1.cross(xn).normalized();
	}

	diff=(diff1+diff2).normalized();

	*vec_y = diff.cross(vec_x_last);
	if(vec_y->norm() < 0.001) { vec_x_last[0]=1; vec_x_last[1]=0; vec_x_last[2]=0; *vec_y = diff.cross(vec_x_last); }
	if(vec_y->norm() < 0.001) { vec_x_last[0]=0; vec_x_last[1]=1; vec_x_last[2]=0; *vec_y = diff.cross(vec_x_last); }
	if(vec_y->norm() < 0.001) { vec_x_last[0]=0; vec_x_last[1]=0; vec_x_last[2]=1; *vec_y = diff.cross(vec_x_last); }
	vec_y->normalize(); 

	*vec_x = vec_y_last.cross(diff);
	if(vec_x->norm() < 0.001) { vec_y_last[0]=1; vec_y_last[1]=0; vec_y_last[2]=0; *vec_x = vec_y_last.cross(diff); }
	if(vec_x->norm() < 0.001) { vec_y_last[0]=0; vec_y_last[1]=1; vec_y_last[2]=0; *vec_x = vec_y_last.cross(diff); }
	if(vec_x->norm() < 0.001) { vec_y_last[0]=0; vec_y_last[1]=0; vec_y_last[2]=1; *vec_x = vec_y_last.cross(diff); }
	vec_x->normalize(); 

	if(diff1.norm() > 0.001 && diff2.norm() > 0.001) {
		beta = (*vec_x).dot(diff1); 
		xfac=sqrt(1-beta*beta);
		beta = (*vec_y).dot(diff1);
		yfac=sqrt(1-beta*beta);

	}
	(*vec_x) /= xfac;
	(*vec_y) /= yfac;
}

std::vector<Vector3d> calculate_path_profile(Vector3d *vec_x, Vector3d *vec_y,Vector3d curpt, const std::vector<Vector2d> &profile) {

	std::vector<Vector3d> result;
	for(int i=0;i<profile.size();i++) {
		result.push_back( Vector3d(
			curpt[0]+(*vec_x)[0]*profile[i][0]+(*vec_y)[0]*profile[i][1],
			curpt[1]+(*vec_x)[1]*profile[i][0]+(*vec_y)[1]*profile[i][1],
			curpt[2]+(*vec_x)[2]*profile[i][0]+(*vec_y)[2]*profile[i][1]
				));
	}
	return result;
}

/*!
   Input to extrude should be sanitized. This means non-intersecting, correct winding order
   etc., the input coming from a library like Clipper.
 */
static std::unique_ptr<Geometry> extrudePolygon(const LinearExtrudeNode& node, const Polygon2d& poly)
{
  bool non_linear = node.twist != 0 || node.scale_x != node.scale_y;
  boost::tribool isConvex{poly.is_convex()};
  // Twist or non-uniform scale makes convex polygons into unknown polyhedrons
  if (isConvex && non_linear) isConvex = unknown;
  PolySetBuilder builder(0, 0, 3, isConvex);
  builder.setConvexity(node.convexity);
  if (node.height <= 0) return std::make_unique<PolySet>(3);

  size_t slices;
  if (node.has_slices) {
    slices = node.slices;
  } else if (node.has_twist) {
    double max_r1_sqr = 0; // r1 is before scaling
    Vector2d scale(node.scale_x, node.scale_y);
    for (const auto& o : poly.outlines())
      for (const auto& v : o.vertices)
        max_r1_sqr = fmax(max_r1_sqr, v.squaredNorm());
    // Calculate Helical curve length for Twist with no Scaling
    if (node.scale_x == 1.0 && node.scale_y == 1.0) {
      slices = (unsigned int)Calc::get_helix_slices(max_r1_sqr, node.height, node.twist, node.fn, node.fs, node.fa);
    } else if (node.scale_x != node.scale_y) {  // non uniform scaling with twist using max slices from twist and non uniform scale
      double max_delta_sqr = 0; // delta from before/after scaling
      Vector2d scale(node.scale_x, node.scale_y);
      for (const auto& o : poly.outlines()) {
        for (const auto& v : o.vertices) {
          max_delta_sqr = fmax(max_delta_sqr, (v - v.cwiseProduct(scale)).squaredNorm());
        }
      }
      size_t slicesNonUniScale;
      size_t slicesTwist;
      slicesNonUniScale = (unsigned int)Calc::get_diagonal_slices(max_delta_sqr, node.height, node.fn, node.fs);
      slicesTwist = (unsigned int)Calc::get_helix_slices(max_r1_sqr, node.height, node.twist, node.fn, node.fs, node.fa);
      slices = std::max(slicesNonUniScale, slicesTwist);
    } else { // uniform scaling with twist, use conical helix calculation
      slices = (unsigned int)Calc::get_conical_helix_slices(max_r1_sqr, node.height, node.twist, node.scale_x, node.fn, node.fs, node.fa);
    }
  } else if (node.scale_x != node.scale_y) {
    // Non uniform scaling, w/o twist
    double max_delta_sqr = 0; // delta from before/after scaling
    Vector2d scale(node.scale_x, node.scale_y);
    for (const auto& o : poly.outlines()) {
      for (const auto& v : o.vertices) {
        max_delta_sqr = fmax(max_delta_sqr, (v - v.cwiseProduct(scale)).squaredNorm());
      }
    }
    slices = Calc::get_diagonal_slices(max_delta_sqr, node.height, node.fn, node.fs);
  } else {
    // uniform or [1,1] scaling w/o twist needs only one slice
    slices = 1;
  }

  // Calculate outline segments if appropriate.
  Polygon2d seg_poly;
  bool is_segmented = false;
  if (node.has_segments) {
    // Set segments = 0 to disable
    if (node.segments > 0) {
      for (const auto& o : poly.outlines()) {
        if (o.vertices.size() >= node.segments) {
          seg_poly.addOutline(o);
        } else {
          seg_poly.addOutline(splitOutlineByFn(o, node.twist, node.scale_x, node.scale_y, node.segments, slices));
        }
      }
      is_segmented = true;
    }
  } else if (non_linear) {
    if (node.fn > 0.0) {
      for (const auto& o : poly.outlines()) {
        if (o.vertices.size() >= node.fn) {
          seg_poly.addOutline(o);
        } else {
          seg_poly.addOutline(splitOutlineByFn(o, node.twist, node.scale_x, node.scale_y, node.fn, slices));
        }
      }
    } else { // $fs and $fa based segmentation
      auto fa_segs = static_cast<unsigned int>(std::ceil(360.0 / node.fa));
      for (const auto& o : poly.outlines()) {
        if (o.vertices.size() >= fa_segs) {
          seg_poly.addOutline(o);
        } else {
          // try splitting by $fs, then check if $fa results in less segments
          auto fsOutline = splitOutlineByFs(o, node.twist, node.scale_x, node.scale_y, node.fs, slices);
          if (fsOutline.vertices.size() >= fa_segs) {
            seg_poly.addOutline(splitOutlineByFn(o, node.twist, node.scale_x, node.scale_y, fa_segs, slices));
          } else {
            seg_poly.addOutline(std::move(fsOutline));
          }
        }
      }
    }
    is_segmented = true;
  }

  const Polygon2d& polyref = is_segmented ? seg_poly : poly;

  double h1, h2;
  if (node.center) {
    h1 = -node.height / 2.0;
    h2 = +node.height / 2.0;
  } else {
    h1 = 0;
    h2 = node.height;
  }

#ifdef ENABLE_PYTHON  
  if(node.profile_func != NULL)
  {
	Outline2d lowerFace;
	Outline2d upperFace;
	double lower_h=h1, upper_h=h2;
	double lower_scalex=1.0, upper_scalex=1.0;
	double lower_scaley=1.0, upper_scaley=1.0;
	double lower_rot=0.0, upper_rot=0.0;
        if(node.twist_func != NULL) {
          lower_rot = python_doublefunc(node.twist_func, 0);
        } else lower_rot=0;

	// Add Bottom face
	lowerFace = alterprofile(python_getprofile(node.profile_func, node.fn, 0),lower_scalex, lower_scaley,node.origin_x, node.origin_y, 0, 0, lower_rot);
	Polygon2d botface;
        botface.addOutline(lowerFace);
    	std::unique_ptr<PolySet> ps_bot = botface.tessellate();
	translate_PolySet(*ps_bot, Vector3d(0, 0, lower_h));
  	for (auto& p : ps_bot->indices) {
	    std::reverse(p.begin(), p.end());
	}
	builder.append(*ps_bot);
  	for (unsigned int i = 1; i <= slices; i++) {
		upper_h=i*node.height/slices;
    		upper_scalex = 1 - i * (1 - node.scale_x) / slices;
    		upper_scaley = 1 - i * (1 - node.scale_y) / slices;
        	if(node.twist_func != NULL) {
	          upper_rot = python_doublefunc(node.twist_func, i/(double)slices);
        	} else upper_rot=i*node.twist /slices;
		if(node.center) upper_h -= node.height/2;
		upperFace = alterprofile(python_getprofile(node.profile_func, node.fn, upper_h), upper_scalex, upper_scaley , node.origin_x, node.origin_y, 0, 0, upper_rot);
		if(lowerFace.vertices.size() == upperFace.vertices.size()) {
			unsigned int n=lowerFace.vertices.size();
			for(unsigned int j=0;j<n;j++) {
				builder.appendPoly(3);
				append_linear_vertex(builder,&lowerFace,(j+0)%n, lower_h);
				append_linear_vertex(builder,&lowerFace,(j+1)%n, lower_h);
				append_linear_vertex(builder,&upperFace,(j+1)%n, upper_h);

				builder.appendPoly(3);
				append_linear_vertex(builder,&lowerFace,(j+0)%n, lower_h);
				append_linear_vertex(builder,&upperFace,(j+1)%n, upper_h);
				append_linear_vertex(builder,&upperFace,(j+0)%n, upper_h);
			}
		}

		lowerFace = upperFace;
		lower_h = upper_h;
		lower_scalex = upper_scalex;
		lower_scaley = upper_scaley;
		lower_rot = upper_rot;
	}
	// Add Top face
	Polygon2d topface;
        topface.addOutline(upperFace);
	std::unique_ptr<PolySet> ps_top = topface.tessellate();
	translate_PolySet(*ps_top, Vector3d(0, 0, upper_h));
	builder.append(*ps_top);
  }
  else
#endif  
{
  // Create bottom face.
  auto ps_bottom = polyref.tessellate(); // bottom
  // Flip vertex ordering for bottom polygon
  for (auto& p : ps_bottom->indices) {
    std::reverse(p.begin(), p.end());
  }
  translate_PolySet(*ps_bottom, Vector3d(0, 0, h1));
  builder.append(*ps_bottom);

  // Create slice sides.
  double rot1, rot2;
#ifdef ENABLE_PYTHON  
    if(node.twist_func != NULL) {
      rot1 = python_doublefunc(node.twist_func, 0);
    } else
#endif
  rot1=0;
  for (unsigned int j = 0; j < slices; j++) {
#ifdef ENABLE_PYTHON	  
    if(node.twist_func != NULL) {
      rot2 = python_doublefunc(node.twist_func, (double)(j+1.0)/(double) slices);
    } else
#endif
    rot2 = node.twist * (j + 1) / slices;
    double height1 = h1 + (h2 - h1) * j / slices;
    double height2 = h1 + (h2 - h1) * (j + 1) / slices;
    Vector2d scale1(1 - (1 - node.scale_x) * j / slices,
                    1 - (1 - node.scale_y) * j / slices);
    Vector2d scale2(1 - (1 - node.scale_x) * (j + 1) / slices,
                    1 - (1 - node.scale_y) * (j + 1) / slices);
    add_slice(builder, polyref, rot1, rot2, height1, height2, scale1, scale2);
    rot1=rot2;
  }

  // Create top face.
  // If either scale components are 0, then top will be zero-area, so skip it.
  if (node.scale_x != 0 && node.scale_y != 0) {
    Polygon2d top_poly(polyref);
    Eigen::Affine2d trans(Eigen::Scaling(node.scale_x, node.scale_y) * Eigen::Affine2d(rotate_degrees(-rot2)));
    top_poly.transform(trans);
    auto ps_top = top_poly.tessellate();
    translate_PolySet(*ps_top, Vector3d(0, 0, h2));
    builder.append(*ps_top);
  }
}
  return builder.build();
}

static std::unique_ptr<Geometry> extrudePolygon(const PathExtrudeNode& node, const Polygon2d& poly)
{
  int i;
  PolySetBuilder builder;
  builder.setConvexity(node.convexity);
  std::vector<Vector3d> path_os;
  std::vector<double> length_os;
  gboolean intersect=false;

  // Round the corners with radius
  int xdir_offset = 0; // offset in point list to apply the xdir
  std::vector<Vector3d> path_round; 
  int m = node.path.size();
  for(i=0;i<node.path.size();i++)
  {
	int draw_arcs=0;
	Vector3d diff1, diff2,center,arcpt;
	int secs;
	double ang;
	Vector3d cur=node.path[i].head<3>();
	double r=node.path[i][3];
	do
	{
		if(i == 0 && node.closed == 0) break;
		if(i == m-1 && node.closed == 0) break;

		Vector3d prev=node.path[(i+m-1)%m].head<3>();
		Vector3d next=node.path[(i+1)%m].head<3>();
		diff1=(prev-cur).normalized();
		diff2=(next-cur).normalized();
		Vector3d diff=(diff1+diff2).normalized();
	
		ang=acos(diff1.dot(-diff2));
		double arclen=ang*r;
		center=cur+(r/cos(ang/2.0))*diff;

		secs=node.fn;
		int secs_a,secs_s;
		secs_a=(int) ceil(180.0*ang/(G_PI*node.fa));
		if(secs_a > secs) secs=secs_a;

		secs_s=(int) ceil(arclen/node.fs);
		if(secs_s > secs) secs=secs_s;


		if(r == 0) break;
		if(secs  == 0)  break;
		draw_arcs=1;
	}
	while(false);
	if(draw_arcs) {
		draw_arcs=1;
		Vector3d diff1n=diff1.cross(diff2.cross(diff1)).normalized();
		for(int j=0;j<=secs;j++) {
			arcpt=center
				-diff1*r*sin(ang*j/(double) secs)
				-diff1n*r*cos(ang*j/(double) secs);
  			path_round.push_back(arcpt);
		}
		if(node.closed > 0 && i == 0) xdir_offset=secs; // user wants to apply xdir on this point
	} else path_round.push_back(cur);

  }

  // xdir_offset is claculated in in next step automatically
  //
  // Create oversampled path with fs. for streights
  path_os.push_back(path_round[xdir_offset]);
  length_os.push_back(0);
  m = path_round.size();
  int ifinal=node.closed?m:m-1;

  for(int i=1;i<=ifinal;i++) {
	  Vector3d prevPt = path_round[(i+xdir_offset-1)%m];
	  Vector3d curPt = path_round[(i+xdir_offset)%m];
	  Vector3d seg=curPt - prevPt;
	  double length_seg = seg.norm();
	  int split=ceil(length_seg/node.fs);
	  if(node.twist == 0 && node.scale_x == 1.0 && node.scale_y == 1.0
			  ) split=1;
	  for(int j=1;j<=split;j++) {
		double ratio=(double)j/(double)split;
	  	path_os.push_back(prevPt+seg*ratio);
	  	length_os.push_back((i-1+(double)j/(double)split)/(double) (path_round.size()-1));
	  }
  }
  if(node.closed) { // let close do its last pt itself
	  path_os.pop_back();
	  length_os.pop_back();
  }

  Vector3d lastPt, curPt, nextPt;
  Vector3d vec_x_last(node.xdir_x,node.xdir_y,node.xdir_z);
  Vector3d vec_y_last(0,0,0);
  vec_x_last.normalize();

  // in case of custom profile,poly shall exactly have one dummy outline,will be replaced
  for(const Outline2d &profile2d: poly.outlines()) {
  
    std::vector<Vector3d> lastProfile;
    std::vector<Vector3d> startProfile; 
    unsigned int m=path_os.size();
    int mfinal=(node.closed == true)?m+1:m-1;
    for (unsigned int i = 0; i <= mfinal; i++) {
        std::vector<Vector3d> curProfile; 
	double cur_twist;

#ifdef ENABLE_PYTHON
        if(node.twist_func != NULL) {
          cur_twist = python_doublefunc(node.twist_func, length_os[i]);
        } else 
#endif	
	cur_twist=node.twist *length_os[i];

	double cur_scalex=1.0+(node.scale_x-1.0)*length_os[i];
	double cur_scaley=1.0+(node.scale_y-1.0)*length_os[i];
	Outline2d profilemod;
	#ifdef ENABLE_PYTHON  
	if(node.profile_func != NULL)
	{
		Outline2d tmpx=python_getprofile(node.profile_func, node.fn, length_os[i%m]);
        	profilemod = alterprofile(tmpx,cur_scalex,cur_scaley,node.origin_x, node.origin_y,0, 0, cur_twist);
	}
	else
	#endif  
        profilemod = alterprofile(profile2d,cur_scalex,cur_scaley,node.origin_x, node.origin_y,0, 0, cur_twist);

	unsigned int n=profilemod.vertices.size();
	curPt = path_os[i%m];
	if(i > 0) lastPt = path_os[(i-1)%m]; else lastPt = path_os[i%m]; 
	if(node.closed == true) {
		nextPt = path_os[(i+1)%m];
	} else {
		if(i < m-1 ) nextPt = path_os[(i+1)%m];  else  nextPt = path_os[i%m]; 
	}
  	Vector3d vec_x, vec_y;
	if(i != m+1) {
		calculate_path_dirs(lastPt, curPt,nextPt,vec_x_last, vec_y_last, &vec_x, &vec_y);
		curProfile = calculate_path_profile(&vec_x, &vec_y,curPt,  profilemod.vertices);
	} else 	curProfile = startProfile;
	if(i == 1 && node.closed == true) startProfile=curProfile;

	if((node.closed == false && i == 1) || ( i >= 2)){ // create ring
		// collision detection
		Vector3d vec_z_last = vec_x_last.cross(vec_y_last);
		// check that all new points are above old plane lastPt, vec_z_last
		for(unsigned int j=0;j<n;j++) {
			double dist=(curProfile[j]-lastPt).dot(vec_z_last);
			if(dist < 0) intersect=true;
		}
		
		for(unsigned int j=0;j<n;j++) {
			builder.appendPoly(3);
			builder.appendVertex( builder.vertexIndex(Vector3d(lastProfile[(j+0)%n][0], lastProfile[(j+0)%n][1], lastProfile[(j+0)%n][2])));
			builder.appendVertex( builder.vertexIndex(Vector3d(lastProfile[(j+1)%n][0], lastProfile[(j+1)%n][1], lastProfile[(j+1)%n][2])));
			builder.appendVertex( builder.vertexIndex(Vector3d( curProfile[(j+1)%n][0],  curProfile[(j+1)%n][1],  curProfile[(j+1)%n][2])));
			builder.appendPoly(3);
			builder.appendVertex( builder.vertexIndex(Vector3d(lastProfile[(j+0)%n][0], lastProfile[(j+0)%n][1], lastProfile[(j+0)%n][2])));
			builder.appendVertex( builder.vertexIndex(Vector3d( curProfile[(j+1)%n][0],  curProfile[(j+1)%n][1],  curProfile[(j+1)%n][2])));
			builder.appendVertex(builder.vertexIndex(Vector3d(  curProfile[(j+0)%n][0],  curProfile[(j+0)%n][1],  curProfile[(j+0)%n][2])));
		}
	}
       if(node.closed == false && (i == 0 || i == m-1)) {
		Polygon2d face_poly;
		face_poly.addOutline(profilemod);
		std::unique_ptr<PolySet> ps_face = face_poly.tessellate(); 

		if(i == 0) {
			// Flip vertex ordering for bottom polygon
			for (auto & polygon : ps_face->indices) {
				std::reverse(polygon.begin(), polygon.end());
			}
		}
		for (auto &p3d : ps_face->indices) { 
			std::vector<Vector2d> p2d;
			for(int i=0;i<p3d.size();i++) {
				Vector3d pt = ps_face->vertices[p3d[i]];
				p2d.push_back(Vector2d(pt[0],pt[1]));
			}
			std::vector<Vector3d> newprof = calculate_path_profile(&vec_x, &vec_y,(i == 0)?curPt:nextPt,  p2d);
			builder.appendPoly(newprof.size());
			for(Vector3d pt: newprof) {
				builder.appendVertex(builder.vertexIndex(pt));
			}
		}
	}

	vec_x_last = vec_x.normalized();
	vec_y_last = vec_y.normalized();
	
	lastProfile = curProfile;
    }

  }
  if(intersect == true && node.allow_intersect == false) {
  	return std::unique_ptr<PolySet>();
  }
  return builder.build();
}

/*!
   input: List of 2D objects
   output: 3D PolySet
   operation:
    o Union all children
    o Perform extrude
 */
Response GeometryEvaluator::visit(State& state, const LinearExtrudeNode& node)
{
  if (state.isPrefix() && isSmartCached(node)) return Response::PruneTraversal;
  if (state.isPostfix()) {
    std::shared_ptr<const Geometry> geom;
    if (!isSmartCached(node)) {
      std::shared_ptr<const Geometry> geometry;
      if (!node.filename.empty()) {
        DxfData dxf(node.fn, node.fs, node.fa, node.filename, node.layername, node.origin_x, node.origin_y, node.scale_x);

        auto p2d = dxf.toPolygon2d();
        if (p2d) geometry = ClipperUtils::sanitize(*p2d);
      } else {
        geometry = applyToChildren2D(node, OpenSCADOperator::UNION);
      }
      if (geometry) {
        const auto polygons = std::dynamic_pointer_cast<const Polygon2d>(geometry);
        auto extruded = extrudePolygon(node, *polygons);
        assert(extruded);
        geom = std::move(extruded);
      }
    } else {
      geom = smartCacheGet(node, false);
    }
    addToParent(state, node, geom);
    node.progress_report();
  }
  return Response::ContinueTraversal;
}

Response GeometryEvaluator::visit(State& state, const PathExtrudeNode& node)
{
  if (state.isPrefix() && isSmartCached(node)) return Response::PruneTraversal;
  if (state.isPostfix()) {
    std::shared_ptr<const Geometry> geom;
    if (!isSmartCached(node)) {
      std::shared_ptr<Geometry> geometry =  applyToChildren2D(node, OpenSCADOperator::UNION);
      if (geometry) {
        const auto polygons = std::dynamic_pointer_cast<const Polygon2d>(geometry);
        auto extruded = extrudePolygon(node, *polygons);
        assert(extruded);
        geom = std::move(extruded);
      }
    } else {
      geom = smartCacheGet(node, false);
    }
    addToParent(state, node, geom);
    node.progress_report();
  }
  return Response::ContinueTraversal;
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
static std::unique_ptr<Geometry> rotatePolygon(const RotateExtrudeNode& node, const Polygon2d& poly)
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

  fragments = (unsigned int)std::ceil(fmax(Calc::get_fragments_from_r(max_x - min_x, node.fn, node.fs, node.fa) * std::abs(node.angle) / 360, 1));

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
		builder.append(*ps_last);

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
				builder.appendPoly(3);
				append_rotary_vertex(builder,&lastFace,(j+0)%n, last_ang);
				append_rotary_vertex(builder,&lastFace,(j+1)%n, last_ang);
				append_rotary_vertex(builder,&curFace,(j+1)%n, cur_ang);
				builder.appendPoly(3);
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
		builder.append(*ps_cur);
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
    builder.append(*ps_start);

    auto ps_end = poly.tessellate();
    Transform3d rot2(angle_axis_degrees(node.angle, Vector3d::UnitZ()) * angle_axis_degrees(90, Vector3d::UnitX()));
    ps_end->transform(rot2);
    if (flip_faces) {
      for (auto& p : ps_end->indices) {
        std::reverse(p.begin(), p.end());
      }
    }
    builder.append(*ps_end);
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
        builder.appendPoly({
		rings[j % 2][(i + 1) % o.vertices.size()],
		rings[(j + 1) % 2][(i + 1) % o.vertices.size()],
		rings[j % 2][i]
	});		

        builder.appendPoly({
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

/*!
   input: List of 2D objects
   output: 3D PolySet
   operation:
    o Union all children
    o Perform extrude
 */
Response GeometryEvaluator::visit(State& state, const RotateExtrudeNode& node)
{
  if (state.isPrefix() && isSmartCached(node)) return Response::PruneTraversal;
  if (state.isPostfix()) {
    std::shared_ptr<const Geometry> geom;
    if (!isSmartCached(node)) {
      std::shared_ptr<const Polygon2d> geometry;
      if (!node.filename.empty()) {
        DxfData dxf(node.fn, node.fs, node.fa, node.filename, node.layername, node.origin_x, node.origin_y, node.scale);
        auto p2d = dxf.toPolygon2d();
        if (p2d) geometry = ClipperUtils::sanitize(*p2d);
      } else {
        geometry = applyToChildren2D(node, OpenSCADOperator::UNION);
      }
      if (geometry) {
	geom = rotatePolygon(node, *geometry);
      }
    } else {
      geom = smartCacheGet(node, false);
    }
    addToParent(state, node, geom);
    node.progress_report();
  }
  return Response::ContinueTraversal;
}

static int pullObject_calccut(const PullNode &node, Vector3d p1, Vector3d p2,Vector3d &r)
{
	Vector3d dir=p2-p1;
	Vector3d res;
	Vector3d v1, v2;
	Vector3d z(0,0,1);
	v1=node.dir.cross(dir);
	v2=node.dir.cross(v1);
	if(linsystem(dir,v1,v2,node.anchor-p1,res)) return 1;

	if(res[0] < 0 || res[0] > 1) return 1;
	r =p1+dir*res[0];
	return 0;
}
static void pullObject_addtri(PolySetBuilder &builder,Vector3d a, Vector3d b, Vector3d c)
{
	builder.appendPoly(3);
	builder.prependVertex(builder.vertexIndex(Vector3d(a[0], a[1], a[2])));
	builder.prependVertex(builder.vertexIndex(Vector3d(b[0], b[1], b[2])));
	builder.prependVertex(builder.vertexIndex(Vector3d(c[0], c[1], c[2])));
}
static std::unique_ptr<PolySet> pullObject(const PullNode& node, const PolySet *ps)
{
  PolySetBuilder builder(0,0,3,true);
  auto ps_tess = PolySetUtils::tessellate_faces( *ps);
  for(int i=0;i<ps_tess->indices.size();i++) {
	  auto pol = ps_tess->indices[i];

	  //count upper points
	  int upper=0;
	  int lowind=0;
	  int highind=0;

	  for(int j=0;j<3;j++) { 
		Vector3d pt=ps_tess->vertices[pol[j]];
		float dist=(pt-node.anchor).dot(node.dir);
		if(dist > 0) { upper++; highind += j; } else {lowind += j; }
	  }
	  switch(upper)
	  {
		  case 0:
	  		builder.appendPoly(3);
			for(int j=0;j<3;j++) { 
			        builder.prependVertex(pol[j]);
			}
			break;
		  case 1:
			{
				std::vector<int> pol1;
				pol1.push_back(pol[(highind+1)%3]);
				pol1.push_back(pol[(highind+2)%3]);
				pol1.push_back(pol[highind]);
				// pol1[2] ist oben 
				//
				Vector3d p02, p12;
				if(pullObject_calccut(node, ps_tess->vertices[pol1[0]],ps_tess->vertices[pol1[2]],p02)) break;
				if(pullObject_calccut(node, ps_tess->vertices[pol1[1]],ps_tess->vertices[pol1[2]],p12)) break;

				pullObject_addtri(builder, ps_tess->vertices[pol1[0]],ps_tess->vertices[pol1[1]], p12);
				pullObject_addtri(builder, ps_tess->vertices[pol1[0]], p12,p02);

				pullObject_addtri(builder, p02,p12,p12+node.dir);
				pullObject_addtri(builder, p02,p12+node.dir,p02+node.dir);

				pullObject_addtri(builder, p02+node.dir,p12+node.dir,ps_tess->vertices[pol1[2]]+node.dir);
			}
		  case 2: 
			{
				std::vector<int> pol1;
				pol1.push_back(pol[(lowind+1)%3]);
				pol1.push_back(pol[(lowind+2)%3]);
				pol1.push_back(pol[lowind]);
				// pol1[2] ist unten 
				//
				Vector3d p02, p12;
				if(pullObject_calccut(node, ps_tess->vertices[pol1[0]],ps_tess->vertices[pol1[2]],p02)) break;
				if(pullObject_calccut(node, ps_tess->vertices[pol1[1]],ps_tess->vertices[pol1[2]],p12)) break;

				pullObject_addtri(builder, ps_tess->vertices[pol1[2]],p02, p12);

				pullObject_addtri(builder, p12, p02, p02+node.dir);
				pullObject_addtri(builder, p12, p02+node.dir,p12+node.dir) ;

				pullObject_addtri(builder, p12+node.dir,p02+node.dir,ps_tess->vertices[pol1[0]]+node.dir);
				pullObject_addtri(builder, p12+node.dir,ps_tess->vertices[pol1[0]]+node.dir,ps_tess->vertices[pol1[1]]+node.dir);
			}
			break;
		  case 3:
	  		builder.appendPoly(3);
			for(int j=0;j<3;j++) { 
				Vector3d pt=ps_tess->vertices[pol[j]]+node.dir;
			        builder.prependVertex(builder.vertexIndex(Vector3d(pt[0],pt[1], pt[2])));
			}
			break;
	  }
  }

  return builder.build();
}

Response GeometryEvaluator::visit(State& state, const PullNode& node)
{
  std::shared_ptr<const Geometry> newgeom;
  std::shared_ptr<const Geometry> geom = applyToChildren3D(node, OpenSCADOperator::UNION).constptr();
  if (geom) {
    if(std::shared_ptr<const PolySet> ps = std::dynamic_pointer_cast<const PolySet>(geom)) {
      std::unique_ptr<Geometry> ps_pulled =  pullObject(node,ps.get());
      newgeom = std::move(ps_pulled);
      addToParent(state, node, newgeom);
      node.progress_report();
    }
  }
  return Response::ContinueTraversal;
}

/*!
   FIXME: Not in use
 */
Response GeometryEvaluator::visit(State& /*state*/, const AbstractPolyNode& /*node*/)
{
  assert(false);
  return Response::AbortTraversal;
}

std::shared_ptr<const Geometry> GeometryEvaluator::projectionCut(const ProjectionNode& node)
{
  std::shared_ptr<const Geometry> geom;
  std::shared_ptr<const Geometry> newgeom = applyToChildren3D(node, OpenSCADOperator::UNION).constptr();
  if (newgeom) {
#ifdef ENABLE_CGAL
    auto Nptr = CGALUtils::getNefPolyhedronFromGeometry(newgeom);
    if (Nptr && !Nptr->isEmpty()) {
      auto poly = CGALUtils::project(*Nptr, node.cut_mode);
      if (poly) {
        poly->setConvexity(node.convexity);
        geom = std::move(poly);
      }
    }
#endif
  }
  return geom;
}

std::shared_ptr<const Geometry> GeometryEvaluator::projectionNoCut(const ProjectionNode& node)
{
  std::shared_ptr<const Geometry> geom;
  std::vector<std::unique_ptr<Polygon2d>> tmp_geom;
  BoundingBox bounds;
  for (const auto& [chnode, chgeom] : this->visitedchildren[node.index()]) {
    if (chnode->modinst->isBackground()) continue;

    // Clipper version of Geometry projection
    // Clipper doesn't handle meshes very well.
    // It's better in V6 but not quite there. FIXME: stand-alone example.
    // project chgeom -> polygon2d
    if (auto chPS = PolySetUtils::getGeometryAsPolySet(chgeom)) {
      if (auto poly = PolySetUtils::project(*chPS)) {
	bounds.extend(poly->getBoundingBox());
	tmp_geom.push_back(std::move(poly));
      }
    }
  }
  int pow2 = ClipperUtils::getScalePow2(bounds);

  ClipperLib::Clipper sumclipper;
  for (auto &poly : tmp_geom) {
    ClipperLib::Paths result = ClipperUtils::fromPolygon2d(*poly, pow2);
    // Using NonZero ensures that we don't create holes from polygons sharing
    // edges since we're unioning a mesh
    result = ClipperUtils::process(result, ClipperLib::ctUnion, ClipperLib::pftNonZero);
    // Add correctly winded polygons to the main clipper
    sumclipper.AddPaths(result, ClipperLib::ptSubject, true);
  }

  ClipperLib::PolyTree sumresult;
  // This is key - without StrictlySimple, we tend to get self-intersecting results
  sumclipper.StrictlySimple(true);
  sumclipper.Execute(ClipperLib::ctUnion, sumresult, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
  if (sumresult.Total() > 0) {
    geom = ClipperUtils::toPolygon2d(sumresult, pow2);
  }

  return geom;
}


/*!
   input: List of 3D objects
   output: Polygon2d
   operation:
    o Union all children
    o Perform projection
 */
Response GeometryEvaluator::visit(State& state, const ProjectionNode& node)
{
  if (state.isPrefix() && isSmartCached(node)) return Response::PruneTraversal;
  if (state.isPostfix()) {
    std::shared_ptr<const Geometry> geom;
    if (isSmartCached(node)) {
      geom = smartCacheGet(node, false);
    } else {
      if (node.cut_mode) {
        geom = projectionCut(node);
      } else {
        geom = projectionNoCut(node);
      }
    }
    addToParent(state, node, geom);
    node.progress_report();
  }
  return Response::ContinueTraversal;
}

/*!
   input: List of 2D or 3D objects (not mixed)
   output: any Geometry
   operation:
    o Perform cgal operation
 */
Response GeometryEvaluator::visit(State& state, const CgalAdvNode& node)
{
  if (state.isPrefix() && isSmartCached(node)) return Response::PruneTraversal;
  if (state.isPostfix()) {
    std::shared_ptr<const Geometry> geom;
    if (!isSmartCached(node)) {
      switch (node.type) {
      case CgalAdvType::MINKOWSKI: {
        ResultObject res = applyToChildren(node, OpenSCADOperator::MINKOWSKI);
        geom = res.constptr();
        // If we added convexity, we need to pass it on
        if (geom && geom->getConvexity() != node.convexity) {
	  std::shared_ptr<Geometry> editablegeom;
          // If we got a const object, make a copy
          if (res.isConst()) editablegeom = geom->copy();
          else editablegeom = res.ptr();
          geom = editablegeom;
          editablegeom->setConvexity(node.convexity);
        }
        break;
      }
      case CgalAdvType::HULL: {
        geom = applyToChildren(node, OpenSCADOperator::HULL).constptr();
        break;
      }
      case CgalAdvType::FILL: {
        geom = applyToChildren(node, OpenSCADOperator::FILL).constptr();
        break;
      }
      case CgalAdvType::RESIZE: {
        ResultObject res = applyToChildren(node, OpenSCADOperator::UNION);
        auto editablegeom = res.asMutableGeometry();
        geom = editablegeom;
        if (editablegeom) {
          editablegeom->setConvexity(node.convexity);
          editablegeom->resize(node.newsize, node.autosize);
        }
        break;
      }
      default:
        assert(false && "not implemented");
      }
    } else {
      geom = smartCacheGet(node, state.preferNef());
    }
    addToParent(state, node, geom);
    node.progress_report();
  }
  return Response::ContinueTraversal;
}

Response GeometryEvaluator::visit(State& state, const AbstractIntersectionNode& node)
{
  if (state.isPrefix()) {
    if (isSmartCached(node)) return Response::PruneTraversal;
    state.setPreferNef(true); // Improve quality of CSG by avoiding conversion loss
  }
  if (state.isPostfix()) {
    std::shared_ptr<const Geometry> geom;
    if (!isSmartCached(node)) {
      geom = applyToChildren(node, OpenSCADOperator::INTERSECTION).constptr();
    } else {
      geom = smartCacheGet(node, state.preferNef());
    }
    addToParent(state, node, geom);
    node.progress_report();
  }
  return Response::ContinueTraversal;
}

#if defined(ENABLE_EXPERIMENTAL) && defined(ENABLE_CGAL)
static std::unique_ptr<Geometry> roofOverPolygon(const RoofNode& node, const Polygon2d& poly)
{
  std::unique_ptr<PolySet> roof;
  if (node.method == "voronoi") {
    roof = roof_vd::voronoi_diagram_roof(poly, node.fa, node.fs);
    roof->setConvexity(node.convexity);
  } else if (node.method == "straight") {
    roof = roof_ss::straight_skeleton_roof(poly);
    roof->setConvexity(node.convexity);
  } else {
    assert(false && "Invalid roof method");
  }

  return roof;
}

Response GeometryEvaluator::visit(State& state, const RoofNode& node)
{
  if (state.isPrefix() && isSmartCached(node)) return Response::PruneTraversal;
  if (state.isPostfix()) {
    std::shared_ptr<const Geometry> geom;
    if (!isSmartCached(node)) {
      const auto polygon2d = applyToChildren2D(node, OpenSCADOperator::UNION);
      if (polygon2d) {
	std::unique_ptr<Geometry> roof;
        try {
          roof = roofOverPolygon(node, *polygon2d);
        } catch (RoofNode::roof_exception& e) {
          LOG(message_group::Error, node.modinst->location(), this->tree.getDocumentPath(),
              "Skeleton computation error. " + e.message());
          roof = std::make_unique<PolySet>(3);
        }
        assert(roof);
        geom = std::move(roof);
      }
    } else {
      geom = smartCacheGet(node, false);
    }
    addToParent(state, node, geom);
  }
  return Response::ContinueTraversal;
}
#endif
