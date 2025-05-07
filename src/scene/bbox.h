#ifndef CGL_BBOX_H
#define CGL_BBOX_H

#include <utility>
#include <algorithm>

#include "scene/ray.h"

HOST_DEVICE inline bool intersect_bbox(
    Ray &r,
    const Vector3D &mn,
    const Vector3D &mx,
    float &t0,
    float &t1) {

  // Precompute
  const float inv_dx = r.inv_d.x;
  const float inv_dy = r.inv_d.y;
  const float inv_dz = r.inv_d.z;

  // X slab
  float tx1 = (mn.x - r.o.x) * inv_dx;
  float tx2 = (mx.x - r.o.x) * inv_dx;
  float tmin = fminf(tx1, tx2);
  float tmax = fmaxf(tx1, tx2);
  // early exit if miss or outside [min_t,max_t]
  if (tmax < r.min_t || tmin > r.max_t || tmin > tmax) 
    return false;

  // Y slab
  float ty1 = (mn.y - r.o.y) * inv_dy;
  float ty2 = (mx.y - r.o.y) * inv_dy;
  float tymin = fminf(ty1, ty2);
  float tymax = fmaxf(ty1, ty2);
  // tighten interval
  tmin = fmaxf(tmin, tymin);
  tmax = fminf(tmax, tymax);
  // early exit again
  if (tmax < r.min_t || tmin > r.max_t || tmin > tmax) 
    return false;

  // Z slab
  float tz1 = (mn.z - r.o.z) * inv_dz;
  float tz2 = (mx.z - r.o.z) * inv_dz;
  float tzmin = fminf(tz1, tz2);
  float tzmax = fmaxf(tz1, tz2);
  // final tighten
  tmin = fmaxf(tmin, tzmin);
  tmax = fminf(tmax, tzmax);
  if (tmax < r.min_t || tmin > r.max_t || tmin > tmax) 
    return false;

  // Hit!
  t0 = tmin;
  t1 = tmax;
  return true;
}

struct BBox {

  Vector3D max;	    ///< min corner of the bounding box
  Vector3D min;	    ///< max corner of the bounding box
  Vector3D extent;  ///< extent of the bounding box (min -> max)

  /**
   * Constructor.
   * The default constructor creates a new bounding box which contains no
   * points.
   */
  BBox() {
    max = Vector3D(-INFINITY, -INFINITY, -INFINITY);
    min = Vector3D( INFINITY,  INFINITY,  INFINITY);
    extent = max - min;
  }

  /**
   * Constructor.
   * Creates a bounding box that includes a single point.
   */
  BBox(const Vector3D p) : min(p), max(p) { extent = max - min; }

  /**
   * Constructor.
   * Creates a bounding box with given bounds.
   * \param min the min corner
   * \param max the max corner
   */
  BBox(const Vector3D min, const Vector3D max) :
       min(min), max(max) { extent = max - min; }

  /**
   * Constructor.
   * Creates a bounding box with given bounds (component wise).
   */
  BBox(const float minX, const float minY, const float minZ,
       const float maxX, const float maxY, const float maxZ) {
    min = Vector3D(minX, minY, minZ);
    max = Vector3D(maxX, maxY, maxZ);
		extent = max - min;
  }

  /**
   * Expand the bounding box to include another (union).
   * If the given bounding box is contained within *this*, nothing happens.
   * Otherwise *this* is expanded to the minimum volume that contains the
   * given input.
   * \param bbox the bounding box to be included
   */
  void expand(const BBox& bbox) {
    min.x = std::min(min.x, bbox.min.x);
    min.y = std::min(min.y, bbox.min.y);
    min.z = std::min(min.z, bbox.min.z);
    max.x = std::max(max.x, bbox.max.x);
    max.y = std::max(max.y, bbox.max.y);
    max.z = std::max(max.z, bbox.max.z);
    extent = max - min;
  }

  /**
   * Expand the bounding box to include a new point in space.
   * If the given point is already inside *this*, nothing happens.
   * Otherwise *this* is expanded to a minimum volume that contains the given
   * point.
   * \param p the point to be included
   */
  void expand(const Vector3D p) {
    min.x = std::min(min.x, p.x);
    min.y = std::min(min.y, p.y);
    min.z = std::min(min.z, p.z);
    max.x = std::max(max.x, p.x);
    max.y = std::max(max.y, p.y);
    max.z = std::max(max.z, p.z);
    extent = max - min;
  }

  Vector3D centroid() const {
    return (min + max) / 2;
  }

  /**
   * Compute the surface area of the bounding box.
   * \return surface area of the bounding box.
   */
  float surface_area() const {
    if (empty()) return 0.0;
    return 2 * (extent.x * extent.z +
                extent.x * extent.y +
                extent.y * extent.z);
  }

  /**
   * Check if bounding box is empty.
   * Bounding box that has no size is considered empty. Note that since
   * bounding box are used for objects with positive volumes, a bounding
   * box of zero size (empty, or contains a single vertex) are considered
   * empty.
   */
  bool empty() const {
    return min.x > max.x || min.y > max.y || min.z > max.z;
  }

  /**
   * Ray - bbox intersection.
   * Intersects ray with bounding box, does not store shading information.
   * \param r the ray to intersect with
   * \param t0 lower bound of intersection time
   * \param t1 upper bound of intersection time
   */
  HOST_DEVICE bool intersect(Ray& r, float& t0, float& t1) const {
    return intersect_bbox(r, min, max, t0, t1);
  }
};

#endif // CGL_BBOX_H
