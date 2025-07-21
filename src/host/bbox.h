#ifndef BBOX_H
#define BBOX_H

#include "scene/geometry.h"

struct BBox {

  Vector3D max;	    ///< min corner of the bounding box
  Vector3D min;	    ///< max corner of the bounding box
  Vector3D extent;  ///< extent of the bounding box (min -> max)

  BBox() {
    max = Vector3D{-INFINITY,-INFINITY,-INFINITY};
    min = Vector3D{INFINITY,INFINITY,INFINITY};
    extent = vector3d_sub(max, min);
  }

  BBox(const Vector3D p) : min(p), max(p) { extent = vector3d_sub(max, min); }

  void expand(const BBox& bbox) {
    min.x = std::min(min.x, bbox.min.x);
    min.y = std::min(min.y, bbox.min.y);
    min.z = std::min(min.z, bbox.min.z);
    max.x = std::max(max.x, bbox.max.x);
    max.y = std::max(max.y, bbox.max.y);
    max.z = std::max(max.z, bbox.max.z);
    extent = vector3d_sub(max, min);
  }

  void expand(const Vector3D p) {
    min.x = std::min(min.x, p.x);
    min.y = std::min(min.y, p.y);
    min.z = std::min(min.z, p.z);
    max.x = std::max(max.x, p.x);
    max.y = std::max(max.y, p.y);
    max.z = std::max(max.z, p.z);
    extent = vector3d_sub(max, min);
  }

  Vector3D centroid() const {
    return vector3d_scale(vector3d_add(min, max), 0.5);
  }

  float surface_area() const {
    return 2 * (extent.x * extent.z +
                extent.x * extent.y +
                extent.y * extent.z);
  }
};

#endif // BBOX_H
