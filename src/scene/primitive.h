#ifndef CGL_STATICSCENE_PRIMITIVE_H
#define CGL_STATICSCENE_PRIMITIVE_H

#include "pathtracer/intersection.h"
#include "scene/bbox.h"

namespace CGL { namespace SceneObjects {

struct CudaPrimitive {

  DEVICE bool intersect(Ray& r, CudaIntersection* i);
  BBox get_bbox() const { 
    return bbox;
  }

  Vector3D p1, p2, p3;
  Vector3D n1, n2, n3;
  double area;
  uint32_t bsdf_idx;

  BBox bbox; 

  CudaPrimitive() {}
  CudaPrimitive(Vector3D p1, Vector3D p2, Vector3D p3, 
                Vector3D n1, Vector3D n2, Vector3D n3,
                uint16_t bsdf_idx) 
    : p1(p1), p2(p2), p3(p3), n1(n1), n2(n2), n3(n3), 
      bsdf_idx(bsdf_idx) {
    area = 0.5 * cross(p2 - p1, p3 - p1).norm();
    bbox = BBox(p1);
    bbox.expand(p2);
    bbox.expand(p3);
  } 
};


} // namespace SceneObjects
} // namespace CGL

#endif //CGL_STATICSCENE_PRIMITIVE_H
