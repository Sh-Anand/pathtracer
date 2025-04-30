#ifndef CGL_STATICSCENE_PRIMITIVE_H
#define CGL_STATICSCENE_PRIMITIVE_H

#include "pathtracer/intersection.h"
#include "scene/bbox.h"

namespace CGL { namespace SceneObjects {

struct CudaPrimitive {

  DEVICE bool intersect(Ray& r, CudaIntersection* i);

  Vector3D p1, p2, p3;
  Vector3D n1, n2, n3;
  uint32_t bsdf_idx;
};


} // namespace SceneObjects
} // namespace CGL

#endif //CGL_STATICSCENE_PRIMITIVE_H
