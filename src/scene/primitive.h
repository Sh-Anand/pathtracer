#ifndef CGL_STATICSCENE_PRIMITIVE_H
#define CGL_STATICSCENE_PRIMITIVE_H

#include "pathtracer/intersection.h"
#include "scene/bbox.h"
#include <cstddef>

namespace CGL { namespace SceneObjects {

struct CudaPrimitive {

  DEVICE bool intersect(Ray& r, CudaIntersection* i, Vector3D* vertices, Vector3D* normals, Vector2D* texcoords, Vector4D* tangets) const;
  DEVICE bool has_intersect(Ray &r, const Vector3D * vertices, double &t) const;

  uint32_t i_p1, i_p2, i_p3;
  uint32_t i_n1, i_n2, i_n3;
  uint32_t i_uv1, i_uv2, i_uv3;
  
  int bsdf_idx;
};


} // namespace SceneObjects
} // namespace CGL

#endif //CGL_STATICSCENE_PRIMITIVE_H
