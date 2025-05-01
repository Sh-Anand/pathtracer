#ifndef CGL_STATICSCENE_LIGHT_H
#define CGL_STATICSCENE_LIGHT_H

#include "scene/primitive.h"
#include "util/vector3D.h"

namespace CGL { namespace SceneObjects {

struct CudaLight {
  CudaLight(const Vector3D rad, 
                const CudaPrimitive tri) 
      : radiance(rad), 
        triangle(tri) {
          area = 0.5 * cross(tri.p2 - tri.p1, tri.p3 - tri.p1).norm();
        }

  Vector3D radiance;
  CudaPrimitive triangle;

  double area;
};

} // namespace SceneObjects
} // namespace CGL

#endif  // CGL_STATICSCENE_BSDF_H
