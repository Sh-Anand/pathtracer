#ifndef CGL_INTERSECT_H
#define CGL_INTERSECT_H

#include <vector>

#include "util/vector2D.h"
#include "util/vector3D.h"
#include "util/vector4D.h"

#include "pathtracer/bsdf.h"

struct CudaIntersection {
  DEVICE CudaIntersection() {
    #ifdef __CUDA_ARCH__
      #include <math_constants.h>
      t = CUDART_INF_F;
    #else
      t = INFINITY;
    #endif
   }
  float t;
  Vector3D n;
  Vector2D uv;
  Vector4D tangent;
  int bsdf_idx;
};

#endif // CGL_INTERSECT_H
