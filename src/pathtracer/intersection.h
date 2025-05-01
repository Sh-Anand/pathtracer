#ifndef CGL_INTERSECT_H
#define CGL_INTERSECT_H

#include <vector>

#include "util/vector3D.h"
#include "util/vector2D.h"

#include "pathtracer/bsdf.h"

namespace CGL {

struct CudaIntersection {
  DEVICE CudaIntersection() : t (INFINITY) { }
  double t;
  Vector3D n;
  Vector2D uv;
  int bsdf_idx;
  int tex_idx;
};

} // namespace CGL

#endif // CGL_INTERSECT_H
