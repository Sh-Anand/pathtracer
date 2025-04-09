#include <curand_kernel.h>
#include "bsdf.h"

namespace CGL {

DEVICE void make_coord_space(Matrix3x3 &o2w, const Vector3D n) {

  Vector3D z = Vector3D(n.x, n.y, n.z);
  Vector3D h = z;
  if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
    h.x = 1.0;
  else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
    h.y = 1.0;
  else
    h.z = 1.0;

  z.normalize();
  Vector3D y = cross(h, z);
  y.normalize();
  Vector3D x = cross(z, y);
  x.normalize();

  o2w[0] = x;
  o2w[1] = y;
  o2w[2] = z;
}

DEVICE Vector3D CudaDiffuseBSDF::f(const Vector3D wo, const Vector3D wi) {
  return reflectance / PI;
}

DEVICE Vector3D CudaEmissionBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

}