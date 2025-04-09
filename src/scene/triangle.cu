#include "triangle.h"

namespace CGL { namespace SceneObjects {

DEVICE bool test_intersect(Ray &r, const Vector3D &p1,
                  const Vector3D &p2, const Vector3D &p3, double &t, double &u,
                  double &v) {
  Vector3D e1 = p2 - p1, e2 = p3 - p1;
  Vector3D normal = cross(e1, e2);
  // early termination
  if (dot(normal, r.d) == 0) {
    return false;
  }

  Vector3D s = r.o - p1, s1 = cross(r.d, e2), s2 = cross(s, e1);
  double rse1 = 1 / (dot(s1, e1));

  t = dot(s2, e2) * rse1, u = dot(s1, s) * rse1, v = dot(s2, r.d) * rse1;
  if (t < r.min_t || t > r.max_t || u < 0 || v < 0 || u + v > 1) {
    return false;
  }

  r.max_t = t;
  return true;
}

DEVICE bool CudaTriangle::intersect(Ray &r, CudaIntersection *isect) {
  double t,u,v;
  if (!test_intersect(r, p1, p2, p3, t, u, v)) {
    return false;
  }

  isect->n = (1 - u - v) * n1 + u * n2 + v * n3;
  isect->t = t; isect->bsdf = bsdf;

  return true;
}

}
}