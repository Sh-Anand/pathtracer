#include "primitive.h"

namespace CGL { namespace SceneObjects {

DEVICE bool sphere_intersect(Ray &r, const Vector3D &o, double r2,
                      double &t1, double &t2) {
  Vector3D ominc = r.o - o;
  double a = dot(r.d, r.d), b = 2 * dot(ominc, r.d), c = dot(ominc, ominc) - r2;
  double discr = b*b - 4*a*c;

  if (discr < 0) {
    return false;
  }

  double r2a = 1/(2*a);
  double b2a = -b*r2a, sqrt_discr_r2a = sqrt(discr) * r2a;
  t1 = b2a - sqrt_discr_r2a;
  t2 = b2a + sqrt_discr_r2a;

  return true;
}

DEVICE bool CudaSphere::intersect(Ray &r, CudaIntersection *i) {
  double t1, t2;
  if (!sphere_intersect(r, o, r2, t1, t2)) {
    return false;
  }

  if (t1 <= r.max_t && t1 >= r.min_t) {
    i->t = t1;
  } else if (t2 <= r.max_t && t2 >= r.min_t) {
    i->t = t2;
  } else {
    return false;
  }

  i->n = normal(r.o + i->t * r.d);
  r.max_t = i->t;
  
  return true;
}

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
  isect->t = t;

  return true;
}

}
}