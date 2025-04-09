#include "sphere.h"

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

  i->bsdf = bsdf;  
  i->n = normal(r.o + i->t * r.d);
  r.max_t = i->t;
  
  return true;
}

}
}