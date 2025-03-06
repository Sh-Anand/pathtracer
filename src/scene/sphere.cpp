#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

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

  r.max_t = t2;
  return true;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  double t1, t2;
  return test(r, t1, t2);
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t1, t2;
  if (!test(r, t1, t2)) {
    return false;
  }


  if (t1 <= r.max_t && t1 >= r.min_t) {
    i->t = t1;
  } else if (t2 <= r.max_t && t2 >= r.min_t) {
    i->t = t2;
  } else {
    return false;
  }

  i->primitive = this;
  i->bsdf = get_bsdf();  
  i->n = normal(r.o + i->t * r.d);
  
  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
