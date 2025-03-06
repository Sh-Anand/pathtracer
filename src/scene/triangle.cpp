#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::test(const Ray &r, double &t, double &u, double &v) const {

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

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.

  double t,u,v;
  return test(r, t, u, v);
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

  double t,u,v;
  if (!test(r, t, u, v)) {
    return false;
  }

  isect->n = (1 - u - v) * n1 + u * n2 + v * n3;
  isect->t = t; isect->primitive = this; isect->bsdf = get_bsdf();

  return true;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
