#ifndef CGL_STATICSCENE_PRIMITIVE_H
#define CGL_STATICSCENE_PRIMITIVE_H

#include <cstdint>
#include "util/vector.h"
typedef struct {
  uint32_t i_p1, i_p2, i_p3;
  uint32_t i_n1, i_n2, i_n3;
  uint32_t i_uv1, i_uv2, i_uv3;
  int bsdf_idx;
} CudaPrimitive;

typedef struct {
  Vector3D o;  ///< origin
  Vector3D d;  ///< direction
  float min_t; ///< treat the ray as a segment (ray "begin" at min_t)
  float max_t; ///< treat the ray as a segment (ray "ends" at max_t)

  Vector3D inv_d;  ///< component wise inverse
} Ray;

typedef struct {
  float t;
  Vector3D n;
  Vector2D uv;
  Vector4D tangent;
  int bsdf_idx;
} CudaIntersection;

DEVICE static inline Vector3D ray_at(const Ray r, float t) {
  return Vector3D{r.o.x + r.d.x * t, r.o.y + r.d.y * t, r.o.z + r.d.z * t};
}

DEVICE static inline bool test_intersect(Ray* r, const Vector3D &p1,
                  const Vector3D &p2, const Vector3D &p3, float &t, float &u,
                  float &v) {
  Vector3D e1 = vector3d_sub(p2, p1), e2 = vector3d_sub(p3, p1);
  Vector3D normal = vector3d_cross(e1, e2);
  // early termination
  if (vector3d_dot(normal, r->d) == 0) {
    return false;
  }

  Vector3D s = vector3d_sub(r->o, p1), s1 = vector3d_cross(r->d, e2), s2 = vector3d_cross(s, e1);
  float rse1 = 1 / (vector3d_dot(s1, e1));

  t = vector3d_dot(s2, e2) * rse1, u = vector3d_dot(s1, s) * rse1, v = vector3d_dot(s2, r->d) * rse1;
  if (t < r->min_t || t > r->max_t || u < 0 || v < 0 || u + v > 1) {
    return false;
  }

  r->max_t = t;
  return true;
}

DEVICE static inline bool primitive_intersect(const CudaPrimitive *p, Ray* r, CudaIntersection* isect, const Vector3D* vertices, const Vector3D* normals, const Vector2D* texcoords, const Vector4D* tangents) {
  Vector3D tp1 = vertices[p->i_p1], tp2 = vertices[p->i_p2], tp3 = vertices[p->i_p3];
  Vector3D tn1 = normals[p->i_p1], tn2 = normals[p->i_p2], tn3 = normals[p->i_p3];
  Vector2D tuv1 = texcoords[p->i_uv1], tuv2 = texcoords[p->i_uv2], tuv3 = texcoords[p->i_uv3];
  Vector4D tt1 = tangents[p->i_uv1], tt2 = tangents[p->i_uv2], tt3 = tangents[p->i_uv3];
  float t,a,b;
  if (!test_intersect(r, tp1, tp2, tp3, t, a, b)) {
    return false;
  }

  float onemab = 1 - a - b;
  isect->n.x = onemab * tn1.x + a * tn2.x + b * tn3.x;
  isect->n.y = onemab * tn1.y + a * tn2.y + b * tn3.y;
  isect->n.z = onemab * tn1.z + a * tn2.z + b * tn3.z;
  isect->uv.x = onemab * tuv1.x + a * tuv2.x + b * tuv3.x;
  isect->uv.y = onemab * tuv1.y + a * tuv2.y + b * tuv3.y;
  isect->tangent.x = onemab * tt1.x + a * tt2.x + b * tt3.x;
  isect->tangent.y = onemab * tt1.y + a * tt2.y + b * tt3.y;
  isect->tangent.z = onemab * tt1.z + a * tt2.z + b * tt3.z;
  isect->tangent.w = onemab * tt1.w + a * tt2.w + b * tt3.w;
  isect->t = t;
  isect->bsdf_idx = p->bsdf_idx;

  return true;
}
DEVICE static inline bool primitive_has_intersect(const CudaPrimitive* p, Ray* r, const Vector3D * vertices, float &t) {
  Vector3D tp1 = vertices[p->i_p1], tp2 = vertices[p->i_p2], tp3 = vertices[p->i_p3];
  float a, b;
  if (!test_intersect(r, tp1, tp2, tp3, t, a, b)) {
    return false;
  }

  return true;
}

#endif //CGL_STATICSCENE_PRIMITIVE_H
