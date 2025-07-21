#ifndef CGL_BVH_H
#define CGL_BVH_H

#include "geometry.h"

typedef struct {
  Vector3D bbmin, bbmax;        ///< bounding box of the node
  uint32_t start;
  uint32_t end;
  uint32_t l, r;
} BVHNode;


DEVICE static inline int intersect_bbox(
    Ray r,
    const Vector3D mn,
    const Vector3D mx,
    float *t0,
    float *t1) {

  // Precompute
  const float inv_dx = r.inv_d.x;
  const float inv_dy = r.inv_d.y;
  const float inv_dz = r.inv_d.z;

  // X slab
  float tx1 = (mn.x - r.o.x) * inv_dx;
  float tx2 = (mx.x - r.o.x) * inv_dx;
  float tmin = fminf(tx1, tx2);
  float tmax = fmaxf(tx1, tx2);
  // early exit if miss or outside [min_t,max_t]
  if (tmax < r.min_t || tmin > r.max_t || tmin > tmax) 
    return 0;

  // Y slab
  float ty1 = (mn.y - r.o.y) * inv_dy;
  float ty2 = (mx.y - r.o.y) * inv_dy;
  float tymin = fminf(ty1, ty2);
  float tymax = fmaxf(ty1, ty2);
  // tighten interval
  tmin = fmaxf(tmin, tymin);
  tmax = fminf(tmax, tymax);
  // early exit again
  if (tmax < r.min_t || tmin > r.max_t || tmin > tmax) 
    return 0;

  // Z slab
  float tz1 = (mn.z - r.o.z) * inv_dz;
  float tz2 = (mx.z - r.o.z) * inv_dz;
  float tzmin = fminf(tz1, tz2);
  float tzmax = fmaxf(tz1, tz2);
  // final tighten
  tmin = fmaxf(tmin, tzmin);
  tmax = fminf(tmax, tzmax);
  if (tmax < r.min_t || tmin > r.max_t || tmin > tmax) 
    return 0;

  // Hit!
  *t0 = tmin;
  *t1 = tmax;
  return 1;
}

DEVICE static inline int intersect(Ray *ray, const CudaPrimitive *primitives, const Vector3D *vertices, const Vector3D *normals,
                                     const Vector2D *texcoords, const Vector4D *tangents, const BVHNode *nodes, CudaIntersection *i) {
  constexpr int STACK_SIZE = 20;
  uint32_t stack[STACK_SIZE];
  int stack_ptr = 0;

  stack[stack_ptr++] = 0;
  int hit = 0;

  while (stack_ptr > 0) {
    uint32_t idx = stack[--stack_ptr];
    const BVHNode &node = nodes[idx];

    float t0, t1;
    if (!intersect_bbox(*ray, node.bbmin, node.bbmax, &t0, &t1)) continue;

    if (node.start != node.end) {
      CudaIntersection tmp; tmp.t = INFINITY;
      for (uint32_t p = node.start; p < node.end; p++) {
          if (primitive_intersect(&(primitives[p]), ray, &tmp, vertices, normals, texcoords, tangents) && tmp.t < i->t) {
            hit = 1;
            *i = tmp;
          }
      }
    } else {
      // Push children in reverse order so left is processed first
      if (stack_ptr + 2 > STACK_SIZE) break; // Prevent stack overflow
      stack[stack_ptr++] = node.r;
      stack[stack_ptr++] = node.l;
    }
  }

  return hit;
}

DEVICE static inline int has_intersect(const CudaPrimitive *primitives, const Vector3D *vertices, const BVHNode* nodes, Ray *ray) {
  constexpr int STACK_SIZE = 20;
  uint32_t stack[STACK_SIZE];
  int stack_ptr = 0;

  // start with the root
  stack[stack_ptr++] = 0;
  
  float t;
  // traverse until stack empty
  while (stack_ptr > 0) {
      uint32_t idx = stack[--stack_ptr];
      const BVHNode &node = nodes[idx];

      // 1) bounding‑box test
      float t0, t1;
      if (!intersect_bbox(*ray, node.bbmin, node.bbmax, &t0, &t1))
          continue;

      if (node.start != node.end) {
          // 2) test each primitive in the leaf
          for (uint32_t p = node.start; p < node.end; ++p) {
              if (primitive_has_intersect(&primitives[p], ray, vertices, t)) {
                  return 1;
              }
          }
      } else {
          // 3) push children (no need for order)
          if (stack_ptr + 2 <= STACK_SIZE) {
              stack[stack_ptr++] = node.l;
              stack[stack_ptr++] = node.r;
          }
      }
  }

  return 0;
}



#endif // CGL_BVH_H
