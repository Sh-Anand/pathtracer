#ifndef CGL_BVH_H
#define CGL_BVH_H

#include "geometry.h"

#include <vector>

struct BBox {

  Vector3D max;	    ///< min corner of the bounding box
  Vector3D min;	    ///< max corner of the bounding box
  Vector3D extent;  ///< extent of the bounding box (min -> max)

  BBox() {
    max = Vector3D{-INFINITY,-INFINITY,-INFINITY};
    min = Vector3D{INFINITY,INFINITY,INFINITY};
    extent = vector3d_sub(max, min);
  }

  BBox(const Vector3D p) : min(p), max(p) { extent = vector3d_sub(max, min); }

  void expand(const BBox& bbox) {
    min.x = std::min(min.x, bbox.min.x);
    min.y = std::min(min.y, bbox.min.y);
    min.z = std::min(min.z, bbox.min.z);
    max.x = std::max(max.x, bbox.max.x);
    max.y = std::max(max.y, bbox.max.y);
    max.z = std::max(max.z, bbox.max.z);
    extent = vector3d_sub(max, min);
  }

  void expand(const Vector3D p) {
    min.x = std::min(min.x, p.x);
    min.y = std::min(min.y, p.y);
    min.z = std::min(min.z, p.z);
    max.x = std::max(max.x, p.x);
    max.y = std::max(max.y, p.y);
    max.z = std::max(max.z, p.z);
    extent = vector3d_sub(max, min);
  }

  Vector3D centroid() const {
    return vector3d_scale(vector3d_add(min, max), 0.5);
  }

  float surface_area() const {
    return 2 * (extent.x * extent.z +
                extent.x * extent.y +
                extent.y * extent.z);
  }
};

typedef struct {
  Vector3D bbmin, bbmax;        ///< bounding box of the node
  uint32_t start;
  uint32_t end;
  uint32_t l, r;
} BVHNode;

// CUDA BVH
typedef struct {  
    const CudaPrimitive *primitives;
    const BVHNode* nodes;
    const Vector3D *vertices;
    const Vector3D *normals;
    const Vector2D *texcoords;
    const Vector4D *tangents;
    const uint32_t root;  
} BVHCuda;

HOST_DEVICE static inline bool intersect_bbox(
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
    return false;

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
    return false;

  // Z slab
  float tz1 = (mn.z - r.o.z) * inv_dz;
  float tz2 = (mx.z - r.o.z) * inv_dz;
  float tzmin = fminf(tz1, tz2);
  float tzmax = fmaxf(tz1, tz2);
  // final tighten
  tmin = fmaxf(tmin, tzmin);
  tmax = fminf(tmax, tzmax);
  if (tmax < r.min_t || tmin > r.max_t || tmin > tmax) 
    return false;

  // Hit!
  *t0 = tmin;
  *t1 = tmax;
  return true;
}

void create_bvh(std::vector<CudaPrimitive> &primitives_vec,
                const std::vector<Vector3D> &vertices, 
                const std::vector<Vector3D> &normals, 
                const std::vector<Vector2D> &texcoords,
                const std::vector<Vector4D> &tangents,
                bool debug,
                size_t max_leaf_size,
                BVHCuda** bvh_ret);
int construct_nodes(size_t start, size_t end, size_t max_leaf_size, std::vector<uint32_t> &primitives, std::vector<BBox> &bboxes, std::vector<BVHNode>& nodes);

DEVICE static inline bool intersect(const BVHCuda *bvh, Ray *ray, CudaIntersection *i) {
  constexpr int STACK_SIZE = 20;
  uint32_t stack[STACK_SIZE];
  int stack_ptr = 0;

  stack[stack_ptr++] = 0;
  bool hit = false;

  while (stack_ptr > 0) {
    uint32_t idx = stack[--stack_ptr];
    const BVHNode &node = bvh->nodes[idx];

    float t0, t1;
    if (!intersect_bbox(*ray, node.bbmin, node.bbmax, &t0, &t1)) continue;

    if (node.start != node.end) {
      CudaIntersection tmp; tmp.t = INFINITY;
      for (uint32_t p = node.start; p < node.end; p++) {
          if (primitive_intersect(&(bvh->primitives[p]), ray, &tmp, bvh->vertices, bvh->normals, bvh->texcoords, bvh->tangents) && tmp.t < i->t) {
            hit = true;
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

DEVICE static inline bool has_intersect(const BVHCuda *bvh, Ray *ray) {
  constexpr int STACK_SIZE = 20;
  uint32_t stack[STACK_SIZE];
  int stack_ptr = 0;

  // start with the root
  stack[stack_ptr++] = 0;
  
  float t;
  // traverse until stack empty
  while (stack_ptr > 0) {
      uint32_t idx = stack[--stack_ptr];
      const BVHNode &node = bvh->nodes[idx];

      // 1) bounding‑box test
      float t0, t1;
      if (!intersect_bbox(*ray, node.bbmin, node.bbmax, &t0, &t1))
          continue;

      if (node.start != node.end) {
          // 2) test each primitive in the leaf
          for (uint32_t p = node.start; p < node.end; ++p) {
              if (primitive_has_intersect(&bvh->primitives[p], ray, bvh->vertices, t)) {
                  return true;
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

  return false;
}



#endif // CGL_BVH_H
