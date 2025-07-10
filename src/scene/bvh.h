#ifndef CGL_BVH_H
#define CGL_BVH_H

#include "geometry.h"


#include <cfloat>
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

DEVICE static inline bool intersect(Ray *ray, const CudaPrimitive *primitives, const Vector3D *vertices, const Vector3D *normals,
                                     const Vector2D *texcoords, const Vector4D *tangents, const BVHNode *nodes, CudaIntersection *i) {
  constexpr int STACK_SIZE = 20;
  uint32_t stack[STACK_SIZE];
  int stack_ptr = 0;

  stack[stack_ptr++] = 0;
  bool hit = false;

  while (stack_ptr > 0) {
    uint32_t idx = stack[--stack_ptr];
    const BVHNode &node = nodes[idx];

    float t0, t1;
    if (!intersect_bbox(*ray, node.bbmin, node.bbmax, &t0, &t1)) continue;

    if (node.start != node.end) {
      CudaIntersection tmp; tmp.t = INFINITY;
      for (uint32_t p = node.start; p < node.end; p++) {
          if (primitive_intersect(&(primitives[p]), ray, &tmp, vertices, normals, texcoords, tangents) && tmp.t < i->t) {
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

DEVICE static inline bool has_intersect(const CudaPrimitive *primitives, const Vector3D *vertices, const BVHNode* nodes, Ray *ray) {
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

static int construct_nodes(size_t start, size_t end,
                           size_t max_leaf_size,
                           std::vector<uint32_t> &prims,
                           std::vector<BBox> &bboxes,
                           std::vector<BVHNode> &nodes) {
  // 1) Compute object bounds and centroid bounds
  BBox node_bbox, cent_bbox;
  for (size_t i = start; i < end; ++i) {
    BBox const &b = bboxes[prims[i]];
    node_bbox.expand(b);
    cent_bbox.expand(b.centroid());
  }

  // Create this node
  int idx = nodes.size();
  BVHNode node;
  node.bbmin = node_bbox.min;
  node.bbmax = node_bbox.max;
  node.start = node.end = 0;
  nodes.push_back(node);

  size_t n = end - start;
  if (n <= max_leaf_size) {
    // Make a leaf
    nodes[idx].start = start;
    nodes[idx].end   = end;
    nodes[idx].l = nodes[idx].r = 0;
    return idx;
  }

  // 2) Choose split axis = longest axis of centroid bounds
  Vector3D e = cent_bbox.extent;
  int axis = (e.x > e.y && e.x > e.z) ? 0
           : (e.y > e.z)               ? 1
                                        : 2;

  // 3) Gather centroids and sort exactly by centroid
  std::vector<std::pair<float,uint32_t>> centroids;
  centroids.reserve(n);
  for (size_t i = start; i < end; ++i) {
    auto p = prims[i];
    if (axis == 0)
      centroids.emplace_back(bboxes[p].centroid().x, p);
    else if (axis == 1)
      centroids.emplace_back(bboxes[p].centroid().y, p);
    else
      centroids.emplace_back(bboxes[p].centroid().z, p);
  }
  std::sort(centroids.begin(), centroids.end(),
            [](auto &a, auto &b){ return a.first < b.first; });

  // 4) Compute prefix/suffix bounds for all split positions
  std::vector<BBox> leftBBox(n), rightBBox(n);
  leftBBox[0] = bboxes[centroids[0].second];
  for (size_t i = 1; i < n; ++i) {
    leftBBox[i] = leftBBox[i-1];
    leftBBox[i].expand(bboxes[centroids[i].second]);
  }
  rightBBox[n-1] = bboxes[centroids[n-1].second];
  for (int i = int(n)-2; i >= 0; --i) {
    rightBBox[i] = rightBBox[i+1];
    rightBBox[i].expand(bboxes[centroids[i].second]);
  }

  // 5) Evaluate SAH cost at each possible split
  float invSA = 1.0f / node_bbox.surface_area();
  float bestCost = FLT_MAX;
  size_t bestSplit = 0;
  // we can only split between [1..n-1]
  for (size_t i = 1; i < n; ++i) {
    float leftArea  = leftBBox[i-1].surface_area();
    float rightArea = rightBBox[i].surface_area();
    float cost = 1.0f                                     // traversal cost
               + (i * leftArea + (n - i) * rightArea)    // intersection cost
                 * invSA;
    if (cost < bestCost) {
      bestCost   = cost;
      bestSplit  = i;
    }
  }

  // 6) Partition primitives according to the exact sort
  size_t mid = start + bestSplit;
  for (size_t i = 0; i < bestSplit; ++i)
    prims[start + i] = centroids[i].second;
  for (size_t i = bestSplit; i < n; ++i)
    prims[start + i] = centroids[i].second;

  // Fallback if degenerate
  if (bestSplit == 0 || bestSplit == n) {
    mid = start + n/2;
  }

  // 7) Recurse on each side
  int leftIdx  = construct_nodes(start, mid, max_leaf_size, prims, bboxes, nodes);
  int rightIdx = construct_nodes(mid,   end, max_leaf_size, prims, bboxes, nodes);

  nodes[idx].l = leftIdx;
  nodes[idx].r = rightIdx;
  return idx;
}

static void create_bvh(std::vector<CudaPrimitive> &primitives_vec,
                const std::vector<Vector3D> &vertices, 
                const std::vector<Vector3D> &normals, 
                const std::vector<Vector2D> &texcoords,
                const std::vector<Vector4D> &tangents,
                std::vector<BVHNode> &nodes_vec,
                bool debug,
                size_t max_leaf_size) {

  std::vector<BBox> bboxes;
  bboxes.reserve(primitives_vec.size());
  std::vector<uint32_t> indices(primitives_vec.size());
  for (uint32_t i = 0; i < primitives_vec.size(); i++) {
    indices[i] = i;
    CudaPrimitive &primitive = primitives_vec[i];
    BBox bbox(vertices[primitive.i0]);
    bbox.expand(vertices[primitive.i1]);
    bbox.expand(vertices[primitive.i2]);
    bboxes.push_back(bbox);
  }

  size_t root = construct_nodes(0, primitives_vec.size(), max_leaf_size, indices, bboxes, nodes_vec);

  //reorder primitives according to reordered indices inplace
  std::vector<CudaPrimitive> primitives_vec_reordered(primitives_vec.size());
  for (uint32_t i = 0; i < indices.size(); i++) {
    primitives_vec_reordered[i] = primitives_vec[indices[i]];
  }
  primitives_vec = std::move(primitives_vec_reordered);

  size_t num_nodes = nodes_vec.size();

}



#endif // CGL_BVH_H
