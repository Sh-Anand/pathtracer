#include "bvh.h"

#include <iostream>
#include <cfloat>
#include <stack>

using namespace std;

BVHCuda::~BVHCuda() {
  free(primitives);
  free(nodes);
}

int BVHCuda::construct_bvh(size_t start, size_t end,
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
  nodes.emplace_back(node_bbox.min, node_bbox.max);

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
    float c = bboxes[p].centroid()[axis];
    centroids.emplace_back(c, p);
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
  int leftIdx  = construct_bvh(start, mid, max_leaf_size, prims, bboxes, nodes);
  int rightIdx = construct_bvh(mid,   end, max_leaf_size, prims, bboxes, nodes);

  nodes[idx].l = leftIdx;
  nodes[idx].r = rightIdx;
  return idx;
}





