#include "bvh.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHCuda::~BVHCuda() {
  free(primitives);
  free(nodes);
}

int BVHCuda::construct_bvh(size_t start, size_t end,
                           size_t max_leaf_size,
                           std::vector<CudaPrimitive> &prims,
                           std::vector<BVHNode> &nodes) {
  // 1) Compute object and centroid bounds
  BBox node_bbox, cent_bbox;
  for (size_t i = start; i < end; ++i) {
    BBox b = prims[i].get_bbox();
    node_bbox.expand(b);
    cent_bbox.expand(b.centroid());
  }

  int idx = nodes.size();
  nodes.emplace_back(node_bbox);

  size_t n = end - start;
  if (n <= max_leaf_size) {
    // leaf
    nodes[idx].leaf  = true;
    nodes[idx].start = start;
    nodes[idx].end   = end;
    nodes[idx].l = nodes[idx].r = 0;
    return idx;
  }

  // 2) Choose split axis = longest centroid axis
  Vector3D e = cent_bbox.extent;
  int axis = (e.x > e.y && e.x > e.z) ? 0
           : (e.y > e.z)               ? 1
                                        : 2;

  // 3) Bucket SAH
  const int B = 16;
  struct Bucket { int count = 0; BBox bbox; } buckets[B];
  double minA = cent_bbox.min[axis], maxA = cent_bbox.max[axis];
  double invBin = (maxA>minA) ? B/(maxA-minA) : 0;

  // fill buckets
  for (size_t i = start; i < end; ++i) {
    double c = prims[i].get_bbox().centroid()[axis];
    int b = std::min(int((c - minA)*invBin), B-1);
    buckets[b].count++;
    buckets[b].bbox.expand(prims[i].get_bbox());
  }

  // prefix sums for cost
  double leftCount[B-1]={}, rightCount[B-1]={};
  BBox   leftBBox[B-1], rightBBox[B-1];
  // left side
  int cnt=0;
  for (int i=0; i<B-1; ++i) {
    cnt += buckets[i].count;
    leftCount[i] = cnt;
    if (i==0) leftBBox[i] = buckets[i].bbox;
    else      { leftBBox[i] = leftBBox[i-1]; leftBBox[i].expand(buckets[i].bbox); }
  }
  // right side
  cnt=0;
  for (int i=B-1; i>0; --i) {
    cnt += buckets[i].count;
    rightCount[i-1] = cnt;
    if (i==B-1) rightBBox[i-1] = buckets[i].bbox;
    else        { rightBBox[i-1] = rightBBox[i]; rightBBox[i-1].expand(buckets[i].bbox); }
  }

  // evaluate SAH cost for each split
  double invSA = 1.0/node_bbox.surface_area();
  double bestCost = 1e300;
  int    bestB   = -1;
  for (int i=0; i<B-1; ++i) {
    double cost =  /* travCost=1 */   1
                + /* isectCost=1 */ ( leftCount[i]*leftBBox[i].surface_area()
                                     + rightCount[i]*rightBBox[i].surface_area())
                  * invSA;
    if (cost < bestCost) {
      bestCost = cost;
      bestB    = i;
    }
  }

  // 4) Partition primitives at that bucket boundary
  double splitPos = minA + (bestB+1)/double(B)*(maxA-minA);
  auto midIt = std::partition(prims.begin()+start,
                              prims.begin()+end,
                              [&](const CudaPrimitive &p){
                                return p.get_bbox().centroid()[axis] < splitPos;
                              });
  size_t mid = midIt - prims.begin();
  // fallback if degenerate
  if (mid==start || mid==end)
    mid = start + n/2;

  // 5) recurse
  int left  = construct_bvh(start, mid, max_leaf_size, prims, nodes);
  int right = construct_bvh(mid,   end, max_leaf_size, prims, nodes);

  nodes[idx].leaf = false;
  nodes[idx].l    = left;
  nodes[idx].r    = right;
  return idx;
}


} // namespace SceneObjects
} // namespace CGL
