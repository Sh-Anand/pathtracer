#ifndef CGL_BVH_H
#define CGL_BVH_H

#include "primitive.h"

#include <vector>

namespace CGL { namespace SceneObjects {

/**
 * A node in the BVH accelerator aggregate.
 * The accelerator uses a "flat tree" structure where all the primitives are
 * stored in one vector. A node in the data structure stores only the starting
 * index and the number of primitives in the node and uses this information to
 * index into the primitive vector for actual data. In this implementation all
 * primitives (index + range) are stored on leaf nodes. A leaf node has no child
 * node and its range should be no greater than the maximum leaf size used when
 * constructing the BVH.
 */
struct BVHNode {
  BBox bb;        ///< bounding box of the node
  bool leaf;
  uint32_t start;
  uint32_t end;
  uint32_t l, r;

  BVHNode(BBox b) : bb(b) { }
};

// CUDA BVH
class BVHCuda {
  public:
    BVHCuda(std::vector<CudaPrimitive> &primitives_vec, size_t max_leaf_size = 4);
  
    ~BVHCuda();
    
    DEVICE bool intersect(Ray& r, CudaIntersection* i) const {
      return intersect(r, i, root);
    }

    DEVICE bool intersect(Ray& r, CudaIntersection* i, uint32_t node) const;
    
    CudaPrimitive *primitives;
    BVHNode* nodes;
    uint32_t root;

    int construct_bvh(size_t start, size_t end, size_t max_leaf_size, std::vector<uint32_t> &primitives, std::vector<BBox> &bboxes, std::vector<BVHNode>& nodes);
  
};

} // namespace SceneObjects
} // namespace CGL

#endif // CGL_BVH_H
