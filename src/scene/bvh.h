#ifndef CGL_BVH_H
#define CGL_BVH_H

#include "scene.h"
#include "aggregate.h"

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
  inline bool isLeaf() const { return leaf;}
  std::vector<Primitive*>::const_iterator start;
  std::vector<Primitive*>::const_iterator end;
  size_t l, r;

  BVHNode(BBox b) : bb(b) { }
};

/**
 * Bounding Volume Hierarchy for fast Ray - Primitive intersection.
 * Note that the BVHAccel is an Aggregate (A Primitive itself) that contains
 * all the primitives it was built from. Therefore once a BVHAccel Aggregate
 * is created, the original input primitives can be ignored from the scene
 * during ray intersection tests as they are contained in the aggregate.
 */
class BVHAccel : public Aggregate {
 public:

  BVHAccel () { }

  /**
   * Parameterized Constructor.
   * Create BVH from a list of primitives. Note that the BVHAccel Aggregate
   * stores pointers to the primitives and thus the primitives need be kept
   * in memory for the aggregate to function properly.
   * \param primitives primitives to build from
   * \param max_leaf_size maximum number of primitives to be stored in leaves
   */
  BVHAccel(const std::vector<Primitive*>& primitives, size_t max_leaf_size = 4);

  /**
   * Destructor.
   * The destructor only destroys the Aggregate itself, the primitives that
   * it contains are left untouched.
   */
  ~BVHAccel();

  /**
   * Get the world space bounding box of the aggregate.
   * \return world space bounding box of the aggregate
   */
  BBox get_bbox() const;

  /**
   * Ray - Aggregate intersection.
   * Check if the given ray intersects with the aggregate (any primitive in
   * the aggregate), no intersection information is stored.
   * \param r ray to test intersection with
   * \return true if the given ray intersects with the aggregate,
             false otherwise
   */
  bool has_intersection(const Ray& r) const {
    ++total_rays;
    return has_intersection(r, root);
  }

  bool has_intersection(const Ray& r, size_t node) const;

  /**
   * Ray - Aggregate intersection 2.
   * Check if the given ray intersects with the aggregate (any primitive in
   * the aggregate). If so, the input intersection data is updated to contain
   * intersection information for the point of intersection. Note that the
   * intersected primitive entry in the intersection should be updated to
   * the actual primitive in the aggregate that the ray intersected with and
   * not the aggregate itself.
   * \param r ray to test intersection with
   * \param i address to store intersection info
   * \return true if the given ray intersects with the aggregate,
             false otherwise
   */
  bool intersect(const Ray& r, Intersection* i) const {
    ++total_rays;
    return intersect(r, i, root);
  }

  bool intersect(const Ray& r, Intersection* i, size_t node) const;

  /**
   * Get BSDF of the surface material
   * Note that this does not make sense for the BVHAccel aggregate
   * because it does not have a surface material. Therefore this
   * should always return a null pointer.
   */
  BSDF* get_bsdf() const { return NULL; }

  /**
   * Get entry point (root) - used in visualizer
   */
  size_t get_root() const { return root; }

  mutable unsigned long long total_rays, total_isects;

private:
  std::vector<Primitive*> primitives;
  std::vector<BVHNode> nodes;
  size_t root;
  int construct_bvh(std::vector<Primitive*>::iterator start, std::vector<Primitive*>::iterator end, size_t max_leaf_size);
};

} // namespace SceneObjects
} // namespace CGL

#endif // CGL_BVH_H
