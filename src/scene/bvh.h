#ifndef CGL_BVH_H
#define CGL_BVH_H

#include "scene.h"
#include "aggregate.h"

#include "triangle.h"
#include "sphere.h"

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
  HOST_DEVICE inline bool isLeaf() const { return leaf;}
  uint16_t start;
  uint16_t end;
  uint16_t l, r;

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
  bool intersect(Ray& r, Intersection* i) const {
    ++total_rays;
    return intersect(r, i, root);
  }

  bool intersect(Ray& r, Intersection* i, size_t node) const;

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

  bool isLeaf(size_t idx) const {
    return nodes[idx].isLeaf();
  }

  /**
   * Draw the BVH with OpenGL - used in visualizer
   */
  void draw(const Color& c, float alpha) const { }
  void draw(size_t idx, const Color& c, float alpha) const;

  /**
   * Draw the BVH outline with OpenGL - used in visualizer
   */
  void drawOutline(const Color& c, float alpha) const { }
  void drawOutline(size_t idx, const Color& c, float alpha) const;

  mutable unsigned long long total_rays, total_isects;

  std::vector<Primitive*> primitives;
  std::vector<BVHNode> nodes;
  size_t root;
  int construct_bvh(size_t start, size_t end, size_t max_leaf_size);
};

#ifdef __CUDACC__
#include "curand_kernel.h"
#else
struct curandState;
#endif

// CUDA BVH
class BVHCuda {
  public:
    BVHCuda(BVHAccel* bvh);
  
    ~BVHCuda();
    
    DEVICE bool intersect(Ray& r, CudaIntersection* i) const {
      return intersect(r, i, root);
    }

    DEVICE bool intersect(Ray& r, CudaIntersection* i, uint16_t node) const;
  
    CudaPrimitive* primitives;
    uint16_t num_primitives;

    CudaTriangle* triangles;
    uint16_t num_triangles;
    CudaSphere* spheres;
    uint16_t num_spheres;

    BVHNode* nodes;
    uint16_t num_nodes;
    uint16_t root;

    //BSDFs
    CudaDiffuseBSDF* diffuse_bsdfs;
    uint16_t num_diffuse_bsdfs;
    CudaEmissionBSDF* emission_bsdfs;
    uint16_t num_emission_bsdfs;

    DEVICE Vector3D f (CudaBSDF bsdf, const Vector3D wo, const Vector3D wi) const;

    DEVICE Vector3D get_emission (CudaBSDF bsdf) const {
      if (bsdf.type == CudaBSDFType_Emission) {
        return emission_bsdfs[bsdf.idx].get_emission();
      } else if (bsdf.type == CudaBSDFType_Diffuse) {
        return diffuse_bsdfs[bsdf.idx].get_emission();
      } else {
        return Vector3D(0, 0, 0);
      }
    }

};

} // namespace SceneObjects
} // namespace CGL

#endif // CGL_BVH_H
