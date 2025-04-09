#ifndef CGL_STATICSCENE_PRIMITIVE_H
#define CGL_STATICSCENE_PRIMITIVE_H

#include "pathtracer/intersection.h"
#include "scene/bbox.h"

namespace CGL { namespace SceneObjects {

/**
 * The abstract base class primitive is the bridge between geometry processing
 * and the shading subsystem. As such, its interface contains methods related
 * to both.
 */
class Primitive {
 public:

  /**
   * Get the world space bounding box of the primitive.
   * \return world space bounding box of the primitive
   */
  virtual BBox get_bbox() const = 0;

  /**
   * Ray - Primitive intersection 2.
   * Check if the given ray intersects with the primitive, if so, the input
   * intersection data is updated to contain intersection information for the
   * point of intersection.
   * \param r ray to test intersection with
   * \param i address to store intersection info
   * \return true if the given ray intersects with the primitive,
             false otherwise
   */
  virtual bool intersect(Ray& r, Intersection* i) const = 0;

  /**
   * Get BSDF.
   * Return the BSDF of the surface material of the primitive.
   * Note that the BSDFs are not stored in each primitive but in the
   * SceneObjects the primitive belongs to.
   */
  virtual BSDF* get_bsdf() const = 0;
};

enum CudaPrimitiveType {
  TRIANGLE = 0,
  SPHERE = 1,
};
struct CudaPrimitive {
  CudaPrimitive(uint16_t idx, CudaPrimitiveType type) : idx(idx), type(type) {}
  uint16_t idx;
  CudaPrimitiveType type;
};


} // namespace SceneObjects
} // namespace CGL

#endif //CGL_STATICSCENE_PRIMITIVE_H
