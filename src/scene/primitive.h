#ifndef CGL_STATICSCENE_PRIMITIVE_H
#define CGL_STATICSCENE_PRIMITIVE_H

#include "pathtracer/intersection.h"
#include "scene/bbox.h"

namespace CGL { namespace SceneObjects {

struct CudaSphere {
    CudaSphere(const Vector3D& o, float r) : o(o), r(r), r2(r * r) {}
    DEVICE Vector3D normal(Vector3D p) const {
      return (p - o).unit();
    }
    DEVICE bool intersect(Ray& r, CudaIntersection* i);
    BBox get_bbox() const {
      return BBox(o - Vector3D(r,r,r), o + Vector3D(r,r,r));
    }
    
    Vector3D o; ///< origin of the sphere
    float r;   ///< radius
    float r2;  ///< radius squared
};

struct CudaTriangle {
  DEVICE bool intersect(Ray& r, CudaIntersection* i);
  BBox get_bbox() const { 
    BBox bbox(p1);
    bbox.expand(p2);
    bbox.expand(p3);
    return bbox;
  }

  Vector3D p1, p2, p3;
  Vector3D n1, n2, n3;  
};

enum CudaPrimitiveType {
  TRIANGLE = 0,
  SPHERE = 1,
};

union PrimitiveData {
  PrimitiveData() {}
  CudaTriangle triangle;
  CudaSphere sphere;
};

struct CudaPrimitive {
  PrimitiveData primitive;
  CudaPrimitiveType type;
  uint16_t bsdf_idx;

  CudaPrimitive() : type(TRIANGLE), bsdf_idx(0) {}
  CudaPrimitive(const CudaPrimitive& p) : type(p.type), bsdf_idx(p.bsdf_idx) {
    switch (type) {
      case TRIANGLE:
        primitive.triangle = p.primitive.triangle;
        break;
      case SPHERE:
        primitive.sphere = p.primitive.sphere;
        break;
      default:
        break;
    }
  }

  DEVICE bool intersect(Ray& r, CudaIntersection* isect) {
    isect->bsdf_idx = bsdf_idx;
    switch (type) {
      case TRIANGLE:
        return primitive.triangle.intersect(r, isect);
      case SPHERE:
        return primitive.sphere.intersect(r, isect);
      default:
        return false;
    }
  }

  BBox get_bbox() const {
    switch (type) {
      case TRIANGLE:
        return primitive.triangle.get_bbox();
      case SPHERE:
        return primitive.sphere.get_bbox();
      default:
        return BBox();
    }
  }
};


} // namespace SceneObjects
} // namespace CGL

#endif //CGL_STATICSCENE_PRIMITIVE_H
