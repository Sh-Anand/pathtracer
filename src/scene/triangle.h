#ifndef CGL_STATICSCENE_TRIANGLE_H
#define CGL_STATICSCENE_TRIANGLE_H

#include "object.h"
#include "primitive.h"

namespace CGL { namespace SceneObjects {

/**
 * A single triangle from a mesh.
 * To save space, it holds a pointer back to the data in the original mesh
 * rather than holding the data itself. This means that its lifetime is tied
 * to that of the original mesh. The primitive may refer back to the mesh
 * object for other information such as normal, texcoord, material.
 */
class Triangle : public Primitive {
public:

  /**
   * Constructor.
   * Construct a mesh triangle with the given indicies into the triangle mesh.
   * \param mesh pointer to the mesh the triangle is in
   * \param v1 index of triangle vertex in the mesh's attribute arrays
   * \param v2 index of triangle vertex in the mesh's attribute arrays
   * \param v3 index of triangle vertex in the mesh's attribute arrays
   */
  Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3);

  Triangle() {};

  /**
   * Get the world space bounding box of the triangle.
   * \return world space bounding box of the triangle
   */
  BBox get_bbox() const;

  bool test(Ray& r, double& t, double& u, double& v) const;

  /**
   * Ray - Triangle intersection 2.
   * Check if the given ray intersects with the triangle, if so, the input
   * intersection data is updated to contain intersection information for the
   * point of intersection.
   * \param r ray to test intersection with
   * \param i address to store intersection info
   * \return true if the given ray intersects with the triangle,
             false otherwise
   */
  bool intersect(Ray& r, Intersection* i) const;

  /**
   * Get BSDF.
   * In the case of a triangle, the surface material BSDF is stored in 
   * the mesh it belongs to. 
   */
  BSDF* get_bsdf() const { return bsdf; }

  Vector3D p1, p2, p3;
  Vector3D n1, n2, n3;
  
  BSDF* bsdf;

  BBox bbox;
}; // class Triangle

struct CudaTriangle {

  CudaTriangle(Triangle* triangle, CudaBSDF bsdf) {
    p1 = triangle->p1;
    p2 = triangle->p2;
    p3 = triangle->p3;
    n1 = triangle->n1;
    n2 = triangle->n2;
    n3 = triangle->n3;
    this->bsdf = bsdf;
  }

  DEVICE bool intersect(Ray& r, CudaIntersection* i);

  Vector3D p1, p2, p3;
  Vector3D n1, n2, n3;
  
  CudaBSDF bsdf;
};

} // namespace SceneObjects
} // namespace CGL

#endif //CGL_STATICSCENE_TRIANGLE_H
