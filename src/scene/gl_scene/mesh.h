#ifndef CGL_GLSCENE_MESH_H
#define CGL_GLSCENE_MESH_H

#include "scene/primitive.h"
#include "scene/collada/polymesh_info.h"
#include "util/halfEdgeMesh.h"
#include "scene/gl_scene/scene.h"

namespace CGL { namespace GLScene {

/**
 * A MeshFeature is used to represent an element of the surface selected
 * by the user (e.g., edge, vertex, face).  No matter what kind of feature
 * is selected, the feature is specified relative to some polygon in the
 * mesh.  For instance, if an edge is selected, the MeshFeature will store
 * a pointer to a face containing that edge, as well as the local index of
 * the first (of two) vertices in the polygon corresponding to the edge.
 */
class MeshFeature {
 public:
  MeshFeature() : element(nullptr), w(0.0) { }

  bool isValid() const {
     return element != NULL;
  }

  void invalidate() {
    element = NULL;
  }

  Vector3D bCoords;         ///< Barycentric coordinates of selection
  HalfedgeElement* element; ///< element selected
  double w;                 ///< depth value of selection
  
};


class Mesh : public SceneObject {
 public:

  Mesh(Collada::PolymeshInfo& polyMesh, const Matrix4x4& transform);

  BBox get_bbox();

  BSDF *get_bsdf();

  /**
   * Returns w for collision, and writes barycentric coordinates to baryPtr.
   */
  double triangle_selection_test_4d(const Vector2D& p, const Vector4D& A,
                                    const Vector4D& B, const Vector4D& C,
                                    Vector3D *baryPtr);

  /**
   * Returns t/f for collision, and writes barycentric coordinates to baryPtr.
   */
  bool triangle_selection_test_2d(const Vector2D& p, const Vector2D& A,
                                  const Vector2D& B, const Vector2D& C,
                                  float *uPtr, float *vPtr);

  void get_triangles(vector<CGL::SceneObjects::CudaPrimitive>& triangles, uint32_t bsdf_idx) const;

  // halfEdge mesh
  HalfedgeMesh mesh;

  // material
  BSDF* bsdf;

  Vector3D *positions;  ///< position array
  Vector3D *normals;    ///< normal array

  vector<size_t> indices;  ///< triangles defined by indices
};

} // namespace GLScene
} // namespace CGL

#endif // CGL_GLSCENE_MESH_H
