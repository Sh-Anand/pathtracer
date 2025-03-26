#ifndef CGL_GLSCENE_MESH_H
#define CGL_GLSCENE_MESH_H

#include "scene.h"

#include "scene/collada/polymesh_info.h"
#include "util/halfEdgeMesh.h"

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

  ~Mesh();

  BBox get_bbox();

  BSDF *get_bsdf();
  SceneObjects::SceneObject *get_static_object();

 private:

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

  /**
   * Given that hoveredFeature is pointing to a face, determines which
	 * subfeature (vertex, edge, halfedge, face) it's pointing to within
	 * that face.
   */
  void choose_hovered_subfeature();

  // halfEdge mesh
  HalfedgeMesh mesh;

  // material
  BSDF* bsdf;
};

} // namespace GLScene
} // namespace CGL

#endif // CGL_GLSCENE_MESH_H
