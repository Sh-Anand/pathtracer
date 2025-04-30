#include "mesh.h"

#include <cassert>
#include <sstream>

#include "scene/light.h"

#include "pathtracer/bsdf.h"

#include <unordered_map>

using std::ostringstream;
using std::unordered_map;

using CGL::SceneObjects::CudaPrimitive;

namespace CGL { namespace GLScene {

// For use in choose_hovered_subfeature.
static const double low_threshold  = .1;
static const double mid_threshold  = .2;
static const double high_threshold = 1.0 - low_threshold;

Mesh::Mesh(Collada::PolymeshInfo& polyMesh, const Matrix4x4& transform) {

  // Build halfedge mesh from polygon soup
  vector< vector<size_t> > polygons;
  for (const Collada::Polygon& p : polyMesh.polygons) {
    polygons.push_back(p.vertex_indices);
  }
  vector<Vector3D> vertices = polyMesh.vertices; // DELIBERATE COPY.
  for (int i = 0; i < vertices.size(); i++) {
    vertices[i] = (transform * Vector4D(vertices[i], 1)).projectTo3D();
  }

  // Read texture coordinates.
  vector<Vector2D> texcoords = polyMesh.texcoords; // DELIBERATE COPY.

  mesh.build(polygons, vertices, texcoords);
  if (polyMesh.material) {
    bsdf = polyMesh.material->bsdf;
  } else {
    bsdf = new DiffuseBSDF(Vector3D(0.5f,0.5f,0.5f));
  }

  unordered_map<const Vertex *, int> vertexLabels;
  vector<const Vertex *> verts;

  size_t vertexI = 0;
  for (VertexCIter it = mesh.verticesBegin(); it != mesh.verticesEnd(); it++) {
    const Vertex *v = &*it;
    verts.push_back(v);
    vertexLabels[v] = vertexI;
    vertexI++;
  }

  positions = new Vector3D[vertexI];
  normals   = new Vector3D[vertexI];
  for (int i = 0; i < vertexI; i++) {
    positions[i] = verts[i]->position;
    normals[i]   = verts[i]->normal;
  }

  for (FaceCIter f = mesh.facesBegin(); f != mesh.facesEnd(); f++) {
    HalfedgeCIter h = f->halfedge();
    indices.push_back(vertexLabels[&*h->vertex()]);
    indices.push_back(vertexLabels[&*h->next()->vertex()]);
    indices.push_back(vertexLabels[&*h->next()->next()->vertex()]);
  }

  this->bsdf = bsdf;
}

BBox Mesh::get_bbox() {
  BBox bbox;
  for (VertexIter it = mesh.verticesBegin(); it != mesh.verticesEnd(); it++) {
    bbox.expand(it->position);
  }
  return bbox;
}

void Mesh::get_triangles(vector<CudaPrimitive>& primitives, uint32_t bsdf_idx) const {
  size_t num_triangles = indices.size() / 3;
  for (size_t i = 0; i < num_triangles; ++i) {

    CudaPrimitive primitive {
      positions[indices[i * 3 + 0]],
      positions[indices[i * 3 + 1]],
      positions[indices[i * 3 + 2]],
      normals[indices[i * 3 + 0]],
      normals[indices[i * 3 + 1]],
      normals[indices[i * 3 + 2]], bsdf_idx};
    primitives.push_back(primitive);
  }
}

double Mesh::triangle_selection_test_4d(const Vector2D& p, const Vector4D& A,
                                        const Vector4D& B, const Vector4D& C,
                                        Vector3D *baryPtr) {
  Vector2D a2D(A.x, A.y);
  Vector2D b2D(B.x, B.y);
  Vector2D c2D(C.x, C.y);
  float bU, bV;

  if (triangle_selection_test_2d(p, a2D, b2D, c2D, &bU, &bV)) {
    baryPtr->x = 1.0 - bU - bV;
    baryPtr->y = bV;
    baryPtr->z = bU;
    return A.w + (C.w - A.w) * bU + (B.w - A.w) * bV;
  }
  return -1.0;
}


bool Mesh::triangle_selection_test_2d(const Vector2D& p, const Vector2D& A,
                                      const Vector2D& B, const Vector2D& C,
                                      float *uPtr, float *vPtr) {
  // Compute vectors
  Vector2D v0 = C - A;
  Vector2D v1 = B - A;
  Vector2D v2 = p - A;

  // Compute dot products
  double dot00 = dot(v0, v0);
  double dot01 = dot(v0, v1);
  double dot02 = dot(v0, v2);
  double dot11 = dot(v1, v1);
  double dot12 = dot(v1, v2);

  // Compute barycentric coordinates
  double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
  double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

  // Check if point is in triangle.
  bool output = (u >= 0) && (v >= 0) && (u + v < 1);

  if (output) {
    *uPtr = u;
    *vPtr = v;
  }

  return output;
}

BSDF* Mesh::get_bsdf() {
  return bsdf;
}


} // namespace GLScene
} // namespace CGL
