#include "bvh.h"
#include <cstddef>

namespace CGL { namespace SceneObjects {

BVHCuda::BVHCuda(std::vector<CudaPrimitive> &primitives_vec, std::vector<Vector3D> &vertices, std::vector<Vector3D> &normals, size_t max_leaf_size) {

  std::vector<BBox> bboxes;
  bboxes.reserve(primitives_vec.size());
  std::vector<uint32_t> indices(primitives_vec.size());
  for (uint32_t i = 0; i < primitives_vec.size(); i++) {
    indices[i] = i;
    CudaPrimitive &primitive = primitives_vec[i];
    BBox bbox(vertices[primitive.i_p1]);
    bbox.expand(vertices[primitive.i_p2]);
    bbox.expand(vertices[primitive.i_p3]);
    bboxes.push_back(bbox);
  }

  std::vector<BVHNode> nodes_vec;
  root = construct_bvh(0, primitives_vec.size(), max_leaf_size, indices, bboxes, nodes_vec);

  //reorder primitives according to reordered indices inplace
  std::vector<CudaPrimitive> primitives_vec_reordered(primitives_vec.size());
  for (uint32_t i = 0; i < indices.size(); i++) {
    primitives_vec_reordered[i] = primitives_vec[indices[i]];
  }
  primitives_vec = std::move(primitives_vec_reordered);

  
  size_t num_primitives = primitives_vec.size();
  size_t num_nodes = nodes_vec.size();
  size_t num_vertices = vertices.size();
  size_t num_normals = normals.size();

  CUDA_ERR(cudaMalloc(&primitives, num_primitives * sizeof(CudaPrimitive)));
  CUDA_ERR(cudaMalloc(&nodes, num_nodes * sizeof(BVHNode)));
  CUDA_ERR(cudaMalloc(&this->vertices, num_vertices * sizeof(Vector3D)));
  CUDA_ERR(cudaMalloc(&this->normals, num_normals * sizeof(Vector3D)));


  CUDA_ERR(cudaMemcpy(primitives, primitives_vec.data(), num_primitives * sizeof(CudaPrimitive), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(nodes, nodes_vec.data(), num_nodes * sizeof(BVHNode), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(this->vertices, vertices.data(), num_vertices * sizeof(Vector3D), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(this->normals, normals.data(), num_normals * sizeof(Vector3D), cudaMemcpyHostToDevice));

  std::cout<< "BVHCuda: " << num_primitives << " primitives, " << num_nodes << " nodes, " << std::endl;
}

DEVICE bool BVHCuda::intersect(Ray &ray, CudaIntersection *i, uint32_t root_idx) const {
  constexpr int STACK_SIZE = 64;
  uint32_t stack[STACK_SIZE];
  int stack_ptr = 0;

  stack[stack_ptr++] = root_idx;
  bool hit = false;

  while (stack_ptr > 0) {
    uint32_t idx = stack[--stack_ptr];
    const BVHNode &node = nodes[idx];

    double t0, t1;
    if (!node.bb.intersect(ray, t0, t1)) continue;

    if (node.leaf) {
      CudaIntersection tmp;
      for (uint32_t p = node.start; p < node.end; p++) {
          if (primitives[p].intersect(ray, &tmp, vertices, normals) && tmp.t < i->t) {
            hit = true;
            *i = tmp;
          }
      }
    } else {
      // Push children in reverse order so left is processed first
      if (stack_ptr + 2 > STACK_SIZE) break; // Prevent stack overflow
      stack[stack_ptr++] = node.r;
      stack[stack_ptr++] = node.l;
    }
  }

  return hit;
}

}
}