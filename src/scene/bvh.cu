#include "bvh.h"

namespace CGL { namespace SceneObjects {

BVHCuda::BVHCuda(std::vector<CudaPrimitive> &primitives_vec, size_t max_leaf_size) {

  std::vector<BVHNode> nodes_vec;
  root = construct_bvh(0, primitives_vec.size(), max_leaf_size, primitives_vec, nodes_vec);
  num_primitives = primitives_vec.size();
  num_nodes = nodes_vec.size();

  CUDA_ERR(cudaMalloc(&primitives, num_primitives * sizeof(CudaPrimitive)));
  CUDA_ERR(cudaMalloc(&nodes, num_nodes * sizeof(BVHNode)));

  CUDA_ERR(cudaMemcpy(primitives, primitives_vec.data(), num_primitives * sizeof(CudaPrimitive), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(nodes, nodes_vec.data(), num_nodes * sizeof(BVHNode), cudaMemcpyHostToDevice));

  std::cout<< "BVHCuda: " << num_primitives << " primitives, " << num_nodes << " nodes, " << std::endl;
  std::cout<< "root: " << root << std::endl;
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
          if (primitives[p].intersect(ray, &tmp) && tmp.t < i->t) {
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