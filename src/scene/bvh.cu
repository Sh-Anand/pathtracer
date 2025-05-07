#include "bvh.h"
#include <cstddef>

 

BVHCuda::BVHCuda(std::vector<CudaPrimitive> &primitives_vec,
                const std::vector<Vector3D> &vertices, 
                const std::vector<Vector3D> &normals, 
                const std::vector<Vector2D> &texcoords,
                const std::vector<Vector4D> &tangents,
                bool debug,
                size_t max_leaf_size) {

  DEBUG(debug,
  std::cout << "Building BVHCuda" << std::endl;
  std::cout << "Vertices size: " << vertices.size() << std::endl;
  std::cout << "Normals size: " << normals.size() << std::endl;
  std::cout << "Texcoords size: " << texcoords.size() << std::endl;
  std::cout << "Tangets size: " << tangents.size() << std::endl;
  std::cout << "Primitives size: " << primitives_vec.size() << std::endl;
  )

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
  size_t num_texcoords = texcoords.size();
  size_t num_tangents = tangents.size();

  CUDA_ERR(cudaMalloc(&primitives, num_primitives * sizeof(CudaPrimitive)));
  CUDA_ERR(cudaMalloc(&nodes, num_nodes * sizeof(BVHNode)));
  CUDA_ERR(cudaMalloc(&this->vertices, num_vertices * sizeof(Vector3D)));
  CUDA_ERR(cudaMalloc(&this->normals, num_normals * sizeof(Vector3D)));
  CUDA_ERR(cudaMalloc(&this->texcoords, num_texcoords * sizeof(Vector2D)));
  CUDA_ERR(cudaMalloc(&this->tangents, num_tangents * sizeof(Vector4D)));


  CUDA_ERR(cudaMemcpy(primitives, primitives_vec.data(), num_primitives * sizeof(CudaPrimitive), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(nodes, nodes_vec.data(), num_nodes * sizeof(BVHNode), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(this->vertices, vertices.data(), num_vertices * sizeof(Vector3D), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(this->normals, normals.data(), num_normals * sizeof(Vector3D), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(this->texcoords, texcoords.data(), num_texcoords * sizeof(Vector2D), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(this->tangents, tangents.data(), num_tangents * sizeof(Vector4D), cudaMemcpyHostToDevice));

  DEBUG(debug,
  std::cout<< "BVHCuda Built: " << num_nodes << " nodes" << std::endl;
  )
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

    float t0, t1;
    if (!node.bb.intersect(ray, t0, t1)) continue;

    if (node.leaf) {
      CudaIntersection tmp;
      for (uint32_t p = node.start; p < node.end; p++) {
          if (primitives[p].intersect(ray, &tmp, vertices, normals, texcoords, tangents) && tmp.t < i->t) {
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

DEVICE bool BVHCuda::has_intersect(Ray &ray, uint32_t root_idx) const {
  constexpr int STACK_SIZE = 64;
  uint32_t stack[STACK_SIZE];
  int stack_ptr = 0;

  // start with the root
  stack[stack_ptr++] = root_idx;
  
  float t;
  // traverse until stack empty
  while (stack_ptr > 0) {
      uint32_t idx = stack[--stack_ptr];
      const BVHNode &node = nodes[idx];

      // 1) bounding‑box test
      float t0, t1;
      if (!node.bb.intersect(ray, t0, t1))
          continue;

      if (node.leaf) {
          // 2) test each primitive in the leaf
          CudaIntersection tmp;
          for (uint32_t p = node.start; p < node.end; ++p) {
              if (primitives[p].has_intersect(ray, vertices, t)) {
                  return true;
              }
          }
      } else {
          // 3) push children (no need for order)
          if (stack_ptr + 2 <= STACK_SIZE) {
              stack[stack_ptr++] = node.l;
              stack[stack_ptr++] = node.r;
          }
      }
  }

  return false;
}