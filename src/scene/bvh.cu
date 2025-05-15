#include "bvh.h"
#include <cstddef>

void create_bvh(std::vector<CudaPrimitive> &primitives_vec,
                const std::vector<Vector3D> &vertices, 
                const std::vector<Vector3D> &normals, 
                const std::vector<Vector2D> &texcoords,
                const std::vector<Vector4D> &tangents,
                bool debug,
                size_t max_leaf_size,
                BVHCuda **bvh_ret) {

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
  size_t root = construct_nodes(0, primitives_vec.size(), max_leaf_size, indices, bboxes, nodes_vec);

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

  CudaPrimitive* primitives_bvh;
  BVHNode* nodes_bvh;
  Vector3D* vertices_bvh;
  Vector3D* normals_bvh;
  Vector2D* texcoords_bvh;
  Vector4D* tangents_bvh;

  CUDA_ERR(cudaMalloc(&primitives_bvh, num_primitives * sizeof(CudaPrimitive)));
  CUDA_ERR(cudaMalloc(&nodes_bvh, num_nodes * sizeof(BVHNode)));
  CUDA_ERR(cudaMalloc(&vertices_bvh, num_vertices * sizeof(Vector3D)));
  CUDA_ERR(cudaMalloc(&normals_bvh, num_normals * sizeof(Vector3D)));
  CUDA_ERR(cudaMalloc(&texcoords_bvh, num_texcoords * sizeof(Vector2D)));
  CUDA_ERR(cudaMalloc(&tangents_bvh, num_tangents * sizeof(Vector4D)));


  CUDA_ERR(cudaMemcpy(primitives_bvh, primitives_vec.data(), num_primitives * sizeof(CudaPrimitive), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(nodes_bvh, nodes_vec.data(), num_nodes * sizeof(BVHNode), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(vertices_bvh, vertices.data(), num_vertices * sizeof(Vector3D), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(normals_bvh, normals.data(), num_normals * sizeof(Vector3D), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(texcoords_bvh, texcoords.data(), num_texcoords * sizeof(Vector2D), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(tangents_bvh, tangents.data(), num_tangents * sizeof(Vector4D), cudaMemcpyHostToDevice));

  DEBUG(debug,
  std::cout<< "BVHCuda Built: " << num_nodes << " nodes" << std::endl;
  )

  BVHCuda bvh_tmp = (BVHCuda) {
    .primitives = primitives_bvh,
    .nodes = nodes_bvh,
    .vertices = vertices_bvh,
    .normals = normals_bvh,
    .texcoords = texcoords_bvh,
    .tangents = tangents_bvh
  };

  BVHCuda *bvh = (BVHCuda*) malloc(sizeof(BVHCuda));
  memcpy(bvh, &bvh_tmp, sizeof(BVHCuda));

  *bvh_ret = bvh;
}