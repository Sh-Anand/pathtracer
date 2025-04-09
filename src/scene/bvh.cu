#include <curand_kernel.h>
#include "bvh.h"

namespace CGL { namespace SceneObjects {


CudaBSDF to_cuda_bsdf(const BSDF *bsdf, size_t idx) {
  CudaBSDF cuda_bsdf;
  // dynamic cst BSDF and figure out type:
  if (const DiffuseBSDF *diffuse_bsdf = dynamic_cast<const DiffuseBSDF *>(bsdf)) {
    cuda_bsdf.type = CudaBSDFType_Diffuse;
  } else if (const MicrofacetBSDF *microfacet_bsdf = dynamic_cast<const MicrofacetBSDF *>(bsdf)) {
    cuda_bsdf.type = CudaBSDFType_Microfacet;
  } else if (const MirrorBSDF *mirror_bsdf = dynamic_cast<const MirrorBSDF *>(bsdf)) {
    cuda_bsdf.type = CudaBSDFType_Mirror;
  } else if (const RefractionBSDF *refraction_bsdf = dynamic_cast<const RefractionBSDF *>(bsdf)) {
    cuda_bsdf.type = CudaBSDFType_Refraction;
  } else if (const GlassBSDF *glass_bsdf = dynamic_cast<const GlassBSDF *>(bsdf)) {
    cuda_bsdf.type = CudaBSDFType_Glass;
  } else if (const EmissionBSDF *emission_bsdf = dynamic_cast<const EmissionBSDF *>(bsdf)) {
    cuda_bsdf.type = CudaBSDFType_Emission;
  } else {
    std::cerr << "Unknown BSDF type" << std::endl;
    exit(1);
  }
  cuda_bsdf.idx = idx;
  return cuda_bsdf;
}

BVHCuda::BVHCuda(BVHAccel *bvh) {
  std::vector<CudaPrimitive> primitives_vec;
  std::vector<CudaSphere> spheres_vec;
  std::vector<CudaTriangle> triangles_vec;
  std::vector<CudaDiffuseBSDF> diffuse_bsdfs_vec;
  std::vector<CudaEmissionBSDF> emission_bsdfs_vec;

  num_triangles = 0;
  num_spheres = 0;
  num_diffuse_bsdfs = 0;
  num_emission_bsdfs = 0;

  for (size_t i = 0; i < bvh->primitives.size(); i++) {
    Primitive *primitive = bvh->primitives[i];
    size_t bsdf_idx;
    if (DiffuseBSDF *diffuse_bsdf = dynamic_cast<DiffuseBSDF *>(primitive->get_bsdf())) {
      diffuse_bsdfs_vec.push_back(CudaDiffuseBSDF(diffuse_bsdf));
      bsdf_idx = num_diffuse_bsdfs++;
    } else if (EmissionBSDF *emission_bsdf = dynamic_cast<EmissionBSDF *>(primitive->get_bsdf())) {
      emission_bsdfs_vec.push_back(CudaEmissionBSDF(emission_bsdf));
      bsdf_idx = num_emission_bsdfs++;
    } else {
      std::cerr << "Unknown BSDF type" << std::endl;
      exit(1);
    }

    if (Triangle *t = dynamic_cast<Triangle *>(primitive)) {
      triangles_vec.push_back(CudaTriangle(t, to_cuda_bsdf(primitive->get_bsdf(), bsdf_idx)));
      primitives_vec.push_back(CudaPrimitive(num_triangles, TRIANGLE));
      num_triangles++;
    } else if (Sphere *s = dynamic_cast<Sphere *>(primitive)) {
      spheres_vec.push_back(CudaSphere(s, to_cuda_bsdf(primitive->get_bsdf(), bsdf_idx)));
      primitives_vec.push_back(CudaPrimitive(num_spheres, SPHERE));
      num_spheres++;
    } else {
      std::cerr << "Unknown primitive type" << std::endl;
      exit(1);
    }
  }

  num_primitives = primitives_vec.size();
  num_nodes = bvh->nodes.size();
  
  CUDA_ERR(cudaMalloc(&primitives, num_primitives * sizeof(CudaPrimitive)));
  CUDA_ERR(cudaMalloc(&spheres, num_spheres * sizeof(CudaSphere)));
  CUDA_ERR(cudaMalloc(&triangles, num_triangles * sizeof(CudaTriangle)));
  CUDA_ERR(cudaMalloc(&nodes, num_nodes * sizeof(BVHNode)));
  CUDA_ERR(cudaMalloc(&diffuse_bsdfs, num_diffuse_bsdfs * sizeof(CudaDiffuseBSDF)));
  CUDA_ERR(cudaMalloc(&emission_bsdfs, num_emission_bsdfs * sizeof(CudaEmissionBSDF)));

  CUDA_ERR(cudaMemcpy(primitives, primitives_vec.data(), num_primitives * sizeof(CudaPrimitive), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(spheres, spheres_vec.data(), num_spheres * sizeof(CudaSphere), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(triangles, triangles_vec.data(), num_triangles * sizeof(CudaTriangle), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(nodes, bvh->nodes.data(), bvh->nodes.size() * sizeof(BVHNode), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(diffuse_bsdfs, diffuse_bsdfs_vec.data(), num_diffuse_bsdfs * sizeof(CudaDiffuseBSDF), cudaMemcpyHostToDevice));
  CUDA_ERR(cudaMemcpy(emission_bsdfs, emission_bsdfs_vec.data(), num_emission_bsdfs * sizeof(CudaEmissionBSDF), cudaMemcpyHostToDevice));

  root = bvh->root;

  std::cout<< "BVHCuda: " << num_primitives << " primitives, " << num_nodes << " nodes, " << std::endl;
  std::cout<< "BVHCuda: " << num_diffuse_bsdfs << " diffuse BSDFs, " << num_emission_bsdfs << " emission BSDFs" << std::endl;
  std::cout<< "BVHCuda: " << num_triangles << " triangles, " << num_spheres << " spheres" << std::endl;
  std::cout<< "root: " << root << std::endl;
}

DEVICE bool BVHCuda::intersect(Ray &ray, CudaIntersection *i, uint16_t root_idx) const {
  constexpr int STACK_SIZE = 64;
  uint16_t stack[STACK_SIZE];
  int stack_ptr = 0;

  stack[stack_ptr++] = root_idx;
  bool hit = false;

  while (stack_ptr > 0) {
    uint16_t idx = stack[--stack_ptr];
    const BVHNode &node = nodes[idx];

    double t0, t1;
    if (!node.bb.intersect(ray, t0, t1)) continue;

    if (node.isLeaf()) {
      CudaIntersection tmp;
      for (uint16_t p = node.start; p < node.end; p++) {
        switch (primitives[p].type) {
          case CudaPrimitiveType::TRIANGLE:
            if (triangles[primitives[p].idx].intersect(ray, &tmp) && tmp.t < i->t) {
              hit = true;
              *i = tmp;
            }
            break;
          case CudaPrimitiveType::SPHERE:
            if (spheres[primitives[p].idx].intersect(ray, &tmp) && tmp.t < i->t) {
              hit = true;
              *i = tmp;
            }
            break;
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

DEVICE Vector3D BVHCuda::f (CudaBSDF bsdf, const Vector3D wo, const Vector3D wi) const {
switch (bsdf.type) {
    case CudaBSDFType_Diffuse:
        return diffuse_bsdfs[bsdf.idx].f(wo, wi);
    case CudaBSDFType_Emission:
        return emission_bsdfs[bsdf.idx].f(wo, wi);
    default:
        return Vector3D(0, 0, 0);
}
}

}
}