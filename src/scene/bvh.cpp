#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  nodes = std::vector<BVHNode>();
  root = construct_bvh(0, primitives.size(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  primitives.clear();
  nodes.clear();
}

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
  num_diffuse_bsdfs = diffuse_bsdfs_vec.size();
  num_nodes = bvh->nodes.size();


  primitives = (CudaPrimitive *) malloc(num_primitives * sizeof(CudaPrimitive));
  spheres = (CudaSphere *) malloc(num_spheres * sizeof(CudaSphere));
  triangles = (CudaTriangle *) malloc(num_triangles * sizeof(CudaTriangle));
  nodes = (BVHNode *) malloc(num_nodes * sizeof(BVHNode));
  diffuse_bsdfs = (CudaDiffuseBSDF *) malloc(num_diffuse_bsdfs * sizeof(CudaDiffuseBSDF));
  emission_bsdfs = (CudaEmissionBSDF *) malloc(num_emission_bsdfs * sizeof(CudaEmissionBSDF));

  memcpy(primitives, primitives_vec.data(), num_primitives * sizeof(CudaPrimitive));
  memcpy(spheres, spheres_vec.data(), num_spheres * sizeof(CudaSphere));
  memcpy(triangles, triangles_vec.data(), num_triangles * sizeof(CudaTriangle));
  memcpy(nodes, bvh->nodes.data(), bvh->nodes.size() * sizeof(BVHNode));
  memcpy(diffuse_bsdfs, diffuse_bsdfs_vec.data(), num_diffuse_bsdfs * sizeof(CudaDiffuseBSDF));
  memcpy(emission_bsdfs, emission_bsdfs_vec.data(), num_emission_bsdfs * sizeof(CudaEmissionBSDF));

  root = bvh->root;

  std::cout<< "BVHCuda: " << num_primitives << " primitives, " << num_nodes << " nodes, " << std::endl;
  std::cout<< "BVHCuda: " << num_diffuse_bsdfs << " diffuse BSDFs, " << num_emission_bsdfs << " emission BSDFs" << std::endl;
  std::cout<< "BVHCuda: " << num_triangles << " triangles, " << num_spheres << " spheres" << std::endl;
  std::cout<< "root: " << root << std::endl;
}

BVHCuda::~BVHCuda() {
  free(primitives);
  free(spheres);
  free(triangles);
  free(nodes);
}

BBox BVHAccel::get_bbox() const { return nodes[root].bb; }

int BVHAccel::construct_bvh(size_t start, size_t end, size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bbox;

  for (size_t p = start; p < end; p++) {
    BBox bb = (primitives[p])->get_bbox();
    bbox.expand(bb);
  }

  BVHNode node(bbox);

  if (end - start <= max_leaf_size) {
    node.start = start;
    node.end = end;
    node.leaf = true;
    node.l = 0;
    node.r = 0;
  } else {
    auto sort_x = [](Primitive *a, Primitive *b) {
      return a->get_bbox().centroid().x < b->get_bbox().centroid().x;
    };
    auto sort_y = [](Primitive *a, Primitive *b) {
      return a->get_bbox().centroid().y < b->get_bbox().centroid().y;
    };
    auto sort_z = [](Primitive *a, Primitive *b) {
      return a->get_bbox().centroid().z < b->get_bbox().centroid().z;
    };
    

    size_t best_axis = 0;
    size_t best_index = 0;
    double best_cost = std::numeric_limits<double>::infinity();

    auto start_it = primitives.begin() + start;
    auto end_it = primitives.begin() + end;
    for (size_t axis = 0; axis < 3; axis++) {
      if (axis == 0) {
        std::sort(start_it, end_it, sort_x);
      } else if (axis == 1) {
        std::sort(start_it, end_it, sort_y);
      } else {
        std::sort(start_it, end_it, sort_z);
      }

      std::vector<BBox> left(end-start+1), right(end-start+1);
      BBox s_bbox, e_bbox;
      for (size_t p = start; p < end; p++) {
        s_bbox.expand((primitives[p])->get_bbox());
        left[p - start] = s_bbox;
      }
      for (size_t p = end; p-- > start;) {
        e_bbox.expand(primitives[p]->get_bbox());
        right[p - start] = e_bbox;
      }

      for (size_t p = start + 1; p < end; p++) {
        double cost = left[p - start - 1].surface_area() * (p - start) + right[p - start].surface_area() * (end - p);
        if (cost < best_cost) {
          best_cost = cost;
          best_axis = axis;
          best_index = p - start;
        }
      }
    }

    if (best_axis == 0) {
      std::sort(start_it, end_it, sort_x);
    } else if (best_axis == 1) {
      std::sort(start_it, end_it, sort_y);
    } else {
      std::sort(start_it, end_it, sort_z);
    }

    auto mid = start + best_index;
    size_t l = construct_bvh(start, mid, max_leaf_size), r = construct_bvh(mid, end, max_leaf_size);
    node.leaf = false;
    node.l = l;
    node.r = r;
  }

  nodes.push_back(node);
  return nodes.size() - 1;
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, size_t idx) const {
  const BVHNode &node = nodes[idx];

  double t0, t1;
  if (!node.bb.intersect(ray, t0, t1)) {
    return false;
  }

  if (node.isLeaf()) {
    bool hit = false;
    Intersection tmp;
    for (size_t p = node.start; p < node.end; p++) {
      total_isects++;
      if (primitives[p]->intersect(ray, &tmp) && tmp.t < i->t) {
        hit = true;
        *i = tmp;
      }
    }
    return hit;
  }

  bool hit_left  = intersect(ray, i, node.l);
  bool hit_right = intersect(ray, i, node.r);
  return hit_left || hit_right;
}

bool BVHCuda::intersect(const Ray &ray, CudaIntersection *i, size_t idx) const {
  const BVHNode &node = nodes[idx];

  double t0, t1;
  if (!node.bb.intersect(ray, t0, t1)) {
    return false;
  }

  if (node.isLeaf()) {
    bool hit = false;
    CudaIntersection tmp;
    for (size_t p = node.start; p < node.end; p++) {
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
    return hit;
  }

  bool hit_left  = intersect(ray, i, node.l);
  bool hit_right = intersect(ray, i, node.r);
  return hit_left || hit_right;
}

Vector3D BVHCuda::sample_f (CudaBSDF bsdf, const Vector3D wo, Vector3D *wi, double* pdf) const {
  if (bsdf.type == CudaBSDFType_Diffuse) {
    return diffuse_bsdfs[bsdf.idx].sample_f(wo, wi, pdf);
  } else if (bsdf.type == CudaBSDFType_Emission) {
    return emission_bsdfs[bsdf.idx].sample_f(wo, wi, pdf);
  } else {
    std::cerr << "Unknown BSDF type" << std::endl;
    exit(1);
  }
}

Vector3D BVHCuda::f (CudaBSDF bsdf, const Vector3D wo, const Vector3D wi) const {
  if (bsdf.type == CudaBSDFType_Diffuse) {
    return diffuse_bsdfs[bsdf.idx].f(wo, wi);
  } else if (bsdf.type == CudaBSDFType_Emission) {
    return emission_bsdfs[bsdf.idx].f(wo, wi);
  } else {
    std::cerr << "Unknown BSDF type" << std::endl;
    exit(1);
  }
}

} // namespace SceneObjects
} // namespace CGL
