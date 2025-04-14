#include "bvh.h"

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

BVHCuda::~BVHCuda() {
  free(primitives);
  free(spheres);
  free(triangles);
  free(nodes);
}

BBox BVHAccel::get_bbox() const { return nodes[root].bb; }

void BVHAccel::draw(size_t idx, const Color &c, float alpha) const {
  BVHNode node = nodes[idx];
  if (node.isLeaf()) {
    for (size_t p = node.start; p < node.end; p++) {
      (primitives[p])->draw(c, alpha);
    }
  }
}

void BVHAccel::drawOutline(size_t idx, const Color &c, float alpha) const {
  BVHNode node = nodes[idx];
  if (node.isLeaf()) {
    for (size_t p = node.start; p < node.end; p++) {
      (primitives[p])->drawOutline(c, alpha);
    }
  }
}

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

bool BVHAccel::intersect(Ray &ray, Intersection *i, size_t idx) const {
  const BVHNode &node = nodes[idx];

  float t0, t1;
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

} // namespace SceneObjects
} // namespace CGL
