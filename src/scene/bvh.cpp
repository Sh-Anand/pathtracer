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
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  primitives.clear();
  nodes.clear();
}

BBox BVHAccel::get_bbox() const { return nodes[root].bb; }

void BVHAccel::draw(size_t idx, const Color &c, float alpha) const {
  BVHNode node = nodes[idx];
  if (node.isLeaf()) {
    for (auto p = node.start; p != node.end; p++) {
      (*p)->draw(c, alpha);
    }
  }
}

void BVHAccel::drawOutline(size_t idx, const Color &c, float alpha) const {
  BVHNode node = nodes[idx];
  if (node.isLeaf()) {
    for (auto p = node.start; p != node.end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  }
}

int BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bbox;

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
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

    for (size_t axis = 0; axis < 3; axis++) {
      if (axis == 0) {
        std::sort(start, end, sort_x);
      } else if (axis == 1) {
        std::sort(start, end, sort_y);
      } else {
        std::sort(start, end, sort_z);
      }

      std::vector<BBox> left(end-start+1), right(end-start+1);
      BBox s_bbox, e_bbox;
      for (auto p = start; p != end; p++) {
        s_bbox.expand((*p)->get_bbox());
        left[p - start] = s_bbox;
      }
      for (auto p = end - 1; p >= start; p--) {
        e_bbox.expand((*p)->get_bbox());
        right[p - start] = e_bbox;
      }

      for (auto p = start + 1; p != end; p++) {
        double cost = left[p - start - 1].surface_area() * (p - start) + right[p - start].surface_area() * (end - p);
        if (cost < best_cost) {
          best_cost = cost;
          best_axis = axis;
          best_index = p - start;
        }
      }
    }

    if (best_axis == 0) {
      std::sort(start, end, sort_x);
    } else if (best_axis == 1) {
      std::sort(start, end, sort_y);
    } else {
      std::sort(start, end, sort_z);
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

bool BVHAccel::has_intersection(const Ray &ray, size_t idx) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  BVHNode node = nodes[idx];
  double t0, t1;
  if (!node.bb.intersect(ray, t0, t1)) {
    return false;
  }

  if (node.isLeaf()) {
    for (auto p = node.start; p != node.end; p++) {
      total_isects++;
      if ((*p)->has_intersection(ray)) {
        return true;
      }
    }
    return false;
  } else {
    return has_intersection(ray, node.l) || has_intersection(ray, node.r);
  }

}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, size_t idx) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.

  bool hit = false;

  double t0, t1;

  i->t = ray.max_t;

  Intersection t_i;
  std::stack<size_t> stack;
  stack.push(idx);
  while (!stack.empty()) {
    BVHNode n = nodes[stack.top()];
    stack.pop();
    if (!n.bb.intersect(ray, t0, t1)) {
      continue;
    }
    if (n.isLeaf()) {
      for (auto p = n.start; p != n.end; p++) {
        total_isects++;
        if ((*p)->intersect(ray, &t_i)) {
          if (t_i.t < i->t) {
            hit = true;
            *i = t_i;
          }
        }
      }
    } else {
      stack.push(n.l);
      stack.push(n.r);
    }
  } 

  return hit;
}

} // namespace SceneObjects
} // namespace CGL
