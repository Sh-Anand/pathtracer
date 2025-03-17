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
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
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

  BVHNode *node = new BVHNode(bbox);

  if (end - start <= max_leaf_size) {
    node->start = start;
    node->end = end;
    return node;
  }

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
  
  node->l = construct_bvh(start, mid, max_leaf_size);
  node->r = construct_bvh(mid, end, max_leaf_size);
  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  double t0, t1;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false;
  }

  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->has_intersection(ray)) {
        return true;
      }
    }
    return false;
  } else {
    return has_intersection(ray, node->l) || has_intersection(ray, node->r);
  }

}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.

  bool hit = false;

  double t0, t1;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false;
  }

  i->t = ray.max_t;

  Intersection t_i;
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->intersect(ray, &t_i)) {
        if (t_i.t < i->t) {
          hit = true;
          *i = t_i;
        }
      }
    }
  } else {
    bool hit_l = intersect(ray, &t_i, node->l);
    if (hit_l && t_i.t < i->t) *i = t_i;
    bool hit_r = intersect(ray, &t_i, node->r);
    if (hit_r && t_i.t < i->t) *i = t_i;
    hit = hit_l || hit_r;
  }

  return hit;
}

} // namespace SceneObjects
} // namespace CGL
