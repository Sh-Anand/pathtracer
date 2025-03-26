#include "bbox.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
  
  double tmin, tmax, timinx, timaxx, timiny, timaxy, timinz, timaxz;
  double r_rdx = r.inv_d.x, r_rdy = r.inv_d.y, r_rdz = r.inv_d.z;
  double minox = min.x - r.o.x, maxox = max.x - r.o.x;
  double minoy = min.y - r.o.y, maxoy = max.y - r.o.y;
  double minoz = min.z - r.o.z, maxoz = max.z - r.o.z;
  
  timinx = minox * r_rdx;
  timaxx = maxox * r_rdx;
  tmin = std::min(timinx, timaxx);
  tmax = std::max(timinx, timaxx);

  timiny = minoy * r_rdy;
  timaxy = maxoy * r_rdy;
  tmin = std::max(tmin, std::min(timiny, timaxy));
  tmax = std::min(tmax, std::max(timiny, timaxy));

  timinz = minoz * r_rdz;
  timaxz = maxoz * r_rdz;
  tmin = std::max(tmin, std::min(timinz, timaxz));
  tmax = std::min(tmax, std::max(timinz, timaxz));

  if (tmin > tmax || tmin > r.max_t || tmax < r.min_t || tmax < 0) {
    return false;
  }

  t0 = tmin;
  t1 = tmax;

  return true;

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
