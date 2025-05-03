#include "bbox.h"

namespace CGL {
  
  HOST_DEVICE bool BBox::intersect(Ray& r, float& t0, float& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
  
  float tmin, tmax, timinx, timaxx, timiny, timaxy, timinz, timaxz;
  float r_rdx = r.inv_d.x, r_rdy = r.inv_d.y, r_rdz = r.inv_d.z;
  float minox = min.x - r.o.x, maxox = max.x - r.o.x;
  float minoy = min.y - r.o.y, maxoy = max.y - r.o.y;
  float minoz = min.z - r.o.z, maxoz = max.z - r.o.z;
  
  timinx = minox * r_rdx;
  timaxx = maxox * r_rdx;
  tmin = fminf(timinx, timaxx);
  tmax = fmaxf(timinx, timaxx);

  timiny = minoy * r_rdy;
  timaxy = maxoy * r_rdy;
  tmin = fmaxf(tmin, fminf(timiny, timaxy));
  tmax = fminf(tmax, fmaxf(timiny, timaxy));

  timinz = minoz * r_rdz;
  timaxz = maxoz * r_rdz;
  tmin = fmaxf(tmin, fminf(timinz, timaxz));
  tmax = fminf(tmax, fmaxf(timinz, timaxz));

  if (tmin > tmax || tmin > r.max_t || tmax < r.min_t || tmax < 0) {
    return false;
  }

  t0 = tmin;
  t1 = tmax;

  return true;

}
}