#include "camera.h"

namespace CGL {
/**
 * This function generates a ray from camera perspective, passing through camera / sensor plane (x,y)
 */

HOST_DEVICE inline float d_radians(float degrees) {
  return degrees * (PI_F / 180.0);
}

DEVICE Ray CudaCamera::generate_ray(float x, float y) {

  // TODO (Part 1.1):
  // compute position of the input sensor sample coordinate on the
  // canonical sensor plane one unit away from the pinhole.
  // Note: hFov and vFov are in degrees.
  //
  Vector3D sensor = Vector3D((x - 0.5) * 2 * tanf(d_radians(hFov) * 0.5), (y - 0.5) * 2 * tanf(d_radians(vFov) * 0.5), -1);
  Vector3D dir = (c2w * sensor);
  // printf("dir: %lf %lf %lf\n", dir.x, dir.y, dir.z);
  Ray camera_ray = Ray(pos, dir.unit());
  camera_ray.min_t = nClip; camera_ray.max_t = fClip;

  return camera_ray;
}

}