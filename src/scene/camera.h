#ifndef CGL_CAMERA_H
#define CGL_CAMERA_H

#include <iostream>

#include "util/matrix.h"

#include "math.h"
#include "geometry.h"

struct CameraInfo {

	Vector3D view_dir;
	Vector3D up_dir;

  float hFov, vFov, nClip, fClip;
};

/**
 * Camera.
 */
class Camera {
 public:

  /*
    Rotate by the specified amount around the target.
  */
  void rotate_by(const float dPhi, const float dTheta) {
    phi = clamp_T(phi + dPhi, 0.0f, PI);
    theta += dTheta;
    compute_position();
  }

  // Computes pos, screenXDir, screenYDir from target, r, phi, theta.
  void compute_position() {
    float sinPhi = sin(phi);
    if (sinPhi == 0) {
      phi += EPS_F;
      sinPhi = sin(phi);
    }
    const Vector3D dirToCamera{-r * sinPhi * sinf(theta),
                              r * cosf(phi),
                              r * sinPhi * cosf(theta)};
    pos = vector3d_add(targetPos, dirToCamera);
    Vector3D upVec{0, sinPhi > 0 ? 1.0f : -1.0f, 0.f};
    Vector3D screenXDir = vector3d_unit(vector3d_cross(upVec, dirToCamera));
    Vector3D screenYDir = vector3d_unit(vector3d_cross(dirToCamera, screenXDir));

    c2w.c[0] = screenXDir;
    c2w.c[1] = screenYDir;
    c2w.c[2] = vector3d_unit(dirToCamera);  
  }

  // Field of view aspect ratio, clipping planes.
  float hFov, vFov, ar, nClip, fClip;

  // Current position and target point (the point the camera is looking at).
  Vector3D pos, targetPos;

  // Orientation relative to target, and min & max distance from the target.
  float phi, theta, r, minR, maxR;

  // camera-to-world rotation matrix (note: also need to translate a
  // camera-space point by 'pos' to perform a full camera-to-world
  // transform)
  Matrix3x3 c2w;
};

typedef struct {
  float hFov, vFov, nClip, fClip;
  Vector3D pos;
  Matrix3x3 c2w;
} CudaCamera;

HOST_DEVICE inline float d_radians(float degrees) {
  return degrees * (PI / 180.0);
}

DEVICE static inline Ray generate_ray(CudaCamera *cam, float x, float y) {
  Vector3D sensor = Vector3D{(x - 0.5f) * 2 * tanf(d_radians(cam->hFov) * 0.5f), (y - 0.5f) * 2 * tanf(d_radians(cam->vFov) * 0.5f),-1};
  Vector3D dir = matrix3x3_vector_multiply(&cam->c2w, &sensor);
  Ray camera_ray;
  camera_ray.o = cam->pos; camera_ray.d = vector3d_unit(dir); camera_ray.inv_d = vector3d_rcp(camera_ray.d);
  camera_ray.min_t = cam->nClip; camera_ray.max_t = cam->fClip;

  return camera_ray;
}


#endif // CGL_CAMERA_H
