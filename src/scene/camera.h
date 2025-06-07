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
    Sets the field of view to match screen screenW/H.
    NOTE: data and screenW/H will almost certainly disagree about the aspect
          ratio. screenW/H are treated as the source of truth, and the field
          of view is expanded along whichever dimension is too narrow.
    NOTE2: info.hFov and info.vFov are expected to be in DEGREES.
  */
  void configure(const CameraInfo& info,
                 size_t screenW, size_t screenH);

  /*
    Phi and theta are in RADIANS.
  */
  void place(const Vector3D targetPos, const float phi, const float theta,
             const float r, const float minR, const float maxR);

  std::string param_string() {
    return "";
  }

  /*
    Copies just placement data from the other camera.
  */
  void copy_placement(const Camera& other);

  /*
    Updates the screen size to be the specified size, keeping screenDist
    constant.
  */
  void set_screen_size(const size_t screenW, const size_t screenH);

  /*
    Translates the camera such that a value at distance d directly in front of
    the camera moves by (dx, dy). Note that dx and dy are in screen coordinates,
    while d is in world-space coordinates (like pos/dir/up).
  */
  void move_by(const float dx, const float dy, const float d);

  /*
    Move the specified amount along the view axis.
  */
  void move_forward(const float dist);

  /*
    Rotate by the specified amount around the target.
  */
  void rotate_by(const float dPhi, const float dTheta);

  Ray generate_ray_for_thin_lens(float x, float y, float rndR, float rndTheta) const;

  // Lens aperture and focal distance for depth of field effects.
  float lensRadius;
  float focalDistance;

  // Computes pos, screenXDir, screenYDir from target, r, phi, theta.
  void compute_position();

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

  // Info about screen to render to; it corresponds to the camera's full field
  // of view at some distance.
  size_t screenW, screenH;
  float screenDist;
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
