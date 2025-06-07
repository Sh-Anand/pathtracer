#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "util/vector.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

/**
 * Sets the field of view to match screen screenW/H.
 * NOTE: data and screenW/H will almost certainly disagree about the aspect
 *       ratio. screenW/H are treated as the source of truth, and the field
 *       of view is expanded along whichever dimension is too narrow.
 * NOTE2: info.hFov and info.vFov are expected to be in DEGREES.
 */
void Camera::configure(const CameraInfo& info, size_t screenW, size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;
  nClip = info.nClip;
  fClip = info.fClip;
  hFov = info.hFov;
  vFov = info.vFov;

  float ar1 = tan(radians(hFov) / 2) / tan(radians(vFov) / 2);
  ar = static_cast<float>(screenW) / screenH;
  if (ar1 < ar) {
    // hFov is too small
    hFov = 2 * degrees(atan(tan(radians(vFov) / 2) * ar));
  } else if (ar1 > ar) {
    // vFov is too small
    vFov = 2 * degrees(atan(tan(radians(hFov) / 2) / ar));
  }
  screenDist = ((float) screenH) / (2.0 * tan(radians(vFov) / 2));
}

/**
 * This function places the camera at the target position and sets the arguments.
 * Phi and theta are in RADIANS.
 */
void Camera::place(const Vector3D targetPos, const float phi,
                   const float theta, const float r, const float minR,
                   const float maxR) {
  float r_ = min(max(r, minR), maxR);
  float phi_ = (sin(phi) == 0) ? (phi + EPS_F) : phi;
  this->targetPos = targetPos;
  this->phi = phi_;
  this->theta = theta;
  this->r = r_;
  this->minR = minR;
  this->maxR = maxR;
  compute_position();
}

/**
 * This function copies the camera placement state.
 */
void Camera::copy_placement(const Camera& other) {
  pos = other.pos;
  targetPos = other.targetPos;
  phi = other.phi;
  theta = other.theta;
  minR = other.minR;
  maxR = other.maxR;
  c2w = other.c2w;
}

/**
 * This sets the screen size & compute the new FOV.
 */
void Camera::set_screen_size(const size_t screenW, const size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;
  ar = 1.0 * screenW / screenH;
  hFov = 2 * degrees(atan(((float) screenW) / (2 * screenDist)));
  vFov = 2 * degrees(atan(((float) screenH) / (2 * screenDist)));
}

/**
 * This function translates the camera position
 */
void Camera::move_by(const float dx, const float dy, const float d) {
  const float scaleFactor = d / screenDist;
  const Vector3D displacement =
    vector3d_add(vector3d_scale(c2w.c[0], (dx * scaleFactor)), vector3d_scale(c2w.c[1], (dy * scaleFactor)));
  pos = vector3d_add(pos, displacement);
  targetPos = vector3d_add(targetPos, displacement);
}

/**
 * This function rotates the camera position
 */
void Camera::rotate_by(const float dPhi, const float dTheta) {
  phi = clamp_T(phi + dPhi, 0.0f, PI);
  theta += dTheta;
  compute_position();
}

/**
 * This function computes the camera position, basis vectors, and the view matrix
 */
void Camera::compute_position() {
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

  cout << "Camera position: " << pos.x << ", " << pos.y << ", " << pos.z << endl;
  cout << "Camera target: " << targetPos.x << ", " << targetPos.y << ", " << targetPos.z << endl;
  cout << "Camera up: " << upVec.x << ", " << upVec.y << ", " << upVec.z << endl;
  cout << "Camera screen X direction: " << screenXDir.x << ", " << screenXDir.y << ", " << screenXDir.z << endl;
  cout << "Camera screen Y direction: " << screenYDir.x << ", " << screenYDir.y << ", " << screenYDir.z << endl;
  cout << "Camera direction: " << dirToCamera.x << ", " << dirToCamera.y << ", " << dirToCamera.z << endl;


  c2w.c[0] = screenXDir;
  c2w.c[1] = screenYDir;
  c2w.c[2] = vector3d_unit(dirToCamera);
}