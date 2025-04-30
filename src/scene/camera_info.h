#ifndef CGL_COLLADA_CAMERAINFO_H
#define CGL_COLLADA_CAMERAINFO_H

#include "util/vector3D.h"

namespace CGL {

/*
  Note that hFov_ and vFov_ are expected to be in DEGREES.
*/
struct CameraInfo {

	Vector3D view_dir;
	Vector3D up_dir;

  float hFov, vFov, nClip, fClip;
};

} // namespace CGL

#endif // CGL_COLLADA_CAMERAINFO_H
