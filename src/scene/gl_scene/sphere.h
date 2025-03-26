#ifndef CGL_GLSCENE_SPHERE_H
#define CGL_GLSCENE_SPHERE_H

#include "scene.h"

#include "../collada/sphere_info.h"

namespace CGL { namespace GLScene {

class Sphere : public SceneObject {
 public:
  Sphere(const Collada::SphereInfo& sphereInfo, const Vector3D position,
         const double scale);

  BBox get_bbox();

  BSDF* get_bsdf();
  SceneObjects::SceneObject *get_static_object();

private:
  double r;
  Vector3D p;
  BSDF* bsdf;
};

} // namespace GLScene
} // namespace CGL

#endif //CGL_GLSCENE_SPHERE_H
