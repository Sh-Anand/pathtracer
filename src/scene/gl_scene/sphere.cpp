#include "sphere.h"

#include "scene/object.h"

#include "pathtracer/bsdf.h"

namespace CGL { namespace GLScene {

Sphere::Sphere(const Collada::SphereInfo& info, 
               const Vector3D position, const double scale) : 
  p(position), r(info.radius * scale) { 
  if (info.material) {
    bsdf = info.material->bsdf;
  } else {
    bsdf = new DiffuseBSDF(Vector3D(0.5f,0.5f,0.5f));    
  }
}

BBox Sphere::get_bbox() {
  return BBox(p.x - r, p.y - r, p.z - r, p.x + r, p.y + r, p.z + r);
}

BSDF* Sphere::get_bsdf() {
  return bsdf;
}

SceneObjects::SceneObject *Sphere::get_static_object() {
  return new SceneObjects::SphereObject(p, r, bsdf);
}

} // namespace GLScene
} // namespace CGL
