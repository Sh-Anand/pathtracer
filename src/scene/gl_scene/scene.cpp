#include "scene.h"

namespace CGL { namespace GLScene {

BBox Scene::get_bbox() {
  BBox bbox;
  for (SceneObject *obj : objects) {
    bbox.expand(obj->get_bbox());
  }
  return bbox;
}

} // namespace GLScene
} // namespace CGL
