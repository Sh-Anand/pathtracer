#include "scene.h"

using std::cout;
using std::endl;

namespace CGL { namespace GLScene {

BBox Scene::get_bbox() {
  BBox bbox;
  for (SceneObject *obj : objects) {
    bbox.expand(obj->get_bbox());
  }
  return bbox;
}

SceneObjects::Scene *Scene::get_static_scene() {
  std::vector<SceneObjects::SceneObject *> staticObjects;
  std::vector<SceneObjects::SceneLight *> staticLights;

  for (SceneObject *obj : objects) {
    staticObjects.push_back(obj->get_static_object());
  }
  for (SceneLight *light : lights) {
    staticLights.push_back(light->get_static_light());
  }

  return new SceneObjects::Scene(staticObjects, staticLights);
}

} // namespace GLScene
} // namespace CGL
