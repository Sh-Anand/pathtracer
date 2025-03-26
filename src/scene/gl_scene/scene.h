#ifndef CGL_GLSCENE_SCENE_H
#define CGL_GLSCENE_SCENE_H

#include <string>
#include <vector>
#include <iostream>

#include "CGL/CGL.h"
#include "CGL/color.h"

#include "scene/bbox.h"
#include "scene/scene.h"

#include "pathtracer/ray.h"

namespace CGL { namespace GLScene {

struct SelectionInfo {
  std::vector<std::string> info;
};

/**
 * Interface that all physical objects in the scene conform to.
 * Note that this doesn't include properties like material that may be treated
 * as separate entities in a COLLADA file, or lights, which are treated
 * specially.
 */
class SceneObject {
public:

  /**
   * Given a transformation matrix from local to space to world space, returns
   * a bounding box of the object in world space. Note that this doesn't have
   * to be the smallest possible bbox, in case that's difficult to compute.
   */
  virtual BBox get_bbox() = 0;

  /**
   * Converts this object to an immutable, raytracer-friendly form. Passes in a
   * local-space-to-world-space transformation matrix, because the raytracer
   * expects all the objects to be
   */
  virtual SceneObjects::SceneObject *get_static_object() = 0;
};


/**
 * A light.
 */
class SceneLight {
 public:
  virtual SceneObjects::SceneLight *get_static_light() const = 0;
};

/**
 * The scene that meshEdit generates and works with.
 */
class Scene {
 public:
  Scene(std::vector<SceneObject *> objects, std::vector<SceneLight *> lights) {
    this->objects = objects;
    this->lights = lights;
    this->selectionIdx = -1;
    this->hoverIdx = -1;
  }

  /**
   * Gets a bounding box for the entire scene in world space coordinates.
   * May not be the tightest possible.
   */
  BBox get_bbox();

  /**
   * Builds a static scene that's equivalent to the current scene and is easier
   * to use in raytracing, but doesn't allow modifications.
   */
  SceneObjects::Scene *get_static_scene();

  std::vector<SceneObject*> objects;
  std::vector<SceneLight*> lights;

 private:
  SelectionInfo selectionInfo;
  int selectionIdx, hoverIdx;
};

} // namespace GLScene
} // namespace CGL

#endif // CGL_GLSCENE_GLSCENE_H
