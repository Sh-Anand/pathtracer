#ifndef CGL_APPLICATION_H
#define CGL_APPLICATION_H

// STL
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <string>
#include <vector>

// COLLADA
#include "scene/collada/collada.h"
#include "scene/collada/light_info.h"
#include "scene/collada/sphere_info.h"
#include "scene/collada/polymesh_info.h"
#include "scene/collada/material_info.h"
#include "scene/collada/camera_info.h"

// MeshEdit
#include "scene/gl_scene/scene.h"
#include "util/halfEdgeMesh.h"

// RaytracedRenderer
#include "pathtracer/raytraced_renderer.h"
#include "util/image.h"

// Shared modules
#include "pathtracer/camera.h"

// GLTF parser
#include "util/tiny_gltf.h"
#include "util/vector3D.h"

using namespace std;

namespace CGL {

  class VisualDebugger;

struct AppConfig {

  AppConfig () {

    pathtracer_ns_aa = 1;
    pathtracer_max_ray_depth = 1;
    pathtracer_accumulate_bounces = true;
    pathtracer_ns_area_light = 1;

    pathtracer_ns_diff = 1;
    pathtracer_ns_glsy = 1;
    pathtracer_ns_refr = 1;

    pathtracer_num_threads = 1;
    pathtracer_envmap = NULL;

    pathtracer_samples_per_patch = 32;
    pathtracer_max_tolerance = 0.05f;
    pathtracer_direct_hemisphere_sample = false;

    pathtracer_filename = "";
    pathtracer_lensRadius = 0.0;
    pathtracer_focalDistance = 4.7;
  }

  size_t pathtracer_ns_aa;
  size_t pathtracer_max_ray_depth;
  bool pathtracer_accumulate_bounces; // whether we accumulate light bounce or only sample from the last bounce
  size_t pathtracer_ns_area_light;

  size_t pathtracer_ns_diff;
  size_t pathtracer_ns_glsy;
  size_t pathtracer_ns_refr;

  size_t pathtracer_num_threads;
  HDRImageBuffer* pathtracer_envmap;

  float pathtracer_max_tolerance;
  size_t pathtracer_samples_per_patch;

  bool pathtracer_direct_hemisphere_sample;

  string pathtracer_filename;

  double pathtracer_lensRadius;
  double pathtracer_focalDistance;
};

class Application {
 public:

  Application(AppConfig config, bool gl = true);

  ~Application();

  void init();
  void render() {

  }

  std::string name() {
    return "Application";
  }
  std::string info() {
    return "Path Tracer";
  }

  void resize(size_t w, size_t h);
  void load(Collada::SceneInfo* sceneInfo);
  void load_from_gltf_model(const tinygltf::Model &model);
  void render_to_file(std::string filename, size_t x, size_t y, size_t dx, size_t dy) { 
    set_up_pathtracer();
    renderer->render_to_file(filename, x, y, dx, dy, lights, bsdfs); 
  }

  void load_camera(std::string filename) {
    camera.load_settings(filename);
  }

private:
  void set_up_pathtracer();

  GLScene::Scene *scene;
  RaytracedRenderer* renderer;

  // View Frustrum Variables.
  // On resize, the aspect ratio is changed. On reset_camera, the position and
  // orientation are reset but NOT the aspect ratio.
  Camera camera;
  Camera canonicalCamera;

  size_t screenW;
  size_t screenH;

  // Length of diagonal of bounding box for the mesh.
  // Guranteed to not have the camera occlude with the mes.
  double canonical_view_distance;

  // Initialization functions to get the opengl cooking with oil.
  void init_camera(Collada::CameraInfo& camera, const Matrix4x4& transform);
  void init_material(Collada::MaterialInfo& material);
  void ParseNode(const tinygltf::Model &model, int nodeIdx, const Matrix4x4 &parentTransform);

  std::vector<Vector3D> vertices;
  std::vector<Vector3D> normals;
  std::vector<CudaPrimitive> primitives;
  std::vector<CudaBSDF> bsdfs;
  std::vector<CudaLight> lights;

  std::string filename;

}; // class Application

} // namespace CGL

  #endif // CGL_APPLICATION_H
