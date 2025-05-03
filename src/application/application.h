#ifndef CGL_APPLICATION_H
#define CGL_APPLICATION_H

// STL
#include <cmath>
#include <cstddef>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <string>
#include <vector>

// COLLADA
#include "scene/camera_info.h"

// RaytracedRenderer
#include "pathtracer/raytraced_renderer.h"
#include "util/image.h"

// Shared modules
#include "pathtracer/camera.h"
#include "pathtracer/texture.h"

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

    total_image_generated = 1;
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

  size_t total_image_generated;
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
  void load_from_gltf_model(const tinygltf::Model &model);
  void render_to_file(std::string filename, size_t x, size_t y, size_t dx, size_t dy) { 
    set_up_pathtracer();
    renderer->set_cuda_camera();
    renderer->copy_host_device_pt(lights, bsdfs, textures);
    renderer->render_to_file(filename, x, y, dx, dy); 
  }

  void render_to_video(std::string filename, size_t x, size_t y, size_t dx, size_t dy, size_t num_images){
    const double TOTAL_ROTATION = M_PI;
    double angle_per_image = TOTAL_ROTATION / (double)num_images;
    size_t dot_pos = filename.find_last_of('.');
    auto name = filename.substr(0, dot_pos);
    auto dot_extension = filename.substr(dot_pos);

    set_up_pathtracer();
    renderer->set_cuda_camera();
    renderer->copy_host_device_pt(lights, bsdfs, textures);

    for(size_t i = 0; i < num_images; ++i){
      std::cout << "Rendering image: " << i << std::endl;
      std::ostringstream oss;
      oss << std::setw(4) << std::setfill('0') << i;
      auto filename_per_image = name + oss.str() + dot_extension;
      camera.rotate_by(0, angle_per_image);
      renderer->set_cuda_camera();
      renderer->update_camera();
      renderer->render_to_file(filename_per_image, x, y, dx, dy); 
    }
  }

  void load_camera(std::string filename) {
    camera.load_settings(filename);
  }

private:
  void set_up_pathtracer();

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
  void init_camera(CameraInfo& camera, const Matrix4x4& transform);
  void ParseMaterial(const tinygltf::Model&);
  void ParseNode(const tinygltf::Model &model, int nodeIdx, const Matrix4x4 &parentTransform);
  void ParseTexture(const tinygltf::Model &model);

  std::vector<Vector3D> vertices;
  std::vector<Vector3D> normals;
  std::vector<Vector4D> tangents;
  std::vector<Vector2D> texcoords;
  std::vector<CudaPrimitive> primitives;
  std::vector<CudaBSDF> bsdfs;
  std::vector<CudaLight> lights;
  std::vector<CudaTexture> textures;

  std::string filename;

}; // class Application

} // namespace CGL

  #endif // CGL_APPLICATION_H
