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

// RaytracedRenderer
#include "raytraced_renderer.h"

// Shared modules
#include "scene/camera.h"

// GLTF parser
#include "util/tiny_gltf.h"
#include "util/vector.h"

using namespace std;

struct AppConfig {

  AppConfig () {
    pathtracer_max_ray_depth = 1;
    pathtracer_accumulate_bounces = true;
    pathtracer_ns_area_light = 1;

    pathtracer_filename = "";

    total_image_generated = 1;

    debug = false;
    restir = false;
  }

  size_t pathtracer_max_ray_depth;
  bool pathtracer_accumulate_bounces; // whether we accumulate light bounce or only sample from the last bounce
  size_t pathtracer_ns_area_light;

  string pathtracer_filename;

  size_t total_image_generated;

  bool debug;
  bool restir;
};

class Application {
 public:

  Application(AppConfig config);

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
    const double TOTAL_ROTATION = M_PI * 2;
    double angle_per_image = TOTAL_ROTATION / (double)num_images;
    size_t dot_pos = filename.find_last_of('.');
    auto name = filename.substr(0, dot_pos);
    auto dot_extension = filename.substr(dot_pos);

    set_up_pathtracer();
    renderer->set_cuda_camera();
    renderer->copy_host_device_pt(lights, bsdfs, textures);

    for(size_t i = 0; i < num_images; ++i){
      std::ostringstream oss;
      oss << std::setw(4) << std::setfill('0') << i;
      auto filename_per_image = name + oss.str() + dot_extension;
      camera.rotate_by(0, angle_per_image);
      renderer->set_cuda_camera();
      renderer->update_camera();
      renderer->render_to_file(filename_per_image, x, y, dx, dy); 
    }
  }

private:
  void set_up_pathtracer();

  RaytracedRenderer* renderer;

  // View Frustrum Variables.
  // On resize, the aspect ratio is changed. On reset_camera, the position and
  // orientation are reset but NOT the aspect ratio.
  Camera camera;

  size_t screenW;
  size_t screenH;

  // Length of diagonal of bounding box for the mesh.
  // Guranteed to not have the camera occlude with the mes.
  double canonical_view_distance;

  // Initialization functions to get the opengl cooking with oil.
  void init_camera(CameraInfo& camera);
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

#endif // CGL_APPLICATION_H
