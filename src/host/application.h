#ifndef CGL_APPLICATION_H
#define CGL_APPLICATION_H

#include <iostream>
#include <string>
#include <vector>

#include "raytraced_renderer.h"
#include "parser/loader.h"

using namespace std;

struct AppConfig {

  AppConfig () {
    pathtracer_max_ray_depth = 1;
    pathtracer_accumulate_bounces = true;
    pathtracer_ns_area_light = 1;
    total_image_generated = 1;
    debug = false;
    restir = false;
  }

  size_t pathtracer_max_ray_depth;
  bool pathtracer_accumulate_bounces; // whether we accumulate light bounce or only sample from the last bounce
  size_t pathtracer_ns_area_light;
  size_t total_image_generated;
  bool debug;
  bool restir;
};

class Application {
 public:

  Application(AppConfig config, string sceneFilePath, int w, int h);

  ~Application();

  void render_to_file(std::string filename);

  void render_to_video(std::string filename, size_t num_images);

  RaytracedRenderer* renderer;

  size_t screenW;
  size_t screenH;

  Loader *loader;

  std::string filename;

}; // class Application

#endif // CGL_APPLICATION_H
