#include "application.h"

#include "parser/loader.h"

#include <cmath>
#include <iostream>
#include <unistd.h>

#define msg(s) cerr << "[PathTracer] " << s << endl;

Application::Application(AppConfig config) {
  renderer = new RaytracedRenderer (
    config.pathtracer_accumulate_bounces,
    config.pathtracer_max_ray_depth,
    config.pathtracer_ns_area_light,
    config.pathtracer_filename,
    config.debug,
    config.restir
  );
  filename = config.pathtracer_filename;

  texcoords.clear();
  vertices.clear();
  normals.clear();
  tangents.clear();
  bsdfs.clear();
  lights.clear();
  textures.clear();
  
  texcoords.push_back(Vector2D{0, 0}); // dummy texcoord for all non-textured materials
  tangents.push_back(Vector4D{0,0,0,0}); // dummy tangent for all non-bump mapped materials
}

Application::~Application() {
  delete renderer;
}

void Application::init() {
  screenW = 800; screenH = 600; // Default value
}

void Application::resize(size_t w, size_t h) {
  screenW = w;
  screenH = h;
  renderer->set_frame_size(w, h);
}

void Application::set_up_pathtracer() {
  renderer->set_camera(&camera);
  renderer->set_frame_size(screenW, screenH);
  renderer->build_accel(primitives, vertices, normals, texcoords, tangents);
}

void usage(const char *binaryName) {
  printf("Usage: %s [options] <scenefile>\n", binaryName);
  printf("Program Options:\n");
  printf("  -l  <INT>        Number of samples per area light\n");
  printf("  -g  <INT>        Number of total image generated\n");
  printf("  -R  <INT>        Enable ReSTIR-GI\n");
  printf("  -d  <INT>        Enable debug mode\n");
  printf("  -m  <INT>        Maximum ray depth\n");
  printf("  -o  <INT>        Include direct light in render \n");
  printf("  -f  <FILENAME>   Image (.png) file to save output to in windowless "
         "mode\n");
  printf(
      "  -r  <INT> <INT>  Width and height of output image (i fwindowless)\n");
  printf("  -h               Print this help message\n");
  printf("\n");
}

int main(int argc, char **argv) {
  // get the options
  AppConfig config;
  int opt;
  size_t w = 0, h = 0, x = -1, y = 0, dx = 0, dy = 0;
  string output_file_name, cam_settings = "";
  string sceneFilePath;
  while ((opt = getopt(argc, argv, "l:m:o:h:f:r:d:R:g:")) !=
          -1) { // for each option...
    switch (opt) {
    case 'f':
      output_file_name = string(optarg);
      break;
    case 'r':
      w = atoi(argv[optind - 1]);
      h = atoi(argv[optind]);
      optind++;
      break;
    case 'l':
      config.pathtracer_ns_area_light = atoi(optarg);
      break;
    case 'g':
      config.total_image_generated = atoi(optarg);
      break;
    case 't':
      break;
    case 'm':
      config.pathtracer_max_ray_depth = atoi(optarg);
      break;
    case 'o':
      config.pathtracer_accumulate_bounces = atoi(optarg) > 0;
      break;
    case 'd':
      config.debug = atoi(optarg) > 0;
      break;
    case 'R':
      config.restir = atoi(optarg) > 0;
      break;
    default:
      usage(argv[0]);
      return 1;
    }

    // print usage if no argument given
    if (optind >= argc) {
      usage(argv[0]);
      return 1;
    }

    sceneFilePath = argv[optind];
  }
  string sceneFile = sceneFilePath.substr(sceneFilePath.find_last_of('/') + 1);
  sceneFile = sceneFile.substr(0, sceneFile.find(".dae"));
  config.pathtracer_filename = sceneFile;

  w = w ? w : 800; // default width
  h = h ? h : 600; // default height

  // Load scene
  cout << "[PathTracer] Loading scene from " << sceneFilePath << endl;
  Loader* loader = new Loader(w, h, sceneFilePath);
  
  // create application
  Application *app = new Application(config);

  // write straight to file without opening a window if -f option provided
  app->init();

  if (w && h) {
    app->resize(w, h);
  }

  app->bsdfs = loader->parser.bsdfs;
  app->camera = loader->parser.camera;
  app->lights = loader->parser.lights;
  app->normals = loader->parser.normals;
  app->primitives = loader->parser.primitives;
  app->tangents = loader->parser.tangents;
  app->texcoords = loader->parser.texcoords;
  app->textures = loader->parser.textures;
  app->vertices = loader->parser.vertices;

  if(config.total_image_generated == 1){
    app->render_to_file(output_file_name, x, y, dx, dy);
  }else{
    app->render_to_video(output_file_name, x, y, dx, dy, config.total_image_generated);
  }
  return 0;
}