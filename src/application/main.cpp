#include "application.h"
typedef uint32_t gid_t;
#include "util/image.h"
typedef uint32_t gid_t;

#include <iostream>
#include <unistd.h>

using namespace std;
using namespace CGL;

#define msg(s) cerr << "[PathTracer] " << s << endl;

void usage(const char *binaryName) {
  printf("Usage: %s [options] <scenefile>\n", binaryName);
  printf("Program Options:\n");
  printf("  -s  <INT>        Number of camera rays per pixel\n");
  printf("  -l  <INT>        Number of samples per area light\n");
  printf("  -t  <INT>        Number of render threads\n");
  printf("  -m  <INT>        Maximum ray depth\n");
  printf("  -o  <INT>        Accumulate Bounces of Light \n");
  printf("  -e  <PATH>       Path to environment map\n");
  printf("  -b  <FLOAT>      The size of the aperture\n");
  printf("  -d  <FLOAT>      The focal distance\n");
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
  while ((opt = getopt(argc, argv, "s:l:t:m:o:e:h:H:f:r:c:b:d:a:p:")) !=
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
    case 'p':
      x = atoi(argv[optind - 1]);
      y = atoi(argv[optind - 0]);
      dx = atoi(argv[optind + 1]);
      dy = atoi(argv[optind + 2]);
      optind += 3;
      break;
    case 's':
      config.pathtracer_ns_aa = atoi(optarg);
      break;
    case 'l':
      config.pathtracer_ns_area_light = atoi(optarg);
      break;
    case 't':
      config.pathtracer_num_threads = atoi(optarg);
      break;
    case 'm':
      config.pathtracer_max_ray_depth = atoi(optarg);
      break;
    case 'o':
      config.pathtracer_accumulate_bounces = atoi(optarg) > 0;
      break;
    case 'e':
      std::cout << "[PathTracer] Loading environment map " << optarg
                << std::endl;
      std::cout << "Environment map not implemented" << std::endl;
      std::abort();
      break;
    case 'c':
      cam_settings = string(optarg);
      break;
    case 'b':
      config.pathtracer_lensRadius = atof(optarg);
      break;
    case 'd':
      config.pathtracer_focalDistance = atof(optarg);
      break;
    case 'a':
      config.pathtracer_samples_per_patch = atoi(argv[optind - 1]);
      config.pathtracer_max_tolerance = atof(argv[optind]);
      optind++;
      break;
    case 'H':
      config.pathtracer_direct_hemisphere_sample = true;
      optind--;
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
  msg("Input scene file: " << sceneFilePath);
  string sceneFile = sceneFilePath.substr(sceneFilePath.find_last_of('/') + 1);
  sceneFile = sceneFile.substr(0, sceneFile.find(".dae"));
  config.pathtracer_filename = sceneFile;

  // parse scene
  Collada::SceneInfo *sceneInfo = new Collada::SceneInfo();
  if (Collada::ColladaParser::load(sceneFilePath.c_str(), sceneInfo) < 0) {
    delete sceneInfo;
    exit(0);
  }

  // create application
  Application *app = new Application(config, false);

  msg("Rendering using " << config.pathtracer_num_threads << " threads");

  // write straight to file without opening a window if -f option provided
  app->init();
  app->load(sceneInfo);
  delete sceneInfo;

  if (w && h)
    app->resize(w, h);

  if (cam_settings != "")
    app->load_camera(cam_settings);

  app->render_to_file(output_file_name, x, y, dx, dy);
  return 0;
}
