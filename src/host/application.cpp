#include "application.h"

#include <unistd.h>

#define msg(s) cerr << "[PathTracer] " << s << endl;

Application::Application(AppConfig config, string sceneFilePath, int w, int h) {
  renderer = new RaytracedRenderer (
    config.pathtracer_accumulate_bounces,
    config.pathtracer_max_ray_depth,
    config.pathtracer_ns_area_light,
    config.debug,
    config.restir
  );
  loader = new Loader(w, h, sceneFilePath);
  screenW = w;
  screenH = h;
  renderer->set_frame_size(w, h);
  renderer->set_camera(&loader->parser.camera);
  renderer->set_frame_size(screenW, screenH);
  renderer->copy_host_device_pt(loader->parser.primitives, 
        loader->parser.vertices, loader->parser.normals, 
        loader->parser.texcoords, loader->parser.tangents, 
        loader->parser.lights, loader->parser.bsdfs, loader->parser.textures);
}

Application::~Application() {
  delete renderer;
  delete loader;
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
  size_t w = 0, h = 0;
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

  w = w ? w : 800; // default width
  h = h ? h : 600; // default height

  // Load scene
  cout << "[PathTracer] Loading scene from " << sceneFilePath << endl;

  // create application
  Application *app = new Application(config, sceneFilePath, w, h);

  if(config.total_image_generated == 1){
    app->render_to_file(output_file_name);
  }else{
    app->render_to_video(output_file_name, config.total_image_generated);
  }
  return 0;
}

void Application::render_to_file(std::string filename) { 
    renderer->render_to_file(filename); 
}

void Application::render_to_video(std::string filename, size_t num_images){
  const double TOTAL_ROTATION = M_PI * 2;
  double angle_per_image = TOTAL_ROTATION / (double)num_images;
  size_t dot_pos = filename.find_last_of('.');
  auto name = filename.substr(0, dot_pos);
  auto dot_extension = filename.substr(dot_pos);

  // UNSUPPORTED: This is a placeholder for video rendering.
  msg("Rendering to video is not supported yet. Please use the single image rendering option.");
}