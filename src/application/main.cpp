#include "main.h"
#include <cstdint>
typedef uint32_t gid_t;
#include <iostream>
#include <unistd.h>

#include "glTF.h"

#include "util/lodepng.h"

using namespace std;

#define msg(s) cerr << "[PathTracer] " << s << endl;

void usage(const char *binaryName) {
  printf("Usage: %s [options] <scenefile>\n", binaryName);
  printf("Program Options:\n");
  printf("  -s  <INT>        Number of camera rays per pixel\n");
  printf("  -l  <INT>        Number of samples per area light\n");
  printf("  -g  <INT>        Number of total image generated\n");
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

void Main::parse_scene(string sceneFilePath) {
  string sceneFile = sceneFilePath.substr(sceneFilePath.find_last_of('/') + 1);
  sceneFile = sceneFile.substr(0, sceneFile.find(".dae"));
  pathtracer_filename = sceneFile;

  parse_glTF(sceneFilePath, screenW, screenH,
        vertices, normals, texcoords, tangents,
        primitives, lights, bsdfs, textures,
        camera);
}

void Main::save_image(std::string filename) {

  ImageBuffer* buffer = &frameBuffer;
  if (filename == "") {
    time_t rawtime;
    time (&rawtime);

    time_t t = time(nullptr);
    tm *lt = localtime(&t);
    std::stringstream ss;
    ss << filename << "_screenshot_" << lt->tm_mon+1 << "-" << lt->tm_mday << "_" 
      << lt->tm_hour << "-" << lt->tm_min << "-" << lt->tm_sec << ".png";
    filename = ss.str();  
  }

  uint32_t* frame = &buffer->data[0];
  size_t w = buffer->w;
  size_t h = buffer->h;
  uint32_t* frame_out = new uint32_t[w * h];
  for(size_t i = 0; i < h; ++i) {
    memcpy(frame_out + i * w, frame + (h - i - 1) * w, 4 * w);
  }
  
  for (size_t i = 0; i < w * h; ++i) {
    frame_out[i] |= 0xFF000000;
  }

  DEBUG(debug, fprintf(stderr, "[PathTracer] Saving to file: %s... ", filename.c_str());)
  lodepng::encode(filename, (unsigned char*) frame_out, w, h);
  DEBUG(debug, fprintf(stderr, "Done!\n");)
  
  delete[] frame_out;
}

void Main::render_to_file(std::string filename, size_t x, size_t y, size_t dx, size_t dy) { 
  frameBuffer.resize(screenW, screenH);
  pt_host->set_frame_size(screenW, screenH);
  pt_host->camera = CudaCamera(camera);
  build_accel(primitives, vertices, normals, texcoords, tangents, lights, bsdfs, textures);

  DEBUG(debug, fprintf(stderr, "[PathTracer] Rendering to file: %s... ", filename.c_str());)
  gpu_raytrace();
  save_image(filename); 
  DEBUG(debug, fprintf(stderr, "Done!\n");)
}

void Main::render_to_video(std::string filename, size_t x, size_t y, size_t dx, size_t dy, size_t num_images){
  const double TOTAL_ROTATION = M_PI * 2;
  double angle_per_image = TOTAL_ROTATION / (double)num_images;
  size_t dot_pos = filename.find_last_of('.');
  auto name = filename.substr(0, dot_pos);
  auto dot_extension = filename.substr(dot_pos);

  frameBuffer.resize(screenW, screenH);
  pt_host->set_frame_size(screenW, screenH);
  build_accel(primitives, vertices, normals, texcoords, tangents, lights, bsdfs, textures);  
  for(size_t i = 0; i < num_images; ++i){
    std::ostringstream oss;
    oss << std::setw(4) << std::setfill('0') << i;
    auto filename_per_image = name + oss.str() + dot_extension;
    camera.rotate_by(0, angle_per_image);
    pt_host->camera = CudaCamera(camera);
    update_camera();
    gpu_raytrace();
    save_image(filename_per_image); 
  }
}

int Main::main(int argc, char **argv) {

  // get the options
  int opt;
  size_t w = 0, h = 0, x = -1, y = 0, dx = 0, dy = 0;
  string output_file_name;
  string sceneFilePath;
  while ((opt = getopt(argc, argv, "s:l:t:m:e:h:f:r:d:p:g:")) !=
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
      pathtracer_ns_aa = atoi(optarg);
      break;
    case 'l':
      pathtracer_ns_area_light = atoi(optarg);
      break;
    case 'g':
      total_image_generated = atoi(optarg);
      break;
    case 't':
      break;
    case 'm':
      pathtracer_max_ray_depth = atoi(optarg);
      break;
    case 'e':
      std::cout << "[PathTracer] Loading environment map " << optarg
                << std::endl;
      std::cout << "Environment map not implemented" << std::endl;
      std::abort();
      break;
    case 'd':
      debug = atoi(optarg) > 0;
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

  // set cam
  // init camera
  CameraInfo cameraInfo;
  cameraInfo.hFov = 50;
  cameraInfo.vFov = 35;
  cameraInfo.nClip = 0.01;
  cameraInfo.fClip = 100;
  camera.configure(cameraInfo, screenW, screenH);

  parse_scene(sceneFilePath);

  bsdfs = bsdfs;
  lights = lights;
  textures = textures;
  vertices = vertices;
  normals = normals;
  texcoords = texcoords;
  primitives = primitives;
  tangents = tangents;

  if (w && h) {
    screenW = w;
    screenH = h;
    camera.set_screen_size(w, h);
    camera = camera;
  }

  if(total_image_generated == 1){
    render_to_file(output_file_name, x, y, dx, dy);
  }else{
    render_to_video(output_file_name, x, y, dx, dy, total_image_generated);
  }
  return 0;
}

int main(int argc, char **argv) {
  Main main;
  return main.main(argc, argv);
}
