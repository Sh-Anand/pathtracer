#include "application.h"
#include "bvh_host.h"
#include "scene/serializer/text_serializer.h"
#include "scene/serializer/binary_serializer.h"

#include <unistd.h>

#define msg(s) cerr << "[PathTracer] " << s << endl;

Application::Application(AppConfig config, string sceneFilePath, string output_file_name, int serializer_type, int w, int h) {
  loader = new Loader(w, h, sceneFilePath);
  screenW = w;
  screenH = h;

  // write scene to file
  // build BVH //
  DEBUG(config.debug, 
  fprintf(stdout, "[PathTracer] Building BVH from %lu primitives... ", loader->parser.primitives.size()); 
  fflush(stdout);
  )
  std::chrono::time_point<std::chrono::steady_clock> t0 = std::chrono::steady_clock::now();

  std::vector<BVHNode> nodes;

  create_bvh(loader->parser.primitives, loader->parser.vertices, loader->parser.normals, loader->parser.texcoords, loader->parser.tangents, nodes, config.debug, 2);
  std::chrono::time_point<std::chrono::steady_clock> t1 = std::chrono::steady_clock::now();
  DEBUG(config.debug, 
  fprintf(stdout, "Done! (%.4f sec)\n", (std::chrono::duration<float>(t1 - t0)).count());
  )

  /* Calculate total size for debugging */
  size_t const_size = sizeof(config.pathtracer_max_ray_depth) +
                      sizeof(config.pathtracer_ns_area_light) +
                      sizeof(config.pathtracer_accumulate_bounces);
  size_t cam_size = sizeof(CudaCamera);
  size_t light_size = sizeof(CudaLight) * loader->parser.lights.size();
  size_t bsdf_size = sizeof(CudaBSDF) * loader->parser.bsdfs.size();
  size_t tex_size = 0;
  for (const auto& tex : loader->parser.textures) {
    tex_size += sizeof(tex.width) + sizeof(tex.height) + sizeof(tex.channels) + tex.width * tex.height * tex.channels;
  }
  size_t prim_size = sizeof(CudaPrimitive) * loader->parser.primitives.size();
  size_t vertex_size = sizeof(Vector3D) * loader->parser.vertices.size();
  size_t normal_size = sizeof(Vector3D) * loader->parser.normals.size();
  size_t texcoord_size = sizeof(Vector2D) * loader->parser.texcoords.size();
  size_t tangent_size = sizeof(Vector4D) * loader->parser.tangents.size();
  size_t bvh_size = sizeof(BVHNode) * nodes.size();
  size_t rand_state_size = sizeof(RNGState) * w * h; // for each pixel
  size_t image_buffer_size = sizeof(PixelData) * w * h; // for each pixel
  size_t samples_size = sizeof(Sample) * w * h; // for each pixel
  size_t reservoirs_size = sizeof(Reservoir) * w * h; // for each pixel
  size_t total_size = cam_size + light_size + bsdf_size + tex_size + prim_size + vertex_size + normal_size + texcoord_size + tangent_size + bvh_size + rand_state_size + image_buffer_size + const_size + samples_size + reservoirs_size;

  DEBUG(config.debug,
  cout << "Camera size: " << cam_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "Lights num: " << loader->parser.lights.size() << ", size: " << light_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "BSDFs num: " << loader->parser.bsdfs.size() << ", size: " << bsdf_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "Textures num: " << loader->parser.textures.size() << ", size: " << tex_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "Primitives num: " << loader->parser.primitives.size() << ", size: " << prim_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "Vertices num: " << loader->parser.vertices.size() << ", size: " << vertex_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "Normals num: " << loader->parser.normals.size() << ", size: " << normal_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "Texcoords num: " << loader->parser.texcoords.size() << ", size: " << texcoord_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "Tangents num: " << loader->parser.tangents.size() << ", size: " << tangent_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "BVH nodes num: " << nodes.size() << ", size: " << bvh_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "RNG states num: " << w * h << ", size: " << rand_state_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "Image buffer num: " << w * h << ", size: " << image_buffer_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "Constants size: " << const_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "Samples num: " << w * h << ", size: " << samples_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "Reservoirs num: " << w * h << ", size: " << reservoirs_size / (1024.0 * 1024.0) << " MB" << endl;
  cout << "Serializer type: " << serializer_type << endl;
  cout << "Total size: " << total_size / (1024.0 * 1024.0) << " MB" << endl;
  )

  bool CUDA = false;
  size_t dot_pos = output_file_name.find_last_of('.');
  if (dot_pos != std::string::npos) {
    std::string extension = output_file_name.substr(dot_pos);
    if (extension == ".cu") {
      CUDA = true;
      DEBUG(config.debug, cout << "CUDA mode enabled (detected .cu extension)" << endl;)
    }
  }

  Serializer *serializer;
  if (serializer_type == 0) {
    serializer = new TextSerializer();
  } else if (serializer_type == 1) {
    serializer = new BinarySerializer();
  } else {
    std::cerr << "Unknown serializer type: " << serializer_type << std::endl;
    return;
  }
  serializer->CUDA = CUDA;
  FILE* f = fopen(output_file_name.c_str(), "w");
  if (!f) { perror("open baked_data.cu"); return; }

  /* 1) Headers */
  serializer->serialize_headers(f);
  /* 1.6) Camera */
  serializer->serialize(f, loader->parser.camera);
  /* 2) Lights */
  serializer->serialize_lights(f, loader->parser.lights);
  /* 3) BSDFs */
  serializer->serialize_bsdfs(f, loader->parser.bsdfs);
  /* 4) Textures */
  serializer->serialize_textures(f, loader->parser.textures);
  /* 5) Primitives */
  serializer->serialize_primitives(f, loader->parser.primitives);
  /* 6) Vertices */
  serializer->serialize(f, loader->parser.vertices, "vertices");
  /* 7) Normals */
  serializer->serialize(f, loader->parser.normals, "normals");
  /* 8) Texcoords */
  serializer->serialize(f, loader->parser.texcoords, "texcoords");
  /* 9) Tangents */
  serializer->serialize(f, loader->parser.tangents, "tangents");
  /* 10) BVH */
  serializer->serialize_bvh_nodes(f, nodes);
  /* 11) RNG states */
  serializer->serializeEmpty<RNGState>(f, "rand_states", "RNGState", w*h);
  /* 12) Image buffer */
  serializer->serializeEmpty<PixelData>(f, "pixel", "PixelData", w*h);
  /* 13) Samples */
  serializer->serializeEmpty<Sample>(   f, "initialSampleBuffer", "Sample", w*h);
  /* 13.1) SampleMetadata */
  serializer->serializeEmpty<SampleMetadata>(f,"initialSampleMetadataBuffer", "SampleMetadata", w*h);
  /* 14) Reservoirs */
  serializer->serializeEmpty<Reservoir>(f, "temporalReservoirBufferGI", "Reservoir", w*h);
  /* 15) Rays_traced */
  serializer->serializeEmpty<uint8_t>(f, "rays_traced", "uint8_t", w*h);
  /* 16) Consts */
  serializer->serialize_const(f, "w", w);
  serializer->serialize_const(f, "h", h);
  serializer->serialize_const(f, "max_ray_depth", config.pathtracer_max_ray_depth);
  serializer->serialize_const(f, "ns_area_light", config.pathtracer_ns_area_light);
  serializer->serialize_const(f, "accumulate", config.pathtracer_accumulate_bounces);
  serializer->serialize_const(f, "restir", (int)config.restir);

  fclose(f);

  DEBUG(config.debug,
  std::cout << "Done cooking..." << std::endl;
  )

}

Application::~Application() {
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
  printf("  -S  <INT>        Type of serializer to use (0 - text, 1 - binary)\n");
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
  string output_file_name = "../bakes/baked_data.cu";
  string sceneFilePath;
  int serializer_type = 0;

  while ((opt = getopt(argc, argv, "l:m:o:h:f:r:d:R:g:S:")) !=
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
    case 'S':
      serializer_type = atoi(optarg);
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
  Application *app = new Application(config, sceneFilePath, output_file_name, serializer_type, w, h);
  return 0;
}