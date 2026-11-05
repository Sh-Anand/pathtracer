// gluon host: loads a scene, builds its BVH, uploads it and launches the path
// tracer through gluon's driver, then writes the rendered image as a PFM.
#include <unistd.h>

#include <cstdio>
#include <vector>

#include "rad.h"

#include "host/parser/loader.h" // before bvh_host.h, which relies on its includes
#include "host/bvh_host.h"
#include "target/gluon/scene.h"
#include "util/reservoir.h"

// copy v to fresh device memory and return its address.  Synchronous, since the
// DMA reads host memory only when the command processor runs it.
template <class T>
static uint32_t upload(const std::vector<T> &v) {
  if (v.empty())
    return 0;
  void *d;
  radMalloc(&d, v.size() * sizeof(T));
  radMemcpyAsync(d, (void *)v.data(), v.size() * sizeof(T), radMemcpyDir_H2D);
  radStreamSynchronize();
  return (uint32_t)(uintptr_t)d;
}

// zeroed device buffer of n elements
template <class T>
static uint32_t zeros(size_t n) {
  return upload(std::vector<T>(n));
}

// PFM: little-endian RGB floats, bottom row first
static void write_pfm(const char *path, const std::vector<PixelData> &pixels, int w, int h) {
  FILE *f = fopen(path, "wb");
  if (!f) {
    perror(path);
    exit(1);
  }
  fprintf(f, "PF\n%d %d\n-1.0\n", w, h);
  for (const PixelData &p : pixels)
    fwrite(&p.data, sizeof(float), 3, f);
  fclose(f);
}

static void usage(const char *bin) {
  fprintf(stderr,
          "Usage: %s [options] <scenefile>\n"
          "  -r <W> <H>  image size (default 16 16)\n"
          "  -l <INT>    samples per area light (default 1)\n"
          "  -m <INT>    maximum ray depth (default 1)\n"
          "  -o <INT>    accumulate bounces (default 1)\n"
          "  -R <INT>    enable ReSTIR-GI (default 0)\n"
          "  -f <FILE>   output PFM (default pathtracer.pfm)\n",
          bin);
  exit(1);
}

int main(int argc, char **argv) {
  int w = 16, h = 16, ns_area_light = 1, max_ray_depth = 1, accumulate = 1, restir = 0;
  const char *out = "pathtracer.pfm";
  int opt;
  while ((opt = getopt(argc, argv, "r:l:m:o:R:f:")) != -1) {
    switch (opt) {
    case 'r': // -r W H
      if (optind >= argc)
        usage(argv[0]);
      w = atoi(optarg);
      h = atoi(argv[optind++]);
      break;
    case 'l': ns_area_light = atoi(optarg); break;
    case 'm': max_ray_depth = atoi(optarg); break;
    case 'o': accumulate = atoi(optarg); break;
    case 'R': restir = atoi(optarg) > 0; break;
    case 'f': out = optarg; break;
    default: usage(argv[0]);
    }
  }
  if (optind != argc - 1)
    usage(argv[0]);

  Loader loader(w, h, argv[optind]);
  Parser &p = loader.parser;
  std::vector<BVHNode> nodes;
  create_bvh(p.primitives, p.vertices, p.normals, p.texcoords, p.tangents, nodes, false, 2);

  // texture pixels first, then the table pointing at them
  std::vector<GluonTexture> textures;
  for (const CudaTexture &t : p.textures) {
    std::vector<uint8_t> data(t.data, t.data + (size_t)t.width * t.height * t.channels);
    textures.push_back({t.width, t.height, t.channels, upload(data)});
  }

  size_t n = (size_t)w * h;
  GluonScene s{};
  s.camera = {p.camera.hFov, p.camera.vFov, p.camera.nClip, p.camera.fClip, p.camera.pos, p.camera.c2w};
  s.lights = upload(p.lights);
  s.bsdfs = upload(p.bsdfs);
  s.textures = upload(textures);
  s.primitives = upload(p.primitives);
  s.vertices = upload(p.vertices);
  s.normals = upload(p.normals);
  s.texcoords = upload(p.texcoords);
  s.tangents = upload(p.tangents);
  s.nodes = upload(nodes);
  s.rand_states = zeros<RNGState>(n);
  s.samples = zeros<Sample>(n);
  s.sample_metadata = zeros<SampleMetadata>(n);
  s.reservoirs = zeros<Reservoir>(n);
  s.pixel = zeros<PixelData>(n);
  s.rays_traced = zeros<uint8_t>(n);
  s.num_lights = p.lights.size();
  s.w = w;
  s.h = h;
  s.max_ray_depth = max_ray_depth;
  s.ns_area_light = ns_area_light;
  s.accumulate = accumulate;
  s.restir = restir;

  radParamBuf params;
  params.push((void *)(uintptr_t)upload(std::vector<GluonScene>{s}));
  radDim3 grid{(unsigned)(w + GLUON_BLOCK - 1) / GLUON_BLOCK, (unsigned)(h + GLUON_BLOCK - 1) / GLUON_BLOCK, 1};
  radDim3 block{GLUON_BLOCK, GLUON_BLOCK, 1};
  printf("[PathTracer] %dx%d, %zu primitives, %zu BVH nodes, %zu lights\n",
         w, h, p.primitives.size(), nodes.size(), p.lights.size());
  radKernelLaunch("pathtrace_kernel", grid, block, 0, &params);
  if (restir)
    radKernelLaunch("spatial_resampling_kernel", grid, block, 0, &params);

  std::vector<PixelData> pixels(n);
  radMemcpyAsync(pixels.data(), (void *)(uintptr_t)s.pixel, n * sizeof(PixelData), radMemcpyDir_D2H);
  radStreamSynchronize();
  write_pfm(out, pixels, w, h);
  printf("[PathTracer] wrote %s\n", out);
  return 0;
}
