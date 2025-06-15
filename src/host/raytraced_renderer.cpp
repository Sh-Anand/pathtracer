#include "raytraced_renderer.h"
#include "scene/material.h"
#include "scene/camera.h"
#include "scene/light.h"
#include "scene/geometry.h"

#include <stack>
#include <random>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <cmath>

using std::min;
using std::max;

RaytracedRenderer::RaytracedRenderer(bool accumulate,
                       size_t max_ray_depth, size_t ns_area_light,
                       bool debug,
                       bool restir) {
  pt_host = (PathTracer*) malloc(sizeof(PathTracer));

  pt_host->max_ray_depth = max_ray_depth;                        // Maximum recursion ray depth
  pt_host->ns_area_light = ns_area_light;                        // Number of samples for area light
  pt_host->accumulate = accumulate;


  this->debug = debug;
  this->restir = restir;

  camera = NULL;
}

void RaytracedRenderer::set_camera(Camera *camera) {

  this->camera = camera;
}

void RaytracedRenderer::set_frame_size(size_t width, size_t height) {
  frameBuffer.w = width; 
  frameBuffer.h = height;

  pt_host->sampleBuffer.w = width;
  pt_host->sampleBuffer.h = height;
}


void RaytracedRenderer::set_cuda_camera(){
  pt_host->sampleBuffer.w = frameBuffer.w;
  pt_host->sampleBuffer.h = frameBuffer.h;
  pt_host->camera.c2w = camera->c2w;
  pt_host->camera.pos = camera->pos;
  pt_host->camera.fClip = camera->fClip;
  pt_host->camera.nClip = camera->nClip;
  pt_host->camera.hFov = camera->hFov;
  pt_host->camera.vFov = camera->vFov;
}

void RaytracedRenderer::render_to_file(std::string filename) {
  // launch threads
  DEBUG(debug,
  fprintf(stdout, "[PathTracer] Rendering... "); fflush(stdout);
  )
  
  gpu_raytrace();

  save_image(filename);

  DEBUG(debug, 
  fprintf(stdout, "[PathTracer] Rendering completed.\n");
  )
}

// ACES‐style filmic curve (RRT+ODT approximation)
static inline float aces_film(float x) {
    const float A = 2.51f, B = 0.03f, C = 2.43f, D = 0.59f, E = 0.14f;
    float v = (x*(A*x + B)) / (x*(C*x + D) + E);
    return std::clamp(v, 0.0f, 1.0f);
}

void RaytracedRenderer::save_image(const std::string filename) {
    // Tonemapping parameters (good defaults for a 4K laptop display)
    const float gamma    = 2.2f;    // display gammaz
    const float level    = 1.0f;    // exposure stops: final exposure = 2^level
    const float key      = 0.18f;   // middle gray
    const float wht      = 5.0f;    // white point (scene brightness clamp)

    // Precompute exposure and white‐point normalization
    float exposure   = std::pow(2.0f, level);
    float whiteScale = 1.0f / aces_film(wht * key * exposure);

    auto &buf = frameBuffer;
    int w = buf.w, h = buf.h;

    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs) throw std::runtime_error("Could not open " + filename);

    // PFM header: "PF" = color, width height, scale (neg = little‐endian)
    ofs << "PF\n" << w << " " << h << "\n" << "-1.0\n";
    DEBUG(debug,
    cout << "Writing " << filename << "...\n";
    cout << "Image size " << w << " " << h << "\n";
    )
    // Write pixels bottom→top
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        Vector3D c = buf.pixel[x + y * w].data;  // HDR radiance

        // 1) Exposure & key (middle gray)
        c.x *= exposure * key;
        c.y *= exposure * key;
        c.z *= exposure * key;

        // 2) ACES filmic curve + white‐point normalization
        c.x = aces_film(c.x) * whiteScale;
        c.y = aces_film(c.y) * whiteScale;
        c.z = aces_film(c.z) * whiteScale;

        // 3) Clamp to [0,1]
        c.x = std::clamp(c.x, 0.0f, 1.0f);
        c.y = std::clamp(c.y, 0.0f, 1.0f);
        c.z = std::clamp(c.z, 0.0f, 1.0f);

        // 4) Gamma‐encode to sRGB
        c.x = std::pow(c.x, 1.0f / gamma);
        c.y = std::pow(c.y, 1.0f / gamma);
        c.z = std::pow(c.z, 1.0f / gamma);

        // 5) Write as three floats (R, G, B)
        ofs.write(reinterpret_cast<const char*>(&c.x), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&c.y), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&c.z), sizeof(float));
      }
    }

    ofs.close();
}
