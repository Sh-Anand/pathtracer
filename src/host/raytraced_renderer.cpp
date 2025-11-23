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
#include <utility>

#include <OpenEXR/ImfOutputFile.h>
#include <OpenEXR/ImfHeader.h>
#include <OpenEXR/ImfChannelList.h>
#include <OpenEXR/ImfFrameBuffer.h>
#include <OpenEXR/ImfIO.h>

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
    const auto &buf = frameBuffer;
    int W = int(buf.w), H = int(buf.h);

    // split filename so we can emit rgb/albedo/normal separately
    auto split_ext = [](const std::string &path) -> std::pair<std::string, std::string> {
        auto pos = path.find_last_of('.');
        if (pos == std::string::npos) return {path, ".exr"};
        return {path.substr(0, pos), path.substr(pos)};
    };
    auto [base, ext] = split_ext(filename);

    // Prepare buffer pointers for flipped write
    const size_t pixelSize = sizeof(PixelData);
    const size_t lineSize  = pixelSize * W;
    char *basePtr = reinterpret_cast<char*>(buf.pixel);

    auto write_three_channel = [&](const std::string &out_name,
                                   size_t c0, size_t c1, size_t c2) {
        Imf::Header header(W, H);
        auto &ch = header.channels();
        ch.insert("R",  Imf::Channel(Imf::FLOAT));
        ch.insert("G",  Imf::Channel(Imf::FLOAT));
        ch.insert("B",  Imf::Channel(Imf::FLOAT));

        Imf::OutputFile file(out_name.c_str(), header);

        Imf::FrameBuffer fb;
        auto insertSliceFlipped = [&](const char *name, size_t offset){
          char *start = basePtr + (H - 1) * lineSize + offset;
          fb.insert(name, Imf::Slice(
            Imf::FLOAT,
            start,
            pixelSize,
            -lineSize
          ));
        };

        insertSliceFlipped("R", c0);
        insertSliceFlipped("G", c1);
        insertSliceFlipped("B", c2);

        file.setFrameBuffer(fb);
        file.writePixels(H);
    };

    write_three_channel(base + "_rgb" + ext,
                        offsetof(PixelData, data)   + sizeof(float)*0,
                        offsetof(PixelData, data)   + sizeof(float)*1,
                        offsetof(PixelData, data)   + sizeof(float)*2);

    write_three_channel(base + "_alb" + ext,
                        offsetof(PixelData, albedo) + sizeof(float)*0,
                        offsetof(PixelData, albedo) + sizeof(float)*1,
                        offsetof(PixelData, albedo) + sizeof(float)*2);

    write_three_channel(base + "_nrm" + ext,
                        offsetof(PixelData, normal) + sizeof(float)*0,
                        offsetof(PixelData, normal) + sizeof(float)*1,
                        offsetof(PixelData, normal) + sizeof(float)*2);
}
