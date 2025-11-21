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

    // 1) Build the EXR header
    Imf::Header header(W, H);
    auto &ch = header.channels();
    ch.insert("R",  Imf::Channel(Imf::FLOAT));
    ch.insert("G",  Imf::Channel(Imf::FLOAT));
    ch.insert("B",  Imf::Channel(Imf::FLOAT));
    ch.insert("ALBEDO_R", Imf::Channel(Imf::FLOAT));
    ch.insert("ALBEDO_G", Imf::Channel(Imf::FLOAT));
    ch.insert("ALBEDO_B", Imf::Channel(Imf::FLOAT));
    ch.insert("NX", Imf::Channel(Imf::FLOAT));
    ch.insert("NY", Imf::Channel(Imf::FLOAT));
    ch.insert("NZ", Imf::Channel(Imf::FLOAT));

    // 2) Create the file
    Imf::OutputFile file(filename.c_str(), header);

    // 3) Prepare the frame buffer with flipped Y
    Imf::FrameBuffer frameBufferExr;
    const size_t pixelSize = sizeof(PixelData);
    const size_t lineSize  = pixelSize * W;

    // We want buf.pixel[y=0] (first row in memory) → EXR scanline H-1,
    // and buf.pixel[y=H-1] → scanline 0.  So:
    char *base = reinterpret_cast<char*>(buf.pixel);

    auto insertSliceFlipped = [&](const char *name, size_t offset){
      // start pointer = beginning of *last* row + offset
      char *start = base + (H - 1) * lineSize + offset;
      // xStride = +pixelSize (move right in memory), yStride = -lineSize (move up)
      frameBufferExr.insert(name, Imf::Slice(
        Imf::FLOAT,
        start,
        pixelSize,
        -lineSize
      ));
    };

    // data.x/y/z → R/G/B
    insertSliceFlipped("R", offsetof(PixelData, data)   + sizeof(float)*0);
    insertSliceFlipped("G", offsetof(PixelData, data)   + sizeof(float)*1);
    insertSliceFlipped("B", offsetof(PixelData, data)   + sizeof(float)*2);
    // albedo.x/y/z → ALBEDO_R/G/B
    insertSliceFlipped("ALBEDO_R", offsetof(PixelData, albedo) + sizeof(float)*0);
    insertSliceFlipped("ALBEDO_G", offsetof(PixelData, albedo) + sizeof(float)*1);
    insertSliceFlipped("ALBEDO_B", offsetof(PixelData, albedo) + sizeof(float)*2);
    // normal.x/y/z → NX/NY/NZ
    insertSliceFlipped("NX", offsetof(PixelData, normal) + sizeof(float)*0);
    insertSliceFlipped("NY", offsetof(PixelData, normal) + sizeof(float)*1);
    insertSliceFlipped("NZ", offsetof(PixelData, normal) + sizeof(float)*2);

    // 4) Write out all scanlines
    file.setFrameBuffer(frameBufferExr);
    file.writePixels(H);
}

