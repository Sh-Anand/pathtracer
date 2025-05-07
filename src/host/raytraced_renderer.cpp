#include "raytraced_renderer.h"
#include "scene/bsdf.h"
#include "scene/camera.h"
#include "scene/light.h"
#include "scene/ray.h"

#include <stack>
#include <random>
#include <algorithm>
#include <sstream>

#include "util/vector3D.h"
#include "util/matrix3x3.h"
#include "util/lodepng.h"

using std::min;
using std::max;

/**
 * Raytraced Renderer is a render controller that in this case.
 * It controls a path tracer to produce an rendered image from the input parameters.
 *
 * A pathtracer with BVH accelerator and BVH visualization capabilities.
 */
RaytracedRenderer::RaytracedRenderer(size_t ns_aa,
                       size_t max_ray_depth, size_t ns_area_light,
                       std::string filename,
                       float lensRadius,
                       float focalDistance,
                       bool debug) {
  pt_host = new PathTracer();

  pt_host->ns_aa = ns_aa;                                        // Number of samples per pixel
  pt_host->max_ray_depth = max_ray_depth;                        // Maximum recursion ray depth
  pt_host->ns_area_light = ns_area_light;                        // Number of samples for area light

  this->lensRadius = lensRadius;
  this->focalDistance = focalDistance;

  this->filename = filename;

  this->debug = debug;

  camera = NULL;
}

/**
 * Destructor.
 * Frees all the internal resources used by the pathtracer.
 */
RaytracedRenderer::~RaytracedRenderer() {
  delete pt_host;
}

/**
 * This DOES NOT take ownership of the camera, and doesn't delete it ever.
 * \param camera the camera to use in rendering
 */
void RaytracedRenderer::set_camera(Camera *camera) {

  camera->focalDistance = focalDistance;
  camera->lensRadius = lensRadius;
  this->camera = camera;
}

/**
 * Sets the pathtracer's frame size.
 * \param width width of the frame
 * \param height height of the frame
 */
void RaytracedRenderer::set_frame_size(size_t width, size_t height) {
  frameBuffer.resize(width, height);

  pt_host->sampleBuffer.resize(width, height);
}

bool RaytracedRenderer::has_valid_configuration() {
  return camera;
}

void RaytracedRenderer::set_cuda_camera(){
  pt_host->sampleBuffer.resize(frameBuffer.w, frameBuffer.h);
  pt_host->camera = CudaCamera(*camera);
}

void RaytracedRenderer::render_to_file(std::string filename, size_t x, size_t y, size_t dx, size_t dy) {
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

void RaytracedRenderer::save_image(std::string filename) {

  ImageBuffer* buffer = &frameBuffer;
  if (filename == "") {
    time_t rawtime;
    time (&rawtime);

    time_t t = time(nullptr);
    tm *lt = localtime(&t);
    std::stringstream ss;
    ss << this->filename << "_screenshot_" << lt->tm_mon+1 << "-" << lt->tm_mday << "_" 
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
