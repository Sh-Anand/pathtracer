#include "raytraced_renderer.h"
#include "bsdf.h"
#include "pathtracer/ray.h"

#include <stack>
#include <random>
#include <algorithm>
#include <sstream>

#include "CGL/CGL.h"
#include "CGL/vector3D.h"
#include "CGL/matrix3x3.h"
#include "CGL/lodepng.h"

#include "scene/sphere.h"
#include "scene/triangle.h"
#include "scene/light.h"

using namespace CGL::SceneObjects;

using std::min;
using std::max;

namespace CGL {

/**
 * Raytraced Renderer is a render controller that in this case.
 * It controls a path tracer to produce an rendered image from the input parameters.
 *
 * A pathtracer with BVH accelerator and BVH visualization capabilities.
 */
RaytracedRenderer::RaytracedRenderer(size_t ns_aa,
                       size_t max_ray_depth, bool isAccumBounces, size_t ns_area_light,
                       size_t ns_diff, size_t ns_glsy, size_t ns_refr,
                       size_t num_threads,
                       size_t samples_per_batch,
                       float max_tolerance,
                       HDRImageBuffer* envmap,
                       bool direct_hemisphere_sample,
                       string filename,
                       double lensRadius,
                       double focalDistance) {
  pt = new PathTracer();

  pt->ns_aa = ns_aa;                                        // Number of samples per pixel
  pt->max_ray_depth = max_ray_depth;                        // Maximum recursion ray depth
  pt->isAccumBounces = isAccumBounces;                      // Accumulate Bounces Along Path
  pt->ns_area_light = ns_area_light;                        // Number of samples for area light
  pt->ns_diff = ns_diff;                                    // Number of samples for diffuse surface
  pt->ns_glsy = ns_diff;                                    // Number of samples for glossy surface
  pt->ns_refr = ns_refr;                                    // Number of samples for refraction
  pt->samplesPerBatch = samples_per_batch;                  // Number of samples per batch
  pt->maxTolerance = max_tolerance;                         // Maximum tolerance for early termination
  pt->direct_hemisphere_sample = direct_hemisphere_sample;  // Whether to use direct hemisphere sampling vs. Importance Sampling

  this->lensRadius = lensRadius;
  this->focalDistance = focalDistance;

  this->filename = filename;

  if (envmap) {
    pt->envLight = new EnvironmentLight(envmap);
  } else {
    pt->envLight = NULL;
  }

  bvh = NULL;
  scene = NULL;
  camera = NULL;

  imageTileSize = 32;                     // Size of the rendering tile.
  numWorkerThreads = num_threads;         // Number of threads
  workerThreads.resize(numWorkerThreads);

}

/**
 * Destructor.
 * Frees all the internal resources used by the pathtracer.
 */
RaytracedRenderer::~RaytracedRenderer() {

  delete bvh;
  delete pt;

}

/**
 * This DOES take ownership of the scene, and therefore deletes it if a new
 * scene is later passed in.
 * \param scene pointer to the new scene to be rendered
 */
void RaytracedRenderer::set_scene(Scene *scene) {
  if (pt->envLight != nullptr) {
    scene->lights.push_back(pt->envLight);
  }

  this->scene = scene;
  build_accel();
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
  frame_w = width;
  frame_h = height;

  frameBuffer.resize(width, height);

  pt->set_frame_size(width, height);
}

bool RaytracedRenderer::has_valid_configuration() {
  return scene && camera;
}

void RaytracedRenderer::start_tiled_processing(void (PathTracer::*func)(size_t, size_t)) {
  size_t width = frameBuffer.w, height = frameBuffer.h;
  
  workQueue.clear();  
  frameBuffer.clear();
  num_tiles_w = width / imageTileSize + 1;
  num_tiles_h = height / imageTileSize + 1;
  tilesTotal = num_tiles_w * num_tiles_h;
  tilesDone = 0;
  tile_samples.resize(num_tiles_w * num_tiles_h);
  memset(&tile_samples[0], 0, num_tiles_w * num_tiles_h * sizeof(int));

  // populate the tile work queue
  for (size_t y = 0; y < height; y += imageTileSize) {
      for (size_t x = 0; x < width; x += imageTileSize) {
          workQueue.put_work(WorkItem(x, y, imageTileSize, imageTileSize, func));
      }
  }
  for (int i=0; i<numWorkerThreads; i++) {
      workerThreads[i] = new std::thread(&RaytracedRenderer::worker_thread, this);
  }

  // wait for all threads to finish
  for (int i=0; i<numWorkerThreads; i++) {
      workerThreads[i]->join();
      delete workerThreads[i];
  }
}

void RaytracedRenderer::start_raytracing() {
  pt->clear();
  pt->set_frame_size(frameBuffer.w, frameBuffer.h);

  pt->bvh = bvh;
  pt->camera = camera;
  pt->scene = scene;

  bvh->total_isects = 0; bvh->total_rays = 0;
  // launch threads
  fprintf(stdout, "[PathTracer] Rendering... "); fflush(stdout);

  start_tiled_processing(&PathTracer::raytrace_pixel);
}

void RaytracedRenderer::render_to_file(string filename, size_t x, size_t y, size_t dx, size_t dy) {
  start_raytracing();
  start_tiled_processing(&PathTracer::temporal_resampling);
  start_tiled_processing(&PathTracer::spatial_resampling);
  start_tiled_processing(&PathTracer::render_final_sample);
  pt->write_to_framebuffer(frameBuffer, 0, 0, frameBuffer.w, frameBuffer.h);
  save_image(filename);
  fprintf(stdout, "[PathTracer] Job completed.\n");
}


void RaytracedRenderer::build_accel() {

  // collect primitives //
  fprintf(stdout, "[PathTracer] Collecting primitives... "); fflush(stdout);
  timer.start();
  vector<Primitive *> primitives;
  for (SceneObject *obj : scene->objects) {
    const vector<Primitive *> &obj_prims = obj->get_primitives();
    primitives.reserve(primitives.size() + obj_prims.size());
    primitives.insert(primitives.end(), obj_prims.begin(), obj_prims.end());
  }
  timer.stop();
  fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

  // build BVH //
  fprintf(stdout, "[PathTracer] Building BVH from %lu primitives... ", primitives.size()); 
  fflush(stdout);
  timer.start();
  bvh = new BVHAccel(primitives);
  timer.stop();
  fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());
}

/**
 * Raytrace a tile of the scene and update the frame buffer. Is run
 * in a worker thread.
 */
void RaytracedRenderer::process_tile(int tile_x, int tile_y,
                               int tile_w, int tile_h, void (PathTracer::*func)(size_t, size_t)) {
  size_t w = frame_w;
  size_t h = frame_h;

  size_t tile_start_x = tile_x;
  size_t tile_start_y = tile_y;

  size_t tile_end_x = std::min(tile_start_x + tile_w, w);
  size_t tile_end_y = std::min(tile_start_y + tile_h, h);

  size_t tile_idx_x = tile_x / imageTileSize;
  size_t tile_idx_y = tile_y / imageTileSize;
  size_t num_samples_tile = tile_samples[tile_idx_x + tile_idx_y * num_tiles_w];

  for (size_t y = tile_start_y; y < tile_end_y; y++) {
    for (size_t x = tile_start_x; x < tile_end_x; x++) {
      // generate samples
      (pt->*func)(x, y);
    }
  }

  tile_samples[tile_idx_x + tile_idx_y * num_tiles_w] += 1;

//  pt->write_to_framebuffer(frameBuffer, tile_start_x, tile_start_y, tile_end_x, tile_end_y);
}

void RaytracedRenderer::worker_thread() {

  Timer timer;
  timer.start();

  WorkItem work;
  while (workQueue.try_get_work(&work)) {
    process_tile(work.tile_x, work.tile_y, work.tile_w, work.tile_h, work.func);
    { 
      ++tilesDone;
    }
  }
}

void RaytracedRenderer::save_image(string filename) {

  ImageBuffer* buffer = &frameBuffer;
  if (filename == "") {
    time_t rawtime;
    time (&rawtime);

    time_t t = time(nullptr);
    tm *lt = localtime(&t);
    stringstream ss;
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

  fprintf(stderr, "[PathTracer] Saving to file: %s... ", filename.c_str());
  lodepng::encode(filename, (unsigned char*) frame_out, w, h);
  fprintf(stderr, "Done!\n");
  
  delete[] frame_out;
}

}  // namespace CGL
