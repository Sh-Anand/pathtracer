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
  frameBuffer.resize(width, height);

  pt->set_frame_size(width, height);
}

bool RaytracedRenderer::has_valid_configuration() {
  return scene && camera;
}

void RaytracedRenderer::render_to_file(string filename, size_t x, size_t y, size_t dx, size_t dy) {
  pt->clear();
  pt->set_frame_size(frameBuffer.w, frameBuffer.h);

  pt->bvh = bvh;
  pt->camera = camera;
  pt->scene = scene;

  bvh->total_isects = 0; bvh->total_rays = 0;
  // launch threads
  fprintf(stdout, "[PathTracer] Rendering... "); fflush(stdout);

  // loop over all pixels opposite direction
  for (size_t i = 0; i < frameBuffer.w; ++i)
    for (size_t j = 0; j < frameBuffer.h; ++j)
      pt->raytrace_pixel(i, j);

  for (size_t i = 0; i < frameBuffer.w; ++i)
    for (size_t j = 0; j < frameBuffer.h; ++j)
      pt->temporal_resampling(i, j);

  for (size_t i = 0; i < frameBuffer.w; ++i)
    for (size_t j = 0; j < frameBuffer.h; ++j)
      pt->spatial_resampling(i, j);

  for (size_t i = 0; i < frameBuffer.w; ++i)
    for (size_t j = 0; j < frameBuffer.h; ++j)
      pt->render_final_sample(i, j);

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
