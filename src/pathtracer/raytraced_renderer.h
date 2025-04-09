#ifndef CGL_RAYTRACER_H
#define CGL_RAYTRACER_H

#include <stack>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <algorithm>

#include "CGL/timer.h"

#include "scene/bvh.h"
#include "pathtracer/camera.h"
#include "pathtracer/sampler.h"
#include "util/image.h"
#include "util/work_queue.h"
#include "pathtracer/intersection.h"

#include "scene/scene.h"
using CGL::SceneObjects::Scene;

#include "scene/environment_light.h"
using CGL::SceneObjects::EnvironmentLight;

using CGL::SceneObjects::BVHNode;
using CGL::SceneObjects::BVHAccel;

#include "pathtracer.h"

namespace CGL {

/**
 * A pathtracer with BVH accelerator and BVH visualization capabilities.
 * It is always in exactly one of the following states:
 * -> INIT: is missing some data needed to be usable, like a camera or scene.
 * -> READY: fully configured, but not rendering.
 * -> VISUALIZE: visualizatiNG BVH aggregate.
 * -> RENDERING: rendering a scene.
 * -> DONE: completed rendering a scene.
 */
class RaytracedRenderer {
public:

  /**
   * Default constructor.
   * Creates a new pathtracer instance.
   */
  RaytracedRenderer(size_t ns_aa = 1, 
             size_t max_ray_depth = 4, bool is_accumulate_bounces =false, size_t ns_area_light = 1,
             size_t ns_diff = 1, size_t ns_glsy = 1, size_t ns_refr = 1,
             size_t num_threads = 1,
             size_t samples_per_batch = 32,
             float max_tolerance = 0.05f,
             HDRImageBuffer* envmap = NULL,
             bool direct_hemisphere_sample = false,
             string filename = "",
             double lensRadius = 0.25,
             double focalDistance = 4.7);

  /**
   * Destructor.
   * Frees all the internal resources used by the pathtracer.
   */
  ~RaytracedRenderer();

  /**
   * If in the INIT state, configures the pathtracer to use the given scene. If
   * configuration is done, transitions to the READY state.
   * This DOES take ownership of the scene, and therefore deletes it if a new
   * scene is later passed in.
   * \param scene pointer to the new scene to be rendered
   */
  void set_scene(Scene* scene);

  /**
   * If in the INIT state, configures the pathtracer to use the given camera. If
   * configuration is done, transitions to the READY state.
   * This DOES NOT take ownership of the camera, and doesn't delete it ever.
   * \param camera the camera to use in rendering
   */
  void set_camera(Camera* camera);

  /**
   * Sets the pathtracer's frame size. If in a running state (VISUALIZE,
   * RENDERING, or DONE), transitions to READY b/c a changing window size
   * would invalidate the output. If in INIT and configuration is done,
   * transitions to READY.
   * \param width width of the frame
   * \param height height of the frame
   */
  void set_frame_size(size_t width, size_t height);

  void render_to_file(std::string filename, size_t x, size_t y, size_t dx, size_t dy);

  /**
   * Save rendered result to png file.
   */
  void save_image(std::string filename="");

 private:

  /**
   * Used in initialization.
   */
  bool has_valid_configuration();

  /**
   * Build acceleration structures.
   */
  void build_accel();

  void build_lights();

  void gpu_raytrace();

  void copy_host_device_pt();


  PathTracer *pt;
  PathTracer *pt_cuda;

  // Configurables //

  Scene* scene;         ///< current scene
  Camera* camera;       ///< current camera

  double lensRadius;
  double focalDistance;

  // Components //

  BVHAccel* bvh;                 ///< BVH accelerator aggregate
  BVHCuda* bvh_cuda;             ///< BVH accelerator aggregate for cuda
  ImageBuffer frameBuffer;       ///< frame buffer
  Timer timer;                   ///< performance test timer

  std::vector<CudaLight> lights;
  CudaLightBundle *light_data;

  std::string filename;
};

}  // namespace CGL

#endif  // CGL_RAYTRACER_H
