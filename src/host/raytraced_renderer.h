#ifndef CGL_RAYTRACER_H
#define CGL_RAYTRACER_H

#include <stack>
#include <vector>
#include <algorithm>
#include <chrono>


#include "scene/bvh.h"
#include "scene/camera.h"

#include "target/pathtracer.h"

class RaytracedRenderer {
public:

  RaytracedRenderer(size_t ns_aa = 1, 
             size_t max_ray_depth = 4, size_t ns_area_light = 1,
             std::string filename = "",
             float lensRadius = 0.25,
             float focalDistance = 4.7,
             bool debug = false);

  void set_camera(Camera* camera);

  void set_frame_size(size_t width, size_t height);

  void render_to_file(std::string filename, size_t x, size_t y, size_t dx, size_t dy);

  void set_cuda_camera();

  void update_camera();

  void save_image(std::string filename="");

  void build_accel(std::vector<CudaPrimitive> &primitives, 
                   std::vector<Vector3D> &vertices,
                   std::vector<Vector3D> &normals,
                   std::vector<Vector2D> &texcoords,
                   std::vector<Vector4D> &tangents);

  void gpu_raytrace();

  void copy_host_device_pt(std::vector<CudaLight> &lights,
                           std::vector<CudaBSDF> &bsdfs, std::vector<CudaTexture> &textures);

  PathTracer *pt_host;
  PathTracer *pt_target;

  // Configurables //

  Camera* camera;       ///< current camera

  // Components //

  BVHCuda* bvh;             ///< BVH accelerator aggregate for cuda
  HDRImageBuffer frameBuffer;       ///< frame buffer

  std::string filename;

  bool debug;
};

#endif  // CGL_RAYTRACER_H
