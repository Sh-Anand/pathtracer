#ifndef CGL_RAYTRACER_H
#define CGL_RAYTRACER_H

#include <stack>
#include <vector>
#include <algorithm>
#include <chrono>


#include "scene/bvh.h"
#include "scene/camera.h"
#include "scene/light.h"
#include "scene/material.h"

class RaytracedRenderer {
public:

  RaytracedRenderer(bool accumulate = 1,
             size_t max_ray_depth = 4, size_t ns_area_light = 1,
             bool debug = false,
             bool restir = false);

  void set_camera(Camera* camera);

  void set_frame_size(size_t width, size_t height);

  void render_to_file(std::string filename);

  void save_image(std::string filename="");

  // void gpu_raytrace();

  void copy_host_device_pt(std::vector<CudaPrimitive> &primitives, 
                                            std::vector<Vector3D> &vertices,
                                            std::vector<Vector3D> &normals, 
                                            std::vector<Vector2D> &texcoords,
                                            std::vector<Vector4D> &tangents, 
                                            std::vector<CudaLight> &lights, 
                                            std::vector<CudaBSDF> &bsdfs, 
                                            std::vector<CudaTexture> &textures);

  // Configurables //

  Camera* camera;       ///< current camera

  // Components //
  PixelData *pixel;
  int w, h;
  bool debug;
  bool restir;
  uint16_t max_ray_depth;
  uint16_t ns_area_light;           
  bool accumulate;
};

#endif  // CGL_RAYTRACER_H
