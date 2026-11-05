#pragma once

#include <stdint.h>

#include "scene/camera.h"

// threads per block in x and y; the host launches ceil(w/8) x ceil(h/8) blocks
#define GLUON_BLOCK 8

// Kernel launch argument: the scene's device buffers.  Device addresses are
// 32 bits, so they are stored as uint32_t to keep one layout on host and device.
struct GluonScene {
  CudaCamera camera;
  uint32_t lights, bsdfs, textures, primitives, vertices, normals, texcoords, tangents, nodes;
  uint32_t rand_states, samples, sample_metadata, reservoirs, pixel, rays_traced;
  int32_t num_lights, w, h, max_ray_depth, ns_area_light, accumulate, restir;
};

// CudaTexture as the device lays it out: its data pointer is 32 bits there
struct GluonTexture {
  uint16_t width, height;
  uint8_t channels;
  uint32_t data;
};
