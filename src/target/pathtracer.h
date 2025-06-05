#ifndef CGL_PATHTRACER_H
#define CGL_PATHTRACER_H

#include "scene/material.h"
#include "scene/bvh.h"
#include "scene/camera.h"
#include "scene/light.h"
#include "util/reservoir.h"

// host + device force-inline
#define CUDA_INLINE __device__ __host__ __forceinline__
#define K_PI_F 3.14159265359f

#pragma once
typedef struct {
  size_t w;
  size_t h;
  Vector3D* data;
} HDRImageBuffer;
typedef struct {
    uint16_t max_ray_depth; ///< maximum allowed ray depth (applies to all rays)
    uint16_t ns_area_light; ///< number samples per area light source
    bool accumulate;
    const BVHCuda* bvh;                 ///< BVH accelerator aggregate
    CudaCamera camera;       ///< current camera
    RNGState* rand_states;       ///< random state for each thread
    const CudaLight *lights; 
    uint16_t num_lights;
    const CudaBSDF *bsdfs;
    const CudaTexture *textures;

    Sample* initialSampleBuffer;
    Reservoir* temporalReservoirBufferDirect;
    Reservoir* temporalReservoirBufferGI;
    uint8_t* rays_traced;
    HDRImageBuffer sampleBuffer;
} PathTracer;


#endif  // CGL_RAYTRACER_H
