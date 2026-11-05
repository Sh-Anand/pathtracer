#include <stddef.h>

#include <rad_defs.h>

#include "target/pathtracer.h"
#include "target/gluon/scene.h"

static_assert(sizeof(GluonTexture) == sizeof(CudaTexture) &&
              offsetof(GluonTexture, data) == offsetof(CudaTexture, data),
              "GluonTexture must match the device layout of CudaTexture");

// the scene pathtracer.h declares, bound from the launch argument
int max_ray_depth, ns_area_light, accumulate, num_lights, w, h, restir;
CudaCamera camera;
const CudaLight *lights;
const CudaBSDF *bsdfs;
const CudaTexture *textures;
const CudaPrimitive *primitives;
const Vector3D *vertices, *normals;
const Vector2D *texcoords;
const Vector4D *tangents;
const BVHNode *nodes;
RNGState *rand_states;
Sample *initialSampleBuffer;
SampleMetadata *initialSampleMetadataBuffer;
Reservoir *temporalReservoirBufferGI;
PixelData *pixel;
uint8_t *rays_traced;

// every thread binds the same values, so the racing writes are benign
static void bind(const GluonScene *s) {
  camera = s->camera;
  lights = (const CudaLight *)s->lights;
  bsdfs = (const CudaBSDF *)s->bsdfs;
  textures = (const CudaTexture *)s->textures;
  primitives = (const CudaPrimitive *)s->primitives;
  vertices = (const Vector3D *)s->vertices;
  normals = (const Vector3D *)s->normals;
  texcoords = (const Vector2D *)s->texcoords;
  tangents = (const Vector4D *)s->tangents;
  nodes = (const BVHNode *)s->nodes;
  rand_states = (RNGState *)s->rand_states;
  initialSampleBuffer = (Sample *)s->samples;
  initialSampleMetadataBuffer = (SampleMetadata *)s->sample_metadata;
  temporalReservoirBufferGI = (Reservoir *)s->reservoirs;
  pixel = (PixelData *)s->pixel;
  rays_traced = (uint8_t *)s->rays_traced;
  num_lights = s->num_lights;
  w = s->w;
  h = s->h;
  max_ray_depth = s->max_ray_depth;
  ns_area_light = s->ns_area_light;
  accumulate = s->accumulate;
  restir = s->restir;
}

// this thread's pixel; false if it lies outside the image
static bool pixel_xy(uint32_t *x, uint32_t *y) {
  uint32_t tx, ty, bx, by;
  asm volatile("csrr %0, %1" : "=r"(tx) : "i"(CSR_THREAD_IDX_X));
  asm volatile("csrr %0, %1" : "=r"(ty) : "i"(CSR_THREAD_IDX_Y));
  asm volatile("csrr %0, %1" : "=r"(bx) : "i"(CSR_BLOCK_IDX_X));
  asm volatile("csrr %0, %1" : "=r"(by) : "i"(CSR_BLOCK_IDX_Y));
  *x = bx * GLUON_BLOCK + tx;
  *y = by * GLUON_BLOCK + ty;
  return *x < (uint32_t)w && *y < (uint32_t)h;
}

extern "C" void pathtrace_kernel(const GluonScene *s) {
  bind(s);
  uint32_t x, y;
  if (pixel_xy(&x, &y))
    raytrace_pixel_temporal_sample(x, y);
}

extern "C" void spatial_resampling_kernel(const GluonScene *s) {
  bind(s);
  uint32_t x, y;
  if (pixel_xy(&x, &y))
    spatial_resampling(x, y);
}
