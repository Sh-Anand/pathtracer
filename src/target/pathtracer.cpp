#include "scene/geometry.h"
#include "scene/material.h"
#include "scene/bvh.h"
#include "scene/camera.h"
#include "scene/light.h"
#include "util/cuda_defs.h"
#include "util/reservoir.h"

#include "vx_print.h"
#include "vx_spawn.h"

// read only start
extern const int max_ray_depth;
extern const int ns_area_light;
extern const int accumulate;
extern const CudaCamera camera;       ///< current camera
extern const CudaLight lights[];
extern const int num_lights;
extern const CudaBSDF bsdfs[];
extern const CudaTexture textures[];
extern const CudaPrimitive primitives[];
extern const Vector3D vertices[];
extern const Vector3D normals[];
extern const Vector2D texcoords[];
extern const Vector4D tangents[];
extern const BVHNode nodes[];
extern const int w;
extern const int h;
extern const int restir;
// read only end

// read/write start
extern RNGState rand_states[];
extern Sample initialSampleBuffer[];
extern SampleMetadata initialSampleMetadataBuffer[];
extern Reservoir temporalReservoirBufferGI[];
extern PixelData pixel[];
extern uint8_t rays_traced[];
// read/write end

static inline Vector3D get_emission(const CudaBSDF *bsdfs,
                                           const CudaTexture *textures,
                                           const CudaIntersection isect) {
  CudaBSDF bsdf = bsdfs[isect.bsdf_idx];
  Vector2D uv = isect.uv;
  Vector3D emission = bsdf.emission;
  if (bsdf.emission_idx >= 0) {
    Vector4D tc = sample_texture(textures[bsdf.emission_idx], uv);
    emission.x *= tc.x;
    emission.y *= tc.y;
    emission.z *= tc.z;
  }
  return emission;
}

static inline __attribute__((always_inline)) Vector3D estimate_direct_lighting_importance(Ray r, const CudaIntersection isect, int idx) {
  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = ray_at(r, isect.t);
  const Vector3D w_out = vector3d_neg(r.d);
  Vector3D L_out = Vector3D{};
  //NOTE: wi here is in worldpsace,

  float occlusion; //ignored for dir lighting
  RNGState rand_state = rand_states[idx];
  uint8_t l = 0;
  while (l++ < ns_area_light) {
    int i = next_u32(&rand_state) % num_lights;
    CudaLight L = lights[i];
    Vector3D wi;
    float   distToL, pdfL;
    Vector3D Li = sample_L(&L, hit_p, &wi, &distToL, &pdfL,
                           &rand_state, vertices);

    float cosNL = fmaxf(vector3d_dot(isect.n, wi), 0.0);
    if (pdfL > 0 && cosNL > 0) {
      // shadow test
      Ray shadow; shadow.o = hit_p; shadow.d = wi; shadow.inv_d = vector3d_rcp(wi);
      shadow.min_t = EPS_F;
      shadow.max_t = distToL - EPS_F;
      int has_shadow = has_intersect(primitives, vertices, nodes, &shadow);
      if (!has_shadow) {
        // BRDF eval and PDF of sampling that same wi via BSDF
        Vector3D f_val = f(bsdfs, textures, isect, w_out, wi, &occlusion);
        L_out = vector3d_add(L_out, vector3d_mul(f_val, vector3d_scale(Li, (cosNL/ pdfL))));
      }
    }
  }

  L_out = vector3d_scale(L_out, 1.0f / ns_area_light);

  rand_states[idx] = rand_state;
  return L_out;
}


#define RRT 0.7f

static inline void raytrace_pixel_temporal_sample(uint16_t x, uint16_t y) {
  CudaIntersection isect{}; isect.t = INFINITY;
  
  Ray r;
  uint32_t idx = x + y * w;
  init_gpu_rng(&rand_states[idx], 1234 + idx);

  Vector2D sample;
  sample.x = float(x) + next_float(&rand_states[idx]);
  sample.y = float(y) + next_float(&rand_states[idx]);
  r = generate_ray(&camera, sample.x / w, sample.y / h);

  float pdf_f = 0.0f;
  if (intersect(&r, primitives, vertices, normals, texcoords, tangents, nodes, &isect)) {
    // perturb normal
    int normal_idx = bsdfs[isect.bsdf_idx].normal_idx;
    if (normal_idx >= 0) {
      Vector3D N = isect.n;
      Vector3D T = Vector3D{isect.tangent.x, isect.tangent.y, isect.tangent.z};
      T = vector3d_unit(vector3d_sub(T, vector3d_scale(N, vector3d_dot(N, T))));
      Vector3D B = vector3d_scale(vector3d_cross(N, T), isect.tangent.w);

      Vector4D c = sample_texture(textures[normal_idx], isect.uv);
      Vector3D n_tangent = Vector3D{c.x * 2.0f - 1.0f,
                                    c.y * 2.0f - 1.0f,
                                    c.z * 2.0f - 1.0f};
      Vector3D perturbed = vector3d_unit(vector3d_add(vector3d_scale(T, n_tangent.x), vector3d_add(vector3d_scale(B, n_tangent.y), vector3d_scale(N, n_tangent.z))));

      Vector3D diff = vector3d_sub(perturbed, N);
      float diff_len = vector3d_norm(diff);
      // use original if diff small to prevent flickering. TODO: better fix
      if (diff_len < 0.4) {
        isect.n = N;
      } else {
        isect.n = perturbed;
      }
    }
    Vector3D L_out_total{0.0, 0.0, 0.0};
    Vector3D throughput{1.0, 1.0, 1.0};
    Ray current_ray = r;

    uint8_t level = 1;
    uint8_t rays = 0;
    while (level <= max_ray_depth) {
      rays++;

      // hit point & outgoing dir in world space
      Vector3D hit_p  = ray_at(current_ray, isect.t);
      Vector3D w_out = vector3d_neg(current_ray.d);

      // direct lighting
      Vector3D L_out = estimate_direct_lighting_importance(current_ray, isect, idx);
      if (level != 1) {
        L_out_total = vector3d_add(L_out_total, vector3d_mul(throughput, L_out));
      }

      // russian-roulette survival
      float p_survive = (level == 1) ? 1.0f : RRT;
      if (level > 1 &&
          next_float(&rand_states[idx]) >= RRT)
          break;

      // sample BSDF
      Vector3D wi{0,0,0};
      float pdf;
      float occlusion = 1.0;
      Vector3D bsdf_f = sample_f(bsdfs, textures, isect, w_out, &wi, &pdf, &occlusion, &rand_states[idx]);
      wi = vector3d_unit(wi); // ensure wi is normalized
      bsdf_f = vector3d_scale(bsdf_f, occlusion);
      float costheta = fmaxf(vector3d_dot(isect.n, wi), 0.0f);
      Vector3D fcos = vector3d_scale(bsdf_f, costheta);
      throughput = vector3d_mul(throughput, fcos);
      throughput = vector3d_scale(throughput, 1.0f / (pdf * p_survive));

      if (level == 1) {
          Sample* s = &initialSampleBuffer[idx];
          SampleMetadata* sm = &initialSampleMetadataBuffer[idx];
          sm->emittance = accumulate ? vector3d_add(L_out, get_emission(bsdfs, textures, isect)) : Vector3D{};
          s->x_v   = hit_p;
          s->n_v   = isect.n;
          s->z_v   = vector3d_norm2(vector3d_sub(hit_p, r.o));
          pdf_f   = pdf;
          sm->bsdf_f  = bsdf_f;
          throughput = vector3d_scale(throughput, 1 / p_survive);
      } else {
          float costheta = fmaxf(vector3d_dot(isect.n, wi), 0.0f);
          Vector3D fcos = vector3d_scale(bsdf_f, costheta);
          throughput = vector3d_mul(throughput, fcos);
          throughput = vector3d_scale(throughput, 1.0f / (pdf * p_survive));
      }
      if (pdf <= 0.0) {
          break;
      }

      // spawn next ray
      Ray bounce_ray; bounce_ray.o = hit_p; bounce_ray.d = wi; bounce_ray.max_t = INFINITY; bounce_ray.inv_d = vector3d_rcp(bounce_ray.d);
      bounce_ray.min_t = EPS_F;

      CudaIntersection bounce_isect; bounce_isect.t = INFINITY;
      if (!intersect(&bounce_ray, primitives, vertices, normals,
                    texcoords, tangents, nodes, &bounce_isect))
          break;

      if (level == 1) {
          Vector3D bounce_p = ray_at(bounce_ray, bounce_isect.t);
          Sample* s = &initialSampleBuffer[idx];
          s->x_s   = bounce_p;
          s->n_s   = bounce_isect.n;
      }

      // prepare for next iteration
      current_ray = bounce_ray;
      isect       = bounce_isect;
      level = level + 1;
    }

    rays_traced[idx] = rays;
    initialSampleBuffer[idx].L = L_out_total;
  }

  if (restir) {
    Reservoir R = temporalReservoirBufferGI[idx];
    float w = pdf_f > 0 ? illum(initialSampleBuffer[idx].L) / pdf_f : 0.0f;
    update(&R, idx, w, &rand_states[idx]);
    R.W = R.M > 0 && illum(initialSampleBuffer[R.z].L) > 0 ? R.w / (R.M * illum(initialSampleBuffer[R.z].L)) : R.W;

    temporalReservoirBufferGI[idx] = R;
  } else {
    float costheta = fmaxf(vector3d_dot(initialSampleBuffer[idx].n_v, vector3d_unit(vector3d_sub(initialSampleBuffer[idx].x_s, initialSampleBuffer[idx].x_v))), 0.0f);
    float cospdf = pdf_f > 0 ? costheta / pdf_f : 0.0f;
    Vector3D L = vector3d_add(initialSampleMetadataBuffer[idx].emittance, vector3d_mul(vector3d_scale(initialSampleMetadataBuffer[idx].bsdf_f, cospdf), initialSampleBuffer[idx].L));
    pixel[idx] = {
      .data = L,
      .normal = initialSampleBuffer[idx].n_v,
      .depth = initialSampleBuffer[idx].z_v
    };
  }
}

static void rt_entry_point(void *args) {
  uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
  uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) return;
  raytrace_pixel_temporal_sample(x, y);
}

int main() {
  vx_printf("Pathtracer starting...\n");

  vx_printf("RAYTRACING_START\n");

  vx_printf("Width: %d, Height: %d\n", w, h);
  vx_printf("Max Ray Depth: %d, NS Area Light: %d, Accumulate: %s\n, ReSTIR: %s\n",
            max_ray_depth, ns_area_light, accumulate ? "true" : "false", restir ? "true" : "false");

  uint32_t dimension = 2;
  uint32_t block_dim[2] = {4, 4};
  uint32_t grid_dim[2] = {(w + block_dim[0] - 1) / block_dim[0],
                          (h + block_dim[1] - 1) / block_dim[1]};

  vx_spawn_threads(dimension, grid_dim, block_dim, (vx_kernel_func_cb)rt_entry_point, NULL);

  if (!restir) {
    vx_printf("PIXEL_BUFFER_START\n");
    vx_printf("%d, %d\n", w, h);
    for (uint32_t i = 0; i < w; i++) {
      for (uint32_t j = 0; j < h; j++) {
        vx_printf("(%f, %f, %f) ",
            pixel[i + j * w].data.x, pixel[i + j * w].data.y, pixel[i + j * w].data.z);
      }
      vx_printf("\n");
    }
    vx_printf("PIXEL_BUFFER_END\n");
  }

  vx_printf("RAYTRACING_COMPLETE\n");
  return 0;
}