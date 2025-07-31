// read only start
#include "scene/geometry.h"
#include "scene/material.h"
#include "scene/bvh.h"
#include "scene/camera.h"
#include "scene/light.h"
#include "util/cuda_defs.h"
#include "util/reservoir.h"

extern DEVICE const int max_ray_depth;
extern DEVICE const int ns_area_light;
extern DEVICE const int accumulate;
extern DEVICE const CudaCamera camera;       ///< current camera
extern DEVICE const CudaLight lights[];
extern DEVICE const int num_lights;
extern DEVICE const CudaBSDF bsdfs[];
extern DEVICE const CudaTexture textures[];
extern DEVICE const CudaPrimitive primitives[];
extern DEVICE const Vector3D vertices[];
extern DEVICE const Vector3D normals[];
extern DEVICE const Vector2D texcoords[];
extern DEVICE const Vector4D tangents[];
extern DEVICE const BVHNode nodes[];
extern DEVICE const int w;
extern DEVICE const int h;
extern DEVICE const int restir;
// read only end

// read/write start
extern DEVICE RNGState rand_states[];
extern DEVICE Sample initialSampleBuffer[];
extern DEVICE SampleMetadata initialSampleMetadataBuffer[];
extern DEVICE Reservoir temporalReservoirBufferGI[];
extern DEVICE PixelData pixel[];
extern DEVICE uint8_t rays_traced[];
// read/write end

DEVICE static inline Vector3D get_emission(const CudaBSDF *bsdfs,
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

DEVICE static inline Vector3D estimate_direct_lighting_importance(Ray r, const CudaIntersection isect, int idx) {
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

DEVICE static inline void raytrace_pixel_temporal_sample(uint16_t x, uint16_t y) {
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

// Computes jacobian from s1->s2 as defined in Equation 11 of the ReSTIR-GI paper
DEVICE inline float jacobian(
    const Vector3D xq1, const Vector3D xq2,
    const Vector3D xr1, const Vector3D nq2)
{
    Vector3D q1q2 = vector3d_sub(xq1, xq2);
    Vector3D r1q2 = vector3d_sub(xr1, xq2);

    float dist_q  = vector3d_norm2(q1q2);
    float dist_r  = vector3d_norm2(r1q2);
    if (dist_r < EPS_F) return 0.f;

    float abs_cos_q = fabsf(vector3d_dot(nq2, vector3d_scale(q1q2, 1.f/dist_q)));
    float abs_cos_r = fabsf(vector3d_dot(nq2, vector3d_scale(r1q2, 1.f/dist_r)));
    if (abs_cos_q < EPS_F) return 0.f;

    return (abs_cos_r / abs_cos_q) * (dist_q / dist_r);
}

DEVICE static inline void spatial_resampling(uint16_t x, uint16_t y) {
  const uint16_t neighbouring_pixel_radius = floor(0.05 * fminf(w, h));

  uint32_t idx = x + y * w;
  Reservoir Rs = temporalReservoirBufferGI[idx];
  Vector3D q_x_v = initialSampleBuffer[idx].x_v;
  Vector3D q_n_v = initialSampleBuffer[idx].n_v;
  SampleMetadata qm = initialSampleMetadataBuffer[idx];
  RNGState *rand_state = &rand_states[idx];
  const uint8_t max_neighbouring_samples = 9; // ReSTIR GI paper value without temporal sampling

  uint8_t s = 0, retries = 0;
  while (s < max_neighbouring_samples && retries < 20) {
    retries++;
    // Randomly choose a neighbor pixel qn
    int window = 2 * neighbouring_pixel_radius + 1;
    uint16_t sample_x = x + static_cast<int>(next_float(rand_state) * window) - neighbouring_pixel_radius;
    uint16_t sample_y = y + static_cast<int>(next_float(rand_state) * window) - neighbouring_pixel_radius;

    // Ensure the sample is within the frame buffer bounds
    if (sample_x >= w || sample_y >= h || (sample_x == x && sample_y == y)) continue;

    // Retrieve the reservoir from the neighboring pixel
    int idxn = sample_x + sample_y * w;
    Reservoir Rn = temporalReservoirBufferGI[idxn];
    Sample qn = initialSampleBuffer[Rn.z];
    // Discard sample if it failed to intersect second bounce
    if (qn.L.x == 0 && qn.L.y == 0 && qn.L.z == 0) continue;

    // We count this as a sample
    retries = 0;
    s++;

    Sample qs = initialSampleBuffer[Rs.z];
    // Calculate geometric similarity between Rs.z (not q because of resampling) and qn
    if (!are_geometrically_similar(&qs, &qn)) continue;

    // Calculate |Jqn→q| (Jacobian determinant)
    float Jqn_to_q = fabsf(jacobian(qn.x_v, qn.x_s, q_x_v, qn.n_s));
    if (Jqn_to_q < EPS_F) continue;

    // Calculate ˆp′q
    float p_prime_q = illum(qn.L) / Jqn_to_q;

    // visibility test
    // if neighbour's path's point is invisible from the current path's point, p_prime_q = 0
    Ray shadow_ray;
    Vector3D xsmxv = vector3d_sub(qn.x_s, q_x_v);
    shadow_ray.o = q_x_v; shadow_ray.d = vector3d_unit(xsmxv); shadow_ray.inv_d = vector3d_rcp(shadow_ray.d);
    shadow_ray.min_t = EPS_F;
    shadow_ray.max_t = vector3d_norm(xsmxv) - EPS_F;
    if (!has_intersect(primitives, vertices, nodes, &shadow_ray)) p_prime_q = 0;

    // Merge Rn into the current reservoir
    merge(&Rs, Rn, p_prime_q, rand_state);
  }

  Sample S = initialSampleBuffer[Rs.z];
  float phat = illum(S.L);
  Rs.W = Rs.M * phat > 0 ? Rs.w / (Rs.M * phat) : 0;

  float costheta = fmaxf(vector3d_dot(q_n_v, vector3d_unit(vector3d_sub(S.x_s, q_x_v))), 0.0f);
  Vector3D L = vector3d_add(qm.emittance, vector3d_mul(vector3d_scale(qm.bsdf_f, costheta), vector3d_scale(S.L, Rs.W)));
  pixel[idx] = {
    .data = L,
    .normal = S.n_v,
    .depth = S.z_v
  };
}
