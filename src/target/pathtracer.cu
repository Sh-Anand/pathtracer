#include "pathtracer.h"

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


DEVICE static inline Vector3D estimate_direct_lighting_importance(PathTracer* pt, Ray r, const CudaIntersection isect, uint32_t idx) {
  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = ray_at(r, isect.t);
  const Vector3D w_out = vector3d_neg(r.d);
  Vector3D L_out = Vector3D{};
  //NOTE: wi here is in worldpsace,

  RNGState rand_state = pt->rand_states[idx];
  float occlusion;
  int l = 0;
  while (l++ < pt->ns_lights) {
    int light_idx = next_u32(&rand_state) % pt->num_lights;
    CudaLight L = pt->lights[light_idx];
    Vector3D wi;
    float   distToL, pdfL;
    Vector3D Li = sample_L(&L, hit_p, &wi, &distToL, &pdfL,
                           &rand_state, pt->bvh->vertices);

    float cosNL = vector3d_dot(isect.n, wi);
    if (cosNL <=0 || pdfL <= 0) continue;

    // shadow test
    Ray shadow; shadow.o = hit_p; shadow.d = wi; shadow.inv_d = vector3d_rcp(wi);
    shadow.min_t = EPS_F;
    shadow.max_t = distToL - EPS_F;
    if (!has_intersect(pt->bvh, &shadow)) {
      // BRDF eval and PDF of sampling that same wi via BSDF
      Vector3D f_val = f(pt->bsdfs, pt->textures, isect.n, isect.uv, isect.bsdf_idx, w_out, wi, &occlusion);
      L_out = vector3d_add(L_out, vector3d_mul(f_val, vector3d_scale(Li, (cosNL / pdfL))));
    }
  }

  pt->rand_states[idx] = rand_state;
  return L_out;
}

DEVICE static inline Vector3D estimate_direct_lighting_importance_ReSTIR(PathTracer* pt, Ray r, const CudaIntersection isect, uint32_t idx) {
  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = ray_at(r, isect.t);
  const Vector3D w_out = vector3d_neg(r.d);
  Vector3D L_out = Vector3D{};
  //NOTE: wi here is in worldpsace,

  RNGState rand_state = pt->rand_states[idx];
  int l = 0;
  Reservoir R = pt->temporalReservoirBufferDirect[idx];
  Sample s = pt->initialSampleBuffer[idx];
  s.w_out = w_out;
  s.bsdf_idx = isect.bsdf_idx;
  s.uv = isect.uv;
  s.n_v = isect.n;
  while (l++ < pt->ns_lights) {
    int light_idx = next_u32(&rand_state) % pt->num_lights;
    CudaLight L = pt->lights[light_idx];
    s.L_direct = sample_L(&L, hit_p, &s.wi, &s.dist, &s.pdf_L,
                           &rand_state, pt->bvh->vertices);

    float cosNL = vector3d_dot(isect.n, s.wi);
    if (cosNL <=0 || s.pdf_L <= 0) continue;

    float w = illum(s.L_direct) / s.pdf_L;
    update(&R, s, w, &rand_state);
  }

  float rmphat = R.M * illum(R.z.L_direct);
  R.W = rmphat > 0 ? R.w / rmphat : 0;

  pt->rand_states[idx] = rand_state;
  pt->temporalReservoirBufferDirect[idx] = R;

  return L_out;
}


#define RRT 0.7f

DEVICE static inline Vector3D at_least_one_bounce_radiance(PathTracer *pt, Ray r, const CudaIntersection isect_init, uint32_t idx, bool restir) {
  Vector3D L_out_total{0.0, 0.0, 0.0};
  Vector3D throughput{1.0, 1.0, 1.0};
  Ray current_ray = r;
  CudaIntersection isect = isect_init;
  bool first_bounce = true;

  uint8_t level = 1;
  uint8_t rays_traced = 0;
  while (level <= pt->max_ray_depth) {
    rays_traced++;

    // hit point & outgoing dir in world space
    Vector3D hit_p  = ray_at(current_ray, isect.t);
    Vector3D w_out = vector3d_neg(current_ray.d);

    // direct lighting
    Vector3D L_out = restir && first_bounce ? estimate_direct_lighting_importance_ReSTIR(pt, current_ray, isect, idx) : 
                                              estimate_direct_lighting_importance(pt, current_ray, isect, idx);
    L_out_total = vector3d_add(L_out_total, vector3d_mul(throughput, L_out));

    // russian-roulette survival
    float p_survive = (level == 1) ? 1.0f : RRT;
    if (level > 1 &&
        next_float(&pt->rand_states[idx]) >= RRT)
        break;

    // sample BSDF
    Vector3D wi{0,0,0};
    float pdf;
    float occlusion = 1.0;
    Vector3D bsdf_f = sample_f(pt->bsdfs, pt->textures, isect.n, isect.uv, isect.bsdf_idx, w_out, &wi, &pdf, &occlusion, &pt->rand_states[idx]);
    wi = vector3d_unit(wi); // ensure wi is normalized
    float costheta = vector3d_dot(isect.n, wi);
    Vector3D fcos = vector3d_scale(bsdf_f, costheta);
    bsdf_f = vector3d_scale(bsdf_f, occlusion);

    if (first_bounce) {
        Sample* s = &pt->initialSampleBuffer[idx];
        s->x_v   = hit_p;
        s->n_v   = isect_init.n;
        s->z_v   = vector3d_norm2(vector3d_sub(hit_p, r.o));
        s->pdf   = pdf;
        s->bsdf_f  = bsdf_f;
        s->bsdf_idx = isect_init.bsdf_idx;
        s->uv    = isect_init.uv;
        s->w_out = w_out;
    }
    if (pdf <= 0.0) {
        break;
    }

    // update throughput

    throughput = first_bounce ? throughput: vector3d_mul(throughput, fcos);
    throughput = first_bounce ? vector3d_scale(throughput, 1 / p_survive) : vector3d_scale(throughput, 1 / (pdf * p_survive));

    // spawn next ray
    Ray bounce_ray; bounce_ray.o = hit_p; bounce_ray.d = wi; bounce_ray.max_t = INFINITY; bounce_ray.inv_d = vector3d_rcp(bounce_ray.d);
    bounce_ray.min_t = EPS_F;
      level = level + 1;


    CudaIntersection bounce_isect; bounce_isect.t = INFINITY;
    if (!intersect(pt->bvh, &bounce_ray, &bounce_isect))
        break;

    if (first_bounce) {
        Vector3D bounce_p = ray_at(bounce_ray, bounce_isect.t);
        Sample* s = &pt->initialSampleBuffer[idx];
        s->x_s   = bounce_p;
        s->n_s   = bounce_isect.n;
    }

    // prepare for next iteration
    current_ray = bounce_ray;
    isect       = bounce_isect;
    first_bounce = false;
  }

  pt->rays_traced[idx] = rays_traced;
  return vector3d_sub(L_out_total, pt->initialSampleBuffer[idx].L_direct);
}

DEVICE static inline void raytrace_pixel_temporal_sample(PathTracer *pt, uint16_t x, uint16_t y, bool restir) {
  CudaIntersection isect; isect.t = INFINITY;
  
  Ray r;
  uint32_t idx = x + y * pt->sampleBuffer.w;
  pt->initialSampleBuffer[idx] = Sample{};
  init_gpu_rng(&pt->rand_states[idx], 1234 + idx);

  Vector2D origin = Vector2D{float(x), float(y)};
  Vector2D sample;
  sample.x = origin.x + next_float(&pt->rand_states[idx]);
  sample.y = origin.y + next_float(&pt->rand_states[idx]);
  r = generate_ray(&pt->camera, sample.x / pt->sampleBuffer.w, sample.y / pt->sampleBuffer.h);

  Sample S = {};

  if (intersect(pt->bvh, &r, &isect)) {
    // perturb normal
    int normal_idx = pt->bsdfs[isect.bsdf_idx].normal_idx;
    if (normal_idx >= 0) {
      Vector3D N = isect.n;
      Vector3D T = Vector3D{isect.tangent.x, isect.tangent.y, isect.tangent.z};
      T = vector3d_unit(vector3d_sub(T, vector3d_scale(N, vector3d_dot(N, T))));
      Vector3D B = vector3d_scale(vector3d_cross(N, T), isect.tangent.w);

      Vector4D c = sample_texture(pt->textures[normal_idx], isect.uv);
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
    Vector3D L = at_least_one_bounce_radiance(pt, r, isect, idx, restir);
    S = pt->initialSampleBuffer[idx];
    S.emittance = get_emission(pt->bsdfs, pt->textures, isect);
    S.L_indirect = L;
  }

  if (restir) {
    // ReSTIR GI
    Reservoir R_GI = pt->temporalReservoirBufferGI[idx];
    float w_GI = S.pdf > 0 ? illum(S.L_indirect) / S.pdf : 0.0f;
    update(&R_GI, S, w_GI, &pt->rand_states[idx]);
    float phatm = illum(R_GI.z.L_indirect) * R_GI.M;
    R_GI.W = phatm > 0 ? R_GI.w / phatm : R_GI.W;
    pt->temporalReservoirBufferGI[idx] = R_GI;
    pt->initialSampleBuffer[idx] = S;
  } else {
    float cospdf = S.pdf > 0 ? vector3d_dot(S.n_v, vector3d_unit(vector3d_sub(S.x_v, S.x_s))) / S.pdf : 0.0f;
    Vector3D L = vector3d_add(S.emittance, vector3d_add(S.L_direct, vector3d_mul(vector3d_scale(S.bsdf_f, cospdf), S.L_indirect)));
    pt->sampleBuffer.data[idx] = L;
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

DEVICE static inline void spatial_resampling(PathTracer *pt, uint16_t x, uint16_t y) {
  const uint16_t neighbouring_pixel_radius = floor(0.1 * min(pt->sampleBuffer.w, pt->sampleBuffer.h));

  uint32_t idx = x + y * pt->sampleBuffer.w;
  Reservoir Rs = pt->temporalReservoirBufferDirect[idx];
  Reservoir Rs_GI = pt->temporalReservoirBufferGI[idx];
  Sample q = pt->initialSampleBuffer[idx];
  RNGState *rand_state = &pt->rand_states[idx];
  const uint8_t max_neighbouring_samples = 9; // ReSTIR GI paper value without temporal sampling

  uint8_t s = 0, retries = 0;
  while (s < max_neighbouring_samples && retries < 20) {
    retries++;
    // Randomly choose a neighbor pixel qn
    int window = 2 * neighbouring_pixel_radius + 1;
    uint16_t sample_x = x + static_cast<int>(next_float(rand_state) * window) - neighbouring_pixel_radius;
    uint16_t sample_y = y + static_cast<int>(next_float(rand_state) * window) - neighbouring_pixel_radius;

    // Ensure the sample is within the frame buffer bounds
    if (sample_x >= pt->sampleBuffer.w || sample_y >= pt->sampleBuffer.h) continue;

    // ReSTIR spatial resampling
    Reservoir Rn = pt->temporalReservoirBufferDirect[sample_x + sample_y * pt->sampleBuffer.w];
    merge(&Rs, Rn, illum(Rn.z.L_direct), rand_state);

    // ReSTIR-GI spatial resampling
    Reservoir Rn_GI = pt->temporalReservoirBufferGI[sample_x + sample_y * pt->sampleBuffer.w];

    // Discard sample if it failed to intersect second bounce
    if (Rn_GI.z.L_indirect.x != 0 || Rn_GI.z.L_indirect.y != 0 || Rn_GI.z.L_indirect.z != 0) {
      // We count this as a sample
      retries = 0;
      s++;

      // Calculate geometric similarity between Rs_GI.z (not q because of resampling) and qn
      if (!are_geometrically_similar(&Rs_GI.z, &Rn_GI.z)) continue;

      // Calculate |Jqn→q| (Jacobian determinant)
      float Jqn_to_q = fabsf(jacobian(Rn_GI.z.x_v, Rn_GI.z.x_s, q.x_v, Rn_GI.z.n_s));
      if (Jqn_to_q < EPS_F) continue; 

      // Calculate ˆp′q
      float p_prime_q = illum(Rn_GI.z.L_indirect) / Jqn_to_q;

      // visibility test
      // if neighbour's path's point is invisible from the current path's point, p_prime_q = 0
      Ray shadow_ray;
      Vector3D xsmxv = vector3d_sub(Rn_GI.z.x_s, q.x_v);
      shadow_ray.o = q.x_v; shadow_ray.d = vector3d_unit(xsmxv); shadow_ray.inv_d = vector3d_rcp(shadow_ray.d);
      shadow_ray.min_t = EPS_F;
      shadow_ray.max_t = vector3d_norm(xsmxv) - EPS_F;
      if (has_intersect(pt->bvh, &shadow_ray)) p_prime_q = 0;

      // Merge Rn_GI into the current reservoir
      merge(&Rs_GI, Rn_GI, p_prime_q, rand_state);
    }
  }

  float phatm = Rs.M * illum(Rs.z.L_direct);
  Rs.W = phatm > 0 ? Rs.w / phatm : 0;
  float phatmgi = Rs_GI.M * illum(Rs_GI.z.L_indirect);
  Rs_GI.W = phatmgi > 0 ? Rs_GI.w / phatmgi : 0;

  // Compute direct lighting:
  Vector3D L_direct = {};
  Ray shadow_ray;
  shadow_ray.o = q.x_v; shadow_ray.d = Rs.z.wi; shadow_ray.inv_d = vector3d_rcp(Rs.z.wi);
  shadow_ray.min_t = EPS_F;
  shadow_ray.max_t = Rs.z.dist - EPS_F;
  if ((Rs.z.L_direct.x != 0.0f || Rs.z.L_direct.y != 0.0f || Rs.z.L_direct.z != 0.0f) && !has_intersect(pt->bvh, &shadow_ray)) {
    float occlusion;
    Vector3D f_val = f(pt->bsdfs, pt->textures, q.n_v, q.uv, q.bsdf_idx, q.w_out, Rs.z.wi, &occlusion);
    float cosNL = max(vector3d_dot(q.n_v, Rs.z.wi), 0.0f);
    L_direct = vector3d_mul(f_val, vector3d_scale(Rs.z.L_direct, Rs.W));
  }

  Sample S = Rs_GI.z;
  float costheta = fabsf(vector3d_dot(q.n_v, vector3d_unit(vector3d_sub(S.x_s, q.x_v))));
  Vector3D L = vector3d_add(q.emittance, vector3d_add(L_direct, vector3d_mul(vector3d_scale(q.bsdf_f, costheta), vector3d_scale(S.L_indirect, Rs_GI.W))));
  pt->sampleBuffer.data[idx] = L;
}