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

// power­-heuristic MIS weight, β=2
DEVICE static inline float mis_weight(float pA, float pB) {
  float wA = pA*pA;
  float wB = pB*pB;
  float mis =  wA / (wA + wB);
  return mis;
}

// mixture PDF of your metallic‑roughness lobes
DEVICE static inline float bsdf_pdf(const CudaBSDF* bsdfs, 
                             const CudaIntersection isect,
                             const Vector3D wo,
                             const Vector3D wi) {
  Vector3D N = isect.n;
  float NoL = fabs(vector3d_dot(N, wi));
  if (NoL == 0) return 0.0;

  // fetch metallic & roughness
  CudaBSDF bsdf = bsdfs[isect.bsdf_idx];
  float metal    = clampd(bsdf.metallic,  0.0, 1.0);
  float roughness= clampd(bsdf.roughness, 0.02,1.0);
  float onem     = 1.0 - metal;
  float alpha    = roughness * roughness;

  Vector3D base = Vector3D{bsdf.baseColor.x, bsdf.baseColor.y, bsdf.baseColor.z};
  // with—compute luminance of F₀:
  Vector3D F0  = Vector3D{0.04f * onem + base.x * metal, 0.04f * onem + base.y * metal, 0.04f * onem + base.z * metal};
  float   F0_avg = (F0.x + F0.y + F0.z) / 3.0;  
  F0_avg         = clampd(F0_avg, 0.0, 1.0);

  float P_s = F0_avg;   // sample specular lobe with Fresnel weight
  float P_d = 1.0 - P_s;

  // 1) diffuse pdf = (cosθ/π)
  float pdf_diff = P_d * (NoL / M_PI);

  // 2) specular pdf = D(α,NoH)·NoH / (4·VoH)
  Vector3D H   = vector3d_unit(vector3d_add(wo, wi));
  float NoH   = fmaxf(vector3d_dot(N, H), 0.0);
  float VoH   = fmaxf(vector3d_dot(wo, H), 0.0);
  float D     = D_compute(alpha, NoH);
  float pdf_spec = P_s * (D * NoH / (4.0 * VoH));

  return pdf_diff + pdf_spec;
}


DEVICE static inline Vector3D estimate_direct_lighting_importance(PathTracer* pt, Ray r, const CudaIntersection isect, uint32_t idx) {
  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = ray_at(r, isect.t);
  const Vector3D w_out = vector3d_neg(r.d);
  Vector3D L_out = Vector3D{};
  //NOTE: wi here is in worldpsace,

  float occlusion; //ignored for dir lighting
  RNGState rand_state = pt->rand_states[idx];
  for (int i = 0; i < pt->num_lights; ++i) {
    CudaLight L = pt->lights[i];
    Vector3D wi;
    float   distToL, pdfL;
    Vector3D Li = sample_L(&L, hit_p, &wi, &distToL, &pdfL,
                           &rand_state, pt->bvh->vertices);

    float cosNL = fmaxf(vector3d_dot(isect.n, wi), 0.0);
    if (pdfL > 0 && cosNL > 0) {
      // shadow test
      Ray shadow; shadow.o = hit_p; shadow.d = wi; shadow.inv_d = vector3d_rcp(wi);
      shadow.min_t = EPS_F;
      shadow.max_t = distToL;
      if (!has_intersect(pt->bvh, &shadow)) {
        // BRDF eval and PDF of sampling that same wi via BSDF
        Vector3D f_val = f(pt->bsdfs, pt->textures, isect, w_out, wi, &occlusion);
        float  pdfB   = bsdf_pdf(pt->bsdfs, isect, w_out, wi);
        float  w      = mis_weight(pdfL, pdfB);
        L_out = vector3d_add(L_out, vector3d_mul(f_val, vector3d_scale(Li, (cosNL * w / pdfL))));
      }
    }
  }

  Vector3D wi_bsdf;
  float   pdfB;
  Vector3D f_bsdf = sample_f(pt->bsdfs, pt->textures, isect, w_out, &wi_bsdf, &pdfB,
                             &occlusion, &rand_state);
  float cosNL = fmaxf(vector3d_dot(isect.n, wi_bsdf), 0.0);

  if (pdfB > 0 && cosNL > 0) {
    // trace a ray in that direction and see if it hits *any* light
    Ray shadow; shadow.o = hit_p; shadow.d = wi_bsdf; shadow.inv_d = vector3d_rcp(wi_bsdf);
    shadow.min_t = EPS_F;
    shadow.max_t = INFINITY;
    for (int i = 0; i < pt->num_lights; ++i) {
      CudaLight L = pt->lights[i];
      float pdfL;
      if (light_has_intersect(&L, &shadow, &hit_p, &isect.n, pt->bvh->vertices, &pdfL)) {
        // get the light and compute its PDF for this direction
        Vector3D Li = L.radiance;
        float w    = mis_weight(pdfB, pdfL);
        L_out = vector3d_add(L_out, vector3d_mul(f_bsdf, vector3d_scale(Li, (cosNL * w / pdfB))));
      }
    }
  }

  pt->rand_states[idx] = rand_state;
  return L_out;
}


#define RRT 0.7f

DEVICE static inline Vector3D at_least_one_bounce_radiance(PathTracer *pt, Ray r, const CudaIntersection isect_init, uint32_t idx) {
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
    Vector3D L_out = estimate_direct_lighting_importance(pt, current_ray, isect, idx);
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
    Vector3D bsdf_f = sample_f(pt->bsdfs, pt->textures, isect, w_out, &wi, &pdf, &occlusion, &pt->rand_states[idx]);
    wi = vector3d_unit(wi); // ensure wi is normalized
    float costheta = vector3d_dot(isect.n, wi);
    Vector3D fcos = vector3d_scale(bsdf_f, costheta);
    bsdf_f = vector3d_scale(bsdf_f, occlusion);

    if (first_bounce) {
        Sample* s = &pt->initialSampleBuffer[idx];
        s->emittance = L_out;
        s->x_v   = hit_p;
        s->n_v   = isect_init.n;
        s->z_v   = vector3d_norm2(vector3d_sub(hit_p, r.o));
        s->pdf   = pdf;
        s->bsdf_f  = bsdf_f;
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
  return vector3d_sub(L_out_total, pt->initialSampleBuffer[idx].emittance);
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
    Vector3D L = at_least_one_bounce_radiance(pt, r, isect, idx);
    S = pt->initialSampleBuffer[idx];
    S.emittance = vector3d_add(S.emittance, get_emission(pt->bsdfs, pt->textures, isect));
    S.L = L;
  }

  if (restir) {
    Reservoir R = pt->temporalReservoirBufferGI[idx];
    float w = S.pdf > 0 ? p_hat(S) / S.pdf : 0.0f;
    update(&R, S, w, &pt->rand_states[idx]);
    R.W = R.M > 0 && p_hat(R.z) > 0 ? R.w / (R.M * p_hat(R.z)) : R.W;

    pt->temporalReservoirBufferGI[idx] = R;
    pt->initialSampleBuffer[idx] = S;
  } else {
    float cospdf = S.pdf > 0 ? vector3d_dot(S.n_v, vector3d_unit(vector3d_sub(S.x_v, S.x_s))) / S.pdf : 0.0f;
    Vector3D L = vector3d_add(S.emittance, vector3d_mul(vector3d_scale(S.bsdf_f, cospdf), S.L));
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
  const uint16_t neighbouring_pixel_radius = floor(0.05 * min(pt->sampleBuffer.w, pt->sampleBuffer.h));

  uint32_t idx = x + y * pt->sampleBuffer.w;
  Reservoir Rs = pt->temporalReservoirBufferGI[idx];
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

    // Retrieve the reservoir from the neighboring pixel
    Reservoir Rn = pt->temporalReservoirBufferGI[sample_x + sample_y * pt->sampleBuffer.w];

    // Discard sample if it failed to intersect second bounce
    if (Rn.z.L.x == 0 && Rn.z.L.y == 0 && Rn.z.L.z == 0) continue;

    // We count this as a sample
    retries = 0;
    s++;

    // Calculate geometric similarity between Rs.z (not q because of resampling) and qn
    if (!are_geometrically_similar(&Rs.z, &Rn.z)) continue;

    // Calculate |Jqn→q| (Jacobian determinant)
    float Jqn_to_q = fabsf(jacobian(Rn.z.x_v, Rn.z.x_s, q.x_v, Rn.z.n_s));
    if (Jqn_to_q < EPS_F) continue; 

    // Calculate ˆp′q
    float p_prime_q = p_hat(Rn.z) / Jqn_to_q;

    // visibility test
    // if neighbour's path's point is invisible from the current path's point, p_prime_q = 0
    Ray shadow_ray;
    Vector3D xsmxv = vector3d_sub(Rn.z.x_s, q.x_v);
    shadow_ray.o = q.x_v; shadow_ray.d = vector3d_unit(xsmxv); shadow_ray.inv_d = vector3d_rcp(shadow_ray.d);
    shadow_ray.min_t = EPS_F;
    shadow_ray.max_t = vector3d_norm(xsmxv) - EPS_F;
    if (has_intersect(pt->bvh, &shadow_ray)) p_prime_q = 0;

    // Merge Rn into the current reservoir
    merge(&Rs, Rn, p_prime_q, rand_state);
  }

  float phat = p_hat(Rs.z);
  Rs.W = Rs.M * phat > 0 ? Rs.w / (Rs.M * phat) : 0;

  Sample S = Rs.z;
  float costheta = fabsf(vector3d_dot(q.n_v, vector3d_unit(vector3d_sub(S.x_s, q.x_v))));
  Vector3D L = vector3d_add(q.emittance, vector3d_mul(vector3d_scale(q.bsdf_f, costheta), vector3d_scale(S.L, Rs.W)));
  pt->sampleBuffer.data[idx] = L;
}