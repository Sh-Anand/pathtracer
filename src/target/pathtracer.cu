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

DEVICE static inline Vector3D get_base_color(const CudaBSDF *bsdfs,
                                             const CudaTexture *textures,
                                             const CudaIntersection isect) {
  const CudaBSDF &bsdf = bsdfs[isect.bsdf_idx];
  Vector3D base{bsdf.baseColor.x, bsdf.baseColor.y, bsdf.baseColor.z};
  if (bsdf.tex_idx >= 0) {
    Vector4D t = sample_texture(textures[bsdf.tex_idx], isect.uv);
    base = Vector3D{base.x * t.x, base.y * t.y, base.z * t.z};
  }
  return base;
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
  uint8_t l = 0;
  while (l++ < pt->ns_area_light) {
    uint32_t i = next_u32(&rand_state) % pt->num_lights;
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
      shadow.max_t = distToL - EPS_F;
      if (!has_intersect(pt->bvh, &shadow)) {
        // BRDF eval and PDF of sampling that same wi via BSDF
        Vector3D f_val = f(pt->bsdfs, pt->textures, isect, w_out, wi, &occlusion);
        L_out = vector3d_add(L_out, vector3d_mul(f_val, vector3d_scale(Li, (cosNL/ pdfL))));
      }
    }
  }

  L_out = vector3d_scale(L_out, 1.0f / pt->ns_area_light);

  pt->rand_states[idx] = rand_state;
  return L_out;
}


#define RRT 0.7f

DEVICE static inline float at_least_one_bounce_radiance(PathTracer *pt, Ray r, const CudaIntersection isect_init, uint32_t idx) {
  Vector3D L_out_total{0.0, 0.0, 0.0};
  Vector3D throughput{1.0, 1.0, 1.0};
  Ray current_ray = r;
  CudaIntersection isect = isect_init;

  uint8_t level = 1;
  uint8_t rays_traced = 0;
  float pdf_ret = 0.0f;
  while (level <= pt->max_ray_depth) {
    rays_traced++;

    // hit point & outgoing dir in world space
    Vector3D hit_p  = ray_at(current_ray, isect.t);
    Vector3D w_out = vector3d_neg(current_ray.d);

    // direct lighting
    Vector3D L_out = estimate_direct_lighting_importance(pt, current_ray, isect, idx);
    if (level != 1) {
      // add emittance to the first bounce
      L_out_total = vector3d_add(L_out_total, vector3d_mul(throughput, L_out));
    }

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
    bsdf_f = vector3d_scale(bsdf_f, occlusion);

    if (level == 1) {
        Sample* s = &pt->initialSampleBuffer[idx];
        SampleMetadata* sm = &pt->initialSampleMetadataBuffer[idx];
        sm->emittance = pt->accumulate ? vector3d_add(L_out, get_emission(pt->bsdfs, pt->textures, isect)) : Vector3D{};
        s->x_v   = hit_p;
        s->n_v   = isect_init.n;
        s->z_v   = vector3d_norm2(vector3d_sub(hit_p, r.o));
        s->albedo = get_base_color(pt->bsdfs, pt->textures, isect_init);
        pdf_ret   = pdf;
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
    if (!intersect(pt->bvh, &bounce_ray, &bounce_isect))
        break;

    if (level == 1) {
        Vector3D bounce_p = ray_at(bounce_ray, bounce_isect.t);
        Sample* s = &pt->initialSampleBuffer[idx];
        s->x_s   = bounce_p;
        s->n_s   = bounce_isect.n;
    }

    // prepare for next iteration
    current_ray = bounce_ray;
    isect       = bounce_isect;
    level = level + 1;
  }

  pt->rays_traced[idx] = rays_traced;
  pt->initialSampleBuffer[idx].L = L_out_total;
  return pdf_ret;
}

DEVICE static inline void raytrace_pixel_temporal_sample(PathTracer *pt, uint16_t x, uint16_t y, bool restir) {
  CudaIntersection isect; isect.t = INFINITY;
  
  Ray r;
  uint32_t idx = x + y * pt->sampleBuffer.w;
  init_gpu_rng(&pt->rand_states[idx], 1234 + idx);

  Vector2D sample;
  sample.x = float(x) + next_float(&pt->rand_states[idx]);
  sample.y = float(y) + next_float(&pt->rand_states[idx]);
  r = generate_ray(&pt->camera, sample.x / pt->sampleBuffer.w, sample.y / pt->sampleBuffer.h);

  float pdf_f = 0.0f;
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
    pdf_f = at_least_one_bounce_radiance(pt, r, isect, idx);
  }

  if (restir) {
    Reservoir R = pt->temporalReservoirBufferGI[idx];
    float w = pdf_f > 0 ? illum(pt->initialSampleBuffer[idx].L) / pdf_f : 0.0f;
    update(&R, idx, w, &pt->rand_states[idx]);
    R.W = R.M > 0 && illum(pt->initialSampleBuffer[R.z].L) > 0 ? R.w / (R.M * illum(pt->initialSampleBuffer[R.z].L)) : R.W;

    pt->temporalReservoirBufferGI[idx] = R;
  } else {
    float costheta = fmaxf(vector3d_dot(pt->initialSampleBuffer[idx].n_v, vector3d_unit(vector3d_sub(pt->initialSampleBuffer[idx].x_s, pt->initialSampleBuffer[idx].x_v))), 0.0f);
    float cospdf = pdf_f > 0 ? costheta / pdf_f : 0.0f;
    Vector3D L = vector3d_add(pt->initialSampleMetadataBuffer[idx].emittance, vector3d_mul(vector3d_scale(pt->initialSampleMetadataBuffer[idx].bsdf_f, cospdf), pt->initialSampleBuffer[idx].L));
    pt->sampleBuffer.pixel[idx] = {
      .data = L,
      .albedo = pt->initialSampleBuffer[idx].albedo,
      .normal = pt->initialSampleBuffer[idx].n_v,
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

DEVICE static inline void spatial_resampling(PathTracer *pt, uint16_t x, uint16_t y) {
  const uint16_t neighbouring_pixel_radius = floor(0.05 * min(pt->sampleBuffer.w, pt->sampleBuffer.h));

  uint32_t idx = x + y * pt->sampleBuffer.w;
  Reservoir Rs = pt->temporalReservoirBufferGI[idx];
  Vector3D q_x_v = pt->initialSampleBuffer[idx].x_v;
  Vector3D q_n_v = pt->initialSampleBuffer[idx].n_v;
  SampleMetadata qm = pt->initialSampleMetadataBuffer[idx];
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
    if (sample_x >= pt->sampleBuffer.w || sample_y >= pt->sampleBuffer.h || (sample_x == x && sample_y == y)) continue;

    // Retrieve the reservoir from the neighboring pixel
    int idxn = sample_x + sample_y * pt->sampleBuffer.w;
    Reservoir Rn = pt->temporalReservoirBufferGI[idxn];
    Sample qn = pt->initialSampleBuffer[Rn.z];
    // Discard sample if it failed to intersect second bounce
    if (qn.L.x == 0 && qn.L.y == 0 && qn.L.z == 0) continue;

    // We count this as a sample
    retries = 0;
    s++;

    Sample qs = pt->initialSampleBuffer[Rs.z];
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
    if (has_intersect(pt->bvh, &shadow_ray)) p_prime_q = 0;

    // Merge Rn into the current reservoir
    merge(&Rs, Rn, p_prime_q, rand_state);
  }

  Sample S = pt->initialSampleBuffer[Rs.z];
  float phat = illum(S.L);
  Rs.W = Rs.M * phat > 0 ? Rs.w / (Rs.M * phat) : 0;

  float costheta = fmaxf(vector3d_dot(q_n_v, vector3d_unit(vector3d_sub(S.x_s, q_x_v))), 0.0f);
  Vector3D L = vector3d_add(qm.emittance, vector3d_mul(vector3d_scale(qm.bsdf_f, costheta), vector3d_scale(S.L, Rs.W)));
  pt->sampleBuffer.pixel[idx] = {
    .data = L,
    .albedo = S.albedo,
    .normal = S.n_v,
  };
}
