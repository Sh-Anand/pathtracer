#include "pathtracer.h"

DEVICE static inline void make_coord_space(Matrix3x3 *o2w, const Vector3D n) {

  Vector3D z = Vector3D{n.x,n.y,n.z};
  Vector3D h = z;
  if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
    h.x = 1.0;
  else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
    h.y = 1.0;
  else
    h.z = 1.0;

  z = vector3d_unit(z);
  Vector3D y = vector3d_unit(vector3d_cross(h, z));
  Vector3D x = vector3d_unit(vector3d_cross(z, y));

  (*o2w).c[0] = x;
  (*o2w).c[1] = y;
  (*o2w).c[2] = z;
}

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

// following code adapted from https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#appendix-b-brdf-implementation
// wo = V, wi = L
DEVICE static inline Vector3D f( const CudaBSDF *bsdfs,
                          const CudaTexture *textures,
                          const CudaIntersection isect,
                          const Vector3D wo,
                          const Vector3D wi,
                          float *occlusion ) {
  // always initialize occlusion
  *occlusion = 1.0f;

  // fetch material
  CudaBSDF bsdf = bsdfs[isect.bsdf_idx];
  Vector3D N    = isect.n;  
  Vector2D uv   = isect.uv;

  // 1) geometry terms
  Vector3D H   = vector3d_unit(vector3d_add(wo, wi));
  float NoV    = fabsf( vector3d_dot(N, wo) );
  float NoL    = fabsf( vector3d_dot(N, wi) );
  if (NoL == 0.0f || NoV == 0.0f)
    return Vector3D{};                  // zero vector

  float NoH    = vector3d_dot(N, H);
  float VoH    = vector3d_dot(wo, H);
  float LoH    = vector3d_dot(wi, H);

  // 2) base color (albedo)
  Vector3D base{ bsdf.baseColor.x,
                 bsdf.baseColor.y,
                 bsdf.baseColor.z };
  if (bsdf.tex_idx >= 0) {
    Vector4D t = sample_texture(textures[bsdf.tex_idx], uv);
    base       = Vector3D{base.x * t.x, base.y * t.y, base.z * t.z};   // component‑wise
  }

  // 3) metallic/roughness/Occlusion
  float metal     = bsdf.metallic;
  float roughness = bsdf.roughness;
  if (bsdf.orm_idx >= 0) {
    Vector4D orm = sample_texture(textures[bsdf.orm_idx], uv);
    metal        = orm.z;
    roughness    = orm.y;
    *occlusion   = orm.x;
  }

  // clamp & compute derived quantities
  metal     = clampd(metal,     0.0f, 1.0f);
  roughness = clampd(roughness, 0.04f,1.0f);
  float onemmetal = 1.0f - metal;
  float alpha     = roughness * roughness;

  // 4) diffuse term
  Vector3D c_diff  = vector3d_scale(base, onemmetal);
  float sonemmetal = 0.04f * onemmetal;
  Vector3D f0      = Vector3D{sonemmetal + base.x * metal, sonemmetal + base.y * metal, sonemmetal + base.z * metal};
  Vector3D F       = vector3d_add(f0, vector3d_scale((Vector3D{1 - f0.x, 1 - f0.y, 1 - f0.z}), powf(1.0f - VoH, 5.0f)));
  Vector3D one_mF  = Vector3D{1 - F.x, 1 - F.y, 1 - F.z};
  Vector3D diffTerm= vector3d_scale(one_mF, PI_R);
  Vector3D f_diffuse = vector3d_mul(diffTerm, c_diff);   // component‑wise

  // 5) specular term
  float D = D_compute(alpha, NoH);
  float G = G_compute(alpha, NoV, NoL, VoH, LoH) / (4.0f * NoV * NoL);
  Vector3D f_specular = vector3d_scale(F, D * G);

  return vector3d_add(f_diffuse, f_specular);
}


DEVICE static inline Vector3D sample_f(const CudaBSDF* bsdfs,
                                const CudaTexture* textures,
                                const CudaIntersection isect, // isect.n is the world-space shading normal
                                const Vector3D wo_local,   // Outgoing view vector in local shading frame of isect.n
                                Vector3D       *wi_local,  // Output: Incoming light vector in local shading frame of isect.n
                                float         *pdf,        // Output: PDF of sampling wi_local
                                float         *occlusion,
                                RNGState       *rand_state) {
    // Material properties fetched using isect.bsdf_idx and isect.uv
    const CudaBSDF &bsdf = bsdfs[isect.bsdf_idx];
    Vector2D uv = isect.uv;

    Vector3D base_color = Vector3D{bsdf.baseColor.x, bsdf.baseColor.y, bsdf.baseColor.z};
    if (bsdf.tex_idx >= 0) {
        Vector4D t = sample_texture(textures[bsdf.tex_idx], uv);
        base_color = Vector3D{base_color.x * t.x, base_color.y * t.y, base_color.z * t.z};
    }

    float metallic = clampd(bsdf.metallic, 0.0f, 1.0f);
    float roughness = clampd(bsdf.roughness, 0.04f, 1.0f); // Clamping roughness here
    *occlusion = 1.0f; // Default occlusion

    if (bsdf.orm_idx >= 0) {
        Vector4D orm = sample_texture(textures[bsdf.orm_idx], uv);
        *occlusion = orm.x;
        roughness = clampd(orm.y, 0.04f, 1.0f); // Clamp roughness from ORM
        metallic = clampd(orm.z, 0.0f, 1.0f);  // Clamp metallic from ORM
    }
    float one_minus_metallic = 1.0f - metallic;
    float alpha = roughness * roughness;

    // wo_local is in local shading space, where N_shading is (0,0,1)
    // N_dot_V_local is simply the z-component of wo_local
    float N_dot_V_local = fabsf(wo_local.z);
    if (N_dot_V_local == 0.0f) {
        *pdf = 0.0f;
        return Vector3D{};
    }

    Vector3D F0 = Vector3D{
        0.04f * one_minus_metallic + base_color.x * metallic,
        0.04f * one_minus_metallic + base_color.y * metallic,
        0.04f * one_minus_metallic + base_color.z * metallic
    };

    float F0_luminance = (F0.x + F0.y + F0.z) / 3.0f;
    float P_specular = clampd(F0_luminance, 0.0f, 1.0f); // Probability of choosing specular lobe
    float P_diffuse = 1.0f - P_specular;

    float lobe_choice_rand = next_float(rand_state);

    if (lobe_choice_rand < P_diffuse) {
        // --- DIFFUSE LOBE ---
        float r1 = next_float(rand_state); // for sqrt(ksi1) for r
        float r2 = next_float(rand_state); // for 2*pi*ksi2 for phi

        float cos_theta_sample = sqrtf(max(0.0f, 1.0f - r1)); // Cosine of inclination angle
        float sin_theta_sample = sqrtf(r1);                   // Sine of inclination angle (sqrt(1-cos^2))
        float phi_sample = 2.0f * M_PI * r2;                  // Azimuthal angle

        Vector3D sampled_wi_local;
        sampled_wi_local.x = sin_theta_sample * cosf(phi_sample);
        sampled_wi_local.y = sin_theta_sample * sinf(phi_sample);
        sampled_wi_local.z = cos_theta_sample; // Always positive as it's in upper hemisphere

        *wi_local = sampled_wi_local;

        if (sampled_wi_local.z <= 1e-6f) { // PDF would be zero or near-zero
            *pdf = 0.0f;
            return Vector3D{};
        }
        *pdf = (sampled_wi_local.z / M_PI) * P_diffuse; // pdf_diffuse = (cos_theta / PI) * P_diffuse

        // Calculate BRDF value f_r(wo_local, sampled_wi_local) for diffuse
        Vector3D H_local = vector3d_unit(vector3d_add(wo_local, sampled_wi_local));
        float V_dot_H_local = max(0.0f, vector3d_dot(wo_local, H_local));

        Vector3D F_schlick = vector3d_add(F0, vector3d_scale(Vector3D{1.0f - F0.x, 1.0f - F0.y, 1.0f - F0.z}, powf(1.0f - V_dot_H_local, 5.0f)));
        Vector3D c_diff = vector3d_scale(base_color, one_minus_metallic);

        Vector3D one_minus_F_schlick = Vector3D{1.0f - F_schlick.x, 1.0f - F_schlick.y, 1.0f - F_schlick.z};
        Vector3D brdf_numerator = vector3d_mul(c_diff, one_minus_F_schlick);
        return vector3d_scale(brdf_numerator, (1.0f / M_PI)); // c_diff * (1 - F) / PI

    } else {
        // --- SPECULAR LOBE (GGX) ---
        // Sample microfacet normal H_local in local shading space
        float r1 = next_float(rand_state); // For phi_h
        float r2 = next_float(rand_state); // For cos_theta_h

        float phi_h = 2.0f * M_PI * r1;
        float cos_theta_h = sqrtf((1.0f - r2) / (1.0f + (alpha * alpha - 1.0f) * r2));
        float sin_theta_h = sqrtf(max(0.0f, 1.0f - cos_theta_h * cos_theta_h));

        Vector3D H_local; // Sampled microfacet normal in local shading space
        H_local.x = sin_theta_h * cosf(phi_h);
        H_local.y = sin_theta_h * sinf(phi_h);
        H_local.z = cos_theta_h; // Always positive

        // Reflect wo_local around H_local to get wi_local
        *wi_local = vector3d_reflect(vector3d_neg(wo_local), H_local);

        if (wi_local->z <= 1e-6f) { // Reflection goes below surface horizon
            *pdf = 0.0f;
            return Vector3D{};
        }

        // Calculate PDF of sampling wi_local
        float N_dot_H_local = H_local.z; // Since H_local is in shading space, N_shading is (0,0,1)
        float V_dot_H_local = max(0.0f, vector3d_dot(wo_local, H_local));

        if (V_dot_H_local == 0.0f) {
            *pdf = 0.0f;
            return Vector3D{};
        }
        float D_ggx = D_compute(alpha, N_dot_H_local); // D_compute needs N.H
        float pdf_H = D_ggx * N_dot_H_local;           // PDF_GGX(H) = D(H) * (N.H)
        *pdf = (pdf_H / (4.0f * V_dot_H_local)) * P_specular;

        // Calculate BRDF value f_r(wo_local, *wi_local) for specular
        float N_dot_L_local = wi_local->z; // Since *wi_local is in shading space
        // V_dot_H_local already computed
        // L_dot_H_local is also V_dot_H_local because wi is a perfect reflection of -wo around H

        Vector3D F_schlick = vector3d_add(F0, vector3d_scale(Vector3D{1.0f - F0.x, 1.0f - F0.y, 1.0f - F0.z}, powf(1.0f - V_dot_H_local, 5.0f)));
        float G_smith = G_compute(alpha, N_dot_V_local, N_dot_L_local, V_dot_H_local, V_dot_H_local); // G_compute must be Smith G term

        // Specular BRDF = F * D * G / (4 * (N.V) * (N.L))
        // N_dot_V_local = fabsf(wo_local.z)
        // N_dot_L_local = fabsf(wi_local->z) (or just wi_local.z if always positive)
        // The G_compute in f() was divided by (4*NoV*NoL), so f_specular = F*D*G_term_from_f
        // So, here: F_schlick * D_ggx * G_smith / (4 * N_dot_V_local * N_dot_L_local)
        float denominator = 4.0f * N_dot_V_local * N_dot_L_local;
        if (denominator == 0.0f) {
             // Already handled by N_dot_V_local == 0 or wi_local->z <= 0 checks
            return Vector3D{};
        }
        Vector3D specular_term_numerator = vector3d_scale(F_schlick, D_ggx * G_smith);
        return vector3d_scale(specular_term_numerator, 1.0f / denominator);
    }
}

// power­-heuristic MIS weight, β=2
DEVICE static inline float mis_weight(float pA, float pB) {
  float wA = pA*pA;
  float wB = pB*pB;
  return wA / (wA + wB);
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


DEVICE static inline Vector3D estimate_direct_lighting_importance(PathTracer* pt, Ray r, const CudaIntersection isect) {
  Matrix3x3 o2w;

  make_coord_space(&o2w, isect.n);
  Matrix3x3 w2o = matrix3x3_transpose(&o2w);

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = ray_at(r, isect.t);
  const Vector3D minus_out_dir = vector3d_neg(r.d);
  const Vector3D w_out =  matrix3x3_vector_multiply(&w2o, &minus_out_dir);
  Vector3D L_out = Vector3D{};
  //NOTE: wi here is in worldpsace,

  uint16_t x = blockIdx.x * blockDim.x + threadIdx.x;
  uint16_t y = blockIdx.y * blockDim.y + threadIdx.y;

  float occlusion; //ignored for dir lighting
  for (int i = 0; i < pt->num_lights; ++i) {
    CudaLight L = pt->lights[i];
    Vector3D wi;
    float   distToL, pdfL;
    Vector3D Li = sample_L(&L, hit_p, &wi, &distToL, &pdfL,
                           &pt->rand_states[x + y * pt->sampleBuffer.w], pt->bvh->vertices);

    float cosNL = fmaxf(vector3d_dot(isect.n, wi), 0.0);
    if (pdfL > 0 && cosNL > 0) {
      // shadow test
      Ray shadow; shadow.o = hit_p; shadow.d = wi; shadow.inv_d = vector3d_rcp(wi); shadow.depth = 0;
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
                             &occlusion, &pt->rand_states[x + y * pt->sampleBuffer.w]);
  float cosNL = fmaxf(vector3d_dot(isect.n, wi_bsdf), 0.0);

  if (pdfB > 0 && cosNL > 0) {
    // trace a ray in that direction and see if it hits *any* light
    Ray shadow; shadow.o = hit_p; shadow.d = wi_bsdf; shadow.inv_d = vector3d_rcp(wi_bsdf); shadow.depth = 0;
    shadow.min_t = EPS_F;
    shadow.max_t = INFINITY;

    for (int i = 0; i < pt->num_lights; ++i) {
      CudaLight L = pt->lights[i];
      float pdfL;
      if (light_has_intersect(L, &shadow, hit_p, isect.n, pt->bvh->vertices, &pdfL)) {
        // get the light and compute its PDF for this direction
        Vector3D Li = L.radiance;
        float w    = mis_weight(pdfB, pdfL);
        L_out = vector3d_add(L_out, vector3d_mul(f_bsdf, vector3d_scale(Li, (cosNL * w / pdfB))));
      }
    }
    
  }

  return L_out;
}


#define RRT 0.7f

DEVICE static inline Vector3D at_least_one_bounce_radiance(PathTracer *pt, Ray r, const CudaIntersection isect_init) {
  Vector3D L_out_total{0.0, 0.0, 0.0};
  Vector3D throughput{1.0, 1.0, 1.0};
  Ray current_ray = r;
  CudaIntersection isect = isect_init;

  // constant index since x,y don’t change across bounces
  int idx = current_ray.x + current_ray.y * pt->sampleBuffer.w;
  pt->rays_traced[idx] = 0;
  uint8_t level = 1;
  while (level++ <= pt->max_ray_depth) {
    pt->rays_traced[idx]++;
    // build shading frame
    Matrix3x3 o2w;
    make_coord_space(&o2w, isect.n);
    Matrix3x3 w2o = matrix3x3_transpose(&o2w);

    // hit point & outgoing dir in local space
    Vector3D hit_p  = ray_at(current_ray, isect.t);
    Vector3D minus_out_dir = vector3d_neg(current_ray.d);
    Vector3D w_out  = matrix3x3_vector_multiply(&w2o, &minus_out_dir);

    // direct lighting
    Vector3D L_out = estimate_direct_lighting_importance(pt, current_ray, isect);

    L_out_total = vector3d_add(L_out_total, vector3d_mul(throughput, L_out));

    // russian-roulette survival
    float p_survive = (current_ray.depth == 1) ? 1.0f : RRT;
    if (current_ray.depth > 1 &&
        next_float(&pt->rand_states[idx]) >= RRT)
        break;

    // sample BSDF
    Vector3D wi{0,0,0};
    float pdf;
    float occlusion = 1.0;
    Vector3D fcos = sample_f(pt->bsdfs, pt->textures, isect, w_out, &wi, &pdf, &occlusion, &pt->rand_states[idx]);
    fcos = vector3d_scale(fcos, abs_cos_theta(wi) * occlusion);
    if (pdf <= 0.0)
        break;

    // update throughput
    throughput = vector3d_mul(throughput, fcos);
    throughput = vector3d_scale(throughput, 1 / (pdf * p_survive));

    // spawn next ray
    Ray bounce_ray; bounce_ray.o = hit_p; bounce_ray.d = matrix3x3_vector_multiply(&o2w, &wi); bounce_ray.max_t = INFINITY; bounce_ray.inv_d = vector3d_rcp(bounce_ray.d);
    bounce_ray.min_t = EPS_F;
    bounce_ray.depth = current_ray.depth + 1;
    bounce_ray.x = current_ray.x;
    bounce_ray.y = current_ray.y;

    CudaIntersection bounce_isect; bounce_isect.t = INFINITY;
    if (!intersect(pt->bvh, &bounce_ray, &bounce_isect))
        break;

    // prepare for next iteration
    current_ray = bounce_ray;
    isect       = bounce_isect;
  }

  return L_out_total;
}

DEVICE static inline void raytrace_pixel(PathTracer *pt, uint16_t x, uint16_t y) {
  CudaIntersection isect; isect.t = INFINITY;
  
  Ray r;
  uint32_t idx = x + y * pt->sampleBuffer.w;
  // if (x <= 350 || y <= 350 || x >= 360 || y >= 360) {
  //   pt->sampleBuffer.data[idx] = Vector3D{0.0f, 0.0f, 0.0f};
  //   return;
  // }
  // if (x != 351 || y != 353) {
  //   pt->sampleBuffer.data[idx] = Vector3D{0.0f, 0.0f, 0.0f};
  //   return;
  // }
  init_gpu_rng(&pt->rand_states[idx], 1234 + idx);
  size_t spp = pt->ns_aa;
  Vector3D rad = {0.0f, 0.0f, 0.0f};
  float r_spp = 1.0f / spp;
  for (int sap = 0; sap < spp; sap++) {
    Vector2D origin = Vector2D{float(x), float(y)};
    Vector2D sample;
    sample.x = origin.x + next_float(&pt->rand_states[idx]);
    sample.y = origin.y + next_float(&pt->rand_states[idx]);
    r = generate_ray(&pt->camera, sample.x / pt->sampleBuffer.w, sample.y / pt->sampleBuffer.h);
    r.depth = 1, r.x = x, r.y = y;

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
      Vector3D zero_bounce = get_emission(pt->bsdfs, pt->textures, isect);
      Vector3D L = vector3d_add(zero_bounce, at_least_one_bounce_radiance(pt, r, isect));
      rad = vector3d_add(rad, L);
    }
  }
  rad = vector3d_scale(rad, r_spp);
  pt->sampleBuffer.data[idx] = rad;
}