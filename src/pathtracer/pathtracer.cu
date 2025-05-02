#include "pathtracer.h"

using namespace CGL::SceneObjects;
using namespace CGL;

namespace CGL {

///< random state for each thread

DEVICE __inline__ void cosine_weighted_hemisphere_sample_3d(RNGState &rand_state, Vector3D *wi, double *pdf) {
  double Xi1 = next_double(rand_state);
  double Xi2 = next_double(rand_state);

  double r = sqrt(Xi1);
  double theta = 2. * PI * Xi2;
  *pdf = sqrt(1-Xi1) / PI;
  *wi = Vector3D(r*cos(theta), r*sin(theta), sqrt(1-Xi1));
}

DEVICE __inline__ Vector3D sample_L(const CudaLight *light,
                                    const Vector3D         p,
                                    Vector3D*              wi,
                                    double*                distToLight,
                                    double*                pdf,
                                    RNGState&              rand_state,
                                    const Vector3D*        vertices) {
  Vector3D p1 = vertices[light->triangle.i_p1];
  Vector3D p2 = vertices[light->triangle.i_p2];
  Vector3D p3 = vertices[light->triangle.i_p3];
  // 1) Uniformly sample a point on the triangle via barycentrics
  double r1 = next_double(rand_state);
  double r2 = next_double(rand_state);
  if (r1 + r2 > 1.0) {
    r1 = 1.0 - r1;
    r2 = 1.0 - r2;
  }
  Vector3D samplePos = p1
                     + (p2 - p1) * r1
                     + (p3 - p1) * r2;

  // 2) Compute direction & distance from shading point to the sample
  Vector3D d = samplePos - p;
  double  dist = d.norm();
  *distToLight = dist - EPS_D;
  Vector3D dir = d / dist;
  *wi = dir;

  // 3) Compute triangle normal for the geometry term
  Vector3D N = cross((p2 - p1), (p3 - p1)).unit();

  // 4) Convert area‐pdf to solid‐angle pdf:
  //    pdf_ω = (distance²) / (area * cosθ)
  double cosTheta = fmax(dot(N, -dir), 0.0);
  *pdf = (dist * dist) / (light->area * cosTheta);

  // 5) Return the emitted radiance
  return light->radiance;
}

DEVICE __inline__ Vector3D PathTracer::get_emission(const CudaIntersection &isect) {
  CudaBSDF &bsdf = bsdfs[isect.bsdf_idx];
  Vector2D uv = isect.uv;
  Vector3D emission = bsdf.emissiveFactor * bsdf.emissiveStrength;
  if (bsdf.emission_idx >= 0) {
    Vector4D tc = textures[bsdf.emission_idx].sample(uv);
    emission.x *= tc.x * tc.w;
    emission.y *= tc.y * tc.w;
    emission.z *= tc.z * tc.w;
  }
  return emission;
}

DEVICE __inline__ void PathTracer::perturb_normal(CudaIntersection &isect) {
  int normal_idx = bsdfs[isect.bsdf_idx].normal_idx;
  if (normal_idx < 0) return;

  Vector3D N = isect.n;
  Vector3D T = Vector3D(isect.tangent.x,
                        isect.tangent.y,
                        isect.tangent.z);
  T = (T - N * dot(N, T)).unit();
  Vector3D B = cross(N, T) * isect.tangent.w;

  Vector4D c = textures[normal_idx].sample(isect.uv);
  Vector3D n_tangent = Vector3D(c.x, c.y, c.z) * 2.0f - Vector3D(1.0f);

  Vector3D perturbed = (T * n_tangent.x +
                        B * n_tangent.y +
                        N * n_tangent.z).unit();

  Vector3D diff = perturbed - N;
  double diff_len = diff.norm();
  // use original if diff small to prevent flickering. TODO: better fix
  if (diff_len < 0.4) {
    isect.n = N;
  } else {
    isect.n = perturbed;
  }
}

// following code adapted from https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#appendix-b-brdf-implementation
// wo = V, wi = L
DEVICE __inline__ Vector3D PathTracer::f(const CudaIntersection &isect, const Vector3D &wo, const Vector3D &wi, double *occlusion) {
  CudaBSDF &bsdf = bsdfs[isect.bsdf_idx];
  Vector3D N = isect.n; // perturbed normal
  Vector2D uv = isect.uv;

  // 1) geometry terms
  Vector3D H = (wo + wi).unit(); // bisector
  double NoV   = dot(N, wo);
  double NoL   = dot(N, wi);
  if (NoV <= 0) return Vector3D(0.0);
  double NoH   = dot(N, H);
  double VoH   = dot(wo, H);
  double LoH   = dot(wi, H);

  // 2) get base texture
  Vector3D base = Vector3D(bsdf.baseColor.x,
                           bsdf.baseColor.y,
                           bsdf.baseColor.z);
  if (bsdf.tex_idx >= 0) {
    Vector4D t = textures[bsdf.tex_idx].sample(uv);
    base = base * Vector3D(t.x, t.y, t.z);
  }

  // 3) get metallic roughness
  double metal    = bsdf.metallic;
  double roughness= bsdf.roughness;
  if (bsdf.orm_idx >= 0) {
    Vector4D orm = textures[bsdf.orm_idx].sample(uv);
    metal     = orm.z;
    roughness = orm.y;
    *occlusion = orm.x;
  }

  // clamp values : safety
  metal     = clamp_device(metal,     0.0, 1.0);
  roughness = clamp_device(roughness, 0.04, 1.0); // avoid ->0
  double onemmetal = 1.0 - metal;

  double alpha = roughness * roughness;

  // 4) diffuse and specular components
  Vector3D c_diff = base * onemmetal;
  Vector3D f0 = onemmetal * Vector3D(0.04) + metal * base;
  Vector3D F = f0 + (Vector3D(1.0) - f0) * pow(1.0 - VoH, 5.0);
  Vector3D f_diffuse = (Vector3D(1.0) - F) * PI_R * c_diff;

  double D = D_compute(alpha, NoH);
  double V = G_compute(alpha, NoV, NoL, VoH, LoH) / (4.0 * NoV * NoL);
  Vector3D f_specular = F * D * V;

  return f_diffuse + f_specular;
}

// Importance‑sample both diffuse (Lambert) and GGX specular lobes of the metallic‑roughness BRDF.
// Returns f(wo, *wi), writes out *wi, *pdf, and *occlusion.
DEVICE __inline__ Vector3D PathTracer::sample_f(const CudaIntersection &isect,
                                                const Vector3D       &wo,
                                                Vector3D             *wi,
                                                double               *pdf,
                                                double               *occlusion,
                                                RNGState             &rand_state) {
  // 1) Material & normal
  const CudaBSDF &bsdf = bsdfs[isect.bsdf_idx];
  Vector3D N    = isect.n;
  Vector2D uv   = isect.uv;
  *occlusion    = 1.0;

  // 2) Base color
  Vector3D base = Vector3D(bsdf.baseColor.x,
                           bsdf.baseColor.y,
                           bsdf.baseColor.z);
  if (bsdf.tex_idx >= 0) {
    Vector4D t = textures[bsdf.tex_idx].sample(uv);
    base = base * Vector3D(t.x, t.y, t.z);
  }

  // 3) Metallic, roughness, occlusion from ORM
  double metal     = clamp_device(bsdf.metallic,  0.0, 1.0);
  double roughness = clamp_device(bsdf.roughness, 0.02,1.0);
  if (bsdf.orm_idx >= 0) {
    Vector4D orm = textures[bsdf.orm_idx].sample(uv);
    *occlusion   = orm.x;
    roughness    = orm.y;
    metal        = orm.z;
  }
  double onem = 1.0 - metal;

  // // 4) Visibility check
  double NoV = max(dot(N, wo), 0.0);
  if (NoV <= 0.0) {
    *pdf = 0.0;
    return Vector3D(0.0);
  }

  // 5) Precompute F₀ and mixture weights
  double alpha = roughness * roughness;
  Vector3D F0  = Vector3D(0.04) * onem + base * metal;

  double P_d = onem;  // diffuse weight
  double P_s = metal; // specular weight
  double w   = P_d + P_s;
  P_d /= w;
  P_s /= w;

  // 6) Randomly choose lobe
  double u = next_double(rand_state);
  if (u < P_d) {
    // ── DIFFUSE ──
    // sample cosine‑weighted hemisphere
    cosine_weighted_hemisphere_sample_3d(rand_state, wi, pdf);
    *pdf *= P_d;

    // evaluate BRDF
    Vector3D H     = (wo + *wi).unit();
    double VoH     = max(dot(wo, H), 0.0);
    Vector3D F_geo = F0 + (Vector3D(1.0) - F0) * pow(1.0 - VoH, 5.0);
    Vector3D c_diff = base * onem;
    return (Vector3D(1.0) - F_geo) * (1.0 / M_PI) * c_diff;
  } else {
    // ── SPECULAR (GGX) ──
    // (a) sample microfacet normal H via GGX NDF
    double r1 = next_double(rand_state);
    double r2 = next_double(rand_state);
    double phi      = 2.0 * M_PI * r1;
    double cosTheta = sqrt((1.0 - r2) / (1.0 + (alpha*alpha - 1.0) * r2));
    double sinTheta = sqrt(max(0.0, 1.0 - cosTheta*cosTheta));

    Matrix3x3 o2w;
    make_coord_space(o2w, N);
    Vector3D localH = Vector3D(sinTheta * cos(phi),
                               sinTheta * sin(phi),
                               cosTheta);
    Vector3D H = (o2w * localH).unit();

    // (b) reflect view vector about H
    *wi = reflect(-wo, H);

    // (c) compute PDF
    double NoH   = max(dot(N, H), 0.0);
    double VoH   = max(dot(wo, H), 0.0);
    double D     = D_compute(alpha, NoH);
    double pdf_H = D * NoH;
    double pdf_w = pdf_H / (4.0 * VoH);
    *pdf = pdf_w * P_s;

    // (d) evaluate microfacet BRDF
    double NoL = max(dot(N, *wi), 0.0);
    double G   = G_compute(alpha, NoV, NoL, VoH, max(dot(*wi, H), 0.0));
    Vector3D F_geo = F0 + (Vector3D(1.0) - F0) * pow(1.0 - VoH, 5.0);
    return F_geo * (D * G / (4.0 * NoV * NoL));
  }
}



DEVICE Vector3D PathTracer::estimate_direct_lighting_importance(Ray &r,
                                                const CudaIntersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;

  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out = Vector3D(0, 0, 0);
  Vector3D wi;
  double distToLight, pdf;

  //NOTE: wi here is in worldpsace,

  uint16_t x = blockIdx.x * blockDim.x + threadIdx.x;
  uint16_t y = blockIdx.y * blockDim.y + threadIdx.y;

  double occlusion; //ignored for dir lighting
  for (uint16_t i = 0; i < num_lights; i++) {
    CudaLight *light = &lights[i];
    for (int j = 0; j < ns_area_light; j++) {
      Vector3D radiance = sample_L(light, hit_p, &wi, &distToLight, &pdf, rand_states[x + y*sampleBuffer.w], this->bvh->vertices);
      Vector3D wi_o = w2o * wi;
      if (wi_o.z < 0 || radiance == 0) continue;
      Ray shadow_ray = Ray(hit_p, wi);
      shadow_ray.min_t = EPS_D;
      shadow_ray.max_t = distToLight;
      CudaIntersection light_isect;
      if (!bvh->intersect(shadow_ray, &light_isect)) {
        L_out += f(isect, w_out, wi_o, &occlusion) * radiance * abs_cos_theta(wi_o) / pdf;
      }
    }
  }

  L_out /= (num_lights * ns_area_light);
  return L_out;
}


#define RRT 0.7f

DEVICE Vector3D PathTracer::at_least_one_bounce_radiance(Ray& r, const CudaIntersection& isect_init) {
    Vector3D L_out_total(0.0);
    Vector3D throughput(1.0);
    Ray current_ray = r;
    CudaIntersection isect = isect_init;
    bool first_bounce = true;

    // constant index since x,y don’t change across bounces
    int idx = current_ray.x + current_ray.y * sampleBuffer.w;

    while (true) {
        // build shading frame
        Matrix3x3 o2w;
        make_coord_space(o2w, isect.n);
        Matrix3x3 w2o = o2w.T();

        // hit point & outgoing dir in local space
        Vector3D hit_p  = current_ray.o + current_ray.d * isect.t;
        Vector3D w_out  = w2o * (-current_ray.d);

        // direct lighting
        Vector3D L_out = estimate_direct_lighting_importance(current_ray, isect);
        if (first_bounce) {
            initialSampleBuffer[idx].emittance = L_out;
        }
        L_out_total += throughput * L_out;

        // russian-roulette survival
        float p_survive = (current_ray.depth == 1) ? 1.0f : RRT;
        if (current_ray.depth > 1 &&
            next_double(rand_states[idx]) >= RRT)
            break;

        // sample BSDF
        Vector3D wi;
        double pdf;
        double occlusion = 1.0;
        Vector3D fcos = occlusion * sample_f(isect, w_out, &wi, &pdf, &occlusion, rand_states[idx]) * abs_cos_theta(wi);
        if (pdf <= 0.0)
            break;

        // update throughput
        throughput = throughput * (first_bounce ? 1.0 : fcos);
        throughput /= (pdf * p_survive);

        // spawn next ray
        Ray bounce_ray(hit_p, o2w * wi);
        bounce_ray.min_t = EPS_D;
        bounce_ray.depth = current_ray.depth + 1;
        bounce_ray.x = current_ray.x;
        bounce_ray.y = current_ray.y;

        CudaIntersection bounce_isect;
        if (!bvh->intersect(bounce_ray, &bounce_isect))
            break;

        // perturb_normal(bounce_isect);

        if (first_bounce) {
            Vector3D bounce_p = bounce_ray.o + bounce_ray.d * bounce_isect.t;
            Sample* s = &initialSampleBuffer[idx];
            s->x_v   = hit_p;
            s->n_v   = isect.n;
            s->x_s   = bounce_p;
            s->n_s   = bounce_isect.n;
            s->pdf   = pdf;
            s->fcos  = fcos;
        }

        // prepare for next iteration
        current_ray = bounce_ray;
        isect       = bounce_isect;
        first_bounce = false;
    }

    return L_out_total - initialSampleBuffer[idx].emittance;
}

DEVICE Vector3D PathTracer::est_radiance_global_illumination(Ray &r) {
  CudaIntersection isect;
  Vector3D L_out;

  if (!bvh->intersect(r, &isect))
    return L_out;

  L_out = get_emission(isect) + at_least_one_bounce_radiance(r, isect);

  return L_out;
}

DEVICE void PathTracer::raytrace_pixel(uint16_t x, uint16_t y) {
  CudaIntersection isect;
  
  uint16_t num_samples = ns_aa;
  Ray r;
  uint16_t i = 1;
  initialSampleBuffer[x + y * sampleBuffer.w] = Sample();
  init_gpu_rng(rand_states[x + y * sampleBuffer.w], 1234 + x + y * sampleBuffer.w);
  do {
    Vector2D origin = Vector2D(x, y);
    Vector2D sample = origin + Vector2D(next_double(rand_states[x + y * sampleBuffer.w]), next_double(rand_states[x + y * sampleBuffer.w]));
    r = camera.generate_ray(sample.x / sampleBuffer.w, sample.y / sampleBuffer.h);
    r.depth = 1, r.x = x, r.y = y;
  } while (i++ != num_samples && !bvh->intersect(r, &isect));
  if (i == num_samples + 1) {
    initialSampleBuffer[x + y * sampleBuffer.w].L = Vector3D(0, 0, 0);
  } else {
    perturb_normal(isect);
    Vector3D L = at_least_one_bounce_radiance(r, isect);
    initialSampleBuffer[r.x + r.y * sampleBuffer.w].emittance += get_emission(isect);
    initialSampleBuffer[r.x + r.y * sampleBuffer.w].L = L;
  }
}

// Computes jacobian from s1->s2 as defined in Equation 11 of the ReSTIR-GI paper
DEVICE __inline__ double jacobian(const Sample& s1, const Sample& s2) {
    Vector3D xq1 = s1.x_v;
    Vector3D xq2 = s1.x_s;
    Vector3D xr1 = s2.x_v;

    Vector3D nq2 = s1.n_s;

    double cos_phi_q2 = fabs(dot(nq2, (xq1 - xq2).unit())); 
    double cos_phi_r2 = fabs(dot(nq2, (xr1 - xq2).unit()));

    double distance_q = (xq1 - xq2).norm2();
    double distance_r = (xr1 - xq2).norm2();

    return (cos_phi_r2 / cos_phi_q2) * (distance_q / distance_r);
}

DEVICE void PathTracer::temporal_resampling(uint16_t x, uint16_t y) {
  Sample S = initialSampleBuffer[x + y * sampleBuffer.w];
  Reservoir R = Reservoir();

  double w = p_hat(S);
  R.update(S, w, rand_states[x + y * sampleBuffer.w]);
  R.W = R.w / (R.M * p_hat(R.z));

  temporalReservoirBuffer[x + y * sampleBuffer.w] = R;
  spatialReservoirBuffer[x + y * sampleBuffer.w] = R;
}

DEVICE void PathTracer::spatial_resampling(uint16_t x, uint16_t y) {

  const uint16_t neighbouring_pixel_radius = floor(0.1 * min(sampleBuffer.w, sampleBuffer.h));

  Reservoir Rs = spatialReservoirBuffer[x + y * sampleBuffer.w];
  Sample q = initialSampleBuffer[x + y * sampleBuffer.w];
  RNGState rand_state = rand_states[x + y * sampleBuffer.w];
  const uint8_t max_neighbouring_samples = 9; // ReSTIR GI paper value without temporal sampling
  for (uint8_t s = 0; s < max_neighbouring_samples; s++) {
    // Randomly choose a neighbor pixel qn
    int window = 2 * neighbouring_pixel_radius + 1;
    uint16_t sample_x = x + static_cast<int>(next_double(rand_state) * window) - neighbouring_pixel_radius;
    uint16_t sample_y = y + static_cast<int>(next_double(rand_state) * window) - neighbouring_pixel_radius;

    // Ensure the sample is within the frame buffer bounds
    if (sample_x >= sampleBuffer.w || sample_y >= sampleBuffer.h) continue;

    // Retrieve the reservoir from the neighboring pixel
    Reservoir Rn = temporalReservoirBuffer[sample_x + sample_y * sampleBuffer.w];
    // Calculate geometric similarity between q and qn
    if (!are_geometrically_similar(q, Rn.z) || Rn.z.L == Vector3D(0, 0, 0)) continue;

    // Calculate |Jqn→q| (Jacobian determinant)
    double Jqn_to_q = jacobian(Rn.z, q); // Placeholder for actual Jacobian calculation

    // Calculate ˆp′q
    double p_prime_q = (p_hat(Rn.z)) / Jqn_to_q;

    // visibility test
    // if neighbour's path's point is invisible from the current path's point, p_prime_q = 0
    Ray shadow_ray(q.x_v, (Rn.z.x_s - q.x_v).unit());
    shadow_ray.min_t = EPS_D;
    shadow_ray.max_t = (Rn.z.x_s - q.x_v).norm() - EPS_D;
    CudaIntersection isect;
    if (bvh->intersect(shadow_ray, &isect)) p_prime_q = 0;

    // Merge Rn into the current reservoir
    Rs.merge(Rn, p_prime_q, rand_state);
  }

  double phat = p_hat(Rs.z);
  Rs.W = Rs.M * phat > 0 ? Rs.w / (Rs.M * phat) : 0;
  spatialReservoirBuffer[x + y * sampleBuffer.w] = Rs;
  rand_states[x + y * sampleBuffer.w] = rand_state;
}

DEVICE void PathTracer::render_final_sample(uint16_t x, uint16_t y) {
  Reservoir R = spatialReservoirBuffer[x + y * sampleBuffer.w];
  Sample S = R.z;
  Sample initial = initialSampleBuffer[x + y *  sampleBuffer.w];
  Vector3D L = initial.emittance + S.fcos * S.L * R.W;

  sampleBuffer.update_pixel(L, x, y);
}

}