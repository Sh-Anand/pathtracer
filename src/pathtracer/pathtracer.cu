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

DEVICE __inline__ Vector3D sample_f(const CudaBSDF *bsdf, const Vector3D wo, Vector3D *wi, double *pdf, RNGState &rand_state) {
  cosine_weighted_hemisphere_sample_3d(rand_state, wi, pdf);
  return bsdf->f(wo, *wi);
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
  *distToLight = dist;
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

  //NOTE: wi here is in worldpsace, unlike in the previous function

  uint16_t x = blockIdx.x * blockDim.x + threadIdx.x;
  uint16_t y = blockIdx.y * blockDim.y + threadIdx.y;
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
        L_out += bsdfs[isect.bsdf_idx].f(w_out, wi_o) * radiance * abs_cos_theta(wi_o) / pdf;
      }
    }
  }

  L_out /= (num_lights * ns_area_light);
  return L_out;
}


#define RRT 0.7

DEVICE Vector3D PathTracer::at_least_one_bounce_radiance(Ray& r, const CudaIntersection& isect_init) {
  Vector3D L_out_total(0.0);
  Vector3D throughput(1.0);
  Ray current_ray = r;
  CudaIntersection isect = isect_init;

  bool first_bounce = true;

  while (true) {
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    Vector3D hit_p = current_ray.o + current_ray.d * isect.t;
    Vector3D w_out = w2o * (-current_ray.d);
    Vector3D L_out = estimate_direct_lighting_importance(current_ray, isect);

    if (first_bounce) {
      initialSampleBuffer[current_ray.x + current_ray.y * sampleBuffer.w].emittance = L_out;
    }

    L_out_total += throughput * L_out;

    bool not_terminate = current_ray.depth == 1 ||
                         (current_ray.depth < max_ray_depth &&
                          next_double(rand_states[current_ray.x + current_ray.y * sampleBuffer.w]) < RRT);

    if (!not_terminate) break;

    Vector3D wi;
    double pdf;
    Vector3D f = sample_f(&bsdfs[isect.bsdf_idx], w_out, &wi, &pdf,
                            rand_states[current_ray.x + current_ray.y * sampleBuffer.w]);

    if (pdf == 0 || f == Vector3D(0.0)) break;

    throughput = throughput * f * abs_cos_theta(wi) / (pdf * RRT);

    Ray bounce_ray(hit_p, o2w * wi);
    bounce_ray.min_t = EPS_D;
    bounce_ray.depth = current_ray.depth + 1;
    bounce_ray.x = current_ray.x;
    bounce_ray.y = current_ray.y;

    CudaIntersection bounce_isect;
    if (!bvh->intersect(bounce_ray, &bounce_isect)) {
      break;
    }

    if (first_bounce) {
      Vector3D bounce_p = bounce_ray.o + bounce_ray.d * bounce_isect.t;
      Sample *s = &initialSampleBuffer[current_ray.x + current_ray.y * sampleBuffer.w];
      s->x_v = hit_p;
      s->n_v = isect.n;
      s->x_s = bounce_p;
      s->n_s = bounce_isect.n;
      s->pdf = pdf;
      s->fcos = f * abs_cos_theta(wi);
      s->L = throughput;  // matches bounce_radiance / pdf / RRT
    }

    current_ray = bounce_ray;
    isect = bounce_isect;
    first_bounce = false;
  }

  return L_out_total;
}




DEVICE Vector3D PathTracer::est_radiance_global_illumination(Ray &r) {
  CudaIntersection isect;
  Vector3D L_out;

  if (!bvh->intersect(r, &isect))
    return L_out;

  L_out = bsdfs[isect.bsdf_idx].get_emission() + at_least_one_bounce_radiance(r, isect);

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
    Vector3D L = at_least_one_bounce_radiance(r, isect);
    initialSampleBuffer[r.x + r.y * sampleBuffer.w].emittance += bsdfs[isect.bsdf_idx].get_emission();
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