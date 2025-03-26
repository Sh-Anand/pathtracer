#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new JitteredSampler2D(16);
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
  initialSampleBuffer.resize(width * height);
  temporalReservoirBuffer.resize(width * height);
  spatialReservoirBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  initialSampleBuffer.clear();
  temporalReservoirBuffer.clear();
  spatialReservoirBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
  initialSampleBuffer.resize(0);
  temporalReservoirBuffer.resize(0);
  spatialReservoirBuffer.resize(0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
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
  Intersection light_isect;
  Vector3D wi;
  double distToLight, pdf;

  //NOTE: wi here is in worldpsace, unlike in the previous function
  size_t sample_count = 0;
  for (auto light : scene->lights) {
    int num_samples = light->is_delta_light() ? 1 : ns_area_light;
    sample_count += num_samples;
    for (int j = 0; j < num_samples; j++) {
      Vector3D radiance = light->sample_L(hit_p, &wi, &distToLight, &pdf);
      Vector3D wi_o = w2o * wi;
      if (wi_o.z < 0 || radiance == 0) continue;
      Ray shadow_ray = Ray(hit_p, wi);
      shadow_ray.min_t = EPS_D;
      shadow_ray.max_t = distToLight;
      if (!bvh->intersect(shadow_ray, &light_isect)) {
        L_out += isect.bsdf->f(w_out, wi_o) * radiance * abs_cos_theta(wi_o) / pdf;
      }
    }
  }

  L_out /= sample_count;
  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
    return estimate_direct_lighting_importance(r, isect);
}

#define RRT 0.7

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);
  Vector3D L_out = one_bounce_radiance(r, isect);

  bool not_terminate = r.depth == 1 || (r.depth < max_ray_depth && coin_flip(RRT)); 

  if (r.depth == 1) {
    initialSampleBuffer[r.x + r.y * sampleBuffer.w].emittance = L_out;
  }

  if (!not_terminate) {
    return L_out;
  }

  Vector3D wi;
  double pdf;
  Vector3D f = isect.bsdf->sample_f(w_out, &wi, &pdf);
  
  Ray bounce_ray = Ray(hit_p, o2w * wi);
  bounce_ray.min_t = EPS_D;
  bounce_ray.depth = r.depth + 1;

  Intersection bounce_isect;
  if (bvh->intersect(bounce_ray, &bounce_isect)) {
    Vector3D bounce_radiance = at_least_one_bounce_radiance(bounce_ray, bounce_isect);
    Vector3D bounce_radiance_p = bounce_radiance / pdf / RRT;
    L_out += f * abs_cos_theta(wi) * bounce_radiance_p;

    // update initial sample
    if (r.depth == 1) {
      Vector3D bounce_p = bounce_ray.o + bounce_ray.d * bounce_isect.t;
      Sample *s = &initialSampleBuffer[r.x + r.y * sampleBuffer.w];
      s->x_v = hit_p; s->n_v = isect.n; s->x_s = bounce_p; s->n_s = bounce_isect.n;
      s->pdf = pdf; s->fcos = f * abs_cos_theta(wi);
      s->L = bounce_radiance_p;
    }
  }

  return L_out;
}

void PathTracer::est_radiance_global_illumination(size_t x, size_t y) {
  Intersection isect;
  
  size_t num_samples = ns_aa;
  Ray r;
  size_t i = 1;
  do {
    Vector2D origin = Vector2D(x, y);
    Vector2D sample = origin + gridSampler->get_sample();
    r = camera->generate_ray(sample.x / sampleBuffer.w, sample.y / sampleBuffer.h);
    r.depth = 1, r.x = x, r.y = y;
  } while (i++ != num_samples && !bvh->intersect(r, &isect));
  if (i == num_samples + 1) {
    initialSampleBuffer[x + y * sampleBuffer.w].L = envLight ? envLight->sample_dir(r) : Vector3D(0, 0, 0);
  } else {
    Vector3D L = at_least_one_bounce_radiance(r, isect);
    initialSampleBuffer[r.x + r.y * sampleBuffer.w].emittance += zero_bounce_radiance(r, isect);
  }
}

// Computes jacobian from s1->s2 as defined in Equation 11 of the ReSTIR-GI paper
double jacobian(const Sample& s1, const Sample& s2) {
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

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  est_radiance_global_illumination(x,y);          
}

const size_t max_neighbouring_samples = 9; // ReSTIR GI paper value without temporal sampling

void PathTracer::temporal_resampling(size_t x, size_t y) {
  Sample S = initialSampleBuffer[x + y * sampleBuffer.w];
  Reservoir R = temporalReservoirBuffer[x + y * sampleBuffer.w];

  double w = S.pdf > 0 ? p_hat(S) / S.pdf : 0;
  R.update(S, w);
  R.W = R.w / (R.M * p_hat(R.z));

  temporalReservoirBuffer[x + y * sampleBuffer.w] = R;
}

void PathTracer::spatial_resampling(size_t x, size_t y) {

  const size_t neighbouring_pixel_radius = static_cast<size_t>(0.1 * std::min(sampleBuffer.w, sampleBuffer.h));

  Reservoir Rs = spatialReservoirBuffer[x + y * sampleBuffer.w];
  Sample q = initialSampleBuffer[x + y * sampleBuffer.w];
  for (int s = 0; s < max_neighbouring_samples; s++) {
    // Randomly choose a neighbor pixel qn
    int sample_x = x + (rand() % (2 * neighbouring_pixel_radius + 1)) - neighbouring_pixel_radius;
    int sample_y = y + (rand() % (2 * neighbouring_pixel_radius + 1)) - neighbouring_pixel_radius;

    // Ensure the sample is within the frame buffer bounds
    if (sample_x >= sampleBuffer.w || sample_y >= sampleBuffer.h || sample_x < 0 || sample_y < 0) continue;

    // Calculate geometric similarity between q and qn
    Sample qn = initialSampleBuffer[sample_x + sample_y * sampleBuffer.w];
    if (!are_geometrically_similar(q, qn)) continue;

    // Retrieve the reservoir from the neighboring pixel
    Reservoir Rn = temporalReservoirBuffer[sample_x + sample_y * sampleBuffer.w];

    // Calculate |Jqn→q| (Jacobian determinant)
    double Jqn_to_q = jacobian(qn, q); // Placeholder for actual Jacobian calculation

    // Calculate ˆp′q
    double p_prime_q = p_hat(Rn.z) / Jqn_to_q;

    // visibility test
    // if neighbour's path's point is invisible from the current path's point, p_prime_q = 0
    Ray shadow_ray(q.x_v, (Rn.z.x_s - q.x_v).unit());
    shadow_ray.min_t = EPS_D;
    shadow_ray.max_t = (Rn.z.x_s - q.x_v).norm() - EPS_D;
    Intersection isect;
    if (bvh->intersect(shadow_ray, &isect)) p_prime_q = 0;

    // Merge Rn into the current reservoir
    Rs.merge(Rn, p_prime_q);
  }

  double phat = p_hat(Rs.z);
  Rs.W = Rs.M * phat > 0 ? Rs.w / (Rs.M * phat) : 0;
  spatialReservoirBuffer[x + y * sampleBuffer.w] = Rs;
}

void PathTracer::render_final_sample(size_t x, size_t y) {
  Reservoir R = spatialReservoirBuffer[x + y * sampleBuffer.w];
  Sample S = R.z;
  Sample initial = initialSampleBuffer[x + y *  sampleBuffer.w];
  Vector3D L = initial.emittance + initial.fcos * S.L * R.W;
  sampleBuffer.update_pixel(L, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = 1;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
