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
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
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
  Vector3D L_out(0, 0, 0);

  bool ignore_RRT = r.depth == 1 && max_ray_depth > 1;
  bool not_terminate = ignore_RRT || (r.depth < max_ray_depth && coin_flip(RRT)); 

  if (!not_terminate) {
    return one_bounce_radiance(r, isect);
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
    L_out = one_bounce_radiance(r, isect) + f * abs_cos_theta(wi) * bounce_radiance / pdf / RRT;
  } else {
    L_out = one_bounce_radiance(r, isect);
  }
  
  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;

  L_out = zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  int num_samples = ns_aa;
  Vector2D origin = Vector2D(x, y);
  Vector3D pixel_sum = Vector3D(0, 0, 0);

  double s1 = 0, s2 = 0;
  int i = 0;
  for (; i < num_samples; i++) {
    if (i > 0 && i % samplesPerBatch == 0) {
      double one_over_i = 1.0 / i, one_over_im1 = 1.0 / (i - 1);
      double mean = s1 * one_over_i;
      double var = one_over_im1 * (s2 - s1 * s1 * one_over_i);
      double I = 1.96*sqrt(var*one_over_i);
      if (I <= maxTolerance * mean) {
        break;
      }
    }
    Vector2D sample = origin + gridSampler->get_sample();
    Ray r = camera->generate_ray(sample.x / sampleBuffer.w, sample.y / sampleBuffer.h);
    r.depth = 1;
    Vector3D radiance = est_radiance_global_illumination(r);
    double x = radiance.illum();
    s1 += x;
    s2 += x * x;
    pixel_sum += radiance;
  }
  pixel_sum /= (double)i;

  sampleBuffer.update_pixel(pixel_sum, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = i;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
