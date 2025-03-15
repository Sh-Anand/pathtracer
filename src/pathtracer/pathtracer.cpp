#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
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
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out = Vector3D(0, 0, 0);
  Intersection light_isect;

  for (int i = 0; i < num_samples; i++) {
    Vector3D wi = hemisphereSampler->get_sample();
    Ray shadow_ray = Ray(hit_p, o2w * wi);
    shadow_ray.min_t = EPS_D;

    if (bvh->intersect(shadow_ray, &light_isect)) {
      Vector3D light_L = light_isect.bsdf->get_emission();
      L_out += isect.bsdf->f(w_out, wi) * light_L * dot(wi, w2o*isect.n) * 2 * PI;
    }
  }

  L_out /= num_samples;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  return L_out;
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
      Ray shadow_ray = Ray(hit_p, wi);
      shadow_ray.min_t = EPS_D;
      shadow_ray.max_t = distToLight;
      if (!bvh->intersect(shadow_ray, &light_isect)) {
        L_out += isect.bsdf->f(w_out, w2o*wi) * radiance * dot(wi, isect.n) / pdf;
      }
    }
  }

  L_out /= sample_count;
  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light

  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  if (direct_hemisphere_sample) {
    return estimate_direct_lighting_hemisphere(r, isect);
  } else {
    return estimate_direct_lighting_importance(r, isect);
  }
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

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces
  bool ignore_RRT = r.depth == 1 && max_ray_depth > 1;
  bool not_terminate = ignore_RRT || (r.depth < max_ray_depth && (!isAccumBounces || coin_flip(RRT))); 

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
    if (isAccumBounces) {
      L_out = one_bounce_radiance(r, isect) + f * abs_cos_theta(wi) * bounce_radiance / pdf / RRT;
    } else {
      L_out = f * abs_cos_theta(wi) * bounce_radiance / pdf / RRT;
    }
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

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;


  if (max_ray_depth == 0)
    L_out = zero_bounce_radiance(r, isect);
  else if (isAccumBounces)
    L_out = zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
  else
    L_out = at_least_one_bounce_radiance(r, isect);
  // TODO (Part 3): Return the direct illumination.
  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"
  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  Vector3D pixel_sum = Vector3D(0, 0, 0);

  for (int i = 0; i < num_samples; i++) {
    Vector2D sample = origin + gridSampler->get_sample();
    Ray r = camera->generate_ray(sample.x / sampleBuffer.w, sample.y / sampleBuffer.h);
    r.depth = 1;
    pixel_sum += est_radiance_global_illumination(r);
  }
  pixel_sum /= num_samples;

  sampleBuffer.update_pixel(pixel_sum, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
