#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"

#include <curand_kernel.h> 


using namespace CGL::SceneObjects;
using namespace CGL;

namespace CGL {

/**
 * This function creates a object space (basis vectors) from the normal vector
 */

DEVICE __inline__ void cosine_weighted_hemisphere_sample_3d(curandStatePhilox4_32_10 *rand_state, Vector3D *wi, float *pdf) {
  float Xi1 = curand_uniform(rand_state);
  float Xi2 = curand_uniform(rand_state);

  float r = sqrtf(Xi1);
  float theta = 2. * PI_F * Xi2;
  float z = sqrtf(1.0f - Xi1);
  *pdf = z / PI;
  *wi = Vector3D(r * cosf(theta), r * sinf(theta), z);
}

DEVICE __inline__ Vector3D sample_f(CudaDiffuseBSDF *bsdf, const Vector3D wo, Vector3D *wi, float *pdf, curandStatePhilox4_32_10 *rand_state) {
  cosine_weighted_hemisphere_sample_3d(rand_state, wi, pdf);
  return bsdf->f(wo, *wi);
}

DEVICE __inline__ Vector3D sample_f(CudaEmissionBSDF *bsdf, const Vector3D wo, Vector3D *wi, float *pdf, curandStatePhilox4_32_10 *rand_state) {
  cosine_weighted_hemisphere_sample_3d(rand_state, wi, pdf);
  return Vector3D();
}

DEVICE __inline__ Vector3D sample_L(CudaDirectionalLight *light, const Vector3D p, Vector3D* wi,
                                    float* distToLight, float* pdf) {
  *wi = light->dirToLight;
  *distToLight = INFINITY;
  *pdf = 1.0;
  return light->radiance;
}

DEVICE __inline__ Vector3D sample_L(CudaPointLight *light, const Vector3D p, Vector3D* wi,
                             float* distToLight,
                             float* pdf) {
  Vector3D d = light->position - p;
  *wi = d.unit();
  *distToLight = d.norm();
  *pdf = 1.0;
  return light->radiance;
}

DEVICE __inline__ Vector3D sample_L(CudaAreaLight *light, const Vector3D p, Vector3D* wi, 
                             float* distToLight, float* pdf, curandStatePhilox4_32_10 *rand_state) {
  Vector2D sample = Vector2D(curand_uniform(rand_state), curand_uniform(rand_state)) - Vector2D(0.5f, 0.5f);
  Vector3D d = light->position + sample.x * light->dim_x + sample.y * light->dim_y - p;
  double cosTheta = dot(d, light->direction);
  double sqDist = d.norm2();
  double dist = sqrtf(sqDist);
  *wi = d / dist;
  *distToLight = dist;
  *pdf = sqDist / (light->area * fabsf(cosTheta));
  return cosTheta < 0 ? light->radiance : Vector3D();
}

DEVICE __inline__ Vector3D PathTracer::p_sample_L(const CudaLight light, const Vector3D p,
                             Vector3D* wi, float* distToLight,
                             float* pdf, curandStatePhilox4_32_10 *rand_state) {
  switch (light.type) {
    case CudaLightType_Directional:
      return sample_L(&light_data->directional_lights[light.idx], p ,wi, distToLight, pdf);
    case CudaLightType_Point:
      return sample_L(&light_data->point_lights[light.idx], p, wi, distToLight, pdf);
    case CudaLightType_Area:
      return sample_L(&light_data->area_lights[light.idx], p, wi, distToLight, pdf, rand_state);
    default:
      return Vector3D(0, 0, 0);
  }
  return Vector3D();
}

DEVICE __inline__ Vector3D PathTracer::p_sample_f(CudaBSDF bsdf, const Vector3D wo, Vector3D *wi, float* pdf, curandStatePhilox4_32_10 *rand_state) {
  switch (bsdf.type) {
      case CudaBSDFType_Diffuse:
          return sample_f(&bvh->diffuse_bsdfs[bsdf.idx], wo, wi, pdf, rand_state);
      case CudaBSDFType_Emission:
          return sample_f(&bvh->emission_bsdfs[bsdf.idx], wo, wi, pdf, rand_state);
      default:
          return Vector3D(0, 0, 0);
  }
}

DEVICE Vector3D PathTracer::estimate_direct_lighting_importance(Ray &r,
                                                const CudaIntersection &isect, curandStatePhilox4_32_10 *rand_state) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out = Vector3D(0, 0, 0);
  Vector3D wi;
  float distToLight, pdf;

  //NOTE: wi here is in worldpsace, unlike in the previous function
  size_t sample_count = 0;
  for (uint32_t i = 0; i < num_lights; i++) {
    CudaLight light = lights[i];
    int num_samples = light_data->is_delta_light(light) ? 1 : ns_area_light;
    sample_count += num_samples;
    for (int j = 0; j < num_samples; j++) {
      Vector3D radiance = p_sample_L(light, hit_p, &wi, &distToLight, &pdf, rand_state);
      Vector3D wi_o = w2o * wi;
      if (wi_o.z < 0 || radiance == 0) continue;
      Ray shadow_ray = Ray(hit_p, wi);
      shadow_ray.min_t = EPS_F;
      shadow_ray.max_t = distToLight;
      CudaIntersection light_isect;
      if (!bvh->intersect(shadow_ray, &light_isect)) {
        L_out += bvh->f(isect.bsdf, w_out, wi_o) * radiance * abs_cos_theta(wi_o) / pdf;
      }
    }
  }

  L_out /= sample_count;
  return L_out;
}


#define RRT 0.7

DEVICE Vector3D PathTracer::at_least_one_bounce_radiance(Ray &r, const CudaIntersection &isect, curandStatePhilox4_32_10 *rand_state) {
  Vector3D final_radiance = Vector3D();
  Vector3D throughput = Vector3D(1.0f);
  
  Ray current_ray = r;
  CudaIntersection current_isect = isect;

  for (int depth = r.depth; depth <= max_ray_depth; depth++) {
    Matrix3x3 o2w;
    make_coord_space(o2w, current_isect.n);
    Matrix3x3 w2o = o2w.T();
    
    Vector3D hit_p = current_ray.o + current_ray.d * current_isect.t;
    Vector3D w_out = w2o * (-current_ray.d);
    
    // Direct lighting
    Vector3D L_direct = estimate_direct_lighting_importance(current_ray, current_isect, rand_state);
    final_radiance += throughput * L_direct;
    
    bool ignore_RRT = (depth == 1 && max_ray_depth > 1);
    bool survive = ignore_RRT || (depth < max_ray_depth && (curand_uniform(rand_state) < RRT));
    
    if (!survive)
      break;

    // Sample new direction
    Vector3D wi;
    float pdf;
    Vector3D f = p_sample_f(current_isect.bsdf, w_out, &wi, &pdf, rand_state);
    if (pdf == 0.0) break;

    throughput = throughput * f * abs_cos_theta(wi) / pdf / RRT;

    Ray bounce_ray = Ray(hit_p, o2w * wi);
    bounce_ray.min_t = EPS_F;
    bounce_ray.depth = depth + 1;

    CudaIntersection bounce_isect;
    if (!bvh->intersect(bounce_ray, &bounce_isect))
      break;

    current_ray = bounce_ray;
    current_isect = bounce_isect;
  }

  return final_radiance;
}


DEVICE void PathTracer::raytrace_pixel(uint32_t x, uint32_t y) {

  size_t idx = x + y * sampleBuffer.w;
  curandStatePhilox4_32_10 rand_state;
  curand_init(1234 + idx * 9781, idx, 0, &rand_state);
  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  Vector3D pixel_sum = Vector3D(0, 0, 0);
  int i = 0;
  for (; i < num_samples; i++) {
    Vector2D sample = origin + Vector2D(curand_uniform(&rand_state), curand_uniform(&rand_state));
    Ray r = camera.generate_ray(sample.x / sampleBuffer.w, sample.y / sampleBuffer.h);
    r.depth = 1;
    Vector3D radiance = Vector3D(0, 0, 0);
    CudaIntersection isect;
    if (bvh->intersect(r, &isect)) {
      radiance = bvh->get_emission(isect.bsdf) + at_least_one_bounce_radiance(r, isect, &rand_state);
    }
    pixel_sum += radiance;
  }
  pixel_sum /= (float)i;

  sampleBuffer.update_pixel(pixel_sum, x, y);
}
}