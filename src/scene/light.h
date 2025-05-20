#ifndef CGL_STATICSCENE_LIGHT_H
#define CGL_STATICSCENE_LIGHT_H

#include "scene/geometry.h"
#include "util/vector.h"
#include "util/gpu_rand.h"

typedef struct {
  Vector3D radiance;
  CudaPrimitive triangle;
  float area;

  // if point light
  Vector3D position;
  Vector3D direction;
  bool is_point_light = false;
} CudaLight;

DEVICE static inline Vector3D sample_L(
    const CudaLight *light,
    const Vector3D  p,
    Vector3D*       wi,
    float*          distToLight,
    float*          pdf,
    RNGState*       rand_state,
    const Vector3D* vertices) {

  if (light->is_point_light) {
    // 1) Compute vector from shading point → point light
    Vector3D d    = vector3d_sub(light->position, p);
    float    dist = vector3d_norm(d);

    // 2) Return distance (minus epsilon) and direction
    *distToLight  = dist - EPS_F;
    *wi           = vector3d_scale(d, 1.0f / dist);

    // 3) Delta‐distribution pdf
    *pdf = 1.0f;

    // 4) Inverse‐square falloff of intensity
    float invDist2 = 1.0f / (dist * dist);
    return vector3d_scale(light->radiance, invDist2);

  } else {
    Vector3D p1 = vertices[light->triangle.i_p1];
    Vector3D p2 = vertices[light->triangle.i_p2];
    Vector3D p3 = vertices[light->triangle.i_p3];

    // 1) Uniformly sample a point on the triangle via barycentrics
    float r1 = next_float(rand_state);
    float r2 = next_float(rand_state);
    if (r1 + r2 > 1.0f) {
      r1 = 1.0f - r1;
      r2 = 1.0f - r2;
    }

    Vector3D p2mp1 = vector3d_sub(p2, p1), 
             p3mp1 = vector3d_sub(p3, p1);
    Vector3D samplePos = vector3d_add(p1, vector3d_add(vector3d_scale(p2mp1, r1), vector3d_scale(p3mp1, r2)));

    // 2) Compute direction & distance from shading point to the sample
    Vector3D d    = vector3d_sub(samplePos, p);
    float    dist = vector3d_norm(d);
    *distToLight  = dist - EPS_F;
    Vector3D dir  = vector3d_scale(d, 1.0f / dist);
    *wi           = dir;

    // 3) Compute triangle normal for the geometry term
    Vector3D N = vector3d_unit(vector3d_cross(p2mp1, p3mp1));

    // 4) Convert area‐pdf to solid‐angle pdf:
    //    pdf_ω = (distance²) / (area * cosθ)
    float cosTheta = fmaxf(vector3d_dot(N, vector3d_neg(dir)), 0.0f);
    *pdf = (dist * dist) / (light->area * cosTheta);

    // 5) Return the emitted radiance
    return light->radiance;
  }
}

DEVICE static inline bool light_has_intersect(
    const CudaLight *light,
    Ray*             r,
    const Vector3D*  p,
    const Vector3D*  N,
    const Vector3D*  vertices,
    float*           pdf) {
  if (light->is_point_light) {
    // cast a “delta” intersection: does the ray exactly pass through the point?
    Vector3D toLight = vector3d_sub(light->position, r->o);
    // project onto ray direction
    float t = vector3d_dot(toLight, r->d);
    if (t < r->min_t || t > r->max_t) 
      return false;
    // reconstruct hit point and ensure it's within epsilon of the light position
    Vector3D hitP = vector3d_add(r->o, vector3d_scale(r->d, t));
    if (vector3d_norm(vector3d_sub(hitP, light->position)) > EPS_F) 
      return false;
    // delta‐light ⇒ unit pdf
    *pdf = 1.0f;
    return true;

  } else {
    float t;
    bool hit = primitive_has_intersect(&light->triangle, r, vertices, t);
    if (hit) {
      Vector3D samplePos = vector3d_add(r->o, vector3d_scale(r->d, t));
      Vector3D d         = vector3d_sub(samplePos, *p);
      float    dist      = vector3d_norm(d);
      Vector3D dir       = vector3d_scale(d, 1.0f / dist);
      float cosTheta     = fmaxf(vector3d_dot(*N, vector3d_neg(dir)), 0.0f);
      *pdf = (dist * dist) / (light->area * cosTheta);
    }
    return hit;
  }
}

#endif  // CGL_STATICSCENE_LIGHT_H
