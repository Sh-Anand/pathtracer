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
  bool is_cone_light = false;
  float inner_cone_angle = 0.0f;
  float outer_cone_angle = 0.0f;
} CudaLight;

DEVICE static inline Vector3D sample_L(
    const CudaLight *light,
    const Vector3D  p,
    Vector3D*       wi,
    float*          distToLight,
    float*          pdf,
    RNGState*       rand_state,
    const Vector3D* vertices) {

  if (light->is_cone_light) {
    // 1) vector from surface to light
    Vector3D d    = vector3d_sub(light->position, p);
    float   dist = vector3d_norm(d);
    *distToLight  = dist - EPS_F;
    Vector3D dirP = vector3d_scale(d, 1.0f / dist);
    *wi           = dirP;

    // 2) cosine of angle between light axis and this direction
    float cosTheta = vector3d_dot(light->direction, dirP);
    cosTheta = fmaxf(-1.0f, fminf(1.0f, cosTheta));

    // precompute the cosine‐bounds of your cone angles
    float cosInner = cosf(light->inner_cone_angle);
    float cosOuter = cosf(light->outer_cone_angle);

    // 3) linear fall‑off in cosine‐space
    float falloff;
    if      (cosTheta >= cosInner) falloff = 1.0f;
    else if (cosTheta <= cosOuter) falloff = 0.0f;
    else                            falloff = (cosTheta - cosOuter)
                                           / (cosInner - cosOuter);

    // 4) delta‐light PDF
    *pdf = 1.0f;

    // convert stored intensity→radiance via 1/r²:
    return vector3d_scale(light->radiance, falloff / (dist * dist));
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

  if (light->is_cone_light) {
    Vector3D toLight = vector3d_sub(light->position, r->o);
    float   t       = vector3d_dot(toLight, r->d);
    if (t < r->min_t || t > r->max_t) return false;

    Vector3D hitP = vector3d_add(r->o, vector3d_scale(r->d, t));
    if (vector3d_norm(vector3d_sub(hitP, light->position)) > EPS_F) return false;

    float cosTheta = vector3d_dot(
      light->direction,
      vector3d_unit(toLight)
    );
    float cosOuter = cosf(light->outer_cone_angle);
    *pdf = 1.0f;
    return cosTheta >= cosOuter;
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
