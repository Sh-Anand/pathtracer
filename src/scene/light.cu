#include "light.h"

 
DEVICE Vector3D CudaLight::sample_L(const Vector3D         p,
                                    Vector3D*              wi,
                                    float*                distToLight,
                                    float*                pdf,
                                    RNGState&              rand_state,
                                    const Vector3D*        vertices) {

if (is_cone_light) {
  // 1) vector from surface to light
  Vector3D d    = position - p;
  float   dist = d.norm();
  *distToLight  = dist - EPS_F;
  Vector3D dirP = d / dist;    // normalized
  *wi           = dirP;

  // 2) cosine of angle between light axis and this direction
  float cosTheta = dot(direction, dirP);
  cosTheta = fmaxf(-1.0f, fminf(1.0f, cosTheta));

  // precompute the cosine‐bounds of your cone angles
  float cosInner = cos(inner_cone_angle);
  float cosOuter = cos(outer_cone_angle);

  // 3) linear fall‑off in cosine‐space
  float falloff;
  if      (cosTheta >= cosInner) falloff = 1.0;
  else if (cosTheta <= cosOuter) falloff = 0.0;
  else                            falloff = (cosTheta - cosOuter)
                                          / (cosInner - cosOuter);

  // 4) delta‐light PDF
  *pdf = 1.0;

  // convert stored intensity→radiance via 1/r²:
  return radiance * falloff / (dist * dist);
} else {
    Vector3D p1 = vertices[triangle.i_p1];
    Vector3D p2 = vertices[triangle.i_p2];
    Vector3D p3 = vertices[triangle.i_p3];
    // 1) Uniformly sample a point on the triangle via barycentrics
    float r1 = next_float(rand_state);
    float r2 = next_float(rand_state);
    if (r1 + r2 > 1.0) {
      r1 = 1.0 - r1;
      r2 = 1.0 - r2;
    }
    Vector3D samplePos = p1
                      + (p2 - p1) * r1
                      + (p3 - p1) * r2;

    // 2) Compute direction & distance from shading point to the sample
    Vector3D d = samplePos - p;
    float  dist = d.norm();
    *distToLight = dist - EPS_F;
    Vector3D dir = d / dist;
    *wi = dir;

    // 3) Compute triangle normal for the geometry term
    Vector3D N = cross((p2 - p1), (p3 - p1)).unit();

    // 4) Convert area‐pdf to solid‐angle pdf:
    //    pdf_ω = (distance²) / (area * cosθ)
    float cosTheta = fmaxf(dot(N, -dir), 0.0);
    *pdf = (dist * dist) / (area * cosTheta);

    // 5) Return the emitted radiance
    return radiance;
  }
}

DEVICE bool CudaLight::has_intersect(Ray& r, const Vector3D &p, const Vector3D &N, const Vector3D* vertices, float *pdf) const {
  if (is_cone_light) {
      Vector3D toLight = position - r.o;
      float   t       = dot(toLight, r.d);
      if (t < r.min_t || t > r.max_t) return false;
      Vector3D hitP = r.o + r.d * t;
      if ((hitP - position).norm() > EPS_F) return false;

      float cosTheta = dot(direction, (position - r.o).unit());
      float cosOuter = cos(outer_cone_angle);
      *pdf = 1.0;
      return cosTheta >= cosOuter;
  } 
  else {
      float t;
      bool hit = this->triangle.has_intersect(r, vertices, t);
      if (hit) {
          Vector3D samplePos = r.o + r.d * t;
          Vector3D d = samplePos - p;
          float  dist = d.norm();
          Vector3D dir = d / dist;
          float cosTheta = fmaxf(dot(N, -dir), 0.0);
          *pdf = (dist * dist) / (area * cosTheta);
      } 
      return hit;
  }
}