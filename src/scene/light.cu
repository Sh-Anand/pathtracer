#include "light.h"

namespace CGL { namespace SceneObjects {
  DEVICE Vector3D CudaLight::sample_L(const Vector3D         p,
                                      Vector3D*              wi,
                                      double*                distToLight,
                                      double*                pdf,
                                      RNGState&              rand_state,
                                      const Vector3D*        vertices) {

  if (is_cone_light) {
    // 1) vector from surface to light
    Vector3D d    = position - p;
    double   dist = d.norm();
    *distToLight  = dist - EPS_D;
    Vector3D dirP = d / dist;    // normalized
    *wi           = dirP;

    // 2) cosine of angle between light axis and this direction
    double cosTheta = dot(direction, dirP);
    cosTheta = fmax(-1.0, fmin(1.0, cosTheta));

    // precompute the cosine‐bounds of your cone angles
    double cosInner = cos(inner_cone_angle);
    double cosOuter = cos(outer_cone_angle);

    // 3) linear fall‑off in cosine‐space
    double falloff;
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
      *pdf = (dist * dist) / (area * cosTheta);

      // 5) Return the emitted radiance
      return radiance;
    }
  }

  DEVICE bool CudaLight::has_intersect(Ray& r, const Vector3D &p, const Vector3D &N, const Vector3D* vertices, double *pdf) const {
    if (is_cone_light) {
        Vector3D toLight = position - r.o;
        double   t       = dot(toLight, r.d);
        if (t < r.min_t || t > r.max_t) return false;
        Vector3D hitP = r.o + r.d * t;
        if ((hitP - position).norm() > EPS_D) return false;

        double cosTheta = dot(direction, (position - r.o).unit());
        double cosOuter = cos(outer_cone_angle);
        *pdf = 1.0;
        return cosTheta >= cosOuter;
    } 
    else {
        double t;
        bool hit = this->triangle.has_intersect(r, vertices, t);
        if (hit) {
            Vector3D samplePos = r.o + r.d * t;
            Vector3D d = samplePos - p;
            double  dist = d.norm();
            Vector3D dir = d / dist;
            double cosTheta = fmax(dot(N, -dir), 0.0);
            *pdf = (dist * dist) / (area * cosTheta);
        } 
        return hit;
    }
  }
}
}