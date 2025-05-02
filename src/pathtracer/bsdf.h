#ifndef CGL_STATICSCENE_BSDF_H
#define CGL_STATICSCENE_BSDF_H

#include "util/vector3D.h"
#include "util/matrix3x3.h"

#include "util/image.h"

#include <algorithm>

namespace CGL {

// Helper math functions. Assume all vectors are in unit hemisphere //

inline double clamp (double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

DEVICE inline double clamp_device (double n, double lower, double upper) {
  return fmax(lower, fmin(n, upper));
}

inline double cos_theta(const Vector3D w) {
  return w.z;
}

HOST_DEVICE inline double abs_cos_theta(const Vector3D w) {
  return fabsf(w.z);
}

inline double sin_theta2(const Vector3D w) {
  return fmax(0.0, 1.0 - cos_theta(w) * cos_theta(w));
}

inline double sin_theta(const Vector3D w) {
  return sqrt(sin_theta2(w));
}

inline double cos_phi(const Vector3D w) {
  double sinTheta = sin_theta(w);
  if (sinTheta == 0.0) return 1.0;
  return clamp(w.x / sinTheta, -1.0, 1.0);
}

inline double sin_phi(const Vector3D w) {
  double sinTheta = sin_theta(w);
  if (sinTheta) return 0.0;
  return clamp(w.y / sinTheta, -1.0, 1.0);
}

DEVICE void make_coord_space(Matrix3x3& o2w, const Vector3D n);

DEVICE double D_compute(double a, double NoH);

DEVICE double G_compute(double a, double NoV, double NoL, double VoH, double LoH);

struct CudaBSDF {
  CudaBSDF () :
    tex_idx(-1),
    normal_idx(-1),
    baseColor(1.0),
    metallic(0.0),
    roughness(0.0),
    emissiveStrength(1.0) {}
  Vector4D baseColor;      // albedo
  double   metallic;       // [0,1]
  double   roughness;      // [0,1]
  Vector3D emissiveFactor; // KHR_materials_emissive_strength
  double   emissiveStrength; // emission strength
  double   transmissionFactor; // KHR_materials_transmission
  double   thicknessFactor; // KHR_materials_volume
  bool     hasOcclusionTexture;
  int      tex_idx;
  int      normal_idx;
  int      orm_idx;
  int      emission_idx;
};

}  // namespace CGL

#endif  // CGL_STATICSCENE_BSDF_H
