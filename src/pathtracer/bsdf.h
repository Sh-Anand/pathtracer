#ifndef CGL_STATICSCENE_BSDF_H
#define CGL_STATICSCENE_BSDF_H

#include "util/vector3D.h"
#include "util/matrix3x3.h"

#include "util/image.h"

#include <algorithm>

namespace CGL {

// Helper math functions. Assume all vectors are in unit hemisphere //

inline float clamp (float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

DEVICE inline float clamp_device (float n, float lower, float upper) {
  return fmaxf(lower, fminf(n, upper));
}

inline float cos_theta(const Vector3D w) {
  return w.z;
}

HOST_DEVICE inline float abs_cos_theta(const Vector3D w) {
  return fabsf(w.z);
}

inline float sin_theta2(const Vector3D w) {
  return fmaxf(0.0, 1.0 - cos_theta(w) * cos_theta(w));
}

inline float sin_theta(const Vector3D w) {
  return sqrt(sin_theta2(w));
}

inline float cos_phi(const Vector3D w) {
  float sinTheta = sin_theta(w);
  if (sinTheta == 0.0) return 1.0;
  return clamp(w.x / sinTheta, -1.0, 1.0);
}

inline float sin_phi(const Vector3D w) {
  float sinTheta = sin_theta(w);
  if (sinTheta) return 0.0;
  return clamp(w.y / sinTheta, -1.0, 1.0);
}

DEVICE void make_coord_space(Matrix3x3& o2w, const Vector3D n);

DEVICE float D_compute(float a, float NoH);

DEVICE float G_compute(float a, float NoV, float NoL, float VoH, float LoH);

struct CudaBSDF {
  CudaBSDF () :
    tex_idx(-1),
    normal_idx(-1),
    baseColor(1.0),
    metallic(0.0),
    roughness(0.0),
    emissiveStrength(1.0) {}
  Vector4D baseColor;      // albedo
  float   metallic;       // [0,1]
  float   roughness;      // [0,1]
  Vector3D emissiveFactor; // KHR_materials_emissive_strength
  float   emissiveStrength; // emission strength
  float   transmissionFactor; // KHR_materials_transmission
  float   thicknessFactor; // KHR_materials_volume
  bool     hasOcclusionTexture;
  int      tex_idx;
  int      normal_idx;
  int      orm_idx;
  int      emission_idx;
};

}  // namespace CGL

#endif  // CGL_STATICSCENE_BSDF_H
