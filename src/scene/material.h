#ifndef CGL_STATICSCENE_BSDF_H
#define CGL_STATICSCENE_BSDF_H

#include "util/matrix.h"
#include "util/vector.h"

using namespace std;

// Helper math functions. Assume all vectors are in unit hemisphere //

inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

DEVICE inline float clampd (float n, float lower, float upper) {
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
  return clampf(w.x / sinTheta, -1.0, 1.0);
}

inline float sin_phi(const Vector3D w) {
  float sinTheta = sin_theta(w);
  if (sinTheta) return 0.0;
  return clampf(w.y / sinTheta, -1.0, 1.0);
}

typedef struct {
  Vector4D baseColor;      // albedo
  float   metallic;       // [0,1]
  float   roughness;      // [0,1]
  Vector3D emission; // KHR_materials_emissive_strength
  float   transmissionFactor; // KHR_materials_transmission
  float   thicknessFactor; // KHR_materials_volume
  bool     hasOcclusionTexture;
  int      tex_idx;
  int      normal_idx;
  int      orm_idx;
  int      emission_idx;
} CudaBSDF;

typedef struct {
    uint16_t width;
    uint16_t height;
    bool has_alpha;
    uint8_t * data;
} CudaTexture;

DEVICE static inline Vector4D sample(const CudaTexture tex, const Vector2D uv) {
    // wrap or clamp your UVs as needed
    float u_f = uv.x - floorf(uv.x);
    float v_f = uv.y - floorf(uv.y);

    int u = int(u_f * (tex.width  - 1) + 0.5f);
    int v = int(v_f * (tex.height - 1) + 0.5f);

    // clamp to valid
    u = max(0, min(u, tex.width  - 1));
    v = max(0, min(v, tex.height - 1));

    // compute byte index
    int comps = tex.has_alpha ? 4 : 3;
    size_t idx = (size_t(v) * tex.width + size_t(u)) * comps;
    const uint8_t *base = tex.data;

    uint8_t c[4];
    if (tex.has_alpha) {
    // RGBA8
    c[0] = base[idx + 0];
    c[1] = base[idx + 1];
    c[2] = base[idx + 2];
    c[3] = base[idx + 3];
    } else {
    // RGB8 → treat alpha = 255
    c[0] = base[idx + 0];
    c[1] = base[idx + 1];
    c[2] = base[idx + 2];
    c[3] = 255;
    }
    return Vector4D{c[0]*RGB_R, c[1]*RGB_R, c[2]*RGB_R, c[3]*RGB_R};
}

// following code copied from https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#appendix-b-brdf-implementation

// helper: GGX normal distribution
DEVICE inline float D_compute(float a, float NoH_raw) {
  float NoH = fmaxf(NoH_raw, 0.0); // Heaviside step function
  float a2 = a*a;
  float denom = NoH*NoH*(a2 - 1.0) + 1.0;
  return a2 / (PI * denom * denom);
}

DEVICE inline float G_compute(
    float a,    // α = roughness²
    float NoV,  // max(0, N·V)
    float NoL,  // max(0, N·L)
    float VoH,  // vector3d_dot(V, H)
    float LoH   // vector3d_dot(L, H)
) {
  // 1) visibility of microfacet only if H·V>0 and H·L>0
  if (VoH <= 0.0 || LoH <= 0.0) return 0.0;

  // 2) denominator terms per glTF spec:
  //    G₁(X) = (2·X) / (X + sqrt(α² + (1−α²)·X²))
  float a2       = a * a;
  float denomV  = NoV + sqrt(a2 + (1.0 - a2) * NoV * NoV);
  float denomL  = NoL + sqrt(a2 + (1.0 - a2) * NoL * NoL);
  float GV      = (2.0 * NoV) / denomV;
  float GL      = (2.0 * NoL) / denomL;
  return GV * GL;
}


#endif  // CGL_STATICSCENE_BSDF_H
