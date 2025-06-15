#ifndef CGL_STATICSCENE_BSDF_H
#define CGL_STATICSCENE_BSDF_H

#include "util/gpu_rand.h"
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

inline float abs_cos_theta(const Vector3D w) {
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

DEVICE static inline Vector4D sample_texture(const CudaTexture tex, const Vector2D uv) {
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
    float NoL   // max(0, N·L)
) {
  // 1) visibility of microfacet only if H·V>0 and H·L>0
  // must be checked upon function entry

  // 2) denominator terms per glTF spec:
  //    G₁(X) = (2·X) / (X + sqrt(α² + (1−α²)·X²))
  float a2       = a * a;
  float denomV  = NoV + sqrt(a2 + (1.0 - a2) * NoV * NoV);
  float denomL  = NoL + sqrt(a2 + (1.0 - a2) * NoL * NoL);
  float GV      = (2.0 * NoV) / denomV;
  float GL      = (2.0 * NoL) / denomL;
  return GV * GL;
}

// Build an orthonormal basis (T,B,N) from a unit-length normal N.
// Result: T and B are both unit-length, mutually orthogonal, and orthogonal to N.
//
// Reference: Duff et al., "Building an Orthonormal Basis, Revisited", Journal of
// Computer Graphics Techniques, 2017.  (Same math as PBRT & Cycles.)
DEVICE inline void coordinate_system(const Vector3D N,
                                     Vector3D *T, Vector3D *B)
{
  // Handle the sign of N.z to avoid precision loss when N is nearly ±Z.
  float sign = copysignf(1.0f, N.z);
  float a    = -1.0f / (sign + N.z);
  float b    = N.x * N.y * a;

  *T = Vector3D{
      1.0f + sign * N.x * N.x * a,
      sign * b,
      -sign * N.x};

  *B = Vector3D{
      b,
      sign + N.y * N.y * a,
      -N.y};
}

// following code adapted from https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#appendix-b-brdf-implementation
// wo = V, wi = L
DEVICE static inline Vector3D f( const CudaBSDF *bsdfs,
                          const CudaTexture *textures,
                          const CudaIntersection isect,
                          const Vector3D wo,
                          const Vector3D wi,
                          float *occlusion) {
  // always initialize occlusion
  *occlusion = 1.0f;

  // fetch material
  CudaBSDF bsdf = bsdfs[isect.bsdf_idx];
  Vector3D N    = isect.n;  
  Vector2D uv   = isect.uv;

  // 1) geometry terms
  Vector3D H   = vector3d_unit(vector3d_add(wo, wi));
  float NoV    = vector3d_dot(N, wo);
  float NoL    = vector3d_dot(N, wi);
  if (NoL <= 0.0f || NoV <= 0.0f)
    return Vector3D{};                  // zero vector

  float NoH    = vector3d_dot(N, H);
  float VoH    = vector3d_dot(wo, H);
  float LoH    = vector3d_dot(wi, H);

  if (NoH <= 0.0f || VoH <= 0.0f || LoH <= 0.0f)
    return Vector3D{};                  // zero vector

  // 2) base color (albedo)
  Vector3D base{ bsdf.baseColor.x,
                 bsdf.baseColor.y,
                 bsdf.baseColor.z };
  if (bsdf.tex_idx >= 0) {
    Vector4D t = sample_texture(textures[bsdf.tex_idx], uv);
    base       = Vector3D{base.x * t.x, base.y * t.y, base.z * t.z};   // component‑wise
  }

  // 3) metallic/roughness/Occlusion
  float metal     = bsdf.metallic;
  float roughness = bsdf.roughness;
  if (bsdf.orm_idx >= 0) {
    Vector4D orm = sample_texture(textures[bsdf.orm_idx], uv);
    metal        = orm.z;
    roughness    = orm.y;
    *occlusion   = orm.x;
  }

  // clamp & compute derived quantities
  metal     = clampd(metal,     0.0f, 1.0f);
  roughness = clampd(roughness, 0.04f,1.0f);
  float onemmetal = 1.0f - metal;
  float alpha     = roughness * roughness;

  // 4) diffuse term
  Vector3D c_diff  = vector3d_scale(base, onemmetal);
  float sonemmetal = 0.04f * onemmetal;
  Vector3D f0      = Vector3D{sonemmetal + base.x * metal, sonemmetal + base.y * metal, sonemmetal + base.z * metal};
  Vector3D F       = vector3d_add(f0, vector3d_scale((Vector3D{1 - f0.x, 1 - f0.y, 1 - f0.z}), powf(1.0f - fabsf(VoH), 5.0f)));
  Vector3D diffTerm  = Vector3D{(1 - F.x) * PI_R, (1 - F.y) * PI_R, (1 - F.z) * PI_R};
  Vector3D f_diffuse = vector3d_mul(diffTerm, c_diff);   // component‑wise

  // 5) specular term
  float D = D_compute(alpha, NoH);
  float G = VoH != 0 && LoH != 0 ? G_compute(alpha, NoV, NoL) / (4.0f * NoV * NoL) : 0.0f;
  Vector3D f_specular = vector3d_scale(F, D * G);

  return vector3d_add(f_diffuse, f_specular);
}

DEVICE inline void
compute_lobe_probs(const Vector3D  base,
                   float           metallic,
                   float           roughness,
                   float          *P_spec,   // out
                   float          *P_diff)   // out
{
    float onem  = 1.0f - metallic;
    Vector3D F0 = { 0.04f*onem + base.x*metallic,
                    0.04f*onem + base.y*metallic,
                    0.04f*onem + base.z*metallic };

    /* energy weights:   w_spec = lum(F0)
                         w_diff = lum((1−F0)(1−metal)·base)               */
    float w_spec = illum(F0);
    Vector3D one_mF0 = {1.f-F0.x, 1.f-F0.y, 1.f-F0.z};
    float w_diff = illum(vector3d_mul(one_mF0, vector3d_scale(base, onem)));

    float sum = w_spec + w_diff + 1e-6f;
    *P_spec = w_spec / sum;
    *P_diff = w_diff / sum;
}

// Importance‑sample both diffuse (Lambert) and GGX specular lobes of the metallic‑roughness BRDF.
// Returns f(wo, *wi), writes out *wi, *pdf, and *occlusion.
DEVICE static inline Vector3D sample_f(const CudaBSDF* bsdfs,
                                const CudaTexture* textures,
                                const CudaIntersection isect,
                                const Vector3D       wo,
                                Vector3D             *wi,
                                float               *pdf,
                                float               *occlusion,
                                RNGState             *rand_state) {
  // 1) Material & normal
  const CudaBSDF &bsdf = bsdfs[isect.bsdf_idx];
  Vector3D N    = isect.n;
  Vector2D uv   = isect.uv;

  // 2) Base color
  Vector3D base = Vector3D{bsdf.baseColor.x, bsdf.baseColor.y, bsdf.baseColor.z};
  if (bsdf.tex_idx >= 0) {
    Vector4D t = sample_texture(textures[bsdf.tex_idx], uv);
    base = Vector3D{base.x * t.x, base.y * t.y, base.z * t.z};
  }

  // 3) Metallic, roughness, occlusion from ORM
  float metal     = clampd(bsdf.metallic,  0.0, 1.0);
  float roughness = clampd(bsdf.roughness, 0.04,1.0);
  *occlusion = 1.0f;
  if (bsdf.orm_idx >= 0) {
    Vector4D orm = sample_texture(textures[bsdf.orm_idx], uv);
    *occlusion   = orm.x;
    roughness    = orm.y;
    metal        = orm.z;
  }
  float onem  = 1.0f - metal;
  Vector3D F0 = { 0.04f*onem + base.x*metal,
                  0.04f*onem + base.y*metal,
                  0.04f*onem + base.z*metal };

  // 4) Sample specular or diffuse
  float pdf_diff, pdf_spec;
  compute_lobe_probs(base, metal, roughness, &pdf_spec, &pdf_diff);

  // 4.2) Sample specular or diffuse
  float rng = next_float(rand_state);
  Vector3D sample_wi;
  Vector3D res;

  // common compute
  float alpha = fmaxf(roughness * roughness, 0.04f); // avoid zero roughness
  float u1 = next_float(rand_state);
  float u2 = next_float(rand_state);
  float phi = 2.0f * PI * u1;
  Vector3D T, B;
  coordinate_system(N, &T, &B);
  Vector3D one_mF = Vector3D{1.0f - F0.x, 1.0f - F0.y, 1.0f - F0.z};


  if (rng < pdf_diff) {
    // 4.3) Diffuse sample
    /* --- 1. Cosine-weighted sample in local frame (0,0,1) --- */

    float r   = sqrtf(u2);
    float x   = r * cosf(phi);
    float y   = r * sinf(phi);
    float z   = sqrtf(1.0f - u2);          // cosθ

    /* --- 3. Lift to world space --- */
    sample_wi = vector3d_unit( vector3d_add(
                   vector3d_add(vector3d_scale(T, x),
                                 vector3d_scale(B, y)),
                   vector3d_scale(N, z)) );

    /* --- 5. Evaluate *both* lobes at (wo, wi) --- */
    Vector3D H  = vector3d_unit(vector3d_add(wo, sample_wi));
    float    VoH = max(vector3d_dot(wo, H), 0.0f);

    /*   5a. Diffuse BRDF */
    Vector3D c_diff = vector3d_scale(base, onem);
    Vector3D f_diff = vector3d_scale(vector3d_mul(one_mF, c_diff), PI_R);

    /*   5b. Specular BRDF (uses GGX D and G helpers) */
    float NoV   = max(vector3d_dot(N, wo), 0.0f);
    float NoL   = fmaxf(vector3d_dot(N, sample_wi), 0.0f);;

    if (NoV < EPS_F || NoL < EPS_F) {
      *pdf = 0.0f;
      res  = Vector3D{};
      return res;  // return zero vector
    }

    float NoH   = max(vector3d_dot(N, H), 0.0f);
    float D     = D_compute(alpha, NoH);
    float G     = G_compute(alpha, NoV, NoL) / (4.0f * NoV * NoL + EPS_F);
    Vector3D F  = vector3d_add(F0,
                    vector3d_scale(one_mF,
                                   powf(1.0f - VoH, 5.0f)));
    Vector3D f_spec = vector3d_scale(F, D * G);

    /*   5c. Mixture PDF */
    *pdf = fmaxf(pdf_diff * NoL * PI_R + pdf_spec * (D * NoH) / (4.0f * VoH), EPS_F);

    res = vector3d_add(f_diff, f_spec);   // full BRDF
  } else {
    // 4.4) Specular sample
    /* --- 1. GGX half-vector sample (isotropic) -------------------- */
    float cosH   = sqrtf((1.0f - u2) / (1.0f + (alpha - 1.0f) * u2));
    float sinH   = sqrtf(max(0.0f, 1.0f - cosH * cosH));

    /* local H = (sinθ cosφ, sinθ sinφ, cosθ) */
    Vector3D H_local = Vector3D{sinH * cosf(phi), sinH * sinf(phi), cosH};

    /* --- 2. Rotate H_local into world space (same basis as before) */
    Vector3D H = vector3d_unit(vector3d_add(
                  vector3d_add(vector3d_scale(T, H_local.x),
                                vector3d_scale(B, H_local.y)),
                  vector3d_scale(N, H_local.z)));

    /* --- 3. Reflect wo about H to get wi -------------------------- */
    sample_wi = vector3d_reflect(vector3d_neg(wo), H);
    float NoL = max(vector3d_dot(N, sample_wi), 0.0f);
    float NoV = max(vector3d_dot(N,   wo      ), 0.0f);
    float NoH = max(vector3d_dot(N,   H       ), 0.0f);
    float VoH = max(vector3d_dot(wo,  H       ), 0.0f);

    if (NoL < EPS_F || NoV < EPS_F || VoH < EPS_F) {
        *pdf = 0.0f;
        return Vector3D{};
    }
    /* --- 4. PDF for the chosen lobe (mixture) ---------------- */
    float D     = D_compute(alpha, NoH);
    *pdf = fmaxf(pdf_diff * NoL * PI_R + pdf_spec * (D * NoH) / (4.0f * VoH), EPS_F);

    /* --- 5. Full BRDF evaluation ----------------------------- */
    /* 5a. Schlick Fresnel F */
    Vector3D F = vector3d_add(F0,
                  vector3d_scale(Vector3D{1.0f - F0.x, 1.0f - F0.y, 1.0f - F0.z},
                                  powf(1.0f - VoH, 5.0f)));

    /* 5b. Geometry term */
    float G = G_compute(alpha, NoV, NoL) / (4.0f * NoV * NoL + EPS_F);

    /* 5c. Specular BRDF */
    Vector3D f_spec = vector3d_scale(F, D * G);

    /* 5d. Diffuse BRDF (same formula as diffuse branch) */
    Vector3D c_diff = vector3d_scale(base, onem);
    Vector3D f_diff = vector3d_scale(vector3d_mul(one_mF, c_diff), PI_R);

    res = vector3d_add(f_diff, f_spec);
  }

  *wi = sample_wi;
  return res;  // return f(wo, wi)
}



#endif  // CGL_STATICSCENE_BSDF_H
