#ifndef CGL_UTIL_RESERVOIR_H
#define CGL_UTIL_RESERVOIR_H

#include "util/cuda_defs.h"
#include "vector.h"
#include "gpu_rand.h"
#include <cmath>

#ifdef __CUDACC__
  #define FORCE_INLINE __host__ __device__ __forceinline__
#else
  #define FORCE_INLINE inline __attribute__((always_inline))
#endif

struct Sample {
    Vector3D x_v, n_v;       // visible point, normal (normalized)
    float z_v;               // depth
    Vector3D x_s, n_s;       // sample point & normal (normalized)
    Vector3D L;              // outgoing radiance at x_s
    float    pdf;            // pdf of the sample
    Vector3D bsdf_f;           // bsdf_f at visible point
    Vector3D emittance;
};

struct SampleDirect { 
    Vector3D x_v, n_v;       // visible point, normal (normalized)
    Vector3D L;              // Direct lighting
};

struct SampleGI {
    Vector3D x_v, n_v;       // visible point, normal (normalized)
    float z_v;               // depth
    Vector3D x_s, n_s;            // sample point & normal (normalized)
    Vector3D L;              // outgoing radiance at x_s
    Vector3D bsdf_f;           // bsdf_f at visible point
};

struct Reservoir {
    SampleGI z;                // current chosen sample
    float  w;                // sum of weights seen
    float  M;                // number of samples seen
    float  W;                // normalizing constant
};

// precompute thresholds
static constexpr float COS_ANGLE_THRESH = 0.906307f; // cos(25°)
static constexpr float DEPTH_THRESH    = 0.05f;      // 5%
static constexpr float ILLUM_DELTA_THRESH = 1.4f; // 1.4x illumination difference
static constexpr float ILLUM_DELTA_THRESH_INV = 1 / ILLUM_DELTA_THRESH; // inverse threshold for illumination ratio

FORCE_INLINE bool are_geometrically_similar(const SampleGI *s1, const SampleGI *s2) {
    // 1) Angle test: normals within 25°
    float dn = vector3d_dot(s1->n_v, s2->n_v);
    if (dn < COS_ANGLE_THRESH) 
        return false;

    // 2) Depth test: normalized depth difference ≤ 0.05
    //    (Assuming s1.z_v and s2.z_v are camera‑space depths)
    float depthRatio = s1->z_v / max(s2->z_v, 0.001f); // avoid division by zero
    if (depthRatio < 1.0f - DEPTH_THRESH || depthRatio > 1.0f + DEPTH_THRESH)
        return false;

    // 3) Illum delta test: ≤ 2x illumination difference
    float ratio = illum(s1->L) / max(illum(s2->L), 0.001f); // avoid division by zero
    if (ratio > ILLUM_DELTA_THRESH || ratio < ILLUM_DELTA_THRESH_INV)
        return false;

    return true;
}

FORCE_INLINE void to_sample_GI(const Sample *s, SampleGI *s_gi) {
    s_gi->x_v = s->x_v;
    s_gi->n_v = s->n_v;
    s_gi->z_v = s->z_v;
    s_gi->x_s = s->x_s;
    s_gi->n_s = s->n_s;
    s_gi->L   = s->L;
    s_gi->bsdf_f = s->bsdf_f;
}

FORCE_INLINE void update(Reservoir *r,
                         const SampleGI s_new,
                         float w_new,
                         RNGState *rand_state) {
    if (w_new <= 0) return;
    float w_total = r->w + w_new;
    float prob    = (w_total > 0.0f) ? (w_new / w_total) : 0.0f;
    r->w = w_total;
    r->M = r->M + 1.0f;
    if (next_float(rand_state) < prob) {
        r->z = s_new;
    }
}

// merge reservoir r2 into r1, using weight multiplier p_hat2
FORCE_INLINE void merge(Reservoir *r1,
                        const Reservoir r2,
                        float              p_hat2,
                        RNGState          *rand_state) {
    float M0 = r1->M;
    float w_new = p_hat2 * r2.W * r2.M;
    update(r1, r2.z, w_new, rand_state);
    r1->M = M0 + r2.M;
}

#undef FORCE_INLINE
#endif  // CGL_UTIL_RESERVOIR_H
