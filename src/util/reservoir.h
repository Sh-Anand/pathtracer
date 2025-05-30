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
    Vector3D x_v, n_v;       // visible point & normal (normalized)
    Vector3D x_s, n_s;       // sample point & normal (normalized)
    Vector3D L;              // outgoing radiance at x_s
    float    pdf;            // pdf of the sample
    Vector3D fcos;           // product of bsdf and cosine factor
    Vector3D emittance;      // zero + one bounce radiance
};

struct Reservoir {
    Sample z;                // current chosen sample
    float  w;                // sum of weights seen
    float  M;                // number of samples seen
    float  W;                // normalizing constant
};

// precompute thresholds
static constexpr float COS_ANGLE_THRESH = 0.9f;              // cos(25°)
static constexpr float DIST_THRESH      = 0.1f;              // distance threshold
static constexpr float ANGLE_THRESH_RAD = 25.0f * (3.14159265f/180.0f);

FORCE_INLINE bool are_geometrically_similar(const Sample s1, const Sample s2) {
    // normals already assumed unit-length
    float dn = vector3d_dot(s1.n_v, s2.n_v);
    if (dn < -1.0f) dn = -1.0f; else if (dn > 1.0f) dn = 1.0f;
    // compare angles
    if (acosf(dn) > ANGLE_THRESH_RAD) return false;
    // compare distances
    Vector3D dx = vector3d_sub(s1.x_v, s2.x_v);
    return vector3d_norm(dx) <= DIST_THRESH;
}

FORCE_INLINE float p_hat(const Sample s) {
    // ITU‑Rec. BT.709 luminance
    float illum = 0.2126f * s.L.x
                + 0.7152f * s.L.y
                + 0.0722f * s.L.z;
    return (s.pdf > 0.0f) ? (illum / s.pdf) : 0.0f;
}

FORCE_INLINE void update(Reservoir *r,
                         const Sample s_new,
                         float w_new,
                         RNGState *rand_state) {
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
    // w_new = p_hat2 * r2.W * r2.M
    float w_new = p_hat2 * r2.W * r2.M;
    update(r1, r2.z, w_new, rand_state);
    r1->M = M0 + r2.M;
}

FORCE_INLINE void clear(Reservoir *r) {
    r->w = 0.0f;
    r->M = 0.0f;
    r->W = 0.0f;
    // zero out the stored sample
    r->z.x_v      = {0.0f,0.0f,0.0f};
    r->z.n_v      = {0.0f,0.0f,0.0f};
    r->z.x_s      = {0.0f,0.0f,0.0f};
    r->z.n_s      = {0.0f,0.0f,0.0f};
    r->z.L        = {0.0f,0.0f,0.0f};
    r->z.pdf      = 0.0f;
    r->z.fcos     = {0.0f,0.0f,0.0f};
    r->z.emittance= {0.0f,0.0f,0.0f};
}

#undef FORCE_INLINE
#endif  // CGL_UTIL_RESERVOIR_H
