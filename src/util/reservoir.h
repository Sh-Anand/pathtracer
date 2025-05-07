#ifndef CGL_UTIL_RESERVOIR_H
#define CGL_UTIL_RESERVOIR_H

#include "util/cuda_defs.h"
#include "vector3D.h"
#include "gpu_rand.h"


struct Sample {
    Vector3D x_v, n_v; // visible point and normal (normalized)
    Vector3D x_s, n_s; // sample point and normal (normalized)
    Vector3D L; // outgoing radiance at x_s
    float pdf; // pdf of the sample
    Vector3D fcos; // product of bsdf and cosine factor
    Vector3D emittance; // zero + one bounce radiance
    bool is_delta;    // ← new

    DEVICE void clear(){
        x_v = Vector3D(0), n_v = Vector3D(0);
        x_s = Vector3D(0), n_s = Vector3D(0);
        L = Vector3D(0);
        pdf = 0;
        fcos = Vector3D(0);
        emittance = Vector3D(0);
    }
};

#define cos_angle_threshold 0.9f // cos(25 degrees) 
#define distance_threshold 0.1f
DEVICE bool inline are_geometrically_similar(const Sample& s1, const Sample& s2) {
    return acos(dot(s1.n_v.unit(), s2.n_v.unit())) <=  (25.0 * (PI / 180))
        && (s1.x_v - s2.x_v).norm() <= distance_threshold;
}

DEVICE float inline p_hat(const Sample& s) {
    return (s.pdf > 0) ? s.L.illum() / s.pdf : 0.0f;
}

class Reservoir {
    public:
        Sample z;
        float w; // sum of weight of samples so far
        float M; // number of samples so far
        float W;

        HOST_DEVICE Reservoir() : w(0), M(0), W(0) {
            z = Sample();
        }
        
        DEVICE void update(Sample s_new, float w_new, RNGState &rand_state);

        DEVICE void merge(Reservoir r, float p_hat, RNGState &rand_state);

        DEVICE void clear();
};

#endif  // CGL_UTIL_RESERVOIR_H