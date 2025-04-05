#ifndef CGL_UTIL_RESERVOIR_H
#define CGL_UTIL_RESERVOIR_H

#include "CGL/vector3D.h"

namespace CGL {

struct Sample {
    Vector3D x_v, n_v; // visible point and normal (normalized)
    Vector3D x_s, n_s; // sample point and normal (normalized)
    Vector3D L; // outgoing radiance at x_s
    double pdf; // pdf of the sample
    Vector3D fcos; // product of bsdf and cosine factor
    Vector3D emittance; // zero + one bounce radiance
};

#define cos_angle_threshold 0.9f // cos(25 degrees) 
#define distance_threshold 0.1f
bool inline are_geometrically_similar(const Sample& s1, const Sample& s2) {
    // Normal and distance check
    return acos(dot(s1.n_v.unit(), s2.n_v.unit())) <= radians(25.0)
        && (s1.x_v - s2.x_v).norm() <= distance_threshold;
}

float inline p_hat(const Sample& s) {
    return (s.pdf > 0) ? s.L.illum() / s.pdf : 0.0f;
}

class Reservoir {
    public:
        Sample z;
        double w; // sum of weight of samples so far
        double M; // number of samples so far
        double W;

        Reservoir() : w(0), M(0), W(0) {
            z = Sample();
        }
        
        void update(Sample s_new, double w_new) {
            w = w + w_new;
            M = M + 1;  
            if (random_uniform() < (w_new / w)) {
                z = s_new;
            }
        }

        void merge(Reservoir r, double p_hat) {
            double M0 = M;
            this->update(r.z, p_hat * r.W * r.M);
            M = M0 + r.M;
        }
};

}  // namespace CGL

#endif  // CGL_UTIL_RESERVOIR_H