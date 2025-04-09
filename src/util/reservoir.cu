#include "reservoir.h"

namespace CGL {

DEVICE void Reservoir::update(Sample s_new, double w_new, RNGState &rand_state) {
    w = w + w_new;
    M = M + 1;  
    if (next_double(rand_state) < (w_new / w)) {
        z = s_new;
    }
}

DEVICE void Reservoir::merge(Reservoir r, double p_hat, RNGState &rand_state) {
            double M0 = M;
            this->update(r.z, p_hat * r.W * r.M, rand_state);
            M = M0 + r.M;
}

}