#include "reservoir.h"

namespace CGL {

DEVICE void Reservoir::update(Sample s_new, float w_new, RNGState &rand_state) {
    w = w + w_new;
    M = M + 1;  
    if (next_float(rand_state) < (w_new / w)) {
        z = s_new;
    }
}

DEVICE void Reservoir::merge(Reservoir r, float p_hat, RNGState &rand_state) {
    float M0 = M;
    this->update(r.z, p_hat * r.W * r.M, rand_state);
    M = M0 + r.M;
}

DEVICE void Reservoir::clear(){
    w = 0;
    M = 0;
    W = 0;
    z.clear();
}

}