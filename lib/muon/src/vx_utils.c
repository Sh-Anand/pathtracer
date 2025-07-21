#include <vx_utils.h>

#ifdef __cplusplus
extern "C" {
#endif


inline float powf (float x, float y) {
    float result = 1.0f;
    for (int i = 0; i < y; i++) {
        result *= x;
    }
    return result;
}

inline float sqrtf (float x) {
    float result;
    __asm__ ("fsqrt.s %0, %1" : "=f"(result) : "f"(x));
    return result;
}

#ifdef __cplusplus
}
#endif