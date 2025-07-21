// libc replacement functions for muon
#ifndef VX_UTILS_H
#define VX_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

// x^y
float powf (float x, float y);
float cosf (float x);
float sinf (float x);
float sqrtf (float x);
#ifdef __cplusplus
}
#endif

#endif