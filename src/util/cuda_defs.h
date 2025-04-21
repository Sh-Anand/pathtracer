#pragma once
#ifdef __CUDACC__
#  define HOST_DEVICE __host__ __device__
#  define DEVICE __device__
#else
#  define HOST_DEVICE
#  define DEVICE
#endif

#ifdef __CUDACC__
#include <iostream>

#define CUDA_ERR(ans) { gpuAssert((ans), __FILE__, __LINE__); }

inline void gpuAssert(cudaError_t code, const char *file, int line) {
    if (code != cudaSuccess) {
        fprintf(stderr, "CUDA Error: %s (%s:%d)\n", cudaGetErrorString(code), file, line);
        exit(code);
    }
}
#else
#define CUDA_ERR(ans) ans
#endif

#define PI (3.14159265358979323)
#define PI_F (3.1415927f)
#define EPS_F (0.00001f)

#include <cmath>
#include <limits>
#include <algorithm>

/*
  Takes any kind of number and converts from degrees to radians.
*/
template<typename T>
inline T radians(T deg) {
  return deg * (PI_F / 180);
}

/*
  Takes any kind of number and converts from radians to degrees.
*/
template<typename T>
inline T degrees(T rad) {
  return rad * (180 / PI_F);
}

/*
  Takes any kind of number, as well as a lower and upper bound, and clamps the
  number to be within the bound.
  NOTE: x, lo, and hi must all be the same type or compilation will fail. A
        common mistake is to pass an int for x and size_ts for lo and hi.
*/
template<typename T>
inline T clamp_T(T x, T lo, T hi) {
  return std::min(std::max(x, lo), hi);
}