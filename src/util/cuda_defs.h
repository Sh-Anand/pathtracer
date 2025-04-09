#pragma once
#ifdef __CUDACC__
#  define HOST_DEVICE __host__ __device__
#  define DEVICE __device__
#else
#  define HOST_DEVICE
#  define DEVICE
#endif

#ifdef __CUDACC__
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
