#include "raytraced_renderer.h"

#include <cuda_runtime.h>

#include "scene/light.h"

using namespace CGL::SceneObjects;

namespace CGL {

__global__ void kernel_raytrace_temporal(PathTracer* pt) {
    assert (pt != nullptr);
    size_t x = ::blockIdx.x * ::blockDim.x + ::threadIdx.x;
    size_t y = ::blockIdx.y * ::blockDim.y + ::threadIdx.y;
    
    pt->raytrace_pixel(x,y);
    pt->temporal_resampling(x,y);
}

__global__ void kernel_spatial_sample(PathTracer* pt) {
    assert (pt != nullptr);
    size_t x = ::blockIdx.x * ::blockDim.x + ::threadIdx.x;
    size_t y = ::blockIdx.y * ::blockDim.y + ::threadIdx.y;
    
    pt->spatial_resampling(x,y);
    pt->render_final_sample(x,y);
}

void RaytracedRenderer::gpu_raytrace() {
    size_t width = frameBuffer.w;
    size_t height = frameBuffer.h;

    std::cout << "Raytracing on GPU..." << std::endl;


    dim3 blockDim(16, 16);
    dim3 gridDim(
        (width + blockDim.x - 1) / blockDim.x,
        (height + blockDim.y - 1) / blockDim.y
    );
    
    std::cout << "Frame size: " << width << " x " << height << std::endl;
    std::cout << "BlockDim: " << blockDim.x << " x " << blockDim.y << std::endl;
    std::cout << "GridDim: " << gridDim.x << " x " << gridDim.y << std::endl;

    // cudaDeviceSetLimit(cudaLimitStackSize, 8192);

    std::chrono::time_point<std::chrono::steady_clock> t0 = std::chrono::steady_clock::now();


    kernel_raytrace_temporal<<<gridDim, blockDim>>>(pt_cuda);
    CUDA_ERR(cudaGetLastError());
    CUDA_ERR(cudaDeviceSynchronize());
    kernel_spatial_sample<<<gridDim, blockDim>>>(pt_cuda);
    CUDA_ERR(cudaGetLastError());
    CUDA_ERR(cudaDeviceSynchronize());

    std::chrono::time_point<std::chrono::steady_clock> t1 = std::chrono::steady_clock::now();

    std::cout << "Raytracing on GPU done!" << std::endl;
    std::cout << "Time: " << (std::chrono::duration<double>(t1 - t0)).count() << " sec" << std::endl;
    
    PathTracer *pt_tmp = (PathTracer*) malloc(sizeof(PathTracer));
    CUDA_ERR(cudaMemcpy(pt_tmp, pt_cuda, sizeof(PathTracer), cudaMemcpyDeviceToHost));
    pt->sampleBuffer.data = (Vector3D*) malloc(width * height * sizeof(Vector3D));
    CUDA_ERR(cudaMemcpy(pt->sampleBuffer.data, pt_tmp->sampleBuffer.data, width * height * sizeof(Vector3D), cudaMemcpyDeviceToHost));
    
    // write_to_framebuffer
    pt->sampleBuffer.toColor(frameBuffer, 0, 0, frameBuffer.w, frameBuffer.h);
    free (pt->sampleBuffer.data);
    free(pt_tmp);
}

void RaytracedRenderer::build_accel(std::vector<CudaPrimitive> &primitives) {
  // build BVH //
  fprintf(stdout, "[PathTracer] Building BVH from %lu primitives... ", primitives.size()); 
  fflush(stdout);
  std::chrono::time_point<std::chrono::steady_clock> t0 = std::chrono::steady_clock::now();

  bvh_cuda = new BVHCuda(primitives);
  std::chrono::time_point<std::chrono::steady_clock> t1 = std::chrono::steady_clock::now();
  fprintf(stdout, "Done! (%.4f sec)\n", (std::chrono::duration<double>(t1 - t0)).count());
}

void RaytracedRenderer::copy_host_device_pt(std::vector<CudaLight> &lights, std::vector<CudaBSDF> &bsdfs) {
    std::cout << "Copying PathTracer to GPU..." << std::endl;
    std::cout << "BSDFs size: " << bsdfs.size() << std::endl;

    cudaMalloc(&pt->lights, lights.size() * sizeof(CudaLight));
    cudaMemcpy(pt->lights, lights.data(), lights.size() * sizeof(CudaLight), cudaMemcpyHostToDevice);

    cudaMalloc(&pt->bsdfs, bsdfs.size() * sizeof(CudaBSDF));
    cudaMemcpy(pt->bsdfs, bsdfs.data(), bsdfs.size() * sizeof(CudaBSDF), cudaMemcpyHostToDevice);

    pt->num_lights = lights.size();
    pt->num_bsdfs = bsdfs.size();

    cudaMalloc(&pt->bvh, sizeof(BVHCuda));
    cudaMemcpy(pt->bvh, bvh_cuda, sizeof(BVHCuda), cudaMemcpyHostToDevice);

    cudaMalloc(&pt->sampleBuffer.data, frameBuffer.w * frameBuffer.h * sizeof(Vector3D));
    
    cudaMalloc(&pt->initialSampleBuffer, sizeof(Sample) * frameBuffer.w * frameBuffer.h);
    cudaMalloc(&pt->temporalReservoirBuffer, sizeof(Reservoir) * frameBuffer.w * frameBuffer.h);
    cudaMalloc(&pt->spatialReservoirBuffer, sizeof(Reservoir) * frameBuffer.w * frameBuffer.h);

    cudaMalloc(&pt->rand_states, sizeof(RNGState) * frameBuffer.w * frameBuffer.h);

    PathTracer *pt_cuda;
    cudaMalloc(&pt_cuda, sizeof(PathTracer));
    cudaMemcpy(pt_cuda, pt, sizeof(PathTracer), cudaMemcpyHostToDevice);


    this->pt_cuda = pt_cuda;    
}
}