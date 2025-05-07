#include "scene/camera.h"
#include "target/pathtracer.h"
#include "raytraced_renderer.h"

#include <cstddef>
#include <cuda_runtime.h>

#include "scene/light.h"

__global__ void kernel_raytrace_temporal(PathTracer* pt) {
    assert (pt != nullptr);
    uint16_t x = ::blockIdx.x * ::blockDim.x + ::threadIdx.x;
    uint16_t y = ::blockIdx.y * ::blockDim.y + ::threadIdx.y;
    
    pt->raytrace_pixel(x,y);
    pt->temporal_resampling(x,y);
}

__global__ void kernel_spatial_sample(PathTracer* pt) {
    assert (pt != nullptr);
    uint16_t x = ::blockIdx.x * ::blockDim.x + ::threadIdx.x;
    uint16_t y = ::blockIdx.y * ::blockDim.y + ::threadIdx.y;
    
    pt->spatial_resampling(x,y);
    pt->render_final_sample(x,y);
}

void RaytracedRenderer::gpu_raytrace() {
    uint16_t width = frameBuffer.w;
    uint16_t height = frameBuffer.h;

    dim3 blockDim(16, 16);
    dim3 gridDim(
        (width + blockDim.x - 1) / blockDim.x,
        (height + blockDim.y - 1) / blockDim.y
    );
    
    DEBUG(debug, 
    std::cout << "Raytracing on GPU..." << std::endl;
    std::cout << "Frame size: " << width << " x " << height << std::endl;
    std::cout << "BlockDim: " << blockDim.x << " x " << blockDim.y << std::endl;
    std::cout << "GridDim: " << gridDim.x << " x " << gridDim.y << std::endl;
    )

    // cudaDeviceSetLimit(cudaLimitStackSize, 8192);

    std::chrono::time_point<std::chrono::steady_clock> t0 = std::chrono::steady_clock::now();


    kernel_raytrace_temporal<<<gridDim, blockDim>>>(pt_target);
    CUDA_ERR(cudaGetLastError());
    CUDA_ERR(cudaDeviceSynchronize());
    kernel_spatial_sample<<<gridDim, blockDim>>>(pt_target);
    CUDA_ERR(cudaGetLastError());
    CUDA_ERR(cudaDeviceSynchronize());

    std::chrono::time_point<std::chrono::steady_clock> t1 = std::chrono::steady_clock::now();
    float duration = (std::chrono::duration<float>(t1 - t0)).count();
    DEBUG(debug,
    std::cout << "Raytracing on GPU done!" << std::endl;
    std::cout << "Time: " << duration << " sec" << std::endl;
    )
    
    CUDA_ERR(cudaMemcpy(pt_host, pt_target, sizeof(PathTracer), cudaMemcpyDeviceToHost));
    uint8_t *rays_traced = (uint8_t *) malloc(sizeof(uint8_t) * width * height);
    CUDA_ERR(cudaMemcpy(rays_traced, pt_host->rays_traced, sizeof(uint8_t) * width * height, cudaMemcpyDeviceToHost));

    size_t tot_rays_traced = 0;
    for (size_t i = 0; i < width * height; i++)
        tot_rays_traced += rays_traced[i];
    free(rays_traced);
    DEBUG(debug,
    std::cout << "Total rays traced: " << tot_rays_traced << std::endl;
    std::cout << "Rays per second: " << (tot_rays_traced / duration) << std::endl;
    )
    auto data_tmp = pt_host->sampleBuffer.data;
    pt_host->sampleBuffer.data = (Vector3D*) malloc(width * height * sizeof(Vector3D));
    CUDA_ERR(cudaMemcpy(pt_host->sampleBuffer.data, data_tmp, width * height * sizeof(Vector3D), cudaMemcpyDeviceToHost));
    
    // write_to_framebuffer
    pt_host->sampleBuffer.toColor(frameBuffer, 0, 0, frameBuffer.w, frameBuffer.h);
    free (pt_host->sampleBuffer.data);

    // restore back
    pt_host->sampleBuffer.data = data_tmp;
}

void RaytracedRenderer::update_camera(){
    cudaMemcpy(pt_target, pt_host, sizeof(PathTracer), cudaMemcpyHostToDevice);
    CUDA_ERR(cudaGetLastError());
    CUDA_ERR(cudaDeviceSynchronize());
}

void RaytracedRenderer::build_accel(std::vector<CudaPrimitive> &primitives, 
                                    std::vector<Vector3D> &vertices,
                                    std::vector<Vector3D> &normals, 
                                    std::vector<Vector2D> &texcoords,
                                    std::vector<Vector4D> &tangents) {
  // build BVH //
  DEBUG(debug, 
  fprintf(stdout, "[PathTracer] Building BVH from %lu primitives... ", primitives.size()); 
  fflush(stdout);
  )
  std::chrono::time_point<std::chrono::steady_clock> t0 = std::chrono::steady_clock::now();

  bvh = new BVHCuda(primitives, vertices, normals, texcoords, tangents, debug);
  std::chrono::time_point<std::chrono::steady_clock> t1 = std::chrono::steady_clock::now();
  DEBUG(debug, 
  fprintf(stdout, "Done! (%.4f sec)\n", (std::chrono::duration<float>(t1 - t0)).count());
  )
}

void RaytracedRenderer::copy_host_device_pt(std::vector<CudaLight> &lights, std::vector<CudaBSDF> &bsdfs, std::vector<CudaTexture> &textures) {
    DEBUG(debug,
    std::cout << "Copying PathTracer to GPU..." << std::endl;
    std::cout << "BSDFs size: " << bsdfs.size() << std::endl;
    std::cout << "Lights size: " << lights.size() << std::endl;
    std::cout << "Textures size: " << textures.size() << std::endl;
    )

    //lights
    CudaLight *lights_cuda;
    cudaMalloc(&lights_cuda, lights.size() * sizeof(CudaLight));
    cudaMemcpy(lights_cuda, lights.data(), lights.size() * sizeof(CudaLight), cudaMemcpyHostToDevice);
    pt_host->num_lights = lights.size();
    pt_host->lights = lights_cuda;

    //bsdfs
    CudaBSDF *bsdfs_cuda;
    cudaMalloc(&bsdfs_cuda, bsdfs.size() * sizeof(CudaBSDF));
    cudaMemcpy(bsdfs_cuda, bsdfs.data(), bsdfs.size() * sizeof(CudaBSDF), cudaMemcpyHostToDevice);
    pt_host->bsdfs = bsdfs_cuda;

    //textures
    CudaTexture *textures_host = (CudaTexture*) malloc(textures.size() * sizeof(CudaTexture));
    for (size_t i = 0; i < textures.size(); i++) {
        textures_host[i].has_alpha = textures[i].has_alpha;
        textures_host[i].width = textures[i].width;
        textures_host[i].height = textures[i].height;
        int channels = textures[i].has_alpha ? 4 : 3;
        cudaMalloc(&textures_host[i].data, textures[i].width * textures[i].height * channels);
        cudaMemcpy(textures_host[i].data, textures[i].data, textures[i].width * textures[i].height * channels, cudaMemcpyHostToDevice);
    }

    CudaTexture *textures_cuda;
    cudaMalloc(&textures_cuda, textures.size() * sizeof(CudaTexture));
    cudaMemcpy(textures_cuda, textures_host, textures.size() * sizeof(CudaTexture), cudaMemcpyHostToDevice);
    pt_host->textures = textures_cuda;
    free(textures_host);

    //bvh
    BVHCuda *bvh_cuda;
    cudaMalloc(&bvh_cuda, sizeof(BVHCuda));
    cudaMemcpy(bvh_cuda, bvh, sizeof(BVHCuda), cudaMemcpyHostToDevice);
    pt_host->bvh = bvh_cuda;

    cudaMalloc(&pt_host->sampleBuffer.data, frameBuffer.w * frameBuffer.h * sizeof(Vector3D));
    
    cudaMalloc(&pt_host->initialSampleBuffer, sizeof(Sample) * frameBuffer.w * frameBuffer.h);
    cudaMalloc(&pt_host->temporalReservoirBuffer, sizeof(Reservoir) * frameBuffer.w * frameBuffer.h);
    cudaMalloc(&pt_host->spatialReservoirBuffer, sizeof(Reservoir) * frameBuffer.w * frameBuffer.h);
    cudaMalloc(&pt_host->rays_traced, sizeof(uint8_t) * frameBuffer.w * frameBuffer.h);

    cudaMalloc(&pt_host->rand_states, sizeof(RNGState) * frameBuffer.w * frameBuffer.h);

    PathTracer *pt_target;
    cudaMalloc(&pt_target, sizeof(PathTracer));
    cudaMemcpy(pt_target, pt_host, sizeof(PathTracer), cudaMemcpyHostToDevice);


    this->pt_target = pt_target;    
}