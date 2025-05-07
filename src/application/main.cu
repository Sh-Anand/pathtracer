#include "main.h"

#include <chrono>

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

void Main::gpu_raytrace() {
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
    
    DEBUG(debug,
    std::cout << "Raytracing on GPU done!" << std::endl;
    std::cout << "Time: " << (std::chrono::duration<float>(t1 - t0)).count() << " sec" << std::endl;
    )
    
    CUDA_ERR(cudaMemcpy(pt_host, pt_target, sizeof(PathTracer), cudaMemcpyDeviceToHost));
    
    auto data_tmp = pt_host->sampleBuffer.data;
    pt_host->sampleBuffer.data = (Vector3D*) malloc(width * height * sizeof(Vector3D));
    CUDA_ERR(cudaMemcpy(pt_host->sampleBuffer.data, data_tmp, width * height * sizeof(Vector3D), cudaMemcpyDeviceToHost));
    
    // write_to_framebuffer
    pt_host->sampleBuffer.toColor(frameBuffer, 0, 0, frameBuffer.w, frameBuffer.h);
    free (pt_host->sampleBuffer.data);

    // restore back
    pt_host->sampleBuffer.data = data_tmp;
}

void Main::update_camera(){
    cudaMemcpy(pt_target, pt_host, sizeof(PathTracer), cudaMemcpyHostToDevice);
    CUDA_ERR(cudaGetLastError());
    CUDA_ERR(cudaDeviceSynchronize());
}

void Main::build_accel(std::vector<CudaPrimitive> &primitives, 
                                    std::vector<Vector3D> &vertices,
                                    std::vector<Vector3D> &normals, 
                                    std::vector<Vector2D> &texcoords,
                                    std::vector<Vector4D> &tangents,
                                    std::vector<CudaLight> &lights, 
                                    std::vector<CudaBSDF> &bsdfs, 
                                    std::vector<CudaTexture> &textures) {
    // build BVH //
    DEBUG(debug, 
    fprintf(stdout, "[PathTracer] Building BVH from %lu primitives... ", primitives.size()); 
    fflush(stdout);
    )
    std::chrono::time_point<std::chrono::steady_clock> t0 = std::chrono::steady_clock::now();

    bvh_cuda = new BVHCuda(primitives, vertices, normals, texcoords, tangents, debug);
    std::chrono::time_point<std::chrono::steady_clock> t1 = std::chrono::steady_clock::now();
    DEBUG(debug, 
    fprintf(stdout, "Done! (%.4f sec)\n", (std::chrono::duration<float>(t1 - t0)).count());
    )

    DEBUG(debug,
    std::cout << "Copying PathTracer to GPU..." << std::endl;
    std::cout << "BSDFs size: " << bsdfs.size() << std::endl;
    std::cout << "Lights size: " << lights.size() << std::endl;
    std::cout << "Textures size: " << textures.size() << std::endl;
    )

    //lights
    cudaMalloc(&pt_host->lights, lights.size() * sizeof(CudaLight));
    cudaMemcpy(pt_host->lights, lights.data(), lights.size() * sizeof(CudaLight), cudaMemcpyHostToDevice);
    pt_host->num_lights = lights.size();

    //bsdfs
    cudaMalloc(&pt_host->bsdfs, bsdfs.size() * sizeof(CudaBSDF));
    cudaMemcpy(pt_host->bsdfs, bsdfs.data(), bsdfs.size() * sizeof(CudaBSDF), cudaMemcpyHostToDevice);

    //textures
    CudaTexture *textures_cuda = (CudaTexture*) malloc(textures.size() * sizeof(CudaTexture));
    for (size_t i = 0; i < textures.size(); i++) {
        textures_cuda[i].has_alpha = textures[i].has_alpha;
        textures_cuda[i].width = textures[i].width;
        textures_cuda[i].height = textures[i].height;
        int channels = textures[i].has_alpha ? 4 : 3;
        cudaMalloc(&textures_cuda[i].data, textures[i].width * textures[i].height * channels);
        cudaMemcpy(textures_cuda[i].data, textures[i].data, textures[i].width * textures[i].height * channels, cudaMemcpyHostToDevice);
    }
    cudaMalloc(&pt_host->textures, textures.size() * sizeof(CudaTexture));
    cudaMemcpy(pt_host->textures, textures_cuda, textures.size() * sizeof(CudaTexture), cudaMemcpyHostToDevice);
    free(textures_cuda);

    //bvh
    cudaMalloc(&pt_host->bvh, sizeof(BVHCuda));
    cudaMemcpy(pt_host->bvh, bvh_cuda, sizeof(BVHCuda), cudaMemcpyHostToDevice);

    cudaMalloc(&pt_host->sampleBuffer.data, frameBuffer.w * frameBuffer.h * sizeof(Vector3D));

    cudaMalloc(&pt_host->initialSampleBuffer, sizeof(Sample) * frameBuffer.w * frameBuffer.h);
    cudaMalloc(&pt_host->temporalReservoirBuffer, sizeof(Reservoir) * frameBuffer.w * frameBuffer.h);
    cudaMalloc(&pt_host->spatialReservoirBuffer, sizeof(Reservoir) * frameBuffer.w * frameBuffer.h);

    cudaMalloc(&pt_host->rand_states, sizeof(RNGState) * frameBuffer.w * frameBuffer.h);

    PathTracer *pt;
    cudaMalloc(&pt, sizeof(PathTracer));
    cudaMemcpy(pt, pt_host, sizeof(PathTracer), cudaMemcpyHostToDevice);


    pt_target = pt;    
}