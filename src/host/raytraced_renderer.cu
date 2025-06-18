#include "scene/camera.h"
#include "raytraced_renderer.h"

#include <cstddef>
#include <cuda_runtime.h>
#include <filesystem>

#include "scene/serializer.h"

// #include "target/pathtracer.cu"

// void RaytracedRenderer::gpu_raytrace() {
//     uint16_t width = w;
//     uint16_t height = h;

//     dim3 blockDim(16, 16);
//     dim3 gridDim(
//         (width + blockDim.x - 1) / blockDim.x,
//         (height + blockDim.y - 1) / blockDim.y
//     );
    
//     DEBUG(debug, 
//     std::cout << "Raytracing on GPU..." << std::endl;
//     std::cout << "Frame size: " << width << " x " << height << std::endl;
//     std::cout << "BlockDim: " << blockDim.x << " x " << blockDim.y << std::endl;
//     std::cout << "GridDim: " << gridDim.x << " x " << gridDim.y << std::endl;
//     )

//     // cudaDeviceSetLimit(cudaLimitStackSize, 4096);

//     std::chrono::time_point<std::chrono::steady_clock> t0 = std::chrono::steady_clock::now(); 

//     kernel_raytrace_temporal<<<gridDim, blockDim>>>(width, height, restir);
//     CUDA_ERR(cudaGetLastError());
//     CUDA_ERR(cudaDeviceSynchronize());

//     if (restir) {
//         kernel_spatial_sample<<<gridDim, blockDim>>>(width, height);
//         CUDA_ERR(cudaGetLastError());
//         CUDA_ERR(cudaDeviceSynchronize());
//     }

//     std::chrono::time_point<std::chrono::steady_clock> t1 = std::chrono::steady_clock::now();
//     float duration = (std::chrono::duration<float>(t1 - t0)).count();
//     DEBUG(debug,
//     std::cout << "Raytracing on GPU done!" << std::endl;
//     std::cout << "Time: " << duration << " sec" << std::endl;
//     )
// }

void RaytracedRenderer::copy_host_device_pt(std::vector<CudaPrimitive> &primitives, 
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

    std::vector<BVHNode> nodes;

    create_bvh(primitives, vertices, normals, texcoords, tangents, nodes, debug, 2);
    std::chrono::time_point<std::chrono::steady_clock> t1 = std::chrono::steady_clock::now();
    DEBUG(debug, 
    fprintf(stdout, "Done! (%.4f sec)\n", (std::chrono::duration<float>(t1 - t0)).count());
    )

    DEBUG(debug,
    std::cout << "Baking..." << std::endl;
    std::cout << "BSDFs size: " << bsdfs.size() << std::endl;
    std::cout << "Lights size: " << lights.size() << std::endl;
    std::cout << "Textures size: " << textures.size() << std::endl;
    )

    FILE* f = fopen("../src/target/baked_data.cu", "w");
    if (!f) { perror("open baked_data.cu"); return; }

    /* 1) Preamble */
    fprintf(f,
            "// *** AUTO-GENERATED — DO NOT TOUCH ***\n");
    /* 1.5) consts */
    fprintf(f, "__device__ int max_ray_depth = %zu;\n", max_ray_depth);
    fprintf(f, "__device__ int ns_area_light = %zu;\n", ns_area_light);
    fprintf(f, "__device__ bool accumulate = %d;\n", accumulate);
    /* 1.6) Camera */
    serialize(f, *camera);
    /* 2) Lights */
    serialize_lights(f, lights);
    /* 3) BSDFs */
    serialize_bsdfs(f, bsdfs);
    /* 4) Textures */
    serialize_textures(f, textures);
    /* 5) Primitives */
    serialize(f, primitives);
    /* 6) Vertices */
    serialize(f, vertices, "vertices");
    /* 7) Normals */
    serialize(f, normals, "normals");
    /* 8) Texcoords */
    serialize(f, texcoords, "texcoords");
    /* 9) Tangents */
    serialize(f, tangents, "tangents");
    /* 10) BVH */
    serialize_bvh_nodes(f, nodes);
    /* 11) RNG */
    serialize_empty_gpurng(f, w * h);
    /* 12) ImageBuffer */
    serialize_empty_image_buffer(f, w, h);

    fclose(f);

    DEBUG(debug,
    std::cout << "Done cooking..." << std::endl;
    )
}