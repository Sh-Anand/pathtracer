#include "target/pathtracer.h"

#include <chrono>
#include <fstream>
#include <vector>
#include <stdbool.h>

__global__ void kernel_raytrace_temporal(uint16_t width, uint16_t height) {
    uint16_t x = ::blockIdx.x * ::blockDim.x + ::threadIdx.x;
    uint16_t y = ::blockIdx.y * ::blockDim.y + ::threadIdx.y;
    if (x >= width || y >= height) return;
    raytrace_pixel_temporal_sample(x, y);
}

__global__ void kernel_spatial_sample(uint16_t width, uint16_t height) {
    uint16_t x = ::blockIdx.x * ::blockDim.x + ::threadIdx.x;
    uint16_t y = ::blockIdx.y * ::blockDim.y + ::threadIdx.y;
    if (x >= width || y >= height) return;
    spatial_resampling(x,y);
}

static void write_pfm(const char* fname, int W, int H)
{
    PixelData* d_pixels = nullptr;
    CUDA_ERR(cudaGetSymbolAddress((void**)&d_pixels, pixel));

    const size_t N = static_cast<size_t>(W) * H;
    std::vector<PixelData> cpu_fb(N);
    if (N == 0) {
        std::cerr << "Framebuffer is empty, nothing to write." << std::endl;
        return;
    }
    if (d_pixels == nullptr) {
        std::cerr << "Framebuffer pointer is null, nothing to write." << std::endl;
        return;
    }
    CUDA_ERR(cudaMemcpy(cpu_fb.data(), d_pixels,
                        N * sizeof(PixelData),
                        cudaMemcpyDeviceToHost));

    std::ofstream f(fname, std::ios::binary);
    if (!f) {
        std::perror(fname);
        return;
    }

    f << "PF\n";
    f << W << " " << H << "\n"; 
    f << "-1.0\n";

    for (int y = H - 1; y >= 0; --y) { 
        for (int x = 0; x < W; ++x) {
            PixelData& pixel = cpu_fb[x + y * W];
            f.write(reinterpret_cast<const char*>(&pixel.data.z), sizeof(float));  // R
            f.write(reinterpret_cast<const char*>(&pixel.data.y), sizeof(float));  // G
            f.write(reinterpret_cast<const char*>(&pixel.data.x), sizeof(float));  // B
        }
    }
}

int main(int argc, char** argv) {
  std::cout << "Starting raytracing on GPU...\n";
  // 1) copy width/height from device symbols
  uint16_t width  = 0, height = 0;
  int h_max_ray_depth = 0, h_ns_area_light = 0;
  bool h_accumulate = false;

  CUDA_ERR(cudaMemcpyFromSymbol(&width,  w, sizeof(width)));
  CUDA_ERR(cudaMemcpyFromSymbol(&height, h, sizeof(height)));
  CUDA_ERR(cudaMemcpyFromSymbol(&h_max_ray_depth, max_ray_depth, sizeof(h_max_ray_depth)));
  CUDA_ERR(cudaMemcpyFromSymbol(&h_ns_area_light, ns_area_light, sizeof(h_ns_area_light)));
  CUDA_ERR(cudaMemcpyFromSymbol(&h_accumulate, accumulate, sizeof(h_accumulate)));

  std::cout << "Frame size: " << width << "x" << height << "\n";
  std::cout << "Max ray depth: " << h_max_ray_depth << "\n";
  std::cout << "Light samples per hit point: " << h_ns_area_light << "\n";
  std::cout << "Accumulate: " << (h_accumulate ? "yes" : "no") << "\n";

  // 2) parse restir flag (default = 0)
  int restir_h = 0;
  if (argc > 1) {
    restir_h = std::atoi(argv[1]);
  }
  std::cout << "ReSTIR: " << (restir_h ? "enabled" : "disabled") << "\n";

  // 3) build launch dims
  dim3 blockDim(16, 16);
  dim3 gridDim(
    (width  + blockDim.x - 1) / blockDim.x,
    (height + blockDim.y - 1) / blockDim.y
  );

  // 3.5) copy restir flag to device
  CUDA_ERR(cudaMemcpyToSymbol(::restir, &restir_h, sizeof(restir_h)));

  // 4) run temporal kernel
  auto t0 = std::chrono::steady_clock::now();
  kernel_raytrace_temporal<<<gridDim, blockDim>>>(width, height);
  CUDA_ERR(cudaGetLastError());
  CUDA_ERR(cudaDeviceSynchronize());
  auto t1 = std::chrono::steady_clock::now();

  float elapsed = std::chrono::duration<float>(t1 - t0).count();
  std::cout << "Raytracing on GPU done in " << elapsed << " sec\n";

  // 5) copy rays traced from device to host
  uint8_t *h_rays_traced;
  h_rays_traced = new uint8_t[width * height];
  CUDA_ERR(cudaMemcpyFromSymbol(h_rays_traced, rays_traced, width * height * sizeof(uint8_t)));

  size_t total_rays = 0;
  for (uint32_t i = 0; i < width * height; ++i) {
      total_rays += h_rays_traced[i];
  }
  std::cout << "Rays per second: " << (total_rays / elapsed) << "\n";

  // 6) optionally write out to PFM
  if (argc > 2) {
    write_pfm(argv[2], width, height);
    std::cout << "Wrote image to " << argv[2] << "\n";
  }

  return 0;
}