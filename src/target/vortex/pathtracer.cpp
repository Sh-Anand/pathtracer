#include "target/pathtracer.h"

#include "vx_print.h"
#include "vx_spawn.h"

static void path_trace_kernel(void *args) {
  uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
  uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) return;
  raytrace_pixel_temporal_sample(x, y);
}

static void spatial_resampling_kernel(void *args) {
  uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
  uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) return;
  spatial_resampling(x, y);
}

int main() {
  int core_id = vx_core_id();

  if (core_id == 0) {
    vx_printf("Pathtracer starting...\n");

    vx_printf("RAYTRACING_START\n");

    vx_printf("Width: %d, Height: %d\n", w, h);
    vx_printf("Max Ray Depth: %d, NS Area Light: %d, Accumulate: %s, ReSTIR: %s\n",
              max_ray_depth, ns_area_light, accumulate ? "true" : "false", restir ? "true" : "false");
  }

  uint32_t dimension = 2;
  uint32_t block_dim[2] = {4, 4};
  uint32_t grid_dim[2] = {(w + block_dim[0] - 1) / block_dim[0],
                          (h + block_dim[1] - 1) / block_dim[1]};

  vx_spawn_threads(dimension, grid_dim, block_dim, (vx_kernel_func_cb)path_trace_kernel, NULL);
  if (restir) {
    vx_spawn_threads(dimension, grid_dim, block_dim, (vx_kernel_func_cb)spatial_resampling_kernel, NULL);
  }

  core_id = vx_core_id();
  if (core_id == 0) {
    vx_printf("PIXEL_BUFFER_START\n");
    vx_printf("%d, %d\n", w, h);
    for (uint32_t i = 0; i < w; i++) {
      for (uint32_t j = 0; j < h; j++) {
        vx_printf("(%f, %f, %f) ",
            pixel[i + j * w].data.x, pixel[i + j * w].data.y, pixel[i + j * w].data.z);
      }
      vx_printf("\n");
    }
    vx_printf("PIXEL_BUFFER_END\n");
  }

  vx_printf("RAYTRACING_COMPLETE\n");
  return 0;
}