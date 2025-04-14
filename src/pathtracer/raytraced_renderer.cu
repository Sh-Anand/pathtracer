#include "raytraced_renderer.h"

#include <cuda_runtime.h>
#include <curand_kernel.h>

#include "scene/sphere.h"
#include "scene/triangle.h"
#include "scene/light.h"

using namespace CGL::SceneObjects;

namespace CGL {

__global__ void kernel_raytrace(PathTracer* pt) {
    assert (pt != nullptr);
    size_t x = blockIdx.x * blockDim.x + threadIdx.x;
    size_t y = blockIdx.y * blockDim.y + threadIdx.y;
    
    pt->raytrace_pixel(x,y);
}

void RaytracedRenderer::gpu_raytrace() {
    size_t width = frameBuffer.w;
    size_t height = frameBuffer.h;

    cout << "Raytracing on GPU..." << endl;


    dim3 blockDim(16, 16);
    dim3 gridDim(
        (width + blockDim.x - 1) / blockDim.x,
        (height + blockDim.y - 1) / blockDim.y
    );
    
    cout << "Frame size: " << width << " x " << height << endl;
    cout << "BlockDim: " << blockDim.x << " x " << blockDim.y << endl;
    cout << "GridDim: " << gridDim.x << " x " << gridDim.y << endl;

    Timer timer;
    timer.start();

    kernel_raytrace<<<gridDim, blockDim>>>(pt_cuda);
    // Check for errors
    CUDA_ERR(cudaGetLastError());

    // Synchronize
    CUDA_ERR(cudaDeviceSynchronize());
    timer.stop();

    cout << "Raytracing on GPU done!" << endl;
    cout << "Time: " << timer.duration() << " sec" << endl;
    
    PathTracer *pt_tmp = (PathTracer*) malloc(sizeof(PathTracer));
    CUDA_ERR(cudaMemcpy(pt_tmp, pt_cuda, sizeof(PathTracer), cudaMemcpyDeviceToHost));
    pt->sampleBuffer.data = (Vector3D*) malloc(width * height * sizeof(Vector3D));
    CUDA_ERR(cudaMemcpy(pt->sampleBuffer.data, pt_tmp->sampleBuffer.data, width * height * sizeof(Vector3D), cudaMemcpyDeviceToHost));
    
    // write_to_framebuffer
    pt->sampleBuffer.toColor(frameBuffer, 0, 0, frameBuffer.w, frameBuffer.h);
    free (pt->sampleBuffer.data);
    free(pt_tmp);
}

void RaytracedRenderer::build_accel() {

  // collect primitives //
  fprintf(stdout, "[PathTracer] Collecting primitives... "); fflush(stdout);
  timer.start();
  vector<Primitive *> primitives;
  for (SceneObject *obj : scene->objects) {
    const vector<Primitive *> &obj_prims = obj->get_primitives();
    primitives.reserve(primitives.size() + obj_prims.size());
    primitives.insert(primitives.end(), obj_prims.begin(), obj_prims.end());
  }
  timer.stop();
  fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

  // build BVH //
  fprintf(stdout, "[PathTracer] Building BVH from %lu primitives... ", primitives.size()); 
  fflush(stdout);
  timer.start();

  bvh = new BVHAccel(primitives);
  bvh_cuda = new BVHCuda(bvh);
  timer.stop();
  fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

  // initial visualization //
 
  selectionHistory.push(bvh->get_root());
}

void RaytracedRenderer::build_lights() {
  // Build lights:
  lights.clear();
  if (light_data)
    cudaFree(light_data);

  light_data = new CudaLightBundle();
  light_data->num_directional_lights = 0;
  light_data->num_point_lights = 0;
  light_data->num_area_lights = 0;

  std::vector<CudaDirectionalLight> directional_lights;
  std::vector<CudaPointLight> point_lights;
  std::vector<CudaAreaLight> area_lights;

  for (size_t i = 0; i < scene->lights.size(); i++) {
    SceneLight *light = scene->lights[i];
    CudaLight cuda_light;
    if (dynamic_cast<DirectionalLight *>(light)) {
      directional_lights.push_back(CudaDirectionalLight(*(DirectionalLight *) light));
      cuda_light.type = CudaLightType_Directional;
      cuda_light.idx = light_data->num_directional_lights++;
    } else if (dynamic_cast<PointLight *>(light)) {
      point_lights.push_back(CudaPointLight(*(PointLight *) light));
      cuda_light.type = CudaLightType_Point;
      cuda_light.idx = light_data->num_point_lights++;
    } else if (dynamic_cast<AreaLight *>(light)) {
      area_lights.push_back(CudaAreaLight(*(AreaLight *) light));
      cuda_light.type = CudaLightType_Area;
      cuda_light.idx = light_data->num_area_lights++;
    } else {
      std::cout<< "Here?";
      std::cerr << "Unknown light type" << std::endl;
      exit(1);
    }
    lights.push_back(cuda_light);
  }

  CudaLightBundle *light_data_cuda = new CudaLightBundle();
  // copy lights to pt
  cudaMalloc(&pt->lights, lights.size() * sizeof(CudaLight));

  light_data_cuda->num_directional_lights = light_data->num_directional_lights;
  light_data_cuda->num_point_lights = light_data->num_point_lights;
  light_data_cuda->num_area_lights = light_data->num_area_lights;
  cudaMalloc(&light_data_cuda->directional_lights, light_data->num_directional_lights * sizeof(CudaDirectionalLight));
  cudaMalloc(&light_data_cuda->point_lights, light_data->num_point_lights * sizeof(CudaPointLight));
  cudaMalloc(&light_data_cuda->area_lights, light_data->num_area_lights * sizeof(CudaAreaLight));

  cudaMemcpy(pt->lights, lights.data(), lights.size() * sizeof(CudaLight), cudaMemcpyHostToDevice);
  cudaMemcpy(light_data_cuda->directional_lights, directional_lights.data(), light_data->num_directional_lights * sizeof(CudaDirectionalLight), cudaMemcpyHostToDevice);
  cudaMemcpy(light_data_cuda->point_lights, point_lights.data(), light_data->num_point_lights * sizeof(CudaPointLight), cudaMemcpyHostToDevice);
  cudaMemcpy(light_data_cuda->area_lights, area_lights.data(), light_data->num_area_lights * sizeof(CudaAreaLight), cudaMemcpyHostToDevice);

  cudaMalloc(&pt->light_data, sizeof(CudaLightBundle));
  cudaMemcpy(pt->light_data, light_data_cuda, sizeof(CudaLightBundle), cudaMemcpyHostToDevice);

  free (light_data_cuda);

  pt->num_lights = lights.size();

  std::cout << "CudaLights: " << light_data->num_directional_lights << " directional lights, "
            << light_data->num_point_lights << " point lights, "
            << light_data->num_area_lights << " area lights" << std::endl;
}

void RaytracedRenderer::copy_host_device_pt() {
    cudaMalloc(&pt->bvh, sizeof(BVHCuda));
    cudaMemcpy(pt->bvh, bvh_cuda, sizeof(BVHCuda), cudaMemcpyHostToDevice);

    cudaMalloc(&pt->sampleBuffer.data, frameBuffer.w * frameBuffer.h * sizeof(Vector3D));
    
    PathTracer *pt_cuda;
    cudaMalloc(&pt_cuda, sizeof(PathTracer));
    cudaMemcpy(pt_cuda, pt, sizeof(PathTracer), cudaMemcpyHostToDevice);


    this->pt_cuda = pt_cuda;    
}
}