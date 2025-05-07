#include "pathtracer/pathtracer.h"
#include "scene/bvh.h"
#include "scene/light.h"
#include "scene/primitive.h"
#include "util/vector2D.h"
#include "util/vector3D.h"
#include "util/vector4D.h"

#include <vector>

class Main {
public:
// Scene data
std::vector<Vector3D> vertices;
std::vector<Vector3D> normals;
std::vector<Vector4D> tangents;
std::vector<Vector2D> texcoords;
std::vector<CudaPrimitive> primitives;
std::vector<CudaBSDF> bsdfs;
std::vector<CudaLight> lights;
std::vector<CudaTexture> textures;
Camera camera;

//config params
int screenW = 800;
int screenH = 600;
size_t pathtracer_ns_aa;
size_t pathtracer_max_ray_depth;
size_t pathtracer_ns_area_light;
std::string pathtracer_filename;
size_t total_image_generated = 1;
bool debug = false;

// CUDA
PathTracer *pt_host;
PathTracer *pt_target;
BVHCuda* bvh_cuda;             ///< BVH accelerator aggregate for cuda
ImageBuffer frameBuffer;       ///< frame buffer

void parse_scene(std::string sceneFilePath);
void save_image(std::string filename);
void render_to_file(std::string filename, size_t x, size_t y, size_t dx, size_t dy);
void render_to_video(std::string filename, size_t x, size_t y, size_t dx, size_t dy, size_t num_images);
int main(int argc, char **argv);

void build_accel(std::vector<CudaPrimitive> &primitives, 
                                    std::vector<Vector3D> &vertices,
                                    std::vector<Vector3D> &normals, 
                                    std::vector<Vector2D> &texcoords,
                                    std::vector<Vector4D> &tangents,
                                    std::vector<CudaLight> &lights, 
                                    std::vector<CudaBSDF> &bsdfs, 
                                    std::vector<CudaTexture> &textures);

void update_camera();

void gpu_raytrace();
};