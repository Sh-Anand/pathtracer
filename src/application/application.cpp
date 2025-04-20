#include "application.h"

#include "scene/gl_scene/mesh.h"

using Collada::CameraInfo;
using Collada::LightInfo;
using Collada::MaterialInfo;
using Collada::PolymeshInfo;
using Collada::SceneInfo;
using Collada::SphereInfo;

using CGL::SceneObjects::CudaAreaLight;
using CGL::SceneObjects::CudaDirectionalLight;
using CGL::SceneObjects::CudaPointLight;
using CGL::SceneObjects::CudaLightType;
using CGL::SceneObjects::CudaPrimitive;
using CGL::SceneObjects::CudaTriangle;
using CGL::SceneObjects::CudaSphere;

namespace CGL {

Application::Application(AppConfig config, bool gl) {
  renderer = new RaytracedRenderer (
    config.pathtracer_ns_aa,
    config.pathtracer_max_ray_depth,
    config.pathtracer_accumulate_bounces,
    config.pathtracer_ns_area_light,
    config.pathtracer_ns_diff,
    config.pathtracer_ns_glsy,
    config.pathtracer_ns_refr,
    config.pathtracer_num_threads,
    config.pathtracer_samples_per_patch,
    config.pathtracer_max_tolerance,
    config.pathtracer_envmap,
    config.pathtracer_direct_hemisphere_sample,
    config.pathtracer_filename,
    config.pathtracer_lensRadius,
    config.pathtracer_focalDistance
  );
  filename = config.pathtracer_filename;
}

Application::~Application() {
  delete renderer;
}

void Application::init() {
  scene = nullptr;


  // Make a dummy camera so resize() doesn't crash before the scene has been
  // loaded.
  // NOTE: there's a chicken-and-egg problem here, because loadScene
  // requires init, and init requires init_camera (which is only called by
  // loadScene).
  screenW = 800; screenH = 600; // Default value
  CameraInfo cameraInfo;
  cameraInfo.hFov = 50;
  cameraInfo.vFov = 35;
  cameraInfo.nClip = 0.01;
  cameraInfo.fClip = 100;
  camera.configure(cameraInfo, screenW, screenH);
}

void Application::resize(size_t w, size_t h) {
  screenW = w;
  screenH = h;
  camera.set_screen_size(w, h);
  renderer->set_frame_size(w, h);
}

void push_cuda_bsdf(const BSDF *bsdf, vector<CudaBSDF> &bsdfs) {
  // dynamic cst BSDF and figure out type:
  CudaBSDF cuda_bsdf {};
  if (const DiffuseBSDF *diffuse_bsdf = dynamic_cast<const DiffuseBSDF *>(bsdf)) {
    cuda_bsdf.type = CudaBSDFType_Diffuse;
    cuda_bsdf.bsdf.diffuse = CudaDiffuseBSDF{diffuse_bsdf->reflectance};
  } else if (const EmissionBSDF *emission_bsdf = dynamic_cast<const EmissionBSDF *>(bsdf)) {
    cuda_bsdf.type = CudaBSDFType_Emission;
    cuda_bsdf.bsdf.emission = CudaEmissionBSDF{emission_bsdf->radiance};
  } else {
    std::cerr << "Unknown BSDF type" << std::endl;
    exit(1);
  }
  bsdfs.push_back(cuda_bsdf);
}

void Application::load(SceneInfo* sceneInfo) {

  vector<Collada::Node>& nodes = sceneInfo->nodes;
  vector<GLScene::SceneLight *> scene_lights;
  vector<GLScene::SceneObject *> objects;

  lights.clear();
  primitives.clear();
  bsdfs.clear();

  // save camera position to update camera control later
  CameraInfo *c;
  Vector3D c_pos = Vector3D();
  Vector3D c_dir = Vector3D();

  int len = nodes.size();
  for (int i = 0; i < len; i++) {
    Collada::Node& node = nodes[i];
    Collada::Instance *instance = node.instance;
    const Matrix4x4& transform = node.transform;

    switch(instance->type) {
      case Collada::Instance::CAMERA:
        c = static_cast<CameraInfo*>(instance);
        c_pos = (transform * Vector4D(c_pos,1)).to3D();
        c_dir = (transform * Vector4D(c->view_dir,1)).to3D().unit();
        init_camera(*c, transform);
        break;
      case Collada::Instance::LIGHT:
      {
        LightInfo& light_info = static_cast<LightInfo&>(*instance);
        CudaLight light {};
        light.type = (CudaLightType) light_info.light_type;
        switch(light_info.light_type) {
          case Collada::LightType::NONE:
            break;
          case Collada::LightType::AMBIENT:
            std::cerr << "Ambient light type unsupported" << std::endl;
            exit(1);
          case Collada::LightType::DIRECTIONAL:
            light.light.directional = CudaDirectionalLight {light_info.spectrum, (transform * Vector4D(light_info.direction, 1)).to3D().unit()};
            break;
          case Collada::LightType::AREA: {
            Vector3D position = (transform * Vector4D(light_info.position, 1)).to3D();
            Vector3D direction = (transform * Vector4D(light_info.direction, 1)).to3D() - position;
            direction.normalize();
            Vector3D dim_y = (transform * Vector4D(light_info.up, 1)).to3D() - position;
            Vector3D dim_x = (transform * Vector4D(cross(light_info.up, light_info.direction), 1)).to3D() - position;
            light.light.area = CudaAreaLight(light_info.spectrum, position, direction, dim_x, dim_y);
            break;
          }
          case Collada::LightType::POINT:
            light.light.point = CudaPointLight{light_info.spectrum, (transform * Vector4D(light_info.position, 1)).to3D()};
            break;
          case Collada::LightType::SPOT:
            std::cerr << "Spot light type unsupported" << std::endl;
            exit(1);
          default:
            break;
        }
        lights.push_back(light);
        break;
      }
      case Collada::Instance::SPHERE:
      {
        SphereInfo& sphere = static_cast<SphereInfo&>(*instance);
        double r = sphere.radius * (transform * Vector4D(1, 0, 0, 0)).to3D().norm();
        Vector3D o = (transform * Vector4D(0, 0, 0, 1)).projectTo3D();
        BSDF* bsdf = sphere.material ? sphere.material->bsdf : new DiffuseBSDF(Vector3D(0.5f, 0.5f, 0.5f));
        push_cuda_bsdf(bsdf, bsdfs);
        
        CudaPrimitive primitive {};
        primitive.type = CGL::SceneObjects::CudaPrimitiveType::SPHERE;
        primitive.primitive.sphere = CudaSphere{o, r};
        primitive.bsdf_idx = bsdfs.size() - 1;
        primitives.push_back(primitive);
        break;
      }
      case Collada::Instance::POLYMESH:
      {
        GLScene::Mesh mesh(static_cast<PolymeshInfo&>(*instance), transform);
        BSDF* bsdf = mesh.bsdf;
        push_cuda_bsdf(bsdf, bsdfs);

        CudaPrimitive primitive {};
        mesh.get_triangles(primitives, bsdfs.size() - 1);
        break;
      }
      case Collada::Instance::MATERIAL:
        init_material(static_cast<MaterialInfo&>(*instance));
        break;
     }
  }

  scene = new GLScene::Scene(objects, scene_lights);

  BBox bbox;
  for (auto& primitive: primitives) {
    bbox.expand(primitive.get_bbox());
  }

  if (!bbox.empty()) {

    Vector3D target = bbox.centroid();
    canonical_view_distance = bbox.extent.norm() / 2 * 1.5;

    double view_distance = canonical_view_distance * 2;
    double min_view_distance = canonical_view_distance / 10.0;
    double max_view_distance = canonical_view_distance * 20.0;

    canonicalCamera.place(target,
                          acos(c_dir.y),
                          atan2(c_dir.x, c_dir.z),
                          view_distance,
                          min_view_distance,
                          max_view_distance);

    camera.place(target,
                acos(c_dir.y),
                atan2(c_dir.x, c_dir.z),
                view_distance,
                min_view_distance,
                max_view_distance);
  }
}

void Application::init_camera(CameraInfo& cameraInfo,
                              const Matrix4x4& transform) {
  camera.configure(cameraInfo, screenW, screenH);
  canonicalCamera.configure(cameraInfo, screenW, screenH);
}


void Application::init_material(MaterialInfo& material) {
  std::cerr << "Materials not implemented" << std::endl;
  exit(1);
}

void Application::set_up_pathtracer() {
  renderer->set_camera(&camera);
  renderer->set_frame_size(screenW, screenH);
  renderer->build_accel(primitives);
}

} // namespace CGL
