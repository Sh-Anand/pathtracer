#include "application.h"

#include "scene/gl_scene/ambient_light.h"
#include "scene/gl_scene/environment_light.h"
#include "scene/gl_scene/directional_light.h"
#include "scene/gl_scene/area_light.h"
#include "scene/gl_scene/point_light.h"
#include "scene/gl_scene/spot_light.h"
#include "scene/gl_scene/sphere.h"
#include "scene/gl_scene/mesh.h"

using Collada::CameraInfo;
using Collada::LightInfo;
using Collada::MaterialInfo;
using Collada::PolymeshInfo;
using Collada::SceneInfo;
using Collada::SphereInfo;

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
  //renderer->set_frame_size(w, h);
}

void Application::load(SceneInfo* sceneInfo) {

  vector<Collada::Node>& nodes = sceneInfo->nodes;
  vector<GLScene::SceneLight *> lights;
  vector<GLScene::SceneObject *> objects;

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
        lights.push_back(
          init_light(static_cast<LightInfo&>(*instance), transform));
        break;
      }
      case Collada::Instance::SPHERE:
        objects.push_back(
          init_sphere(static_cast<SphereInfo&>(*instance), transform));
        break;
      case Collada::Instance::POLYMESH:
        objects.push_back(
          init_polymesh(static_cast<PolymeshInfo&>(*instance), transform));
        break;
      case Collada::Instance::MATERIAL:
        init_material(static_cast<MaterialInfo&>(*instance));
        break;
     }
  }

  scene = new GLScene::Scene(objects, lights);

  const BBox& bbox = scene->get_bbox();
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

GLScene::SceneLight *Application::init_light(LightInfo& light,
                                        const Matrix4x4& transform) {
  switch(light.light_type) {
    case Collada::LightType::NONE:
      break;
    case Collada::LightType::AMBIENT:
      return new GLScene::AmbientLight(light);
    case Collada::LightType::DIRECTIONAL:
      return new GLScene::DirectionalLight(light, transform);
    case Collada::LightType::AREA:
      return new GLScene::AreaLight(light, transform);
    case Collada::LightType::POINT:
      return new GLScene::PointLight(light, transform);
    case Collada::LightType::SPOT:
      return new GLScene::SpotLight(light, transform);
    default:
      break;
  }
  return nullptr;
}

/**
 * The transform is assumed to be composed of translation, rotation, and
 * scaling, where the scaling is uniform across the three dimensions; these
 * assumptions are necessary to ensure the sphere is still spherical. Rotation
 * is ignored since it's a sphere, translation is determined by transforming the
 * origin, and scaling is determined by transforming an arbitrary unit vector.
 */
GLScene::SceneObject *Application::init_sphere(
    SphereInfo& sphere, const Matrix4x4& transform) {
  const Vector3D& position = (transform * Vector4D(0, 0, 0, 1)).projectTo3D();
  double scale = (transform * Vector4D(1, 0, 0, 0)).to3D().norm();
  return new GLScene::Sphere(sphere, position, scale);
}

GLScene::SceneObject *Application::init_polymesh(
    PolymeshInfo& polymesh, const Matrix4x4& transform) {
  return new GLScene::Mesh(polymesh, transform);
}

void Application::init_material(MaterialInfo& material) {
  // TODO : Support Materials.
}

void Application::set_up_pathtracer() {
  renderer->set_camera(&camera);
  renderer->set_scene(scene->get_static_scene());
  renderer->set_frame_size(screenW, screenH);

}

} // namespace CGL
