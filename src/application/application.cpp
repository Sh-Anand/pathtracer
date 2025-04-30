#include "application.h"

#include "pathtracer/bsdf.h"
#include "scene/gl_scene/mesh.h"

#include "scene/primitive.h"
#include "util/matrix3x3.h"
#include "util/matrix4x4.h"
#include "util/quaternion.h"
#include "util/tiny_gltf.h"
#include "util/transforms.h"
#include "util/vector3D.h"
#include "util/vector4D.h"

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

Vector3D computeDiffuseBSDF(const Vector3D& baseColor, float metallic) {
  Vector3D diffuseColor = baseColor; //(1.0f - metallic) * baseColor;
  return diffuseColor / M_PI;
}

Matrix4x4 GetNodeTransform(const tinygltf::Node &node) {
  Matrix4x4 T(1.0f);

  // if (!node.matrix.empty()) {
  //     return glm::make_mat4(node.matrix.data());
  // }

  Vector3D translation(0.0f), _scale(1.0f);
  Quaternion rotation = Quaternion(0, 0, 0, 1);

  if (!node.translation.empty())
      translation = Vector3D(node.translation[0], node.translation[1], node.translation[2]);
  if (!node.rotation.empty())
      rotation = Quaternion(node.rotation[0], node.rotation[1], node.rotation[2], node.rotation[3]);
  if (!node.scale.empty())
      _scale = Vector3D(node.scale[0], node.scale[1], node.scale[2]);

  T = translate(translation.x, translation.y, translation.z)
    * rotation.rotationMatrix().to4x4()
    * scale(_scale.x, _scale.y, _scale.z);

  return T;
}

CameraInfo cam;

void Application::ParseNode(const tinygltf::Model &model, int nodeIdx, const Matrix4x4 &parentTransform){
  const auto &node = model.nodes[nodeIdx];
  Matrix4x4 worldTransform = parentTransform * GetNodeTransform(node);
  Matrix3x3 normalMatrix = Matrix3x3(worldTransform).inv().T();
  
  if (node.mesh >= 0) {
    const auto &mesh = model.meshes[node.mesh];
    for (const auto &primitive : mesh.primitives) {
        if (primitive.mode != TINYGLTF_MODE_TRIANGLES) continue;
        const auto &posAccessor = model.accessors[primitive.attributes.at("POSITION")];
        const auto &normAccessor = model.accessors[primitive.attributes.at("NORMAL")];

        const auto &posView = model.bufferViews[posAccessor.bufferView];
        const auto &normView = model.bufferViews[normAccessor.bufferView];

        const auto &posBuffer = model.buffers[posView.buffer];
        const auto &normBuffer = model.buffers[normView.buffer];

        const float *posData = reinterpret_cast<const float*>(&posBuffer.data[posView.byteOffset + posAccessor.byteOffset]);
        const float *normData = reinterpret_cast<const float*>(&normBuffer.data[normView.byteOffset + normAccessor.byteOffset]);

        const auto &indexAccessor = model.accessors[primitive.indices];
        const auto &indexView = model.bufferViews[indexAccessor.bufferView];
        const auto &indexBuffer = model.buffers[indexView.buffer];
        const void *indexData = &indexBuffer.data[indexView.byteOffset + indexAccessor.byteOffset];

        auto getIndex = [&](size_t i) -> uint32_t {
            switch (indexAccessor.componentType) {
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: return reinterpret_cast<const uint16_t*>(indexData)[i];
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:   return reinterpret_cast<const uint32_t*>(indexData)[i];
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:  return reinterpret_cast<const uint8_t*>(indexData)[i];
                default: throw std::runtime_error("Unsupported index type");
            }
        };

        for (size_t i = 0; i < indexAccessor.count; i += 3) {
            uint32_t i0 = getIndex(i);
            uint32_t i1 = getIndex(i + 1);
            uint32_t i2 = getIndex(i + 2);

            Vector3D p1 = Vector3D(posData[i0 * 3 + 0], posData[i0 * 3 + 1], posData[i0 * 3 + 2]);
            Vector3D p2 = Vector3D(posData[i1 * 3 + 0], posData[i1 * 3 + 1], posData[i1 * 3 + 2]);
            Vector3D p3 = Vector3D(posData[i2 * 3 + 0], posData[i2 * 3 + 1], posData[i2 * 3 + 2]);

            Vector3D n1 = Vector3D(normData[i0 * 3 + 0], normData[i0 * 3 + 1], normData[i0 * 3 + 2]);
            Vector3D n2 = Vector3D(normData[i1 * 3 + 0], normData[i1 * 3 + 1], normData[i1 * 3 + 2]);
            Vector3D n3 = Vector3D(normData[i2 * 3 + 0], normData[i2 * 3 + 1], normData[i2 * 3 + 2]);

            // Transform to world space
            p1 = (worldTransform * Vector4D(p1, 1.0f)).to3D();
            p2 = (worldTransform * Vector4D(p2, 1.0f)).to3D();
            p3 = (worldTransform * Vector4D(p3, 1.0f)).to3D();

            n1 = (normalMatrix * n1);
            n2 = (normalMatrix * n2);
            n3 = (normalMatrix * n3);

            n1.normalize(); n2.normalize(); n3.normalize();

            CudaTriangle ct = CudaTriangle();
            ct.p1 = p1; ct.p2 = p2; ct.p3 = p3;
            ct.n1 = n1; ct.n2 = n2; ct.n3 = n3;
            CudaPrimitive cprimitive {};
            cprimitive.type = CGL::SceneObjects::CudaPrimitiveType::TRIANGLE;
            cprimitive.primitive.triangle = ct;            
            cprimitive.bsdf_idx = primitive.material;
            primitives.push_back(cprimitive);

            if(bsdfs[cprimitive.bsdf_idx].type == CudaBSDFType_Emission){
              CudaLight clight {};
              clight.type = (CudaLightType) CGL::SceneObjects::CudaLightType::TRIANGLELight;
              // cout << "Parsing light: " << ct.p1 << " " << ct.p2 << " " << ct.p3 << " " << ct.n1 << " " << ct.n2 << " " << ct.n3 << endl;
              clight.light.triangle = CGL::SceneObjects::CudaTriangleLight{bsdfs[cprimitive.bsdf_idx].bsdf.emission.radiance , Vector3D(0, -1, 0), ct};
              lights.push_back(clight);
              // std::cout << "light pushed" << std::endl;
            }
        }
    }
  }else if(node.light >= 0){
    // adding lights
    std::cout << "light" << std::endl;
    CudaLight clight {};

    auto ext = node.extensions.find("KHR_lights_punctual");
    if (ext != node.extensions.end()) {
        int lightIndex = ext->second.Get("light").Get<int>();
        const auto& light = model.extensions.at("KHR_lights_punctual").Get("lights").Get(lightIndex);

        if (light.Get("type").Get<std::string>() == "point") {
          // this is a hack, since we don't have a light type, treat point light as area light
          clight.type = (CudaLightType) Collada::LightType::AREA;
          Vector3D spectrum = Vector3D(10, 10, 10);
          Vector3D direction = Vector3D(0, -1, 0);
          Vector3D dim_x = Vector3D(0.6, 0, 0);
          Vector3D dim_y = Vector3D(0, 1, 0.8);
          // Position is from the node's transform
          Vector3D position = (worldTransform * Vector4D(0, 0, 0, 1)).to3D();
          // Vector3D position = Vector3D(0, 1.49, 0);
          // clight.light.point = CudaPointLight{spectrum, position};
          clight.light.area = CudaAreaLight(spectrum, position, direction, dim_x, dim_y);
          std::cout << "Point light position: (" << position.x << ", " << position.y << ", " << position.z << ")\n";
        }
    }
    
    lights.push_back(clight);
  }else if(node.camera >= 0){
    // adding camera code
    auto gltfCam = model.cameras[node.camera];

    cam.vFov  = degrees(gltfCam.perspective.yfov);
    cam.hFov  = degrees(2.0f * atan(tan(gltfCam.perspective.yfov / 2.0f) * gltfCam.perspective.aspectRatio));
    cam.nClip = gltfCam.perspective.znear;
    cam.fClip = gltfCam.perspective.zfar;

    // Get transform from camera node
    Matrix4x4 camTransform = worldTransform;
    // std::cout << node.translation[0] << "," << node.translation[1] << "," << node.translation[2] << std::endl;

    Vector3D view = camTransform[2].to3D(); // -Z
    view.normalize();
    Vector3D up   = camTransform[1].to3D();  // +Y
    up.normalize();

    cam.view_dir = view;
    cam.up_dir   = up;

    init_camera(cam, camTransform);

  }
  for (int childIdx : node.children) {
    ParseNode(model, childIdx, worldTransform);
  }
}

void Application::ParseMaterial(const tinygltf::Model &model) {
  for(const auto &material: model.materials) {
    if(!material.emissiveFactor.empty() && material.emissiveFactor[0] > 0.0f){ 
      // std::cout << "light" << std::endl;
      Vector3D rad;
      rad.x = (float)material.emissiveFactor[0];
      rad.y = (float)material.emissiveFactor[1];
      rad.z = (float)material.emissiveFactor[2];
      double emissiveStrength = 1.0f;
      if (material.extensions.count("KHR_materials_emissive_strength")) {
        const auto& ext = material.extensions.at("KHR_materials_emissive_strength").Get("emissiveStrength");
        emissiveStrength = (float)ext.Get<double>();
      }
      rad *= emissiveStrength;
      BSDF* bsdf = new EmissionBSDF(rad);
      push_cuda_bsdf(bsdf, bsdfs);
    } else{
      Vector3D baseColor = Vector3D(
        material.pbrMetallicRoughness.baseColorFactor[0],
        material.pbrMetallicRoughness.baseColorFactor[1],
        material.pbrMetallicRoughness.baseColorFactor[2]
      );
      float metallic = material.pbrMetallicRoughness.metallicFactor;
      BSDF* bsdf = new DiffuseBSDF(computeDiffuseBSDF(baseColor, metallic));
      // std::cout << "baseColor: " << baseColor << " metallic: " << metallic << std::endl;
      push_cuda_bsdf(bsdf, bsdfs);
    }
  }
}

void Application::load_from_gltf_model(const tinygltf::Model &model) {

  // load material
  ParseMaterial(model);

  const auto &scene = model.scenes[model.defaultScene > -1 ? model.defaultScene : 0];
  for (int rootNode : scene.nodes) {
    ParseNode(model, rootNode, Matrix4x4(1.0f));
  }

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
                          acos(cam.view_dir.y),
                          atan2(cam.view_dir.x, cam.view_dir.z),
                          view_distance,
                          min_view_distance,
                          max_view_distance);

    camera.place(target,
                acos(cam.view_dir.y),
                atan2(cam.view_dir.x, cam.view_dir.z),
                view_distance,
                min_view_distance,
                max_view_distance);
  }
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
  
  for(auto& primitive: primitives){
    auto& triangle = primitive.primitive.triangle;
    std::cout << triangle.p1 << "," << triangle.p2 << "," << triangle.p3 << std::endl;
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
