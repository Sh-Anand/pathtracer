#include "parser.h"

// gltf stuff
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
// #define TINYGLTF_NOEXCEPTION // optional. disable exception handling.
#include "util/tiny_gltf.h"

using namespace std;

class glTFParser : public Parser {

  public:

  Matrix4x4 GetNodeTransform(const tinygltf::Node &node) {
    Matrix4x4 T = matrix4x4_identity();

    if (!node.matrix.empty()) {
      // tinygltf stores matrix in column‐major order
      Matrix4x4 M;
      for (int i = 0; i < 16; i+=4) {
        M.c[i%4] = Vector4D{float(node.matrix[i]), float(node.matrix[i+1]), float(node.matrix[i+2]), float(node.matrix[i+3])};
      }
      return M;
    }
    Vector3D translation{0.0f, 0.0f, 0.0f}, _scale{1.0f, 1.0f, 1.0f};
    Quaternion rotation = Quaternion(0, 0, 0, 1);

    if (!node.translation.empty())
        translation = Vector3D{float(node.translation[0]), float(node.translation[1]), float(node.translation[2])};
    if (!node.rotation.empty())
        rotation = Quaternion(node.rotation[0], node.rotation[1], node.rotation[2], node.rotation[3]);
    if (!node.scale.empty())
        _scale = Vector3D{float(node.scale[0]), float(node.scale[1]), float(node.scale[2])};

    Matrix4x4 S = scale(_scale.x, _scale.y, _scale.z);
    Matrix3x3 R3 = rotation.rotationMatrix();
    Matrix4x4 R = matrix3x3_to4x4(&R3);
    Matrix4x4 RS = matrix4x4_multiply(&R, &S);
    Matrix4x4 tranl = translate(translation.x, translation.y, translation.z);
    T = matrix4x4_multiply(&tranl, &RS);

    return T;
  }

  void ParseNode(const tinygltf::Model &model, int nodeIdx, const Matrix4x4 &parentTransform){
    const auto &node = model.nodes[nodeIdx];
    Matrix4x4 nodeTransform = GetNodeTransform(node);
    Matrix4x4 worldTransform = matrix4x4_multiply(&parentTransform,  &nodeTransform);
    Matrix3x3 normalMatrix = matrix3x3_from_matrix4x4(&worldTransform);
    // Invert the normal matrix
    normalMatrix = matrix3x3_inverse(&normalMatrix);
    normalMatrix = matrix3x3_transpose(&normalMatrix);

  if (matrix4x4_determinant(&worldTransform) < 0.0f) {
    // flip the handedness
    normalMatrix = matrix3x3_scale(&normalMatrix, -1.0f);
  }
    
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

          // Get texture coordinates if available
          auto uvIt = primitive.attributes.find("TEXCOORD_0");
          const float *uvData = nullptr;
          if (uvIt != primitive.attributes.end()) {
            const auto &uvAccessor = model.accessors[primitive.attributes.at("TEXCOORD_0")];
            const auto &uvView = model.bufferViews[ uvAccessor.bufferView ];
            const auto &uvBuf = model.buffers [uvView.buffer];
            uvData = reinterpret_cast<const float*>(&uvBuf.data[uvView.byteOffset + uvAccessor.byteOffset]);
          }

          // Get tangents if available
          auto tangentIt = primitive.attributes.find("TANGENT");
          const float *tangentData = nullptr;
          if (tangentIt != primitive.attributes.end()) {
            const auto &tangentAccessor = model.accessors[primitive.attributes.at("TANGENT")];
            const auto &tangentView = model.bufferViews[ tangentAccessor.bufferView ];
            const auto &tangentBuf = model.buffers [tangentView.buffer];
            tangentData = reinterpret_cast<const float*>(&tangentBuf.data[tangentView.byteOffset + tangentAccessor.byteOffset]);
          }

          for (size_t i = 0; i < indexAccessor.count; i += 3) {
              uint32_t i0 = getIndex(i);
              uint32_t i1 = getIndex(i + 1);
              uint32_t i2 = getIndex(i + 2);

              Vector3D p1 = Vector3D{posData[i0 * 3 + 0],posData[i0 * 3 + 1],posData[i0 * 3 + 2]};
              Vector3D p2 = Vector3D{posData[i1 * 3 + 0],posData[i1 * 3 + 1],posData[i1 * 3 + 2]};
              Vector3D p3 = Vector3D{posData[i2 * 3 + 0],posData[i2 * 3 + 1],posData[i2 * 3 + 2]};

              Vector3D n1 = Vector3D{normData[i0 * 3 + 0],normData[i0 * 3 + 1],normData[i0 * 3 + 2]};
              Vector3D n2 = Vector3D{normData[i1 * 3 + 0],normData[i1 * 3 + 1],normData[i1 * 3 + 2]};
              Vector3D n3 = Vector3D{normData[i2 * 3 + 0],normData[i2 * 3 + 1],normData[i2 * 3 + 2]};

              // Transform to world space
              Vector4D p1t = Vector4D{p1.x, p1.y, p1.z, 1.0f}, 
                      p2t = Vector4D{p2.x, p2.y, p2.z, 1.0f}, 
                      p3t = Vector4D{p3.x, p3.y, p3.z, 1.0f};
              p1t = matrix4x4_vector_multiply(&worldTransform, &p1t);
              p2t = matrix4x4_vector_multiply(&worldTransform, &p2t);
              p3t = matrix4x4_vector_multiply(&worldTransform, &p3t);
              p1 = vector4d_to3d(p1t);
              p2 = vector4d_to3d(p2t);
              p3 = vector4d_to3d(p3t);

              n1 = vector3d_unit(matrix3x3_vector_multiply(&normalMatrix, &n1));
              n2 = vector3d_unit(matrix3x3_vector_multiply(&normalMatrix, &n2));
              n3 = vector3d_unit(matrix3x3_vector_multiply(&normalMatrix, &n3));

              vertices.push_back(p1);
              vertices.push_back(p2);
              vertices.push_back(p3);
              normals.push_back(n1);
              normals.push_back(n2);
              normals.push_back(n3);

              if (uvIt != primitive.attributes.end()) {
                Vector2D uv1{uvData[i0*2+0], uvData[i0*2 + 1]};
                Vector2D uv2{uvData[i1*2+0], uvData[i1*2 + 1]};
                Vector2D uv3{uvData[i2*2+0], uvData[i2*2 + 1]};
                texcoords.push_back(uv1);
                texcoords.push_back(uv2);
                texcoords.push_back(uv3);
                if (tangentIt != primitive.attributes.end()) {
                  Vector3D t1{tangentData[i0*4+0], tangentData[i0*4+1], tangentData[i0*4+2]};
                  Vector3D t2{tangentData[i1*4+0], tangentData[i1*4+1], tangentData[i1*4+2]};
                  Vector3D t3{tangentData[i2*4+0], tangentData[i2*4+1], tangentData[i2*4+2]};
                  t1 = vector3d_unit(t1); t2 = vector3d_unit(t2), t3 = vector3d_unit(t3); 
                  tangents.push_back(Vector4D{t1.x, t1.y, t1.z, tangentData[i0*4+3]});
                  tangents.push_back(Vector4D{t2.x, t2.y, t2.z, tangentData[i1*4+3]});
                  tangents.push_back(Vector4D{t3.x, t3.y, t3.z, tangentData[i2*4+3]});
                } else {
                  tangents.push_back(Vector4D{0.0f, 0.0f, 0.0f, 0.0f});
                  tangents.push_back(Vector4D{0.0f, 0.0f, 0.0f, 0.0f});
                  tangents.push_back(Vector4D{0.0f, 0.0f, 0.0f, 0.0f});
                }
              } else {
                // if no texture coordinates, add dummy ones
                texcoords.push_back(Vector2D{0.0f, 0.0f});
                texcoords.push_back(Vector2D{0.0f, 0.0f});
                texcoords.push_back(Vector2D{0.0f, 0.0f});
                tangents.push_back(Vector4D{0.0f, 0.0f, 0.0f, 0.0f});
                tangents.push_back(Vector4D{0.0f, 0.0f, 0.0f, 0.0f});
                tangents.push_back(Vector4D{0.0f, 0.0f, 0.0f, 0.0f});
              }

              CudaPrimitive cprimitive {
                  static_cast<uint32_t>(vertices.size() - 3),
                  static_cast<uint32_t>(vertices.size() - 2),
                  static_cast<uint32_t>(vertices.size() - 1),
                  primitive.material,
              };
              primitives.push_back(cprimitive);

              if(bsdfs[cprimitive.bsdf_idx].emission.x > 0.0f || 
                bsdfs[cprimitive.bsdf_idx].emission.y > 0.0f ||
                bsdfs[cprimitive.bsdf_idx].emission.z > 0.0f) {
                CudaLight clight;
                clight.radiance = bsdfs[cprimitive.bsdf_idx].emission;
                clight.triangle = cprimitive;
                clight.area = vector3d_norm(vector3d_cross(vector3d_sub(p2, p1), vector3d_sub(p3, p1))) / 2.0f;
                clight.is_point_light = false;
                lights.push_back(clight);
              }
          }
      }
    }else if (node.camera >= 0) {
      // 1) grab your glTF camera
      const tinygltf::Camera &gCam = model.cameras[node.camera];
      if (gCam.type != "perspective")
        throw std::runtime_error("only perspective cameras supported");

      // assume you have a Camera instance called camera
      // and screenW, screenH in scope.

      // 2) screen parameters
      float ar      = float(screenW) / float(screenH);

      // 3) FOV in degrees
      float yFovRad = float(gCam.perspective.yfov);
      // if glTF aspectRatio was zero, fall back on viewport ar
      float aspect = gCam.perspective.aspectRatio > 0.0f
                    ? float(gCam.perspective.aspectRatio)
                    : ar;

      camera.vFov = yFovRad * (180.0f / M_PI);
      camera.hFov = 2.0f * atanf( tanf(yFovRad * 0.5f) * aspect )
                        * (180.0f / M_PI);

      // 4) clipping planes
      camera.nClip = float(gCam.perspective.znear);
      camera.fClip = float(gCam.perspective.zfar);

      // 7) carve your node’s worldTransform 4×4 into origin + axes
      Vector4D P{0,0,0,1}, X{1,0,0,0}, Y{0,1,0,0}, Z{0,0,1,0};
      Vector4D Pw = matrix4x4_vector_multiply(&worldTransform, &P);
      Vector4D Xw = matrix4x4_vector_multiply(&worldTransform, &X);
      Vector4D Yw = matrix4x4_vector_multiply(&worldTransform, &Y);
      Vector4D Zw = matrix4x4_vector_multiply(&worldTransform, &Z);

      // 8) fill camera position + c2w
      Matrix3x3 Mcol;
      Mcol.c[0] = vector4d_to3d(Xw);
      Mcol.c[1] = vector4d_to3d(Yw);
      Mcol.c[2] = vector4d_to3d(Zw);

      // transpose it so columns ↔ rows
      camera.c2w = matrix3x3_transpose(&Mcol);
      camera.pos = vector4d_to3d(Pw);

      // (no call to place(), we’ve now exactly replicated the glTF node’s
      // translation and rotation, so rays come off at the same spot & angle)
  }else if(node.light >= 0){
      // adding lights
      auto ext = node.extensions.find("KHR_lights_punctual");
      if (ext != node.extensions.end()) {
          const auto& ext = node.extensions.at("KHR_lights_punctual");
          int lightIndex = ext.Get("light").Get<int>();
          const auto& light = model.lights[lightIndex];

          if (light.type == "spot") {
            Vector3D color = {1.0f, 1.0f, 1.0f};
              if (!light.color.empty()) {
                  color = {
                      (float)light.color[0],
                      (float)light.color[1],
                      (float)light.color[2]
                  };
              }
            float intensity = (float)light.intensity; // in lux
            float innerConeAngle = (float)light.spot.innerConeAngle;
            float outerConeAngle = (float)light.spot.outerConeAngle - PI/16;
            // Position is from the node's transform
            Vector4D posv{
              0.0f, 0.0f, 0.0f, 1.0f
            };
            Vector4D dirv{
              0.0f, 0.0f, -1.0f, 0.0f
            };

            posv = matrix4x4_vector_multiply(&worldTransform, &posv);
            dirv = matrix4x4_vector_multiply(&worldTransform, &dirv);
            Vector3D position = vector4d_to3d(posv);
            Vector3D direction = vector4d_to3d(dirv);
            CudaLight clight;
            clight.radiance = vector3d_scale(color, (intensity/1000));
            clight.position = position;
            clight.is_point_light = true;
            cout << "GLTF Point light at " << clight.position.x << ", " << clight.position.y << ", " << clight.position.z << endl;
            cout << "GLTF Point light rad: " << clight.radiance.x << ", " << clight.radiance.y << ", " << clight.radiance.z << endl;
            lights.push_back(clight);
          } else if (light.type == "point") {
            // default white if no color provided
            Vector3D color = {1.0f, 1.0f, 1.0f};
            if (!light.color.empty()) {
                color = {
                    (float)light.color[0],
                    (float)light.color[1],
                    (float)light.color[2]
                };
            }

            // intensity in lux → convert to radiance scale (divide or adjust as needed)
            float intensity = (float)light.intensity; 
            // position from the node's world transform
            Vector4D posv{ 0.0f, 0.0f, 0.0f, 1.0f };
            posv = matrix4x4_vector_multiply(&worldTransform, &posv);
            Vector3D position = vector4d_to3d(posv);

            CudaLight clight;
            // scale color by intensity (and any unit conversion)
            clight.radiance      = vector3d_scale(color, intensity / 1000.0f);
            clight.position      = position;
            clight.is_point_light = true;
            // leave triangle & area unset (ignored for point lights)

            lights.push_back(clight);
        }
      }
    }

    for (int childIdx : node.children) {
      ParseNode(model, childIdx, worldTransform);
    }
  }

  void ParseMaterial(const tinygltf::Model &model) {
    for(const auto &material: model.materials) {
      CudaBSDF bsdf;
      bsdf.baseColor = Vector4D{float(material.pbrMetallicRoughness.baseColorFactor[0])/PI,
                                float(material.pbrMetallicRoughness.baseColorFactor[1])/PI,
                                float(material.pbrMetallicRoughness.baseColorFactor[2])/PI,
                                float(material.pbrMetallicRoughness.baseColorFactor[3])};
      bsdf.metallic = material.pbrMetallicRoughness.metallicFactor;
      bsdf.roughness = material.pbrMetallicRoughness.roughnessFactor;
      bsdf.emission = Vector3D{float(material.emissiveFactor[0]),float(material.emissiveFactor[1]),float(material.emissiveFactor[2])};
      bsdf.emission  = vector3d_scale(bsdf.emission, (material.extensions.count("KHR_materials_emissive_strength") ?
          material.extensions.at("KHR_materials_emissive_strength").Get("emissiveStrength").Get<double>()/2 : 0.0f));

      bsdf.tex_idx = material.pbrMetallicRoughness.baseColorTexture.index;
      bsdf.normal_idx = material.normalTexture.index;
      bsdf.orm_idx = max(material.occlusionTexture.index, material.pbrMetallicRoughness.metallicRoughnessTexture.index);
      bsdf.emission_idx = material.emissiveTexture.index;
      bsdfs.push_back(bsdf);
    }
  }

  void ParseTexture(const tinygltf::Model &model) {
    for (const auto &texture : model.textures) {
      const auto &image = model.images[texture.source];
      // const auto &sampler = model.samplers[texture.sampler]; // we will not use this, only need for mipmapping and clamping

      CudaTexture ctex;
      ctex.width = image.width;
      ctex.height = image.height;
      ctex.has_alpha = (image.component == 4);
      ctex.data = (uint8_t *) malloc(image.width * image.height * image.component);
      memcpy(ctex.data, image.image.data(), image.width * image.height * image.component);
      textures.push_back(ctex);
    }
  }

  void load_from_gltf_model(const tinygltf::Model &model) {
    // load material
    ParseMaterial(model);

    // load textures
    ParseTexture(model);

    const auto &scene = model.scenes[model.defaultScene > -1 ? model.defaultScene : 0];
    for (int rootNode : scene.nodes) {
      ParseNode(model, rootNode, matrix4x4_identity());
    }
  }

  glTFParser(int screenW = 800, int screenH = 600, const string& filePath = "") : Parser(screenW, screenH) {
    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    bool ret = loader.LoadBinaryFromFile(&model, &err, &warn, filePath);
    if (!warn.empty()) {
      printf("Warn: %s\n", warn.c_str());
    }

    if (!err.empty()) {
      printf("Err: %s\n", err.c_str());
    }
    if (!ret) {
      throw std::runtime_error("Failed to parse glTF file: " + filePath);
    }

    load_from_gltf_model(model);
  }

  tinygltf::Model model; // glTF model
  tinygltf::TinyGLTF loader; // glTF loader
  string err; // error message
  string warn; // warning message
};