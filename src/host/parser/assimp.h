#include "parser.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "util/stb_image.h"

#include <stdexcept>
#include <iostream>
#include <unordered_map>

class AssImpParser : public Parser
{
public:
  AssImpParser(int screenW, int screenH, const std::string &filePath)
      : Parser(screenW, screenH)
  {
    scene_ = importer_.ReadFile(filePath,
                                aiProcess_Triangulate | 
                                aiProcess_GenSmoothNormals | 
                                aiProcess_CalcTangentSpace | 
                                aiProcess_JoinIdenticalVertices |
                                aiProcess_RemoveRedundantMaterials |
                                aiProcess_OptimizeMeshes |
                                aiProcess_OptimizeGraph  |
                                aiProcess_EmbedTextures  |
                                aiProcess_FlipUVs);

    if (!scene_)
      throw std::runtime_error(importer_.GetErrorString());

    CudaTexture emptyTexture;
    emptyTexture.width = 128; 
    emptyTexture.height = 1;
    emptyTexture.channels = 1;
    emptyTexture.data = (uint8_t*)calloc(128, 1); // allocate 128 bytes for empty texture

    textures.push_back(emptyTexture); // reserve index 0 for no texture
    texcoords.push_back({}); // reserve index 0 for no texture coordinate
    tangents.push_back({}); // reserve index 0 for no tangent

    ParseMaterialsAndTextures(scene_);
    
    Matrix4x4 identity = matrix4x4_identity();
    ParseNode(scene_, scene_->mRootNode, identity);

    // if no camera was found, create a default one
    if (!scene_->HasCameras()) {
      buildDefaultCam();
    }

    // if no lights were found, create a default light
    if (lights.empty()) {
      CudaLight light;
      light.radiance = {300.0f, 300.0f, 300.0f};
      light.is_point_light = true;
      light.position = camera.pos;
      light.radius = 0.0f; // point light
      lights.push_back(light);
    }
  }

private:
  Assimp::Importer importer_;
  const aiScene *scene_ = nullptr;

  Matrix4x4 AiToMatrix(const aiMatrix4x4& m)
  {
      Matrix4x4 M;
      M.c[0] = {m.a1, m.b1, m.c1, m.d1};
      M.c[1] = {m.a2, m.b2, m.c2, m.d2};
      M.c[2] = {m.a3, m.b3, m.c3, m.d3};
      M.c[3] = {m.a4, m.b4, m.c4, m.d4};   // <- Tx,Ty,Tz end up here
      return M;
  }

  /* -------------------------------------------------------------------------- */
  /* Helper: repeatedly halve until ≤ 256                                       */
  /* -------------------------------------------------------------------------- */
  static uint8_t* downscale_to_max256(uint8_t* src,
                                      int&      w,
                                      int&      h,
                                      int       comp)
  {
      while (w > 256 || h > 256)         /* keep halving both axes             */
      {
          const int newW = w >> 1;       /* assume w,h are even (true for 2048)*/
          const int newH = h >> 1;
          const size_t dstSz = (size_t)newW * newH * comp;
          uint8_t* dst = (uint8_t*)malloc(dstSz);

          for (int y = 0; y < newH; ++y)
          {
              for (int x = 0; x < newW; ++x)
              {
                  const size_t dstOfs = ( (size_t)y * newW + x ) * comp;

                  const size_t srcOfs00 = ( ((size_t)(y*2)   * w + (x*2)  ) * comp );
                  const size_t srcOfs01 = srcOfs00 + comp;
                  const size_t srcOfs10 = srcOfs00 + (size_t)w * comp;
                  const size_t srcOfs11 = srcOfs10 + comp;

                  for (int c = 0; c < comp; ++c)
                  {
                      uint32_t sum =
                          src[srcOfs00 + c] + src[srcOfs01 + c] +
                          src[srcOfs10 + c] + src[srcOfs11 + c];
                      dst[dstOfs + c] = (uint8_t)(sum >> 2);   /* ÷4 */
                  }
              }
          }

          free(src);                     /* recycle */
          src = dst;  w = newW;  h = newH;
      }
      return src;
  }

  /* -------------------------------------------------------------------------- */
  /*  Texture loader with built-in down-sampler                                 */
  /* -------------------------------------------------------------------------- */
  int LoadTexture(const aiString& path)
  {
      static std::unordered_map<std::string, int> cache;
      std::string key = path.C_Str();
      auto it = cache.find(key);
      if (it != cache.end()) return it->second;

      /* ---------- decode (Assimp embedded or external) -------------------- */
      CudaTexture tex;
      unsigned char* data = nullptr;
      int w, h, comp;

      if (!key.empty() && key[0] == '*') {
          /* Embedded -------------------------------------------------------- */
          int texIdx = std::atoi(key.c_str() + 1);
          aiTexture* at = scene_->mTextures[texIdx];
          if (at->mHeight == 0) {
              data = stbi_load_from_memory(reinterpret_cast<unsigned char*>(at->pcData),
                                          at->mWidth, &w, &h, &comp, 0);
          } else {
              w = at->mWidth;  h = at->mHeight;  comp = 4;
              size_t sz = (size_t)w * h * comp;
              data = (unsigned char*)malloc(sz);
              memcpy(data, at->pcData, sz);
          }
      } else {
          /* External -------------------------------------------------------- */
          data = stbi_load(path.C_Str(), &w, &h, &comp, 0);
      }

      if (!data) { std::cerr << "Failed to load texture " << key << '\n'; return -1; }

      /* ---------- cheap down-sample to ≤256×256 --------------------------- */
      data = downscale_to_max256(data, w, h, comp);

      /* ---------- upload to CPU-side texture array ------------------------ */
      tex.width    = (uint16_t)w;
      tex.height   = (uint16_t)h;
      tex.channels = (uint8_t)comp;
      size_t sz    = (size_t)w * h * comp;
      uint8_t *data_tex = (uint8_t*)malloc(sz);
      memcpy(data_tex, data, sz);
      tex.data = data_tex;
      stbi_image_free(data);

      int idx = (int)textures.size();
      textures.push_back(tex);
      cache[key] = idx;
      return idx;
  }


  void ParseMaterialsAndTextures(const aiScene *scene)
  {
    for (unsigned i = 0; i < scene->mNumMaterials; ++i)
    {
      aiMaterial *m = scene->mMaterials[i];
      CudaBSDF bsdf;

      // base color (PBR glTF key or fallback to diffuse)
      aiColor4D col(1, 1, 1, 1);
      if (AI_SUCCESS != m->Get(AI_MATKEY_BASE_COLOR, col))
        m->Get(AI_MATKEY_COLOR_DIFFUSE, col);
      bsdf.baseColor = Vector4D{col.r / PI, col.g / PI, col.b / PI, col.a};

      // metallic / roughness
      float metallic = 0.f, roughness = 1.f;
      m->Get(AI_MATKEY_METALLIC_FACTOR, metallic);
      m->Get(AI_MATKEY_ROUGHNESS_FACTOR, roughness);
      bsdf.metallic = metallic;
      bsdf.roughness = roughness;

      // emission
      aiColor3D em(0, 0, 0);
      m->Get(AI_MATKEY_COLOR_EMISSIVE, em);
      bsdf.emission = Vector3D{em.r, em.g, em.b};
      // KHR_materials_emissive_strength
      float emissive_strength = 0.0f;
      m->Get(AI_MATKEY_EMISSIVE_INTENSITY, emissive_strength);
      bsdf.emission = vector3d_scale(bsdf.emission, emissive_strength);

      // init texture indices
      bsdf.tex_idx = -1;
      bsdf.normal_idx = -1;
      bsdf.orm_idx = -1;
      bsdf.emission_idx = -1;

      aiString p;

      if (m->GetTexture(aiTextureType_BASE_COLOR, 0, &p) == AI_SUCCESS ||
          m->GetTexture(aiTextureType_DIFFUSE, 0, &p) == AI_SUCCESS)
        bsdf.tex_idx = LoadTexture(p);

      if (m->GetTexture(aiTextureType_NORMALS, 0, &p) == AI_SUCCESS)
        bsdf.normal_idx = LoadTexture(p);

      if (m->GetTexture(aiTextureType_METALNESS, 0, &p) == AI_SUCCESS)
        bsdf.orm_idx = LoadTexture(p);

      if (m->GetTexture(aiTextureType_EMISSIVE, 0, &p) == AI_SUCCESS)
        bsdf.emission_idx = LoadTexture(p);

      bsdfs.push_back(bsdf);
    }
  }

  void ParseNode(const aiScene *scene, aiNode *node, const Matrix4x4 &parentXform)
  {
    Matrix4x4 local = AiToMatrix(node->mTransformation);
    Matrix4x4 world = matrix4x4_multiply(&parentXform, &local);
    Matrix3x3 rotation = matrix3x3_from_matrix4x4(&world);
    Matrix3x3 normalMatrix = matrix3x3_inverse(&rotation);
    rotation = matrix3x3_transpose(&rotation);
    normalMatrix = matrix3x3_transpose(&normalMatrix);

    for (unsigned i = 0; i < node->mNumMeshes; ++i)
      ParseMesh(scene->mMeshes[node->mMeshes[i]], world, normalMatrix);

    for (unsigned i = 0; i < node->mNumChildren; ++i)
      ParseNode(scene, node->mChildren[i], world);

    // highly inefficient but we don't care
    for (unsigned i = 0; i < scene->mNumLights; ++i) {
      aiLight* L = scene->mLights[i];
      if (L->mName == node->mName) {
        CudaLight cl;
        // transform position
        Vector4D p{ L->mPosition.x, L->mPosition.y, L->mPosition.z, 1.0f };
        p = matrix4x4_vector_multiply(&world, &p);
        cl.position = vector4d_to3d(p);
        cl.radiance      = Vector3D{min(L->mColorDiffuse.r, 500.0f),
                                    min(L->mColorDiffuse.g, 500.0f),
                                    min(L->mColorDiffuse.b, 500.0f)};
        cl.is_point_light = true; // everything is a point light. probably not true, dont care.
        cl.radius = 0.0f; 
        lights.push_back(cl);
      }
    }

    // take first cam.
    if (scene_->HasCameras() && scene_->mCameras[0] && scene_->mCameras[0]->mName == node->mName) {
      ParseCamera(scene_->mCameras[0], world);
    }

  }

  void ParseMesh(const aiMesh *mesh, const Matrix4x4 &worldTransform, const Matrix3x3 &normalMatrix)
  {
    for (unsigned f = 0; f < mesh->mNumFaces; ++f)
    {
      const aiFace &face = mesh->mFaces[f];
      if (face.mNumIndices != 3)
        continue;

      uint32_t idx0 = face.mIndices[0],
               idx1 = face.mIndices[1],
               idx2 = face.mIndices[2];

      // positions
      aiVector3D a = mesh->mVertices[idx0],
                 b = mesh->mVertices[idx1],
                 c = mesh->mVertices[idx2];
      Vector4D A{a.x, a.y, a.z, 1.f},
          B{b.x, b.y, b.z, 1.f},
          C{c.x, c.y, c.z, 1.f};

      A = matrix4x4_vector_multiply(&worldTransform, &A);
      B = matrix4x4_vector_multiply(&worldTransform, &B);
      C = matrix4x4_vector_multiply(&worldTransform, &C);
      Vector3D p0 = vector4d_to3d(A),
               p1 = vector4d_to3d(B),
               p2 = vector4d_to3d(C);

      // normals
      Vector3D n0{0, 0, 0}, n1{0, 0, 0}, n2{0, 0, 0};
      if (mesh->HasNormals())
      {
        aiVector3D na = mesh->mNormals[idx0],
                   nb = mesh->mNormals[idx1],
                   nc = mesh->mNormals[idx2];
        Vector3D nna{na.x, na.y, na.z},
            nnb{nb.x, nb.y, nb.z},
            nnc{nc.x, nc.y, nc.z};
        nna = matrix3x3_vector_multiply(&normalMatrix, &nna);
        nnb = matrix3x3_vector_multiply(&normalMatrix, &nnb);
        nnc = matrix3x3_vector_multiply(&normalMatrix, &nnc);
        n0 = vector3d_unit(nna);
        n1 = vector3d_unit(nnb);
        n2 = vector3d_unit(nnc);
      }

      // texcoords
      Vector2D uv0{0, 0}, uv1{0, 0}, uv2{0, 0};
      if (mesh->HasTextureCoords(0))
      {
        aiVector3D ta = mesh->mTextureCoords[0][idx0],
                   tb = mesh->mTextureCoords[0][idx1],
                   tc = mesh->mTextureCoords[0][idx2];
        uv0 = Vector2D{ta.x, ta.y};
        uv1 = Vector2D{tb.x, tb.y};
        uv2 = Vector2D{tc.x, tc.y};
      }

      // tangents (and sign via bitangent)
      Vector4D t0{0, 0, 0, 0}, t1{0, 0, 0, 0}, t2{0, 0, 0, 0};
      if (mesh->HasTangentsAndBitangents())
      {
        aiVector3D ta = mesh->mTangents[idx0],
                   tb = mesh->mTangents[idx1],
                   tc = mesh->mTangents[idx2];
        aiVector3D ba = mesh->mBitangents[idx0],
                   bb = mesh->mBitangents[idx1],
                   bc = mesh->mBitangents[idx2];
        auto pack = [&](const aiVector3D &T, const aiVector3D &B, const Vector3D &N)
        {
          Vector3D t{T.x, T.y, T.z};
          float sign = (vector3d_dot(vector3d_cross(N, t), Vector3D{B.x, B.y, B.z}) < 0.f) ? -1.f : 1.f;
          t = vector3d_unit(t);
          return Vector4D{t.x, t.y, t.z, sign};
        };
        t0 = pack(ta, ba, n0);
        t1 = pack(tb, bb, n1);
        t2 = pack(tc, bc, n2);
      }

      // push into your buffers
      vertices.push_back(p0);
      vertices.push_back(p1);
      vertices.push_back(p2);
      normals.push_back(n0);
      normals.push_back(n1);
      normals.push_back(n2);
      texcoords.push_back(uv0);
      texcoords.push_back(uv1);
      texcoords.push_back(uv2);
      tangents.push_back(t0);
      tangents.push_back(t1);
      tangents.push_back(t2);

      // triangle struct
      CudaPrimitive prim;
      prim.i0 = uint32_t(vertices.size()) - 3;
      prim.i1 = uint32_t(vertices.size()) - 2;
      prim.i2 = uint32_t(vertices.size()) - 1;
      prim.bsdf_idx = mesh->mMaterialIndex;
      primitives.push_back(prim);

      // if this material emits, add an area light
      auto &M = bsdfs[prim.bsdf_idx];
      if (M.emission.x > 0 || M.emission.y > 0 || M.emission.z > 0)
      {
        CudaLight L;
        L.triangle = prim;
        L.area = vector3d_norm(vector3d_cross(vector3d_sub(p1, p0), vector3d_sub(p2, p0))) * 0.5f;
        L.radiance = M.emission;
        L.is_point_light = false;
        L.radius = 0.0f;
        L.position = Vector3D{0, 0, 0};
        lights.push_back(L);
      }
    }
  }

  // ————————————————————————————————————————————————————————————————
  // Build a default camera that frames the scene’s bounding sphere
  // and applies user-controlled offsets / rotations.
  //
  //  yawDeg   : +Y CCW (look left)  – use − to look right
  //  pitchDeg : + up (tilt up)      – use − to tilt down
  //  lift     : + up / − down       (in units of scene radius)
  //  strafe   : + right / − left    (in units of scene radius)
  //  dolly    : 1 = fit exactly, <1 closer, >1 farther
  // ————————————————————————————————————————————————————————————————
  void buildDefaultCam(float yawDeg   = -20.0f,
                      float pitchDeg = -20.0f,
                      float lift     = 0.0f,
                      float strafe   = 0.0f,
                      float dolly    = 0.8f)
  {
      /* 1. ------------------------------------------------------------------ */
      BBox bbox;
      for (const auto &p : primitives) {
          BBox tri(vertices[p.i0]);
          tri.expand(vertices[p.i1]);
          tri.expand(vertices[p.i2]);
          bbox.expand(tri);
      }
      Vector3D C = bbox.centroid();
      float    R = 0.5f * vector3d_norm(bbox.extent); // half-diag = “radius”

      /* 2. ------------------------------------------------------------------ */
      camera.ar      = float(screenW) / float(screenH);
      camera.vFov    = 35.0f;          // customise if you like
      camera.hFov    = 50.0f;
      camera.nClip   = 0.01f;
      camera.fClip   = 1000.0f;

      /* 3. Pitch / Yaw ------------------------------------------------------- */
      float yaw   = yawDeg   * (PI / 180.0f);
      float pitch = pitchDeg * (PI / 180.0f);

      float cosP = std::cos(pitch), sinP = std::sin(pitch);
      float cosY = std::cos(yaw)  , sinY = std::sin(yaw);

      Vector3D viewDir = vector3d_unit({
          sinY * cosP,   // x
          sinP,          // y
        -cosY * cosP    // z  (points roughly −Z by default)
      });

      /* 4. Distance so the sphere fits (then scaled by dolly) --------------- */
      float halfFov = 0.5f *
                      std::min(camera.vFov, camera.hFov) * (PI / 180.0f);
      float dist = (R / std::sin(halfFov)) * dolly;

      /* 5. Orthonormal basis ------------------------------------------------- */
      Vector3D upWorld{0.0f, 1.0f, 0.0f};
      Vector3D w = viewDir;                               // forward
      Vector3D u = vector3d_unit(vector3d_cross(upWorld, w)); // right
      Vector3D v = vector3d_cross(w, u);                      // up-corrected

      /* 6. Camera position with lift & strafe ------------------------------- */
      camera.pos = vector3d_sub(C,
                  vector3d_add(vector3d_scale(w, dist),                                  // back off
                              vector3d_add(vector3d_scale(v, lift   * R),               // vertical
                                            vector3d_scale(u, strafe * R))));            // horizontal

      /* 7. Store c2w --------------------------------------------------------- */
      camera.c2w.c[0] = u;
      camera.c2w.c[1] = v;
      camera.c2w.c[2] = vector3d_neg(w);
  }

  void ParseCamera(const aiCamera *C, const Matrix4x4 &world) {
    // 1) basic parameters
    float ar = float(screenW) / float(screenH);

    // 2) FOVs
    float hFovRad = C->mHorizontalFOV;                           // radians, horizontal
    float vFovRad = 2.0f * atanf( tanf(hFovRad * 0.5f) / ar );

    camera.hFov = hFovRad * (180.0f / M_PI);                     // degrees
    camera.vFov = vFovRad * (180.0f / M_PI);

    // 4) clip planes
    camera.nClip = C->mClipPlaneNear;
    camera.fClip = C->mClipPlaneFar;

    // 5) position
    Vector4D pos {C->mPosition.x, C->mPosition.y, C->mPosition.z, 1.0f};
    camera.pos = vector4d_to3d(matrix4x4_vector_multiply(&world, &pos));

    aiMatrix4x4 camMat;
    C->GetCameraMatrix(camMat);
    Matrix4x4 camXform = AiToMatrix(camMat);
    if(camXform.c[0].x < 0) {
      // looking at -ve so flip
      camXform.c[0].x *= -1;
      camXform.c[2].z *= -1;
    }
    Matrix4x4 worldcam = matrix4x4_multiply(&world, &camXform);
    camera.c2w = matrix3x3_from_matrix4x4(&worldcam);
  }

};
