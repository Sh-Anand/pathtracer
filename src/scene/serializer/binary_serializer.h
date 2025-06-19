#include "serializer.h"
#include <cstdint>
#include <cstring>

// helper to write a uint32 count + raw block of POD elements
template<typename T>
void write_array(FILE* f, const T* data, size_t count) {
    uint32_t n = static_cast<uint32_t>(count);
    fwrite(&n, sizeof(n), 1, f);
    fwrite(data, sizeof(T), count, f);
}

class BinarySerializer : public Serializer {
  // Vector<2|3|4>D
  void serialize(FILE* f, const std::vector<Vector2D>& v, std::string) override {
    write_array(f, v.data(), v.size());
  }
  void serialize(FILE* f, const std::vector<Vector3D>& v, std::string) override {
    write_array(f, v.data(), v.size());
  }
  void serialize(FILE* f, const std::vector<Vector4D>& v, std::string) override {
    write_array(f, v.data(), v.size());
  }

  // Camera is a POD: dump its bits directly
  void serialize(FILE* f, const Camera& cam) override {
    fwrite(&cam, sizeof(cam), 1, f);
  }

  // Lights, BSDFs, Primitives, BVHNodes
  void serialize_lights   (FILE* f, const std::vector<CudaLight>&   lights)   override {
    write_array(f, lights.data(), lights.size());
  }
  void serialize_bsdfs    (FILE* f, const std::vector<CudaBSDF>&    bsdfs)    override {
    write_array(f, bsdfs.data(),  bsdfs.size());
  }
  void serialize_primitives(FILE* f, const std::vector<CudaPrimitive>& prims) override {
    write_array(f, prims.data(),  prims.size());
  }
  void serialize_bvh_nodes(FILE* f, const std::vector<BVHNode>&     nodes)    override {
    write_array(f, nodes.data(),  nodes.size());
  }

  // Textures: write count, then for each: width,height,channels, then raw texels
  void serialize_textures(FILE* f, const std::vector<CudaTexture>& tex) override {
    uint32_t n = static_cast<uint32_t>(tex.size());
    fwrite(&n, sizeof(n), 1, f);
    for (const auto &T : tex) {
      uint16_t w = T.width, h = T.height;
      uint32_t c = T.channels;
      fwrite(&w, sizeof(w), 1, f);
      fwrite(&h, sizeof(h), 1, f);
      fwrite(&c, sizeof(c), 1, f);
      size_t sz = size_t(w) * h * c;
      fwrite(T.data, sizeof(uint8_t), sz, f);
    }
  }

  // Empty buffers: just repeat default-constructed POD
  void serialize_empty_gpurng    (FILE* f, int N) override {
    RNGState zero{};
    for (int i = 0; i < N; ++i) fwrite(&zero, sizeof(zero), 1, f);
  }
  void serialize_empty_samples   (FILE* f, int N) override {
    Sample zero{};
    for (int i = 0; i < N; ++i) fwrite(&zero, sizeof(zero), 1, f);
  }
  void serialize_empty_reservoirs(FILE* f, int N, std::string) override {
    Reservoir zero{};
    for (int i = 0; i < N; ++i) fwrite(&zero, sizeof(zero), 1, f);
  }
  void serialize_empty_image_buffer(FILE* f, int W, int H) override {
    PixelData zero{};
    size_t N = size_t(W) * H;
    for (size_t i = 0; i < N; ++i) fwrite(&zero, sizeof(zero), 1, f);
  }

  // name ignored, just write the value
  void serialize_const(FILE* f, const char* name, int value) {
    fwrite(&value, sizeof(value), 1, f);
  }
  void serialize_const(FILE* f, const char* name, bool value) {
    int v = value ? 1 : 0;
    fwrite(&v, sizeof(v), 1, f);
  }

};