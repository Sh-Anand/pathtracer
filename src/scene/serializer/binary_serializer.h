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

  void serialize_headers(FILE* f) override {
    // No headers in binary format
    // This is a no-op
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

  // name ignored, just write the value
  void serialize_const(FILE* f, const char* name, int value) {
    fwrite(&value, sizeof(value), 1, f);
  }
  void serialize_const(FILE* f, const char* name, bool value) {
    int v = value ? 1 : 0;
    fwrite(&v, sizeof(v), 1, f);
  }

protected:
    void serializeEmptyImpl(FILE*          f,
                            const char*    /*var_name – ignored*/,
                            size_t         count,
                            size_t         elem_size,
                            std::string    /*type_name – ignored*/,
                            const std::type_info& /*ti – ignored*/) override
    {
        uint32_t n32 = static_cast<uint32_t>(count);
        fwrite(&n32, sizeof(n32), 1, f);

        constexpr size_t CHUNK = 4096;
        unsigned char zeros[CHUNK] = {0};

        size_t bytes = count * elem_size;
        while (bytes) {
            size_t k = std::min(bytes, CHUNK);
            fwrite(zeros, 1, k, f);
            bytes -= k;
        }
    }

};