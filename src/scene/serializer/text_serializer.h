#include "serializer.h"

static inline void fprint_bool(FILE* f, bool b)   { fputs(b ? "true" : "false", f); }

static void safe_fprintf(FILE* f, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    if (vfprintf(f, fmt, ap) < 0) {
        perror("serializer: write failed");
        exit(EXIT_FAILURE);
    }
    va_end(ap);
}

class TextSerializer : public Serializer {

    void variable_declaration(FILE* f, const char* type, const char* name, size_t count, bool constant = true) {
        std::string decl_str = "extern " + (CUDA ? std::string("__device__ ") : std::string(""));
        decl_str += constant ? "const " : "";
        decl_str += type + std::string(" ") + name;
        decl_str += count == 0 ? "= " : "[" + std::to_string(count) + "] = {";
        safe_fprintf(f, "%s", decl_str.c_str());
    }

    void serialize(FILE* f, const CudaLight& L)
    {
        safe_fprintf(f,
            "{{%.8g,%.8g,%.8g},"
            "{%u,%u,%u},"
            "%.8g,"
            "{%.8g,%.8g,%.8g},"
            "%.8g,",
            L.radiance.x, L.radiance.y, L.radiance.z,
            L.triangle.i0, L.triangle.i1, L.triangle.i2,
            L.area,
            L.position.x, L.position.y, L.position.z,
            L.radius);

        fprint_bool(f, L.is_point_light);
        std::fputs(" },\n", f);
    }

    void serialize(FILE* f, const CudaBSDF& B)
    {
    
        safe_fprintf(f,
            "{{%.8g,%.8g,%.8g,%.8g},"
            "%.8g,%.8g,"
            "{%.8g,%.8g,%.8g},"
            "%d,%d,%d,%d},",
            B.baseColor.x, B.baseColor.y, B.baseColor.z, B.baseColor.w,
            B.metallic, B.roughness,
            B.emission.x, B.emission.y, B.emission.z,
            B.tex_idx, B.normal_idx, B.orm_idx, B.emission_idx);
    }

    void serialize(FILE* f, const CudaTexture& T, int idx)
    {
        safe_fprintf(f,
            "{%hu,%hu,%u,tex_data_%d},",
            T.width, T.height, (unsigned)T.channels, idx);
    }

    void serialize(FILE* f, const CudaPrimitive & P)
    {
        safe_fprintf(f,
            "{%u,%u,%u,%d},",
            P.i0, P.i1, P.i2, P.bsdf_idx);
    }

    void serialize(FILE* f, const std::vector<Vector2D>& vectors, std::string name) override {
        variable_declaration(f, "Vector2D", name.c_str(), vectors.size());
        for (const auto& v : vectors) {
            safe_fprintf(f,
                "{%.8g,%.8g},",
                v.x, v.y);
        }
        safe_fprintf(f, "};\n");
        safe_fprintf(f, "const uint32_t num_%s = %zu;\n", name.c_str(), vectors.size());
    }

    void serialize(FILE* f, const std::vector<Vector3D>& vectors, std::string name) override {
        variable_declaration(f, "Vector3D", name.c_str(), vectors.size());
        for (const auto& v : vectors) {
            safe_fprintf(f,
                "{%.8g,%.8g,%.8g},",
                v.x, v.y, v.z);
        }
        safe_fprintf(f, "};\n");
        safe_fprintf(f, "const uint32_t num_%s = %zu;\n", name.c_str(), vectors.size());
    }

    void serialize(FILE* f, const std::vector<Vector4D>& vectors, std::string name) override {
        variable_declaration(f, "Vector4D", name.c_str(), vectors.size());
        for (const auto& v : vectors) {
            safe_fprintf(f,
                "{%.8g,%.8g,%.8g,%.8g},",
                v.x, v.y, v.z, v.w);
        }
        safe_fprintf(f, "};\n");
        safe_fprintf(f, "const uint32_t num_%s = %zu;\n", name.c_str(), vectors.size());
    }

    void serialize(FILE *f, const Camera &camera) override {
        variable_declaration(f, "CudaCamera", "camera", 0);
        safe_fprintf(f,
            "{%.8g, %.8g, %.8g, %.8g, "
            "{%.8g, %.8g, %.8g}, "
            "{{{%.8g, %.8g, %.8g}, "
            "{%.8g, %.8g, %.8g}, "
            "{%.8g, %.8g, %.8g}}}};\n",
            camera.hFov, camera.vFov, camera.nClip, camera.fClip,
            camera.pos.x, camera.pos.y, camera.pos.z,
            camera.c2w.c[0].x, camera.c2w.c[0].y, camera.c2w.c[0].z,
            camera.c2w.c[1].x, camera.c2w.c[1].y, camera.c2w.c[1].z,
            camera.c2w.c[2].x, camera.c2w.c[2].y, camera.c2w.c[2].z);
    }

    void serialize(FILE* f, const BVHNode &node) {
        safe_fprintf(f,
            "{"
            "{%.8g,%.8g,%.8g}," // bbmin
            "{%.8g,%.8g,%.8g}," // bbmax
            "%u,"                      // start
            "%u,"                      // end
            "%u,"                      // l
            "%u"                       // r
            "},",
            node.bbmin.x, node.bbmin.y, node.bbmin.z,
            node.bbmax.x, node.bbmax.y, node.bbmax.z,
            node.start, node.end, node.l, node.r);
    }

    void serialize(FILE *f, const Sample &sample) {
        safe_fprintf(f,
            "{"
            "{%.8g,%.8g,%.8g}," // x_v
            "{%.8g,%.8g,%.8g}," // n_v
            "{%.8g,%.8g,%.8g}," // x_s
            "{%.8g,%.8g,%.8g}," // n_s
            "%.8g,"               // z_v
            "},",
            sample.x_v.x, sample.x_v.y, sample.x_v.z,
            sample.n_v.x, sample.n_v.y, sample.n_v.z,
            sample.x_s.x, sample.x_s.y, sample.x_s.z,
            sample.n_s.x, sample.n_s.y, sample.n_s.z,
            sample.z_v);
    }

    void serialize(FILE *f, const SampleMetadata &sample_metadata) {
        safe_fprintf(f,
            "{"
            "{%.8g,%.8g,%.8g}," // bsdf_f
            "{%.8g,%.8g,%.8g}," // emittance
            "},", 
            sample_metadata.bsdf_f.x, sample_metadata.bsdf_f.y, sample_metadata.bsdf_f.z,
            sample_metadata.emittance.x, sample_metadata.emittance.y, sample_metadata.emittance.z);
    }

    void serialize(FILE* f, const Reservoir &reservoir) {
        safe_fprintf(f,
            "{%.8g,%.8g,%.8g,%.8g},",
            reservoir.z, reservoir.w, reservoir.M, reservoir.W);
    }

    void serialize(FILE* f, const PixelData& pixel) {
        safe_fprintf(f,
            "{{%.8g,%.8g,%.8g},"
            "{%.8g,%.8g,%.8g},"
            "%.8g},",
            pixel.data.x, pixel.data.y, pixel.data.z,
            pixel.normal.x, pixel.normal.y, pixel.normal.z,
            pixel.depth);
    }

    void serialize_headers(FILE* f) override
    {
        safe_fprintf(f, "#include \"scene/bvh.h\"\n");
        safe_fprintf(f, "#include \"scene/camera.h\"\n");
        safe_fprintf(f, "#include \"scene/light.h\"\n");
        safe_fprintf(f, "#include \"scene/material.h\"\n");
        safe_fprintf(f, "#include \"util/reservoir.h\"\n");
        safe_fprintf(f, "#include \"util/vector.h\"\n\n");
    }

    void serialize_lights(FILE* f, const std::vector<CudaLight>& lights) override
    {
        variable_declaration(f, "CudaLight", "lights", lights.size());

        for (const auto& L : lights) serialize(f, L);

        safe_fprintf(f, "};\n");
        variable_declaration(f, "int", "num_lights", 0);
        safe_fprintf(f, "%zu;\n\n", lights.size());
    }

    void serialize_bsdfs(FILE* f, const std::vector<CudaBSDF>& bsdfs) override
    {
        variable_declaration(f, "CudaBSDF", "bsdfs", bsdfs.size());

        for (const auto& B : bsdfs) serialize(f, B);

        safe_fprintf(f, "};\n");
        safe_fprintf(f, "const uint32_t num_bsdfs = %zu;\n\n", bsdfs.size());
    }

    void serialize_texture_data(FILE* f, const CudaTexture& T, int idx)
    {
        size_t size = static_cast<size_t>(T.width) * T.height * T.channels;

        char name_buf[32];
        snprintf(name_buf, sizeof(name_buf), "tex_data_%d", idx);
        variable_declaration(f, "uint8_t", name_buf, size);

        for (size_t i = 0; i < size; ++i) {
            safe_fprintf(f, "%u,", T.data[i]);
        }
        std::fputs("};\n", f);
    }

    void serialize_textures(FILE* f, const std::vector<CudaTexture>& textures) override
    {
        /* raw byte blobs AFTER the metadata table */
        for (size_t i = 0; i < textures.size(); ++i)
            serialize_texture_data(f, textures[i], static_cast<int>(i));

        variable_declaration(f, "CudaTexture", "textures", textures.size());

        for (size_t i = 0; i < textures.size(); ++i)
            serialize(f, textures[i], static_cast<int>(i));

        safe_fprintf(f, "};\n");
        safe_fprintf(f, "const uint32_t num_textures = %zu;\n\n", textures.size());
    }

    void serialize_primitives(FILE* f, const std::vector<CudaPrimitive>& primitives) override
    {   
        variable_declaration(f, "CudaPrimitive", "primitives", primitives.size());
        for (const auto& P : primitives) 
        {
            serialize(f, P);
        }
        std::fputs("};\n", f);
        safe_fprintf(f, "const uint32_t num_primitives = %zu;\n", primitives.size());
    }

    void serialize_bvh_nodes(FILE* f, const std::vector<BVHNode>& nodes) override
    {
        variable_declaration(f, "BVHNode", "nodes", nodes.size());

        for (const auto& node : nodes) serialize(f, node);

        safe_fprintf(f, "};\n");
        safe_fprintf(f, "const uint32_t num_bvh_nodes = %zu;\n\n", nodes.size());
    }

    void serialize_const(FILE* f, const char* name, int value) override
    {
        variable_declaration(f, "int", name, 0);
        safe_fprintf(f, "%d;\n", value);
    }

    void serialize_const(FILE* f, const char* name, bool value) override
    {
        variable_declaration(f, "bool", name, 0);
        fprint_bool(f, value);
        std::fputs(";\n", f);
    }

protected:
  void serializeEmptyImpl(FILE*           f,
                          const char*     var_name,
                          size_t          count,
                          size_t          /*elem_size*/,
                          std::string    type_name,
                          const std::type_info& ti) override
  {
    variable_declaration(f, type_name.c_str(), var_name, count, false);

    // close array and the size symbol
    safe_fprintf(f,
      "};\n"
      "const uint32_t num_%s = %zu;\n",
      var_name, count);
  }

};