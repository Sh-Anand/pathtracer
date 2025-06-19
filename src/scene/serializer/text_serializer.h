#include "serializer.h"

class TextSerializer : public Serializer {
    /* ---------------------------------------------------------------------
    *  Single-element serializers
    * ------------------------------------------------------------------ */
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

    void serialize(FILE* f, const std::vector<Vector2D>& vectors, std::string name) {
        safe_fprintf(f, "extern __device__ const Vector2D %s[%zu] = {\n", name.c_str(), vectors.size());
        for (const auto& v : vectors) {
            safe_fprintf(f,
                "{%.8g,%.8g},",
                v.x, v.y);
        }
        safe_fprintf(f, "};\n");
        safe_fprintf(f, "size_t num_%s = %zu;\n", name.c_str(), vectors.size());
    }

    void serialize(FILE* f, const std::vector<Vector3D>& vectors, std::string name) {
        safe_fprintf(f, "extern __device__ const Vector3D %s[%zu] = {\n", name.c_str(), vectors.size());
        for (const auto& v : vectors) {
            safe_fprintf(f,
                "{%.8g,%.8g,%.8g},",
                v.x, v.y, v.z);
        }
        safe_fprintf(f, "};\n");
        safe_fprintf(f, "size_t num_%s = %zu;\n", name.c_str(), vectors.size());
    }

    void serialize(FILE* f, const std::vector<Vector4D>& vectors, std::string name) {
        safe_fprintf(f, "extern __device__ const Vector4D %s[%zu] = {\n", name.c_str(), vectors.size());
        for (const auto& v : vectors) {
            safe_fprintf(f,
                "{%.8g,%.8g,%.8g,%.8g},",
                v.x, v.y, v.z, v.w);
        }
        safe_fprintf(f, "};\n");
        safe_fprintf(f, "const size_t num_%s = %zu;\n", name.c_str(), vectors.size());
    }

    void serialize(FILE *f, const Camera &camera) {
        safe_fprintf(f, "#include \"scene/camera.h\"\n\n");
        
        safe_fprintf(f,
            "extern __device__ const CudaCamera camera = {%.8g, %.8g, %.8g, %.8g, "
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
            "{%.8g,%.8g,%.8g}," // emittance
            "{%.8g,%.8g,%.8g}," // x_v
            "{%.8g,%.8g,%.8g}," // n_v
            "{%.8g,%.8g,%.8g}," // x_s
            "{%.8g,%.8g,%.8g}," // n_s
            "%.8g,"               // z_v
            "%.8g,"               // pdf
            "{%.8g,%.8g,%.8g}"  // bsdf_f
            "},",
            sample.emittance.x, sample.emittance.y, sample.emittance.z,
            sample.x_v.x, sample.x_v.y, sample.x_v.z,
            sample.n_v.x, sample.n_v.y, sample.n_v.z,
            sample.x_s.x, sample.x_s.y, sample.x_s.z,
            sample.n_s.x, sample.n_s.y, sample.n_s.z,
            sample.z_v,
            sample.pdf,
            sample.bsdf_f.x, sample.bsdf_f.y, sample.bsdf_f.z);
    }

    void serialize(FILE* f, const Reservoir &reservoir) {
        safe_fprintf(f,
            "{ {{%.8g,%.8g,%.8g},"
            "{%.8g,%.8g,%.8g},"
            "%.8g,"
            "{%.8g,%.8g,%.8g},"
            "{%.8g,%.8g,%.8g},"
            "{%.8g,%.8g,%.8g},"
            "{%.8g,%.8g,%.8g}},"
            "%.8g,%.8g,%.8g },",
            reservoir.z.x_v.x, reservoir.z.x_v.y, reservoir.z.x_v.z,
            reservoir.z.n_v.x, reservoir.z.n_v.y, reservoir.z.n_v.z,
            reservoir.z.z_v,
            reservoir.z.x_s.x, reservoir.z.x_s.y, reservoir.z.x_s.z,
            reservoir.z.n_s.x, reservoir.z.n_s.y, reservoir.z.n_s.z,
            reservoir.z.L.x, reservoir.z.L.y, reservoir.z.L.z,
            reservoir.z.bsdf_f.x, reservoir.z.bsdf_f.y, reservoir.z.bsdf_f.z,
            reservoir.w, reservoir.M, reservoir.W);
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

    /* ---------------------------------------------------------------------
    *  Array serializers
    * ------------------------------------------------------------------ */
    void serialize_lights(FILE* f, const std::vector<CudaLight>& lights)
    {
        safe_fprintf(f, "#include \"scene/light.h\"\n\n");
        safe_fprintf(f, "extern __device__ const CudaLight lights[%zu] = {\n", lights.size());

        for (const auto& L : lights) serialize(f, L);

        safe_fprintf(f, "};\n");
        safe_fprintf(f, "extern __device__ const int num_lights = %zu;\n\n", lights.size());
    }

    void serialize_bsdfs(FILE* f, const std::vector<CudaBSDF>& bsdfs)
    {
        safe_fprintf(f, "#include \"scene/material.h\"\n\n");
        safe_fprintf(f, "extern __device__ const CudaBSDF bsdfs[%zu] = {\n", bsdfs.size());

        for (const auto& B : bsdfs) serialize(f, B);

        safe_fprintf(f, "};\n");
        safe_fprintf(f, "const size_t num_bsdfs = %zu;\n\n", bsdfs.size());
    }

    void serialize_texture_data(FILE* f, const CudaTexture& T, int idx)
    {
        size_t size = static_cast<size_t>(T.width) * T.height * T.channels;

        safe_fprintf(f, "__device__ const uint8_t tex_data_%d[] = {", idx);
        for (size_t i = 0; i < size; ++i) {
            safe_fprintf(f, "%u,", T.data[i]);
        }
        std::fputs("};\n", f);
    }


    void serialize_textures(FILE* f, const std::vector<CudaTexture>& textures)
    {
        /* raw byte blobs AFTER the metadata table */
        for (size_t i = 0; i < textures.size(); ++i)
            serialize_texture_data(f, textures[i], static_cast<int>(i));

        safe_fprintf(f, "extern __device__ const CudaTexture textures[%zu] = {", textures.size());

        for (size_t i = 0; i < textures.size(); ++i)
            serialize(f, textures[i], static_cast<int>(i));

        safe_fprintf(f, "};\n");
        safe_fprintf(f, "const size_t num_textures = %zu;\n\n", textures.size());
    }

    void serialize_primitives(FILE* f, const std::vector<CudaPrimitive>& primitives)
    {   
        safe_fprintf(f, "extern __device__ const CudaPrimitive primitives[%zu] = {\n", primitives.size());
        for (const auto& P : primitives) 
        {
            serialize(f, P);
        }
        std::fputs("};\n", f);
        safe_fprintf(f, "const size_t num_primitives = %zu;\n", primitives.size());
    }

    void serialize_bvh_nodes(FILE* f, const std::vector<BVHNode>& nodes)
    {
        safe_fprintf(f, "#include \"scene/bvh.h\"\n\n");
        safe_fprintf(f, "extern __device__ const BVHNode nodes[%zu] = {\n", nodes.size());

        for (const auto& node : nodes) serialize(f, node);

        safe_fprintf(f, "};\n");
        safe_fprintf(f, "const size_t num_bvh_nodes = %zu;\n\n", nodes.size());
    }

    void serialize_empty_gpurng(FILE* f, int num_samples) {
        safe_fprintf(f, "#include \"util/reservoir.h\"\n\n");
        safe_fprintf(f,"extern __device__ RNGState rand_states[%d]={",num_samples);
        for(int i=0;i<num_samples;++i)safe_fprintf(f,"{0},");
        safe_fprintf(f,"};\n");
    }

    void serialize_empty_samples(FILE* f, int num_samples) {
        safe_fprintf(f, "extern __device__ Sample initialSampleBuffer[%d] = {", num_samples);
        Sample sample = {};
        for (int i = 0; i < num_samples; ++i) serialize(f, sample);

        safe_fprintf(f, "};\n");
        safe_fprintf(f, "size_t num_samples = %d;\n\n", num_samples);
    }

    void serialize_empty_reservoirs(FILE* f, int num_samples, std::string name) {
        safe_fprintf(f, "extern __device__ Reservoir %s[%d] = {", name.c_str(), num_samples);
        Reservoir reservoir = {};
        for (int i = 0; i < num_samples; ++i) serialize(f, reservoir);

        safe_fprintf(f, "};\n");
        safe_fprintf(f, "size_t num_%s = %d;\n\n", name.c_str(), num_samples);
    }

    void serialize_empty_image_buffer(FILE* f, int W, int H) {
        size_t N = W * H;
        safe_fprintf(f, "extern __device__ const int w = %d;\n", W);
        safe_fprintf(f, "extern __device__ const int h = %d;\n", H);
        safe_fprintf(f, "extern __device__ PixelData pixel[%zu] = {", N);
        for (size_t i = 0; i < N; ++i) {
            safe_fprintf(f, "{{0,0,0},{0,0,0},0},");
        }
        safe_fprintf(f, "};\n");
    }

};