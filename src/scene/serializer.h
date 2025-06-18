/* ---------------------------------------------------------------------
 *  Helpers
 * ------------------------------------------------------------------ */
#include <cstdio>
#include <vector>
#include "scene/light.h"
#include "scene/material.h"
#include "util/reservoir.h"

static inline void fprint_bool(FILE* f, bool b)   { fputs(b ? "true" : "false", f); }

/* ---------------------------------------------------------------------
 *  Single-element serializers
 * ------------------------------------------------------------------ */
static void serialize(FILE* f, const CudaLight& L)
{
    std::fprintf(f,
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

static void serialize(FILE* f, const CudaBSDF& B)
{
    std::fprintf(f,
        "{{%.8g,%.8g,%.8g,%.8g},"
        "%.8g,%.8g,"
        "{%.8g,%.8g,%.8g},"
        "%d,%d,%d,%d},",
        B.baseColor.x, B.baseColor.y, B.baseColor.z, B.baseColor.w,
        B.metallic, B.roughness,
        B.emission.x, B.emission.y, B.emission.z,
        B.tex_idx, B.normal_idx, B.orm_idx, B.emission_idx);
}

static void serialize_texture_data(FILE* f, const CudaTexture& T, int idx)
{
    size_t size = static_cast<size_t>(T.width) * T.height * T.channels;

    std::fprintf(f, "__device__ uint8_t tex_data_%d[] = {", idx);
    for (size_t i = 0; i < size; ++i) {
        std::fprintf(f, "%u,", T.data[i]);
    }
    std::fputs("};\n", f);
}

static void serialize(FILE* f, const CudaTexture& T, int idx)
{
    std::fprintf(f,
        "    { %hu, %hu, %u, tex_data_%d },\n",
        T.width, T.height, (unsigned)T.channels, idx);
}

static void serialize(FILE* f, const std::vector<CudaPrimitive>& primitives)
{   
    std::fprintf(f, "__device__ CudaPrimitive _primitives[%zu] = {\n", primitives.size());
    for (const auto& P : primitives) {
        std::fprintf(f,
            "{%u,%u,%u,%d},",
            P.i0, P.i1, P.i2, P.bsdf_idx);
    }
    std::fputs("};\n", f);
    std::fprintf(f, "__device__ CudaPrimitive* primitives = _primitives;\n");
    std::fprintf(f, "size_t num_primitives = %zu;\n", primitives.size());
}

static void serialize(FILE* f, const std::vector<Vector2D>& vectors, std::string name) {
    std::fprintf(f, "__device__ Vector2D _%s[%zu] = {\n", name.c_str(), vectors.size());
    for (const auto& v : vectors) {
        std::fprintf(f,
            "{%.8g,%.8g},",
            v.x, v.y);
    }
    std::fputs("};\n", f);
    std::fprintf(f, "__device__ Vector2D* %s = _%s;\n", name.c_str(), name.c_str());
    std::fprintf(f, "size_t num_%s = %zu;\n", name.c_str(), vectors.size());
}

static void serialize(FILE* f, const std::vector<Vector3D>& vectors, std::string name) {
    std::fprintf(f, "__device__ Vector3D _%s[%zu] = {\n", name.c_str(), vectors.size());
    for (const auto& v : vectors) {
        std::fprintf(f,
            "{%.8g,%.8g,%.8g},",
            v.x, v.y, v.z);
    }
    std::fputs("};\n", f);
    std::fprintf(f, "__device__ Vector3D* %s = _%s;\n", name.c_str(), name.c_str());
    std::fprintf(f, "size_t num_%s = %zu;\n", name.c_str(), vectors.size());
}

static void serialize(FILE* f, const std::vector<Vector4D>& vectors, std::string name) {
    std::fprintf(f, "__device__ Vector4D _%s[%zu] = {\n", name.c_str(), vectors.size());
    for (const auto& v : vectors) {
        std::fprintf(f,
            "{%.8g,%.8g,%.8g,%.8g},",
            v.x, v.y, v.z, v.w);
    }
    std::fputs("};\n", f);
    std::fprintf(f, "__device__ Vector4D* %s = _%s;\n", name.c_str(), name.c_str());
    std::fprintf(f, "size_t num_%s = %zu;\n", name.c_str(), vectors.size());
}

static void serialize(FILE *f, const Camera &camera) {
    std::fprintf(f, "#include \"scene/camera.h\"\n\n");
    
    std::fprintf(f,
        "__device__ CudaCamera camera = {%.8g, %.8g, %.8g, %.8g, "
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

static void serialize(FILE* f, const BVHNode &node) {
    std::fprintf(f,
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

static void serialize(FILE *f, const Sample &sample) {
    std::fprintf(f,
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

static void serialize(FILE* f, const Reservoir &reservoir) {
    std::fprintf(f,
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

static void serialize(FILE* f, const PixelData& pixel) {
    std::fprintf(f,
        "{{%.8g,%.8g,%.8g},"
        "{%.8g,%.8g,%.8g},"
        "%.8g},",
        pixel.data.x, pixel.data.y, pixel.data.z,
        pixel.normal.x, pixel.normal.y, pixel.normal.z,
        pixel.depth);
}

static void serialize_empty_image_buffer(FILE* f, int W, int H) {
    size_t N = W * H;
    std::fprintf(f, "__device__ int w = %d;\n", W);
    std::fprintf(f, "__device__ int h = %d;\n", H);
    std::fprintf(f, "__device__ PixelData _pixel[%zu] = {", N);
    for (size_t i = 0; i < N; ++i) {
        std::fprintf(f, "{{0,0,0},{0,0,0},0},");
    }
    std::fputs("};\n", f);
    std::fprintf(f, "__device__ PixelData* pixel = _pixel;\n");
}

/* ---------------------------------------------------------------------
 *  Array serializers
 * ------------------------------------------------------------------ */
static void serialize_lights(FILE* f, const std::vector<CudaLight>& lights)
{
    std::fprintf(f, "#include \"scene/light.h\"\n\n");
    std::fprintf(f, "__device__ CudaLight _lights[%zu] = {\n", lights.size());

    for (const auto& L : lights) serialize(f, L);

    std::fputs("};\n", f);
    std::fprintf(f, "__device__ CudaLight* lights = _lights;\n");
    std::fprintf(f, "__device__ int num_lights = %zu;\n\n", lights.size());
}

static void serialize_bsdfs(FILE* f, const std::vector<CudaBSDF>& bsdfs)
{
    std::fprintf(f, "#include \"scene/material.h\"\n\n");
    std::fprintf(f, "__device__ CudaBSDF _bsdfs[%zu] = {\n", bsdfs.size());

    for (const auto& B : bsdfs) serialize(f, B);

    std::fputs("};\n", f);
    std::fprintf(f, "__device__ CudaBSDF* bsdfs = _bsdfs;\n");
    std::fprintf(f, "size_t num_bsdfs = %zu;\n\n", bsdfs.size());
}

static void serialize_textures(FILE* f, const std::vector<CudaTexture>& textures)
{
    /* raw byte blobs AFTER the metadata table */
    for (size_t i = 0; i < textures.size(); ++i)
        serialize_texture_data(f, textures[i], static_cast<int>(i));

    std::fprintf(f, "__device__ CudaTexture _textures[%zu] = {", textures.size());

    for (size_t i = 0; i < textures.size(); ++i)
        serialize(f, textures[i], static_cast<int>(i));

    std::fputs("};\n", f);
    std::fprintf(f, "__device__ CudaTexture* textures = _textures;\n");
    std::fprintf(f, "size_t num_textures = %zu;\n\n", textures.size());
}

static void serialize_bvh_nodes(FILE* f, const std::vector<BVHNode>& nodes)
{
    std::fprintf(f, "#include \"scene/bvh.h\"\n\n");
    std::fprintf(f, "__device__ BVHNode _nodes[%zu] = {\n", nodes.size());

    for (const auto& node : nodes) serialize(f, node);

    std::fputs("};\n", f);
    std::fprintf(f, "__device__ const BVHNode* nodes = _nodes;\n");
    std::fprintf(f, "size_t num_bvh_nodes = %zu;\n\n", nodes.size());
}

static void serialize_empty_gpurng(FILE* f, int num_samples) {
    std::fprintf(f, "#include \"util/reservoir.h\"\n\n");
    std::fprintf(f,"__device__ RNGState _rand_states[%d]={",num_samples);
    for(int i=0;i<num_samples;++i)std::fprintf(f,"{0},");
    std::fprintf(f,"};\n__device__ RNGState* rand_states=_rand_states;\n\n");
}

static void serialize_empty_samples(FILE* f, int num_samples) {
    std::fprintf(f, "__device__ Sample _samples[%d] = {", num_samples);
    Sample sample = {};
    for (int i = 0; i < num_samples; ++i) serialize(f, sample);

    std::fputs("};\n", f);
    std::fprintf(f, "__device__ Sample* initialSampleBuffer = _samples;\n");
    std::fprintf(f, "size_t num_samples = %d;\n\n", num_samples);
}

static void serialize_empty_reservoirs(FILE* f, int num_samples, std::string name) {
    std::fprintf(f, "__device__ Reservoir _%s[%d] = {", name.c_str(), num_samples);
    Reservoir reservoir = {};
    for (int i = 0; i < num_samples; ++i) serialize(f, reservoir);

    std::fputs("};\n", f);
    std::fprintf(f, "__device__ Reservoir* %s = _%s;\n", name.c_str(), name.c_str());
    std::fprintf(f, "size_t num_%s = %d;\n\n", name.c_str(), num_samples);
}
