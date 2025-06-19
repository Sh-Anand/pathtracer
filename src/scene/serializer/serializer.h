#pragma once

#include <cstdio>
#include <cstdarg>
#include <string>
#include <vector>
#include "scene/bvh.h"
#include "scene/camera.h"
#include "scene/light.h"
#include "scene/material.h"
#include "util/reservoir.h"

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

class Serializer {
public:
    virtual void serialize(FILE* f, const std::vector<Vector2D>& vectors, std::string name) = 0;
    virtual void serialize(FILE* f, const std::vector<Vector3D>& vectors, std::string name) = 0;
    virtual void serialize(FILE* f, const std::vector<Vector4D>& vectors, std::string name) = 0;
    virtual void serialize(FILE* f, const Camera& camera) = 0;

    virtual void serialize_lights(FILE* f, const std::vector<CudaLight>& lights) = 0;
    virtual void serialize_bsdfs(FILE* f, const std::vector<CudaBSDF>& bsdfs) = 0;
    virtual void serialize_textures(FILE* f, const std::vector<CudaTexture>& textures) = 0;
    virtual void serialize_primitives(FILE* f, const std::vector<CudaPrimitive>& primitives) = 0;
    virtual void serialize_bvh_nodes(FILE* f, const std::vector<BVHNode>& nodes) = 0;
    virtual void serialize_empty_gpurng(FILE* f, int num_samples) = 0;
    virtual void serialize_empty_samples(FILE* f, int num_samples) = 0;
    virtual void serialize_empty_reservoirs(FILE* f, int num_samples, std::string name) = 0;
    virtual void serialize_empty_image_buffer(FILE* f, int W, int H) = 0;
    virtual void serialize_const(FILE* f, const char* name, int value) = 0;
    virtual void serialize_const(FILE* f, const char* name, bool value) = 0;
};
