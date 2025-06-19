#pragma once

#include <cstdio>
#include <cstdarg>
#include <string>
#include <typeinfo>
#include <vector>

#include "scene/bvh.h"
#include "scene/camera.h"
#include "scene/light.h"
#include "scene/material.h"
#include "util/reservoir.h"

class Serializer {
public:
    virtual void serialize(FILE* f, const std::vector<Vector2D>& vectors, std::string name) = 0;
    virtual void serialize(FILE* f, const std::vector<Vector3D>& vectors, std::string name) = 0;
    virtual void serialize(FILE* f, const std::vector<Vector4D>& vectors, std::string name) = 0;
    virtual void serialize(FILE* f, const Camera& camera) = 0;

    virtual void serialize_headers(FILE* f) = 0;
    virtual void serialize_lights(FILE* f, const std::vector<CudaLight>& lights) = 0;
    virtual void serialize_bsdfs(FILE* f, const std::vector<CudaBSDF>& bsdfs) = 0;
    virtual void serialize_textures(FILE* f, const std::vector<CudaTexture>& textures) = 0;
    virtual void serialize_primitives(FILE* f, const std::vector<CudaPrimitive>& primitives) = 0;
    virtual void serialize_bvh_nodes(FILE* f, const std::vector<BVHNode>& nodes) = 0;
    virtual void serialize_const(FILE* f, const char* name, int value) = 0;
    virtual void serialize_const(FILE* f, const char* name, bool value) = 0;

    template<class T>
    void serializeEmpty(FILE* f,
                        const char* var_name,
                        const std::string& type_name,
                        size_t      count)
    {
        serializeEmptyImpl(f, var_name, count,
                           sizeof(T),           // element size for binary
                           type_name,           // type name for text
                           typeid(T));          // run-time tag for text
    }
    protected:
        virtual void serializeEmptyImpl(FILE* f,
                                        const char* var_name,
                                        size_t      count,
                                        size_t      elem_size,
                                        std::string type_name,
                                        const std::type_info& ti) = 0;
};
