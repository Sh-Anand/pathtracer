#include "scene/light.h"
#include "scene/material.h"

__device__ const CudaLight _lights[1] = {
    { { 0.0f, 10.0f, 0.0f }, { 1, 1, 1 }, 100.0f }
};
__device__ const CudaLight *lights = _lights;

__device__ int num_lights = 1;
__device__ CudaBSDF _bsdfs[1] = {
    { { 1.0f, 1.0f, 1.0f, 1.0f }, 0.0f, 0.5f, { 0.0f, 0.0f, 0.0f }, -1, -1, -1, -1 }
};
__device__ CudaBSDF *bsdfs = _bsdfs;
#define NUM_BSDFS 1

__device__ uint8_t dummy_texture_data[4] = { 255, 255, 255, 255 }; // Dummy texture data (white)
__device__ CudaTexture _textures[1] = {
    { 1, 1, 4, dummy_texture_data }
};
__device__ CudaTexture *textures = _textures;

__device__ CudaPrimitive _primitives[1] = {
    { 0, 0, 0, 0 } // Dummy primitive (not used)
};
__device__ CudaPrimitive *primitives = _primitives;
#define NUM_PRIMITIVES 1

__device__ CudaPrimitive _primitives[1] = {
    { 0, 0, 0, 0 } // Dummy primitive (not used)
};
__device__ CudaPrimitive *primitives = _primitives;

__device__ Vector3D _normals[1] = {
    { 0.0f, 1.0f, 0.0f },
};
__device__ Vector3D *normals = _normals;

__device__ Vector2D _texcoords[1] = {
    { 0.0f, 0.0f },
};
__device__ Vector2D *texcoords = _texcoords;

__device__ Vector4D _tangents[1] = {
    { 1.0f, 0.0f, 0.0f, 1.0f } // Dummy tangent
};
__device__ Vector4D *tangents = _tangents;

#define NUM_TEXTURES 1