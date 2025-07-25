// vector.h
#ifndef VECTOR_H
#define VECTOR_H

#include "cuda_defs.h"

#ifdef __CUDACC__
  #define FORCE_INLINE __host__ __device__ __forceinline__
#else
  #define FORCE_INLINE inline __attribute__((always_inline))
#endif

struct Vector2D { float x,y; };
struct Vector3D { float x,y,z; };
struct Vector4D { float x,y,z,w; };

FORCE_INLINE Vector3D vector3d_add  (Vector3D a, Vector3D b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
FORCE_INLINE Vector3D vector3d_sub  (Vector3D a, Vector3D b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
FORCE_INLINE Vector3D vector3d_mul  (Vector3D a, Vector3D b) { return {a.x*b.x, a.y*b.y, a.z*b.z}; }
FORCE_INLINE Vector3D vector3d_neg  (Vector3D v)           { return {-v.x,    -v.y,    -v.z   }; }
FORCE_INLINE Vector3D vector3d_scale(Vector3D v, float s)  { return {v.x*s,   v.y*s,   v.z*s  }; }
FORCE_INLINE Vector3D vector3d_rcp  (Vector3D v)           { return {1.f/v.x, 1.f/v.y, 1.f/v.z}; }

FORCE_INLINE float    vector3d_dot  (Vector3D a, Vector3D b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
FORCE_INLINE Vector3D vector3d_cross(Vector3D a, Vector3D b) {
  return {
    a.y*b.z - a.z*b.y,
    a.z*b.x - a.x*b.z,
    a.x*b.y - a.y*b.x
  };
}

FORCE_INLINE float    vector3d_norm2(Vector3D v)          { return (v.x*v.x + v.y*v.y + v.z*v.z); }
FORCE_INLINE float    vector3d_norm (Vector3D v)          { return sqrtf(vector3d_norm2(v)); }


FORCE_INLINE Vector3D vector3d_unit (Vector3D v)          {
  float norm = vector3d_norm(v);
  if (norm == 0.f) return {0.f, 0.f, 0.f}; 
  float inv = 1.f / norm;
  return {v.x*inv, v.y*inv, v.z*inv};
}

FORCE_INLINE Vector3D vector3d_reflect(Vector3D I, Vector3D N) {
  float d = 2.f * vector3d_dot(I,N);
  return { I.x - N.x*d, I.y - N.y*d, I.z - N.z*d };
}

FORCE_INLINE Vector4D vector4d_unit(const Vector4D v) {
  float inv = 1.f/ sqrtf(v.x*v.x + v.y*v.y + v.z*v.z + v.w*v.w);
  return {v.x*inv, v.y*inv, v.z*inv, v.w*inv};
}
FORCE_INLINE Vector3D vector4d_to3d(const Vector4D v) {
  return {v.x, v.y, v.z};
}
FORCE_INLINE float illum(const Vector3D v) {
  return 0.2126f * v.x + 0.7152f * v.y + 0.0722f * v.z;
}

#undef FORCE_INLINE
#endif // VECTOR_H
