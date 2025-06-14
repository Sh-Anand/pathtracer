// matrix.h
#ifndef MATRIX_H
#define MATRIX_H

#include "vector.h"
#include <cmath>

// 3×3 and 4×4 matrix types
typedef struct { Vector3D c[3]; } Matrix3x3;
typedef struct { Vector4D c[4]; } Matrix4x4;

// ————————————————
// Extract upper‑left 3×3 from 4×4
static inline Matrix3x3 matrix3x3_from_matrix4x4(const Matrix4x4 *A) {
  return {{
    { A->c[0].x, A->c[0].y, A->c[0].z },
    { A->c[1].x, A->c[1].y, A->c[1].z },
    { A->c[2].x, A->c[2].y, A->c[2].z }
  }};
}

// Transpose
static inline Matrix3x3 matrix3x3_transpose(const Matrix3x3 *A) {
  return {{
    { A->c[0].x, A->c[1].x, A->c[2].x },
    { A->c[0].y, A->c[1].y, A->c[2].y },
    { A->c[0].z, A->c[1].z, A->c[2].z }
  }};
}

// Inverse (undefined if det=0)
static inline Matrix3x3 matrix3x3_inverse(const Matrix3x3 *A) {
  float a00=A->c[0].x, a01=A->c[1].x, a02=A->c[2].x;
  float a10=A->c[0].y, a11=A->c[1].y, a12=A->c[2].y;
  float a20=A->c[0].z, a21=A->c[1].z, a22=A->c[2].z;
  float det = a00*(a11*a22 - a12*a21)
            - a01*(a10*a22 - a12*a20)
            + a02*(a10*a21 - a11*a20);
  float inv = 1.f/det;
  return {{
    { (a11*a22 - a12*a21)*inv, (-a01*a22 + a02*a21)*inv, ( a01*a12 - a02*a11)*inv },
    { (-a10*a22 + a12*a20)*inv, ( a00*a22 - a02*a20)*inv, (-a00*a12 + a02*a10)*inv },
    { ( a10*a21 - a11*a20)*inv, (-a00*a21 + a01*a20)*inv, ( a00*a11 - a01*a10)*inv }
  }};
}

// Scale
static inline Matrix3x3 matrix3x3_scale(const Matrix3x3 *A, float s) {
  return {{
    { A->c[0].x*s, A->c[0].y*s, A->c[0].z*s },
    { A->c[1].x*s, A->c[1].y*s, A->c[1].z*s },
    { A->c[2].x*s, A->c[2].y*s, A->c[2].z*s }
  }};
}

// Multiply matrix by vector (column)
HOST_DEVICE static inline Vector3D matrix3x3_vector_multiply(const Matrix3x3 *A, const Vector3D *v) {
  return vector3d_add(
    vector3d_scale(A->c[0], v->x),
    vector3d_add(
      vector3d_scale(A->c[1], v->y),
      vector3d_scale(A->c[2], v->z)
    )
  );
}

// 4x4 -> 3x3
static inline Matrix3x3 matrix4x4_to3x3(const Matrix4x4 *A) {
  return {{
    { A->c[0].x, A->c[0].y, A->c[0].z },
    { A->c[1].x, A->c[1].y, A->c[1].z },
    { A->c[2].x, A->c[2].y, A->c[2].z }
  }};
}

// Lift 3×3 → 4×4 homogeneous
static inline Matrix4x4 matrix3x3_to4x4(const Matrix3x3 *A) {
  return {{
    { A->c[0].x, A->c[0].y, A->c[0].z, 0.f },
    { A->c[1].x, A->c[1].y, A->c[1].z, 0.f },
    { A->c[2].x, A->c[2].y, A->c[2].z, 0.f },
    { 0.f,       0.f,       0.f,       1.f }
  }};
}

// 4×4 Identity
static inline Matrix4x4 matrix4x4_identity() {
  return {{
    {1,0,0,0},
    {0,1,0,0},
    {0,0,1,0},
    {0,0,0,1}
  }};
}

static inline Matrix4x4 matrix4x4_scale(const Matrix4x4 *A, float x) {
  return {{
    { A->c[0].x*x, A->c[0].y*x, A->c[0].z*x, A->c[0].w*x },
    { A->c[1].x*x, A->c[1].y*x, A->c[1].z*x, A->c[1].w*x },
    { A->c[2].x*x, A->c[2].y*x, A->c[2].z*x, A->c[2].w*x },
    { A->c[3].x*x, A->c[3].y*x, A->c[3].z*x, A->c[3].w*x }
  }};
}

// 4×4 Determinant (unrolled)
static inline float matrix4x4_determinant(const Matrix4x4 *A) {
  #define M(i,j) ((i)==0?A->c[j].x:(i)==1?A->c[j].y:(i)==2?A->c[j].z:A->c[j].w)
  float d =
    M(0,3)*M(1,2)*M(2,1)*M(3,0) - M(0,2)*M(1,3)*M(2,1)*M(3,0)
  - M(0,3)*M(1,1)*M(2,2)*M(3,0) + M(0,1)*M(1,3)*M(2,2)*M(3,0)
  + M(0,2)*M(1,1)*M(2,3)*M(3,0) - M(0,1)*M(1,2)*M(2,3)*M(3,0)
  - M(0,3)*M(1,2)*M(2,0)*M(3,1) + M(0,2)*M(1,3)*M(2,0)*M(3,1)
  + M(0,3)*M(1,0)*M(2,2)*M(3,1) - M(0,0)*M(1,3)*M(2,2)*M(3,1)
  - M(0,2)*M(1,0)*M(2,3)*M(3,1) + M(0,0)*M(1,2)*M(2,3)*M(3,1)
  + M(0,3)*M(1,1)*M(2,0)*M(3,2) - M(0,1)*M(1,3)*M(2,0)*M(3,2)
  - M(0,3)*M(1,0)*M(2,1)*M(3,2) + M(0,0)*M(1,3)*M(2,1)*M(3,2)
  + M(0,1)*M(1,0)*M(2,3)*M(3,2) - M(0,0)*M(1,1)*M(2,3)*M(3,2)
  - M(0,2)*M(1,1)*M(2,0)*M(3,3) + M(0,1)*M(1,2)*M(2,0)*M(3,3)
  + M(0,2)*M(1,0)*M(2,1)*M(3,3) - M(0,0)*M(1,2)*M(2,1)*M(3,3)
  - M(0,1)*M(1,0)*M(2,2)*M(3,3) + M(0,0)*M(1,1)*M(2,2)*M(3,3);
  #undef M
  return d;
}

// 4×4 Multiply: C = A * B
static inline Matrix4x4 matrix4x4_multiply(const Matrix4x4 *A, const Matrix4x4 *B) {
  Matrix4x4 C;
  // column 0
  C.c[0].x = A->c[0].x*B->c[0].x + A->c[1].x*B->c[0].y + A->c[2].x*B->c[0].z + A->c[3].x*B->c[0].w;
  C.c[0].y = A->c[0].y*B->c[0].x + A->c[1].y*B->c[0].y + A->c[2].y*B->c[0].z + A->c[3].y*B->c[0].w;
  C.c[0].z = A->c[0].z*B->c[0].x + A->c[1].z*B->c[0].y + A->c[2].z*B->c[0].z + A->c[3].z*B->c[0].w;
  C.c[0].w = A->c[0].w*B->c[0].x + A->c[1].w*B->c[0].y + A->c[2].w*B->c[0].z + A->c[3].w*B->c[0].w;
  // column 1
  C.c[1].x = A->c[0].x*B->c[1].x + A->c[1].x*B->c[1].y + A->c[2].x*B->c[1].z + A->c[3].x*B->c[1].w;
  C.c[1].y = A->c[0].y*B->c[1].x + A->c[1].y*B->c[1].y + A->c[2].y*B->c[1].z + A->c[3].y*B->c[1].w;
  C.c[1].z = A->c[0].z*B->c[1].x + A->c[1].z*B->c[1].y + A->c[2].z*B->c[1].z + A->c[3].z*B->c[1].w;
  C.c[1].w = A->c[0].w*B->c[1].x + A->c[1].w*B->c[1].y + A->c[2].w*B->c[1].z + A->c[3].w*B->c[1].w;
  // column 2
  C.c[2].x = A->c[0].x*B->c[2].x + A->c[1].x*B->c[2].y + A->c[2].x*B->c[2].z + A->c[3].x*B->c[2].w;
  C.c[2].y = A->c[0].y*B->c[2].x + A->c[1].y*B->c[2].y + A->c[2].y*B->c[2].z + A->c[3].y*B->c[2].w;
  C.c[2].z = A->c[0].z*B->c[2].x + A->c[1].z*B->c[2].y + A->c[2].z*B->c[2].z + A->c[3].z*B->c[2].w;
  C.c[2].w = A->c[0].w*B->c[2].x + A->c[1].w*B->c[2].y + A->c[2].w*B->c[2].z + A->c[3].w*B->c[2].w;
  // column 3
  C.c[3].x = A->c[0].x*B->c[3].x + A->c[1].x*B->c[3].y + A->c[2].x*B->c[3].z + A->c[3].x*B->c[3].w;
  C.c[3].y = A->c[0].y*B->c[3].x + A->c[1].y*B->c[3].y + A->c[2].y*B->c[3].z + A->c[3].y*B->c[3].w;
  C.c[3].z = A->c[0].z*B->c[3].x + A->c[1].z*B->c[3].y + A->c[2].z*B->c[3].z + A->c[3].z*B->c[3].w;
  C.c[3].w = A->c[0].w*B->c[3].x + A->c[1].w*B->c[3].y + A->c[2].w*B->c[3].z + A->c[3].w*B->c[3].w;
  return C;
}

// 4×4 Matrix × Vector
static inline Vector4D matrix4x4_vector_multiply(const Matrix4x4 *A, const Vector4D *v) {
  return {
    A->c[0].x*v->x + A->c[1].x*v->y + A->c[2].x*v->z + A->c[3].x*v->w,
    A->c[0].y*v->x + A->c[1].y*v->y + A->c[2].y*v->z + A->c[3].y*v->w,
    A->c[0].z*v->x + A->c[1].z*v->y + A->c[2].z*v->z + A->c[3].z*v->w,
    A->c[0].w*v->x + A->c[1].w*v->y + A->c[2].w*v->z + A->c[3].w*v->w
  };
}

class Quaternion : public Vector4D {
 public:

  Quaternion(float x, float y, float z, float w) : Vector4D{x, y, z, w} { }
  /**
   * @brief Computes the rotation matrix represented by a unit
   * quaternion.
   *
   * @note This does not check that this quaternion is normalized.
   * It formulaically returns the matrix, which will not be a
   * rotation if the quaternion is non-unit.
   */
  Matrix3x3 rotationMatrix() const {
	float m[9] = {
	  1-2*y*y-2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w,
	  2*x*y + 2*z*w, 1-2*x*x-2*z*z, 2*y*z - 2*x*w,
	  2*x*z - 2*y*w, 2*y*z + 2*x*w, 1-2*x*x-2*y*y
	};

  Matrix3x3 R;
  R.c[0] = Vector3D{m[0], m[1], m[2]};
  R.c[1] = Vector3D{m[3], m[4], m[5]};
  R.c[2] = Vector3D{m[6], m[7], m[8]};
  return R;
  }


};

#endif // MATRIX_H
