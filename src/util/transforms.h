#ifndef CGL_TRANSFORMS_H
#define CGL_TRANSFORMS_H

#include "vector.h"
#include "matrix.h"

Matrix4x4 translate(float dx, float dy, float dz){
    Matrix4x4 T;
    T.c[0] = Vector4D{1, 0, 0, 0};
    T.c[1] = Vector4D{0, 1, 0, 0};
    T.c[2] = Vector4D{0, 0, 1, 0};
    T.c[3] = Vector4D{dx, dy, dz, 1};
    return T;
}

Matrix4x4 scale(float sx, float sy, float sz){
    Matrix4x4 S;
    S.c[0] = Vector4D{sx, 0, 0, 0};
    S.c[1] = Vector4D{0, sy, 0, 0};
    S.c[2] = Vector4D{0, 0, sz, 0};
    S.c[3] = Vector4D{0, 0, 0, 1};
    return S;
}


#endif // CGL_TRANSFORMS_H