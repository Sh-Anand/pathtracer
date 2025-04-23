#ifndef CGL_TRANSFORMS_H
#define CGL_TRANSFORMS_H

#include "vector3D.h"
#include "matrix4x4.h"

namespace CGL {

Vector3D operator*(const Matrix4x4 &m, const Vector3D &v);


Matrix4x4 translate(float dx, float dy, float dz){
    return Matrix4x4(
		1, 0, 0, dx,
		0, 1, 0, dy,
		0, 0, 1, dz,
        0, 0, 0, 1
		);
}

Matrix4x4 scale(float sx, float sy, float sz){
    return Matrix4x4(
        sx,   0,   0,   0,  
        0,   sy,   0,   0,
        0,    0,  sz,   0, 
        0,    0,   0,   1
    );
}


// Matrix4x4 rotate(float deg);

}


#endif // CGL_TRANSFORMS_H