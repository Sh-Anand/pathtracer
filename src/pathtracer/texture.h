#ifndef CGL_STATICSCENE_TEXTURE_H
#define CGL_STATICSCENE_TEXTURE_H

#include "util/image.h"
#include "util/vector2D.h"
#include "util/vector4D.h"

#ifndef __CUDACC__
struct uchar4;
#endif


namespace CGL {
    struct CudaTexture {
        uint16_t width;
        uint16_t height;
        bool has_alpha;
        uint8_t * data;
        DEVICE Vector4D sample(const Vector2D &uv);    };
} // CGL

#endif  // CGL_STATICSCENE_TEXTURE_H
