#ifndef CGL_STATICSCENE_TEXTURE_H
#define CGL_STATICSCENE_TEXTURE_H

#include "util/image.h"

namespace CGL {
    struct CudaTexture {
        uint16_t width;
        uint16_t height;
        bool has_alpha;
        uint8_t * data;
    };
} // CGL

#endif  // CGL_STATICSCENE_TEXTURE_H
