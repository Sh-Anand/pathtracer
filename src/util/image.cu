#include <cuda_runtime.h>

#include "image.h"

namespace CGL {
void HDRImageBuffer::resize(size_t w, size_t h) {
    this->w = w;
    this->h = h;
}
}