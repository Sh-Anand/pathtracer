#include "pathtracer.h"

using namespace CGL::SceneObjects;

namespace CGL {

void PathTracer::set_frame_size(uint16_t width, uint16_t height) {
  sampleBuffer.resize(width, height);
}

void PathTracer::clear() {
  bvh = NULL;
}

void PathTracer::write_to_framebuffer(HDRImageBuffer &buffer, ImageBuffer &framebuffer, uint16_t x0,
                                      uint16_t y0, uint16_t x1, uint16_t y1) {
  buffer.toColor(framebuffer, x0, y0, x1, y1);
}

} // namespace CGL
