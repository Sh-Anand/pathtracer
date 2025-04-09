#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
}

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

// void PathTracer::autofocus(Vector2D loc) {
//   Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
//   CudaIntersection isect;

//   bvh->intersect(r, &isect);

//   camera->focalDistance = isect.t;
// }

} // namespace CGL
