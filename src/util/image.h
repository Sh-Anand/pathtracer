#ifndef CGL_IMAGE_H
#define CGL_IMAGE_H

#include "vector3D.h"

#include <vector>
#include <string.h>
#include <cassert>

#include "util/cuda_defs.h"


/**
 * High Dynamic Range image buffer which stores linear space Vector3D
 * values with 32 bit floating points.
 */
struct HDRImageBuffer {

  /**
   * Default constructor.
   * The default constructor creates a zero-sized image.
   */
  HDRImageBuffer() : w(0), h(0) { }

  /**
   * Resize the image buffer.
   * \param w new width of the image
   * \param h new height of the image
   */
  void resize(size_t w, size_t h) { this->w = w; this->h = h; }

  /**
   * Parameterized constructor.
   * Create an image of given size.
   * \param w width of the image
   * \param h height of the image
   */
  HDRImageBuffer(size_t w, size_t h) : w(w), h(h) { data = nullptr; resize(w, h); }

  /**
   * Update the color of a given pixel.
   * \param s new Vector3D value to be set
   * \param x row of the pixel
   * \param y column of the pixel
   */
  DEVICE void update_pixel(const Vector3D& s, size_t x, size_t y) {
    // assert(0 <= x && x < w);
    // assert(0 <= y && y < h);
    data[x + y * w] = s;
  }

  /**
   * Update the color of a given pixel. Blend new pixel color with current
   * pixel data in the buffer according to the blending factor. The result
   * of this would be buffer[i] = s * r + buffer[i] * (1-r);
   * \param s new Vector3D value to be set
   * \param x row of the pixel
   * \param y column of the pixel
   * \param r blending factor
   */
  void update_pixel(const Vector3D& s, size_t x, size_t y, float r) {
    // assert(0 <= x && x < w);
    // assert(0 <= y && y < h);
    data[x + y * w] = s * r + (1 - r) * data[x + y * w];
  }
  
  /**
   * If the buffer is empty
   */
  bool is_empty() { return (w == 0 && h == 0); }

  size_t w; ///< width
  size_t h; ///< height
  Vector3D* data; ///< pixel buffer

}; // class HDRImageBuffer




#endif // CGL_IMAGE_H
