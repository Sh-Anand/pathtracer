#ifndef CGL_IMAGE_H
#define CGL_IMAGE_H

#include "vector3D.h"

#include <vector>
#include <string.h>
#include <cassert>

#include "util/cuda_defs.h"

namespace CGL {

/**
 * Image buffer which stores color space values with RGBA pixel layout using
 * 32 bit unsigned integers (8-bits per color channel, high byte is padding).
 * 
 * Note that the alpha channel exists purely to make interfacing with
 * OpenGL easier, and is assumed 1 everywhere.
 */
struct ImageBuffer {
  /**
   * Default constructor.
   * The default constructor creates a zero-sized image.
   */
  ImageBuffer() : w(0), h(0) {}

  /**
   * Parameterized constructor.
   * Create an image of given size.
   * \param w width of the image
   * \param h height of the image
   */
  ImageBuffer(size_t w, size_t h) : w(w), h(h) { data.resize(w * h); }

  /**
   * Resize the image buffer.
   * \param w new width of the image
   * \param h new height of the image
   */
  void resize(size_t w, size_t h) {
    this->w = w;
    this->h = h;
    data.resize(w * h);
    clear();
  }

  /**
   * Update the color of a given pixel.
   * \param c color value to be set
   * \param x row of the pixel
   * \param y column of the pixel
   */
  void update_pixel(const Vector3D& c, size_t x, size_t y) {
    // x y z = r g b
    assert(x < w);
    assert(y < h);
    uint32_t p = 0;
    p |= ((uint32_t) (clamp_T(0., 1., c.x) * 255)) << 16;
    p |= ((uint32_t) (clamp_T(0., 1., c.y) * 255)) << 8;
    p |= ((uint32_t) (clamp_T(0., 1., c.z) * 255));
    p |= 0xFF000000;
    data[x + y * w] = p;
  }

  /**
   * If the buffer is empty.
   */
  bool is_empty() { return (w == 0 && h == 0); }

  /**
   * Clear image data.
   */
  void clear() {
    uint32_t clear_val = 0xFF000000; // Black with full alpha
    for (size_t i = 0; i < w * h; ++i) {
		data[i] = clear_val;
	}
  }

  size_t w; ///< width
  size_t h; ///< height
  std::vector<uint32_t> data;  ///< pixel buffer
};

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
  void resize(size_t w, size_t h);

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
   * Tonemap and convert to color space image.
   * \param target target color buffer to store output
   * \param gamma gamma value
   * \param level exposure level adjustment
   * \key   key value to map average tone to (higher means brighter)
   * \why   white point (higher means larger dynamic range)
   */
  void tonemap(ImageBuffer& target,
    float gamma, float level, float key, float wht) {

    // compute global log average luminance!
    float avg = 0;
    for (size_t i = 0; i < w * h; ++i) {
      // the small delta value below is used to avoids singularity
      avg += log(0.0000001f + data[i].illum());
    }
    avg = exp(avg / (w * h));


    // apply on pixels
    float one_over_gamma = 1.0f / gamma;
    float exposure = sqrt(pow(2,level));
    for (size_t y = 0; y < h; ++y) {
      for (size_t x = 0; x < w; ++x) {
        Vector3D s = data[x + y * w];
        float l = s.illum();
        s *= key / avg;
        s *= ((l + 1) / (wht * wht)) / (l + 1);
        float r = pow(s.x * exposure, one_over_gamma);
        float g = pow(s.y * exposure, one_over_gamma);
        float b = pow(s.z * exposure, one_over_gamma);
        target.update_pixel(Vector3D(r, g, b), x, y);
      }
    }
  }

  /**
   * Convert the given tile of the buffer to color.
   */
  void toColor(ImageBuffer& target, size_t x0, size_t y0, size_t x1, size_t y1) {

    float gamma = 2.2f;
    float level = 1.0f;
    float one_over_gamma = 1.0f / gamma;
    float exposure = sqrt(pow(2,level));
    for (size_t y = y0; y < y1; ++y) {
      for (size_t x = x0; x < x1; ++x) {
        const Vector3D& s = data[x + y * w];
        float r = std::max(0.0, std::min(pow(s.x * exposure, one_over_gamma), 1.0));
        float g = std::max(0.0, std::min(pow(s.y * exposure, one_over_gamma), 1.0));
        float b = std::max(0.0, std::min(pow(s.z * exposure, one_over_gamma), 1.0));
        target.update_pixel(Vector3D(r, g, b), x, y);
      }
    }
  }

  /**
   * If the buffer is empty
   */
  bool is_empty() { return (w == 0 && h == 0); }

  size_t w; ///< width
  size_t h; ///< height
  Vector3D* data; ///< pixel buffer

}; // class HDRImageBuffer


} // namespace CGL

#endif // CGL_IMAGE_H
