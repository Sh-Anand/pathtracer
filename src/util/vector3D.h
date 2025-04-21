#ifndef CGL_VECTOR3D_H
#define CGL_VECTOR3D_H

#include "cuda_defs.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <ostream>
#include <new>

namespace CGL {

/**
 * Defines 3D vectors.
 */
class Vector3D {
public:

  // components
  float x, y, z;

  HOST_DEVICE Vector3D() : x(0.0), y(0.0), z(0.0) { }

  HOST_DEVICE Vector3D(float x, float y, float z) : x(x), y(y), z(z) { }

  HOST_DEVICE Vector3D(float c) : x(c), y(c), z(c) { }


  HOST_DEVICE Vector3D(const Vector3D& v) : x(v.x), y(v.y), z(v.z) { }

  HOST_DEVICE inline float& operator[](const int& index) {
    return (&x)[index];
  }

  HOST_DEVICE inline const float& operator[](const int& index) const {
    return (&x)[index];
  }

  HOST_DEVICE inline bool operator==(const Vector3D& v) const {
    return v.x == x && v.y == y && v.z == z;
  }

  HOST_DEVICE inline Vector3D operator-(void) const {
    return Vector3D(-x, -y, -z);
  }

  HOST_DEVICE inline Vector3D operator+(const Vector3D& v) const {
    return Vector3D(x + v.x, y + v.y, z + v.z);
  }

  HOST_DEVICE inline Vector3D operator-(const Vector3D& v) const {
    return Vector3D(x - v.x, y - v.y, z - v.z);
  }

  HOST_DEVICE inline Vector3D operator*(const Vector3D& v) const {
    return Vector3D(x * v.x, y * v.y, z * v.z);
  }

  HOST_DEVICE inline Vector3D operator/(const Vector3D& v) const {
    return Vector3D(x / v.x, y / v.y, z / v.z);
  }

  HOST_DEVICE inline Vector3D operator*(const float& c) const {
    return Vector3D(x * c, y * c, z * c);
  }

  HOST_DEVICE inline Vector3D operator/(const float& c) const {
    const float rc = 1.0 / c;
    return Vector3D(rc * x, rc * y, rc * z);
  }

  HOST_DEVICE inline void operator+=(const Vector3D& v) {
    x += v.x; y += v.y; z += v.z;
  }

  HOST_DEVICE inline void operator-=(const Vector3D& v) {
    x -= v.x; y -= v.y; z -= v.z;
  }

  HOST_DEVICE inline void operator*=(const float& c) {
    x *= c; y *= c; z *= c;
  }

  HOST_DEVICE inline void operator/=(const float& c) {
    (*this) *= (1. / c);
  }

  HOST_DEVICE inline Vector3D rcp(void) const {
    return Vector3D(1.0 / x, 1.0 / y, 1.0 / z);
  }

  HOST_DEVICE inline float norm(void) const {
    return sqrtf(x * x + y * y + z * z);
  }

  HOST_DEVICE inline float norm2(void) const {
    return x * x + y * y + z * z;
  }

  HOST_DEVICE inline Vector3D unit(void) const {
    float rNorm = 1. / norm();
    return (*this) * rNorm;
  }

  HOST_DEVICE inline void normalize(void) {
    (*this) /= norm();
  }

  HOST_DEVICE inline float illum() const {
    return 0.2126f * x + 0.7152f * y + 0.0722f * z;
  }
  
}; // class Vector3D

HOST_DEVICE inline Vector3D operator*(const float& c, const Vector3D& v) {
  return Vector3D(c * v.x, c * v.y, c * v.z);
}

HOST_DEVICE inline Vector3D operator/(const float& c, const Vector3D& v) {
  return Vector3D(c / v.x, c / v.y, c / v.z);
}

HOST_DEVICE inline float dot(const Vector3D& u, const Vector3D& v) {
  return u.x * v.x + u.y * v.y + u.z * v.z;
}

HOST_DEVICE inline Vector3D cross(const Vector3D& u, const Vector3D& v) {
  return Vector3D(u.y * v.z - u.z * v.y,
                  u.z * v.x - u.x * v.z,
                  u.x * v.y - u.y * v.x);
}

HOST_DEVICE std::ostream& operator<<(std::ostream& os, const Vector3D& v);

} // namespace CGL

#endif // CGL_VECTOR3D_H
