#ifndef CGL_VECTOR2D_H
#define CGL_VECTOR2D_H

#include "cuda_defs.h"

#include <ostream>
#include <cmath>

namespace CGL {

/**
 * Defines 2D vectors.
 */
class Vector2D {
 public:

  // components
  float x, y;

  /**
   * Constructor.
   * Initializes to vector (0,0).
   */
  HOST_DEVICE Vector2D() : x( 0.0 ), y( 0.0 ) { }

  /**
   * Constructor.
   * Initializes to vector (a,b).
   */
  HOST_DEVICE Vector2D( float x, float y ) : x( x ), y( y ) { }

  /**
   * Constructor.
   * Copy constructor. Creates a copy of the given vector.
   */
  HOST_DEVICE Vector2D( const Vector2D& v ) : x( v.x ), y( v.y ) { }

  // returns reference to the specified component (0-based indexing: x, y)
  HOST_DEVICE inline float& operator[] ( const int& index ) {
    return ( &x )[ index ];
  }

  // returns const reference to the specified component (0-based indexing: x, y)
  HOST_DEVICE inline const float& operator[] ( const int& index ) const {
    return ( &x )[ index ];
  }

  // additive inverse
  HOST_DEVICE inline Vector2D operator-( void ) const {
    return Vector2D( -x, -y );
  }

  // addition
  HOST_DEVICE inline Vector2D operator+( const Vector2D& v ) const {
    Vector2D u = *this;
    u += v;
    return u;
  }

  // subtraction
  HOST_DEVICE inline Vector2D operator-( const Vector2D& v ) const {
    Vector2D u = *this;
    u -= v;
    return u;
  }

  // right scalar multiplication
  HOST_DEVICE inline Vector2D operator*( float r ) const {
    Vector2D vr = *this;
    vr *= r;
    return vr;
  }

  // scalar division
  HOST_DEVICE inline Vector2D operator/( float r ) const {
    Vector2D vr = *this;
    vr /= r;
    return vr;
  }

  // add v
  HOST_DEVICE inline void operator+=( const Vector2D& v ) {
    x += v.x;
    y += v.y;
  }

  // subtract v
  HOST_DEVICE inline void operator-=( const Vector2D& v ) {
    x -= v.x;
    y -= v.y;
  }

  // scalar multiply by r
  HOST_DEVICE inline void operator*=( float r ) {
    x *= r;
    y *= r;
  }

  // scalar divide by r
  HOST_DEVICE inline void operator/=( float r ) {
    x /= r;
    y /= r;
  }

  /**
   * Returns norm.
   */
  HOST_DEVICE inline float norm( void ) const {
    return sqrt( x*x + y*y );
  }

  /**
   * Returns norm squared.
   */
  HOST_DEVICE inline float norm2( void ) const {
    return x*x + y*y;
  }

  /**
   * Returns unit vector parallel to this one.
   */
  HOST_DEVICE inline Vector2D unit( void ) const {
    return *this / this->norm();
  }

}; // class Vector2D

// left scalar multiplication
HOST_DEVICE inline Vector2D operator*( float r, const Vector2D& v ) {
   return v*r;
}

// inner product
HOST_DEVICE inline float dot( const Vector2D& v1, const Vector2D& v2 ) {
  return v1.x*v2.x + v1.y*v2.y;
}

// cross product
HOST_DEVICE inline float cross( const Vector2D& v1, const Vector2D& v2 ) {
  return v1.x*v2.y - v1.y*v2.x;
}

// prints components
std::ostream& operator<<( std::ostream& os, const Vector2D& v );

} // namespace CGL

#endif // CGL_VECTOR2D_H
