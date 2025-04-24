#ifndef CGL_MATRIX4X4_H
#define CGL_MATRIX4X4_H

#include "vector4D.h"

#include <iosfwd>

namespace CGL {

/**
 * Defines a 4x4 matrix.
 * 4x4 matrices are also extremely useful in computer graphics.
 * Written by Bryce Summers on 9/10/2015.
 * Adapted from the Matrix3x3 class.
 *
 * EXTEND_ME : It might be nice to add some combined operations
 *             such as multiplying then adding,
 *             etc to increase arithmetic intensity.
 * I have taken the liberty of removing cross product functionality form 4D Matrices and Vectors.
 */
class Matrix4x4 {

  public:


  // The default constructor.
  Matrix4x4(void) { }

  Matrix4x4(float data) { 
      (*this)(0,0) = data;
      (*this)(1,1) = data;
      (*this)(2,2) = data;
      (*this)(3,3) = data;
  }

  // Constructor for row major form data.
  // Transposes to the internal column major form.
  // REQUIRES: data should be of size 16.
  Matrix4x4(double * data)
  {
    for( int i = 0; i < 4; i++ )
    for( int j = 0; j < 4; j++ )
    {
	  // Transpostion happens within the () query.
	  (*this)(i,j) = data[i*4 + j];
    }

  }

  Matrix4x4(float m00, float m01, float m02, float m03,
      float m10, float m11, float m12, float m13,
      float m20, float m21, float m22, float m23,
      float m30, float m31, float m32, float m33)
      {
      (*this)(0,0) = m00; (*this)(0,1) = m01; (*this)(0,2) = m02; (*this)(0,3) = m03;
      (*this)(1,0) = m10; (*this)(1,1) = m11; (*this)(1,2) = m12; (*this)(1,3) = m13;
      (*this)(2,0) = m20; (*this)(2,1) = m21; (*this)(2,2) = m22; (*this)(2,3) = m23;
      (*this)(3,0) = m30; (*this)(3,1) = m31; (*this)(3,2) = m32; (*this)(3,3) = m33;
      }


  /**
   * Sets all elements to val.
   */
  void zero(double val = 0.0);

  /**
   * Returns the determinant of A.
   */
  double det( void ) const;

  /**
   * Returns the Frobenius norm of A.
   */
  double norm( void ) const;

  /**
   * Returns a fresh 4x4 identity matrix.
   */
  static Matrix4x4 identity( void );

  // No Cross products for 4 by 4 matrix.

  /**
   * Returns the ith column.
   */
        Vector4D& column( int i );
  const Vector4D& column( int i ) const;

  /**
   * Returns the transpose of A.
   */
  Matrix4x4 T( void ) const;

  /**
   * Returns the inverse of A.
   */
  Matrix4x4 inv( void ) const;

  // accesses element (i,j) of A using 0-based indexing
  // where (i, j) is (row, column).
        double& operator()( int i, int j );
  const double& operator()( int i, int j ) const;

  // accesses the ith column of A
        Vector4D& operator[]( int i );
  const Vector4D& operator[]( int i ) const;

  // increments by B
  void operator+=( const Matrix4x4& B );

  // returns -A
  Matrix4x4 operator-( void ) const;

  // returns A-B
  Matrix4x4 operator-( const Matrix4x4& B ) const;

  // returns c*A
  Matrix4x4 operator*( double c ) const;

  // returns A*B
  Matrix4x4 operator*( const Matrix4x4& B ) const;

  // returns A*x
  Vector4D operator*( const Vector4D& x ) const;

  // divides each element by x
  void operator/=( double x );

  protected:

  // 4 by 4 matrices are represented by an array of 4 column vectors.
  Vector4D entries[4];

}; // class Matrix3x3

// returns the outer product of u and v.
Matrix4x4 outer( const Vector4D& u, const Vector4D& v );

// returns c*A
Matrix4x4 operator*( double c, const Matrix4x4& A );

// prints entries
std::ostream& operator<<( std::ostream& os, const Matrix4x4& A );

} // namespace CGL

#endif // CGL_MATRIX4X4_H
