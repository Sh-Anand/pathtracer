#include "matrix3x3.h"

#include <iostream>
#include <cmath>

using namespace std;

void Matrix3x3::zero( float val ) {
  // sets all elements to val
  entries[0] = entries[1] = entries[2] = Vector3D( val, val, val );
}

float Matrix3x3::norm( void ) const {
  return sqrt( entries[0].norm2() +
                entries[1].norm2() +
                entries[2].norm2() );
}

Matrix3x3 Matrix3x3::operator-( void ) const {

  // returns -A
  const Matrix3x3& A( *this );
  Matrix3x3 B;

  B[0] = -A[0];
  B[1] = -A[1];
  B[2] = -A[2];

  return B;
}

void Matrix3x3::operator+=( const Matrix3x3& B ) {

  Matrix3x3& A( *this );

  A[0] += B[0];
  A[1] += B[1];
  A[2] += B[2];
}

Matrix3x3 Matrix3x3::operator-( const Matrix3x3& B ) const {
  const Matrix3x3& A( *this );
  Matrix3x3 C;

  C[0] = A[0] - B[0];
  C[1] = A[1] - B[1];
  C[2] = A[2] - B[2];

  return C;
}

Matrix3x3 Matrix3x3::inv( void ) const {
  const Matrix3x3& A( *this );
  Matrix3x3 B;

  B(0,0) = -A(1,2)*A(2,1) + A(1,1)*A(2,2); B(0,1) =  A(0,2)*A(2,1) - A(0,1)*A(2,2); B(0,2) = -A(0,2)*A(1,1) + A(0,1)*A(1,2);
  B(1,0) =  A(1,2)*A(2,0) - A(1,0)*A(2,2); B(1,1) = -A(0,2)*A(2,0) + A(0,0)*A(2,2); B(1,2) =  A(0,2)*A(1,0) - A(0,0)*A(1,2);
  B(2,0) = -A(1,1)*A(2,0) + A(1,0)*A(2,1); B(2,1) =  A(0,1)*A(2,0) - A(0,0)*A(2,1); B(2,2) = -A(0,1)*A(1,0) + A(0,0)*A(1,1);

  B /= det();

  return B;
}

void Matrix3x3::operator/=( float x ) {
  Matrix3x3& A( *this );
  float rx = 1./x;

  A[0] *= rx;
  A[1] *= rx;
  A[2] *= rx;
}

Matrix3x3 Matrix3x3::identity( void ) {
  Matrix3x3 B;

  B(0,0) = 1.; B(0,1) = 0.; B(0,2) = 0.;
  B(1,0) = 0.; B(1,1) = 1.; B(1,2) = 0.;
  B(2,0) = 0.; B(2,1) = 0.; B(2,2) = 1.;

  return B;
}

std::ostream& operator<<( std::ostream& os, const Matrix3x3& A ) {
  for( int i = 0; i < 3; i++ )
  {
      os << "[ ";

      for( int j = 0; j < 3; j++ )
      {
        os << A(i,j) << " ";
      }

      os << "]" << std::endl;
  }

  return os;
}
