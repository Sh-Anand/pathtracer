#include "matrix3x3.h"

namespace CGL {

  HOST_DEVICE double& Matrix3x3::operator()( int i, int j ) {
    return entries[j][i];
  }

  HOST_DEVICE const double& Matrix3x3::operator()( int i, int j ) const {
    return entries[j][i];
  }

  HOST_DEVICE Vector3D& Matrix3x3::operator[]( int j ) {
      return entries[j];
  }

  HOST_DEVICE const Vector3D& Matrix3x3::operator[]( int j ) const {
    return entries[j];
  }

  HOST_DEVICE Matrix3x3 Matrix3x3::T( void ) const {
    const Matrix3x3& A( *this );
    Matrix3x3 B;

    for( int i = 0; i < 3; i++ )
    for( int j = 0; j < 3; j++ )
    {
       B(i,j) = A(j,i);
    }

    return B;
  }

  HOST_DEVICE Matrix3x3 Matrix3x3::operator*( double c ) const {
    const Matrix3x3& A( *this );
    Matrix3x3 B;

    B[0] = A[0] * c;
    B[1] = A[1] * c;
    B[2] = A[2] * c;

    return B;
  }

  HOST_DEVICE Matrix3x3 operator*( double c, const Matrix3x3& A ) {
    Matrix3x3 cA;

    cA[0] = A[0] * c;
    cA[1] = A[1] * c;
    cA[2] = A[2] * c;

    return cA;
  }

  HOST_DEVICE Matrix3x3 Matrix3x3::operator*( const Matrix3x3& B ) const {
    const Matrix3x3& A( *this );
    Matrix3x3 C;

    C[0] = A * B[0];
    C[1] = A * B[1];
    C[2] = A * B[2];

    return C;
  }

  HOST_DEVICE Vector3D Matrix3x3::operator*( const Vector3D& x ) const {
    return x.x * entries[0] +
           x.y * entries[1] +
           x.z * entries[2];
  }

  HOST_DEVICE double Matrix3x3::det( void ) const {
    const Matrix3x3& A( *this );

    return -A(0,2)*A(1,1)*A(2,0) + A(0,1)*A(1,2)*A(2,0) +
            A(0,2)*A(1,0)*A(2,1) - A(0,0)*A(1,2)*A(2,1) -
            A(0,1)*A(1,0)*A(2,2) + A(0,0)*A(1,1)*A(2,2) ;
  }

  HOST_DEVICE Vector3D& Matrix3x3::column( int i ) {
    return entries[i];
  }

  HOST_DEVICE const Vector3D& Matrix3x3::column( int i ) const {
    return entries[i];
  }
}