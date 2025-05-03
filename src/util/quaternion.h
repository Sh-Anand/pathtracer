#ifndef CGL_QUATERNION_H
#define CGL_QUATERNION_H

#include "util/cuda_defs.h"
#include "matrix3x3.h"
#include "matrix4x4.h"

#include <iosfwd>

namespace CGL {

class Quaternion : public Vector4D {
 public:

  /**
   * Constructor.
   * Initializes to 0,0,0,1
   */
  Quaternion( ) : Vector4D( 0.0, 0.0, 0.0, 1.0 ) { }

  /**
   * Construct from 3D vector and w.
   */
  Quaternion(const Vector3D& v, double w) : Vector4D(v.x, v.y, v.z, w) { }

  Quaternion(const Vector4D& v) : Vector4D(v.x, v.y, v.z, v.w) { }

  Quaternion(double x, double y, double z, double w) : Vector4D(x, y, z, w) { }

  /**
   * Initializes a quaternion that represents a rotation about the given axis
   * and angle.
   */
  void from_axis_angle(const Vector3D& axis, double radians) {
    radians /= 2;
    const Vector3D& nAxis = axis.unit();
    double sinTheta = sin(radians);
    x = sinTheta * nAxis.x;
    y = sinTheta * nAxis.y;
    z = sinTheta * nAxis.z;
    w = cos(radians);
    this->normalize();
  }

  Vector3D complex() const { return Vector3D(x, y, z); }
  void setComplex(const Vector3D& c)
  {
	x = c.x;
	y = c.y;
	z = c.z;
  }

  double real() const { return w; }
  void setReal(double r) { w = r; }

  Quaternion conjugate(void) const
  {
	return Quaternion(-complex(), real());
  }

  /**
   * @brief Computes the inverse of this quaternion.
   *
   * @note This is a general inverse.  If you know a priori
   * that you're using a unit quaternion (i.e., norm() == 1),
   * it will be significantly faster to use conjugate() instead.
   *
   * @return The quaternion q such that q * (*this) == (*this) * q
   * == [ 0 0 0 1 ]<sup>T</sup>.
   */
  Quaternion inverse(void) const
  {
	return conjugate() / norm();
  }


  /**
   * @brief Computes the product of this quaternion with the
   * quaternion 'rhs'.
   *
   * @param rhs The right-hand-side of the product operation.
   *
   * @return The quaternion product (*this) x @p rhs.
   */
  Quaternion product(const Quaternion& rhs) const {
	return Quaternion(y*rhs.z - z*rhs.y + x*rhs.w + w*rhs.x,
					  z*rhs.x - x*rhs.z + y*rhs.w + w*rhs.y,
					  x*rhs.y - y*rhs.x + z*rhs.w + w*rhs.z,
					  w*rhs.w - x*rhs.x - y*rhs.y - z*rhs.z);
  }

  /**
   * @brief Quaternion product operator.
   *
   * The result is a quaternion such that:
   *
   * result.real() = (*this).real() * rhs.real() -
   * (*this).complex().dot(rhs.complex());
   *
   * and:
   *
   * result.complex() = rhs.complex() * (*this).real
   * + (*this).complex() * rhs.real()
   * - (*this).complex().cross(rhs.complex());
   *
   * @return The quaternion product (*this) x rhs.
   */
  Quaternion operator*(const Quaternion& rhs) const {
	return product(rhs);
  }

  /**
   * @brief Returns a matrix representation of this
   * quaternion.
   *
   * Specifically this is the matrix such that:
   *
   * this->matrix() * q.vector() = (*this) * q for any quaternion q.
   *
   * Note that this is @e NOT the rotation matrix that may be
   * represented by a unit quaternion.
   */
  Matrix4x4 matrix() const {

	double m[16] = {
	   w,  -z,  y, x,
	   z,   w, -x, y,
	  -y,   x,  w, z,
	  -x,  -y, -z, w
	};

	return Matrix4x4(m);
  }

  /**
   * @brief Returns a matrix representation of this
   * quaternion for right multiplication.
   *
   * Specifically this is the matrix such that:
   *
   * q.vector().transpose() * this->matrix() = (q *
   * (*this)).vector().transpose() for any quaternion q.
   *
   * Note that this is @e NOT the rotation matrix that may be
   * represented by a unit quaternion.
   */
  Matrix4x4 rightMatrix() const {
	double m[16] = {
	  +w, -z,  y, -x,
	  +z,  w, -x, -y,
	  -y,  x,  w, -z,
	  +x,  y,  z,  w
	};

	return Matrix4x4(m);
  }

  /**
   * @brief Returns this quaternion as a 4-vector.
   *
   * This is simply the vector [x y z w]<sup>T</sup>
   */
  Vector4D vector() const { return Vector4D(x, y, z, w); }

  /**
   * @brief Computes the rotation matrix represented by a unit
   * quaternion.
   *
   * @note This does not check that this quaternion is normalized.
   * It formulaically returns the matrix, which will not be a
   * rotation if the quaternion is non-unit.
   */
  Matrix3x3 rotationMatrix() const {
	double m[9] = {
	  1-2*y*y-2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w,
	  2*x*y + 2*z*w, 1-2*x*x-2*z*z, 2*y*z - 2*x*w,
	  2*x*z - 2*y*w, 2*y*z + 2*x*w, 1-2*x*x-2*y*y
	};

	return Matrix3x3(m);
  }


    /**
     * @brief Returns the scaled-axis representation of this
     * quaternion rotation.
     */
  Vector3D scaledAxis(void) const{

	Quaternion q1 = (Quaternion)unit();

	// s must be positive, because q1 <= 1, due to normalization.
	double s = sqrt(1-q1.w*q1.w);

	// Avoid dividing by 0.
	if (s < 0.001)
	{
	  // if s close to zero then direction of axis not important
	  // if it is important that axis is normalised then replace with x=1; y=z=0;

	  return Vector3D(q1.x, q1.y, q1.z);
	}
	else
	{
	  // normalise axis
	  return Vector3D(q1.x / s, q1.y / s, q1.z / s);
	}

	// NEVER getsgg HERE.

  }

  /**
   * @brief Sets quaternion to be same as rotation by scaled axis w.
   * This is the equal and opposite conversion from the scaledAxis(void) function.
   */
  void scaledAxis(const Vector3D& vec_in)
  {
	double theta = vec_in.norm();

	// Small magnitudes are handled via the default vector.
	if (theta > 0.0001)
	{
	  double s = sin(theta / 2.0);
	  Vector3D W(vec_in / theta * s);
	  x = W.x;
	  y = W.y;
	  z = W.z;
	  w = cos(theta / 2.0);
	}
	else
	{
	  x = y = z = 0;
	  w = 1.0;
	}
  }

  /**
   * @brief Returns a vector rotated by this quaternion.
   *
   * Functionally equivalent to:  (rotationMatrix() * v)
   * or (q * Quaternion(0, v) * q.inverse()).
   *
   * @warning conjugate() is used instead of inverse() for better
   * performance, when this quaternion must be normalized.
   */
  Vector3D rotatedVector(const Vector3D& v) const {
	return (((*this) * Quaternion(v, 0)) * conjugate()).complex();
  }

};

} // namespace CGL

#endif /* CGL_QUATERNION_H */
