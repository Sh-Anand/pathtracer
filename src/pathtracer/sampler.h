#ifndef CGL_SAMPLER_H
#define CGL_SAMPLER_H

#include "CGL/vector2D.h"
#include "CGL/vector3D.h"
#include "CGL/misc.h"
#include "util/random_util.h"

namespace CGL {

/**
 * Interface for generating 2D vector samples
 */
class Sampler2D {
 public:

  /**
   * Virtual destructor.
   */
  virtual ~Sampler2D() { }

  /**
   * Use the Sampler2D to obtain a Vector2D sample
   * according to the particular sampler's distribution.
   */
  virtual Vector2D get_sample() const = 0;

}; // class Sampler2D

/**
 * Interface for generating 3D vector samples
 */
class Sampler3D {
 public:

  /**
   * Virtual destructor.
   */
  virtual ~Sampler3D() { }

  /**
   * Use the Sampler3D to obtain a Vector3D sample
   * according to the particular sampler's distribution.
   */
  virtual Vector3D get_sample() const = 0;

}; // class Sampler3D

/**
 * A Sampler3D implementation with uniform distribution on unit sphere
 */
class UniformSphereSampler3D : public Sampler3D {
public:

  Vector3D get_sample() const;

}; // class UniformHemisphereSampler3D


/**
 * A Sampler2D implementation with uniform distribution on unit square
 */
class UniformGridSampler2D : public Sampler2D {
 public:

  Vector2D get_sample() const;

}; // class UniformSampler2D

class JitteredSampler2D: public Sampler2D {
 private:
  size_t n;
  std::vector<Vector2D> samples;

 public:
  Vector2D get_sample() const;

  void reset() {
    samples.clear();
    for (size_t i = 0; i < n; i++) {
      for (size_t j = 0; j < n; j++) {
        samples.push_back(Vector2D((i + random_uniform()) / n, (j + random_uniform()) / n));
      }
    }
  }

  JitteredSampler2D(size_t n): n(n) {
    reset();
  }

}; // class JitteredSampler2D

/**
 * A Sampler3D implementation with uniform distribution on unit hemisphere
 */
class UniformHemisphereSampler3D : public Sampler3D {
 public:

  Vector3D get_sample() const;

}; // class UniformHemisphereSampler3D

/**
 * A Sampler3D implementation with cosine-weighted distribution on unit
 * hemisphere.
 */
class CosineWeightedHemisphereSampler3D : public Sampler3D {
 public:

  Vector3D get_sample() const;
  // Also returns the pdf at the sample point for use in importance sampling.
  Vector3D get_sample(double* pdf) const;

}; // class UniformHemisphereSampler3D

/**
 * TODO (extra credit) :
 * Jittered sampler implementations
 */

} // namespace CGL

#endif //CGL_SAMPLER_H
