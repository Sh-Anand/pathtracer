#ifndef CGL_RAY_H
#define CGL_RAY_H

#include "util/vector3D.h"
#include "util/matrix4x4.h"

#define PART 5

#define PART_1 (PART >= 1)
#define PART_2 (PART >= 2)
#define PART_3 (PART >= 3)
#define PART_4 (PART >= 4)
#define PART_5 (PART >= 5)

namespace CGL {


struct Ray {
  uint16_t depth;  ///< depth of the Ray

  uint16_t x,y; ///< pixel coordinates

  Vector3D o;  ///< origin
  Vector3D d;  ///< direction
  float min_t; ///< treat the ray as a segment (ray "begin" at min_t)
  float max_t; ///< treat the ray as a segment (ray "ends" at max_t)

  Vector3D inv_d;  ///< component wise inverse

  HOST_DEVICE Ray() {}

  /**
   * Constructor.
   * Create a ray instance with given origin and direction.
   * \param o origin of the ray
   * \param d direction of the ray
   * \param depth depth of the ray
   */
    HOST_DEVICE Ray(const Vector3D o, const Vector3D d, int depth = 0);

  /**
   * Constructor.
   * Create a ray instance with given origin and direction.
   * \param o origin of the ray
   * \param d direction of the ray
   * \param max_t max t value for the ray (if it's actually a segment)
   * \param depth depth of the ray
   */
    HOST_DEVICE Ray(const Vector3D o, const Vector3D d, float max_t, int depth = 0);

  /**
   * Returns the point t * |d| along the ray.
   */
  inline Vector3D at_time(float t) const { return o + t * d; }
};

// structure used for logging rays for subsequent visualization
struct LoggedRay {

    LoggedRay(Ray& r, float hit_t)
        : o(r.o), d(r.d), hit_t(hit_t) {}

    Vector3D o;
    Vector3D d;
    float hit_t;
};

}  // namespace CGL

#endif  // CGL_RAY_H
