#ifndef CGL_STATICSCENE_LIGHT_H
#define CGL_STATICSCENE_LIGHT_H

#include "scene/primitive.h"
#include "util/vector3D.h"
#include "util/gpu_rand.h"

namespace CGL { namespace SceneObjects {

struct CudaLight {
  CudaLight(const Vector3D rad, 
            const Vector3D pos,
            const Vector3D direction,
            float inner_cone_angle,
            float outer_cone_angle) 
      : radiance(rad), 
        position(pos),
        direction(direction),
        inner_cone_angle(inner_cone_angle),
        outer_cone_angle(outer_cone_angle) {
          area = 1;
          is_cone_light = true;
        }
  CudaLight(const Vector3D rad, 
            const CudaPrimitive tri,
            std::vector<Vector3D>& vertices) 
      : radiance(rad), 
        triangle(tri) {
          area = 0.5 * cross(vertices[tri.i_p2] - vertices[tri.i_p1], vertices[tri.i_p3] - vertices[tri.i_p1]).norm();
          is_cone_light = false;
        }

  Vector3D radiance;
  CudaPrimitive triangle;
  float area;

  // if point light
  Vector3D position;
  Vector3D direction;
  bool is_cone_light = false;
  float inner_cone_angle = 0.0;
  float outer_cone_angle = 0.0;

  DEVICE Vector3D sample_L(const  Vector3D         p,
                                  Vector3D*              wi,
                                  float*                distToLight,
                                  float*                pdf,
                                  RNGState&              rand_state,
                                  const Vector3D*        vertices);

  DEVICE bool has_intersect(Ray& r, const Vector3D &p, const Vector3D &N,
                            const Vector3D* vertices, float *pdf) const;

};

} // namespace SceneObjects
} // namespace CGL

#endif  // CGL_STATICSCENE_BSDF_H
