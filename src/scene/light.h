#ifndef CGL_STATICSCENE_LIGHT_H
#define CGL_STATICSCENE_LIGHT_H

#include "util/vector3D.h"

namespace CGL { namespace SceneObjects {

  enum CudaLightType {
    NONE,
    AMBIENT,
    DIRECTIONAL,
    AREA,
    POINT,
    SPOT
  };

struct CudaAmbientLight {
 public:
  bool is_delta_light() const { return false; }
  CudaAmbientLight(Vector3D rad) : radiance(rad) {
    sampleToWorld[0] = Vector3D(1,  0,  0);
    sampleToWorld[1] = Vector3D(0,  0, -1);
    sampleToWorld[2] = Vector3D(0,  1,  0);
  }
  Vector3D radiance;
  Matrix3x3 sampleToWorld;
}; // class InfiniteHemisphereLight

struct CudaDirectionalLight {
  DEVICE bool is_delta_light() const { return true; }
  Vector3D radiance;
  Vector3D dirToLight;
};

struct CudaPointLight {
  DEVICE bool is_delta_light() const { return true; }
  Vector3D radiance;
  Vector3D position;
};

struct CudaAreaLight {
  CudaAreaLight(const Vector3D rad, 
                const Vector3D pos, 
                const Vector3D dir, 
                const Vector3D dim_x, 
                const Vector3D dim_y) 
      : radiance(rad), 
        position(pos), 
        direction(dir), 
        dim_x(dim_x), 
        dim_y(dim_y), 
        area(dim_x.norm() * dim_y.norm()) {}
  DEVICE bool is_delta_light() const { return false; }
  Vector3D radiance;
  Vector3D position;
  Vector3D direction;
  Vector3D dim_x;
  Vector3D dim_y;
  double area;
};

union LightData {
  LightData() {}
  CudaAmbientLight ambient;
  CudaDirectionalLight directional;
  CudaPointLight point;
  CudaAreaLight area;
};
struct CudaLight {
  CudaLightType type;
  LightData light;
  CudaLight() : type(CudaLightType::NONE) {}
  CudaLight(const CudaLight& l) : type(l.type) {
    switch (type) {
      case CudaLightType::AMBIENT:
        light.ambient = l.light.ambient;
        break;
      case CudaLightType::DIRECTIONAL:
        light.directional = l.light.directional;
        break;
      case CudaLightType::POINT:
        light.point = l.light.point;
        break;
      case CudaLightType::AREA:
        light.area = l.light.area;
        break;
      default:
        break;
    }
  }
  DEVICE bool is_delta_light() const {
    switch (type) {
      case CudaLightType::DIRECTIONAL:
        return light.directional.is_delta_light();
      case CudaLightType::POINT:
        return light.point.is_delta_light();
      case CudaLightType::AREA:
        return light.area.is_delta_light();
      default:
        return false;
    }
  }
};

} // namespace SceneObjects
} // namespace CGL

#endif  // CGL_STATICSCENE_BSDF_H
