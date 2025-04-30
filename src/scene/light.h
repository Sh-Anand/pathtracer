#ifndef CGL_STATICSCENE_LIGHT_H
#define CGL_STATICSCENE_LIGHT_H

#include "scene/primitive.h"
#include "util/vector3D.h"

namespace CGL { namespace SceneObjects {

  enum CudaLightType {
    NONE,
    AMBIENT,
    DIRECTIONAL,
    AREA,
    POINT,
    SPOT,
    TRIANGLELight
  };

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

struct CudaTriangleLight {
  CudaTriangleLight(const Vector3D rad, 
                const Vector3D dir, 
                const CudaTriangle tri) 
      : radiance(rad), 
        direction(dir), 
        triangle(tri),  
        area(tri.area()) {}
  DEVICE bool is_delta_light() const { return false; }

  Vector3D radiance;
  Vector3D direction;
  CudaTriangle triangle;

  double area;
};

union LightData {
  LightData() {}
  CudaDirectionalLight directional;
  CudaPointLight point;
  CudaAreaLight area;
  CudaTriangleLight triangle;
};

struct CudaLight {
  CudaLightType type;
  LightData light;
  CudaLight() : type(CudaLightType::NONE) {}
  CudaLight(const CudaLight& l) : type(l.type) {
    switch (type) {
      case CudaLightType::DIRECTIONAL:
        light.directional = l.light.directional;
        break;
      case CudaLightType::POINT:
        light.point = l.light.point;
        break;
      case CudaLightType::AREA:
        light.area = l.light.area;
        break;
      case CudaLightType::TRIANGLELight:
        light.triangle = l.light.triangle;
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
        case CudaLightType::TRIANGLELight:
        return light.triangle.is_delta_light();
      default:
        return false;
    }
  }
};

} // namespace SceneObjects
} // namespace CGL

#endif  // CGL_STATICSCENE_BSDF_H
