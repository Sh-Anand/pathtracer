#ifndef CGL_STATICSCENE_LIGHT_H
#define CGL_STATICSCENE_LIGHT_H

#include "util/vector3D.h"
#include "util/matrix3x3.h"
#include "pathtracer/sampler.h" // UniformHemisphereSampler3D, UniformGridSampler2D
#include "util/image.h"   // HDRImageBuffer

#include "scene.h"  // SceneLight
#include "object.h" // Mesh, SphereObject

namespace CGL { namespace SceneObjects {

// Directional Light //

class DirectionalLight : public SceneLight {
 public:
  DirectionalLight(const Vector3D rad, const Vector3D lightDir);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return true; }

  Vector3D radiance;
  Vector3D dirToLight;

}; // class Directional Light

// Infinite Hemisphere Light //

class InfiniteHemisphereLight : public SceneLight {
 public:
  InfiniteHemisphereLight(const Vector3D rad);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return false; }

  Vector3D radiance;
  Matrix3x3 sampleToWorld;
  UniformHemisphereSampler3D sampler;

}; // class InfiniteHemisphereLight


// Point Light //

class PointLight : public SceneLight {
 public: 
  PointLight(const Vector3D rad, const Vector3D pos);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return true; }

  Vector3D radiance;
  Vector3D position;
  
}; // class PointLight

// Spot Light //

class SpotLight : public SceneLight {
 public:
  SpotLight(const Vector3D rad, const Vector3D pos, 
            const Vector3D dir, double angle);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return true; }

  Vector3D radiance;
  Vector3D position;
  Vector3D direction;
  double angle;

}; // class SpotLight

// Area Light //

class AreaLight : public SceneLight {
 public:
  AreaLight(const Vector3D rad, 
            const Vector3D pos,   const Vector3D dir, 
            const Vector3D dim_x, const Vector3D dim_y);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return false; }

  Vector3D radiance;
  Vector3D position;
  Vector3D direction;
  Vector3D dim_x;
  Vector3D dim_y;
  UniformGridSampler2D sampler;
  double area;

}; // class AreaLight

// Sphere Light //

class SphereLight : public SceneLight {
 public:
  SphereLight(const Vector3D rad, const SphereObject* sphere);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return false; }

  const SphereObject* sphere;
  Vector3D radiance;
  UniformHemisphereSampler3D sampler;

}; // class SphereLight

// Mesh Light

class MeshLight : public SceneLight {
 public:
  MeshLight(const Vector3D rad, const Mesh* mesh);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return false; }

  const Mesh* mesh;
  Vector3D radiance;

}; // class MeshLight

enum CudaLightType {
  CudaLightType_Directional = 0,
  CudaLightType_InfiniteHemisphere = 1,
  CudaLightType_Point = 2,
  CudaLightType_Spot = 3,
  CudaLightType_Area = 4,
  CudaLightType_Sphere = 5,
  CudaLightType_Mesh = 6,
};

struct CudaDirectionalLight {
  CudaDirectionalLight(DirectionalLight &light) : radiance(light.radiance), dirToLight(light.dirToLight) {}
  DEVICE bool is_delta_light() const { return true; }
  Vector3D radiance;
  Vector3D dirToLight;
};

struct CudaPointLight {
  CudaPointLight(PointLight &light) : radiance(light.radiance), position(light.position) {}
  DEVICE bool is_delta_light() const { return true; }
  Vector3D radiance;
  Vector3D position;
};

struct CudaAreaLight {
  CudaAreaLight(AreaLight &light) : radiance(light.radiance), position(light.position), direction(light.direction), dim_x(light.dim_x), dim_y(light.dim_y), area(light.area) {}
  DEVICE bool is_delta_light() const { return false; }
  Vector3D radiance;
  Vector3D position;
  Vector3D direction;
  Vector3D dim_x;
  Vector3D dim_y;
  double area;
};

struct CudaLight {
  uint16_t idx;
  CudaLightType type;
};

struct CudaLightBundle {
  CudaLightBundle() : directional_lights(nullptr), num_directional_lights(0),
                      point_lights(nullptr), num_point_lights(0),
                      area_lights(nullptr), num_area_lights(0) {}
  DEVICE bool is_delta_light(CudaLight light) const;
  CudaDirectionalLight* directional_lights;
  uint16_t num_directional_lights;
  CudaPointLight* point_lights;
  uint16_t num_point_lights;
  CudaAreaLight* area_lights;
  uint16_t num_area_lights;
};

} // namespace SceneObjects
} // namespace CGL

#endif  // CGL_STATICSCENE_BSDF_H
