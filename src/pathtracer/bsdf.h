#ifndef CGL_STATICSCENE_BSDF_H
#define CGL_STATICSCENE_BSDF_H

#include "util/vector3D.h"
#include "util/matrix3x3.h"

#include "util/image.h"

#include <algorithm>

namespace CGL {

// Helper math functions. Assume all vectors are in unit hemisphere //

inline double clamp (double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

inline double cos_theta(const Vector3D w) {
  return w.z;
}

HOST_DEVICE inline double abs_cos_theta(const Vector3D w) {
  return fabsf(w.z);
}

inline double sin_theta2(const Vector3D w) {
  return fmax(0.0, 1.0 - cos_theta(w) * cos_theta(w));
}

inline double sin_theta(const Vector3D w) {
  return sqrt(sin_theta2(w));
}

inline double cos_phi(const Vector3D w) {
  double sinTheta = sin_theta(w);
  if (sinTheta == 0.0) return 1.0;
  return clamp(w.x / sinTheta, -1.0, 1.0);
}

inline double sin_phi(const Vector3D w) {
  double sinTheta = sin_theta(w);
  if (sinTheta) return 0.0;
  return clamp(w.y / sinTheta, -1.0, 1.0);
}

DEVICE void make_coord_space(Matrix3x3& o2w, const Vector3D n);

/**
 * Interface for BSDFs.
 * BSDFs (Bidirectional Scattering Distribution Functions)
 * describe the ratio of incoming light scattered from
 * incident direction to outgoing direction.
 * Scene objects are initialized with a BSDF subclass, used
 * to represent the object's material and associated properties.
 */
class BSDF {
 public:
  const HDRImageBuffer* reflectanceMap;
  const HDRImageBuffer* normalMap;
  virtual ~BSDF() = default;

}; // class BSDF

/**
 * Diffuse BSDF.
 */
class DiffuseBSDF : public BSDF {
 public:

  /**
   * DiffuseBSDFs are constructed with a Vector3D as input,
   * which is stored into the member variable `reflectance`.
   */
  DiffuseBSDF(const Vector3D a) : reflectance(a) { }
  Vector3D reflectance;
}; // class DiffuseBSDF

/**
 * Microfacet BSDF.
 */

class MicrofacetBSDF : public BSDF {
public:

  MicrofacetBSDF(const Vector3D eta, const Vector3D k, double alpha)
    : eta(eta), k(k), alpha(alpha) { }

  double getTheta(const Vector3D w) {
    return acos(clamp(w.z, -1.0 + 1e-5, 1.0 - 1e-5));
  }

  double Lambda(const Vector3D w) {
    double theta = getTheta(w);
    double a = 1.0 / (alpha * tan(theta));
    return 0.5 * (erf(a) - 1.0 + exp(-a * a) / (a * PI));
  }

  Vector3D F(const Vector3D wi);

  double G(const Vector3D wo, const Vector3D wi);

  double D(const Vector3D h);

private:
  Vector3D eta, k;
  double alpha;
}; // class MicrofacetBSDF

/**
 * Mirror BSDF
 */
class MirrorBSDF : public BSDF {
 public:

  MirrorBSDF(const Vector3D reflectance) : reflectance(reflectance) { }

private:

  double roughness;
  Vector3D reflectance;

}; // class MirrorBSDF*/

/**
 * Refraction BSDF.
 */
class RefractionBSDF : public BSDF {
 public:

  RefractionBSDF(const Vector3D transmittance, double roughness, double ior)
    : transmittance(transmittance), roughness(roughness), ior(ior) { }

 private:

  double ior;
  double roughness;
  Vector3D transmittance;

}; // class RefractionBSDF

/**
 * Glass BSDF.
 */
class GlassBSDF : public BSDF {
 public:

  GlassBSDF(const Vector3D transmittance, const Vector3D reflectance,
            double roughness, double ior) :
    transmittance(transmittance), reflectance(reflectance),
    roughness(roughness), ior(ior) { }

 private:

  double ior;
  double roughness;
  Vector3D reflectance;
  Vector3D transmittance;

}; // class GlassBSDF

/**
 * Emission BSDF.
 */
class EmissionBSDF : public BSDF {
 public:

  EmissionBSDF(const Vector3D radiance) : radiance(radiance) { }

  Vector3D radiance;

}; // class EmissionBSDF

enum CudaBSDFType {
  CudaBSDFType_Diffuse,
  CudaBSDFType_Microfacet,
  CudaBSDFType_Mirror,
  CudaBSDFType_Refraction,
  CudaBSDFType_Glass,
  CudaBSDFType_Emission
};

struct CudaDiffuseBSDF {
  Vector3D reflectance;

  DEVICE Vector3D f(const Vector3D wo, const Vector3D wi) const;
  DEVICE Vector3D get_emission() const { return Vector3D(); }
};

struct CudaEmissionBSDF {
  Vector3D radiance;

  DEVICE Vector3D f(const Vector3D wo, const Vector3D wi) const;
  DEVICE Vector3D get_emission() const { return radiance; }
};

union BSDFData {
  BSDFData() {}
  CudaDiffuseBSDF diffuse;
  CudaEmissionBSDF emission;
};
struct CudaBSDF {
  CudaBSDFType type;
  BSDFData bsdf;
  CudaBSDF() : type(CudaBSDFType_Diffuse) {}
  CudaBSDF(const CudaBSDF& b) : type(b.type) {
    switch (type) {
      case CudaBSDFType_Diffuse:
        bsdf.diffuse = b.bsdf.diffuse;
        break;
      case CudaBSDFType_Emission:
        bsdf.emission = b.bsdf.emission;
        break;
      default:
        break;
    }
  }
  DEVICE Vector3D f(const Vector3D wo, const Vector3D wi) const {
    switch (type) {
      case CudaBSDFType_Diffuse:
        return bsdf.diffuse.f(wo, wi);
      case CudaBSDFType_Emission:
        return bsdf.emission.f(wo, wi);
      default:
        return Vector3D();
    }
  }

  DEVICE Vector3D get_emission() const {
    switch (type) {
      case CudaBSDFType_Emission:
        return bsdf.emission.get_emission();
      default:
        return Vector3D();
    }
  }
};


}  // namespace CGL

#endif  // CGL_STATICSCENE_BSDF_H
