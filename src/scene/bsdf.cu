#include "bsdf.h"

DEVICE void make_coord_space(Matrix3x3 &o2w, const Vector3D n) {

  Vector3D z = Vector3D(n.x, n.y, n.z);
  Vector3D h = z;
  if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
    h.x = 1.0;
  else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
    h.y = 1.0;
  else
    h.z = 1.0;

  z.normalize();
  Vector3D y = cross(h, z);
  y.normalize();
  Vector3D x = cross(z, y);
  x.normalize();

  o2w[0] = x;
  o2w[1] = y;
  o2w[2] = z;
}

// following code copied from https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#appendix-b-brdf-implementation

// helper: GGX normal distribution
DEVICE float D_compute(float a, float NoH_raw) {
  float NoH = fmaxf(NoH_raw, 0.0); // Heaviside step function
  float a2 = a*a;
  float denom = NoH*NoH*(a2 - 1.0) + 1.0;
  return a2 / (PI * denom * denom);
}

DEVICE float G_compute(
    float a,    // α = roughness²
    float NoV,  // max(0, N·V)
    float NoL,  // max(0, N·L)
    float VoH,  // dot(V, H)
    float LoH   // dot(L, H)
) {
  // 1) visibility of microfacet only if H·V>0 and H·L>0
  if (VoH <= 0.0 || LoH <= 0.0) return 0.0;

  // 2) denominator terms per glTF spec:
  //    G₁(X) = (2·X) / (X + sqrt(α² + (1−α²)·X²))
  float a2       = a * a;
  float denomV  = NoV + sqrt(a2 + (1.0 - a2) * NoV * NoV);
  float denomL  = NoL + sqrt(a2 + (1.0 - a2) * NoL * NoL);
  float GV      = (2.0 * NoV) / denomV;
  float GL      = (2.0 * NoL) / denomL;
  return GV * GL;
}