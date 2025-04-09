#include <curand_kernel.h>

#include "light.h"

namespace CGL { namespace SceneObjects {

DEVICE bool CudaLightBundle::is_delta_light(CudaLight light) const {
  switch (light.type) {
    case CudaLightType_Directional:
      return directional_lights[light.idx].is_delta_light();
    case CudaLightType_Point:
      return point_lights[light.idx].is_delta_light();
    case CudaLightType_Area:
      return area_lights[light.idx].is_delta_light();
    default:
      return false;
  }
  return false;
}

}
}