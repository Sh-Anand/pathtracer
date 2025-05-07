#ifndef CGL_PATHTRACER_H
#define CGL_PATHTRACER_H

#include "scene/bvh.h"
#include "scene/camera.h"
#include "scene/light.h"
#include "scene/texture.h"
#include "target/intersection.h"
#include "util/reservoir.h"


class PathTracer {
public:
    DEVICE __inline__ void perturb_normal(CudaIntersection &isect);

    DEVICE Vector3D f(const CudaIntersection &isect, 
                                    const Vector3D &wo,
                                    const Vector3D &wi,
                                    float *occlusion);
    DEVICE Vector3D sample_f(const CudaIntersection &isect,
                                            const Vector3D &wo,
                                            Vector3D *wi,
                                            float *pdf,
                                            float *occlusion,
                                            bool   *is_delta,
                                            RNGState &rand_state);
    DEVICE __inline__ float bsdf_pdf(const CudaIntersection &isect,
                                const Vector3D &wo,
                                const Vector3D &wi);

    DEVICE Vector3D estimate_direct_lighting_importance(Ray& r, const CudaIntersection& isect);
    DEVICE Vector3D at_least_one_bounce_radiance(Ray& r, const CudaIntersection& isect);

    // ReSTIR GI //
    Sample* initialSampleBuffer;
    Reservoir* temporalReservoirBuffer;
    Reservoir* spatialReservoirBuffer;
    DEVICE void temporal_resampling(uint16_t x, uint16_t y);
    DEVICE void spatial_resampling(uint16_t x, uint16_t y);
    DEVICE void render_final_sample(uint16_t x, uint16_t y);

    /**
     * Trace a camera ray given by the pixel coordinate.
     */
    DEVICE void raytrace_pixel(uint16_t x, uint16_t y);

    // Integrator sampling settings //

    uint16_t max_ray_depth; ///< maximum allowed ray depth (applies to all rays)
    uint16_t ns_aa;         ///< number of camera rays in one pixel (along one axis)
    uint16_t ns_area_light; ///< number samples per area light source
    uint8_t* rays_traced;

    // Components //

    const BVHCuda* bvh;                 ///< BVH accelerator aggregate
    HDRImageBuffer sampleBuffer;   ///< sample buffer

    CudaCamera camera;       ///< current camera

    RNGState* rand_states;       ///< random state for each thread

    // Lights
    const CudaLight *lights; 
    uint16_t num_lights;

    // BSDFs
    const CudaBSDF *bsdfs;
    // Textures
    const CudaTexture *textures;
};

#endif  // CGL_RAYTRACER_H
