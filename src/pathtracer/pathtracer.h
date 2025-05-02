#ifndef CGL_PATHTRACER_H
#define CGL_PATHTRACER_H

#include "scene/bvh.h"
using CGL::SceneObjects::BVHCuda;

#include "pathtracer/intersection.h"

#include "util/reservoir.h"

#include "scene/light.h"
using CGL::SceneObjects::CudaLight;

#include "pathtracer/camera.h"

#include "pathtracer/texture.h"
namespace CGL {

    class PathTracer {
    public:
        /**
         * Sets the pathtracer's frame size. If in a running state (VISUALIZE,
         * RENDERING, or DONE), transitions to READY b/c a changing window size
         * would invalidate the output. If in INIT and configuration is done,
         * transitions to READY.
         * \param width width of the frame
         * \param height height of the frame
         */
        void set_frame_size(uint16_t width, uint16_t height);

        void write_to_framebuffer(HDRImageBuffer &buffer, ImageBuffer& framebuffer, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

        DEVICE __inline__ Vector3D get_emission(const CudaIntersection &isect);
        DEVICE __inline__ void perturb_normal(CudaIntersection &isect);

        DEVICE Vector3D f(const CudaIntersection &isect, 
                                     const Vector3D &wo,
                                     const Vector3D &wi,
                                     double *occlusion);
        DEVICE Vector3D sample_f(const CudaIntersection &isect,
                                                const Vector3D &wo,
                                                Vector3D *wi,
                                                double *pdf,
                                                double *occlusion,
                                                RNGState &rand_state);
        DEVICE __inline__ double bsdf_pdf(const CudaIntersection &isect,
                                  const Vector3D &wo,
                                  const Vector3D &wi);

        DEVICE Vector3D estimate_direct_lighting_importance(Ray& r, const CudaIntersection& isect);
        DEVICE Vector3D est_radiance_global_illumination(Ray& r);
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

        // Components //

        BVHCuda* bvh;                 ///< BVH accelerator aggregate
        HDRImageBuffer sampleBuffer;   ///< sample buffer

        CudaCamera camera;       ///< current camera

        RNGState* rand_states;       ///< random state for each thread

        // Lights
        CudaLight *lights; 
        uint16_t num_lights;

        // BSDFs
        CudaBSDF *bsdfs;
        // Textures
        CudaTexture *textures;
    };

}  // namespace CGL

#endif  // CGL_RAYTRACER_H
