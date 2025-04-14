#ifndef CGL_PATHTRACER_H
#define CGL_PATHTRACER_H

#include "scene/bvh.h"
using CGL::SceneObjects::BVHCuda;

#include "pathtracer/sampler.h"
#include "pathtracer/intersection.h"

#include "application/renderer.h"

#include "scene/scene.h"
using CGL::SceneObjects::Scene;

#include "scene/environment_light.h"
using CGL::SceneObjects::EnvironmentLight;

#include "scene/light.h"
using CGL::SceneObjects::CudaLight;
using CGL::SceneObjects::CudaLightBundle;

#ifdef __CUDACC__
#include <curand_kernel.h>
#else
struct curandStatePhilox4_32_10;
#endif
namespace CGL {

    class PathTracer {
    public:
        PathTracer();
        ~PathTracer();

        /**
         * Sets the pathtracer's frame size. If in a running state (VISUALIZE,
         * RENDERING, or DONE), transitions to READY b/c a changing window size
         * would invalidate the output. If in INIT and configuration is done,
         * transitions to READY.
         * \param width width of the frame
         * \param height height of the frame
         */
        void set_frame_size(uint32_t width, uint32_t height);

        void write_to_framebuffer(HDRImageBuffer &buffer, ImageBuffer& framebuffer, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1);

        /**
         * If the pathtracer is in READY, delete all internal data, transition to INIT.
         */
        void clear();

        // void autofocus(Vector2D loc);

        /**
         * Trace an ray in the scene.
         */
        DEVICE Vector3D estimate_direct_lighting_importance(Ray& r, const SceneObjects::CudaIntersection& isect, curandStatePhilox4_32_10 *rand_state);
        DEVICE Vector3D at_least_one_bounce_radiance(Ray& r, const SceneObjects::CudaIntersection& isect, curandStatePhilox4_32_10 *rand_state);
        
        DEVICE Vector3D p_sample_L(const CudaLight light, const Vector3D p,
                             Vector3D* wi, float* distToLight,
                             float* pdf, curandStatePhilox4_32_10 *rand_state);
        DEVICE Vector3D p_sample_f (CudaBSDF bsdf, const Vector3D wo, Vector3D *wi, float* pdf, curandStatePhilox4_32_10 *rand_state);

        /**
         * Trace a camera ray given by the pixel coordinate.
         */
        DEVICE void raytrace_pixel(uint32_t x, uint32_t y);

        // Integrator sampling settings //

        uint32_t max_ray_depth; ///< maximum allowed ray depth (applies to all rays)
        uint32_t ns_aa;         ///< number of camera rays in one pixel (along one axis)
        uint32_t ns_area_light; ///< number samples per area light source
        uint32_t ns_diff;       ///< number of samples - diffuse surfaces
        uint32_t ns_glsy;       ///< number of samples - glossy surfaces
        uint32_t ns_refr;       ///< number of samples - refractive surfaces

        uint32_t samplesPerBatch;
        float maxTolerance;
        bool direct_hemisphere_sample; ///< true if sampling uniformly from hemisphere for direct lighting. Otherwise, light sample

        // Components //

        BVHCuda* bvh;                 ///< BVH accelerator aggregate
        HDRImageBuffer sampleBuffer;   ///< sample buffer

        CudaCamera camera;       ///< current camera

        // Lights
        CudaLight *lights; 
        CudaLightBundle *light_data ;
        uint32_t num_lights;

        // Tonemapping Controls //

        float tm_gamma;                           ///< gamma
        float tm_level;                           ///< exposure level
        float tm_key;                             ///< key value
        float tm_wht;                             ///< white point
    };

}  // namespace CGL

#endif  // CGL_RAYTRACER_H
