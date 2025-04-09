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
struct curandState;
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
        void set_frame_size(uint16_t width, uint16_t height);

        void write_to_framebuffer(HDRImageBuffer &buffer, ImageBuffer& framebuffer, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

        /**
         * If the pathtracer is in READY, delete all internal data, transition to INIT.
         */
        void clear();

        // void autofocus(Vector2D loc);

        /**
         * Trace an ray in the scene.
         */
        DEVICE Vector3D estimate_direct_lighting_importance(Ray& r, const SceneObjects::CudaIntersection& isect);

        DEVICE Vector3D est_radiance_global_illumination(Ray& r);
        DEVICE Vector3D at_least_one_bounce_radiance(Ray& r, const SceneObjects::CudaIntersection& isect);
        
        DEVICE Vector3D p_sample_L(const CudaLight light, const Vector3D p,
                             Vector3D* wi, double* distToLight,
                             double* pdf, curandState *rand_state);
        DEVICE Vector3D p_sample_f (CudaBSDF bsdf, const Vector3D wo, Vector3D *wi, double* pdf, curandState *rand_state);

        /**
         * Trace a camera ray given by the pixel coordinate.
         */
        DEVICE void raytrace_pixel(uint16_t x, uint16_t y);

        // Integrator sampling settings //

        uint16_t max_ray_depth; ///< maximum allowed ray depth (applies to all rays)
        uint16_t ns_aa;         ///< number of camera rays in one pixel (along one axis)
        uint16_t ns_area_light; ///< number samples per area light source
        uint16_t ns_diff;       ///< number of samples - diffuse surfaces
        uint16_t ns_glsy;       ///< number of samples - glossy surfaces
        uint16_t ns_refr;       ///< number of samples - refractive surfaces

        uint16_t samplesPerBatch;
        double maxTolerance;
        bool direct_hemisphere_sample; ///< true if sampling uniformly from hemisphere for direct lighting. Otherwise, light sample

        // Components //

        BVHCuda* bvh;                 ///< BVH accelerator aggregate
        HDRImageBuffer sampleBuffer;   ///< sample buffer
        curandState* rand_states;       ///< random state for each thread

        CudaCamera camera;       ///< current camera

        // Lights
        CudaLight *lights; 
        CudaLightBundle *light_data ;
        uint16_t num_lights;

        // Tonemapping Controls //

        double tm_gamma;                           ///< gamma
        double tm_level;                           ///< exposure level
        double tm_key;                             ///< key value
        double tm_wht;                             ///< white point
    };

}  // namespace CGL

#endif  // CGL_RAYTRACER_H
