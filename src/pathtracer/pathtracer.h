#ifndef CGL_PATHTRACER_H
#define CGL_PATHTRACER_H

#include "CGL/timer.h"

#include "scene/bvh.h"
using CGL::SceneObjects::BVHCuda;

#include "pathtracer/sampler.h"
#include "pathtracer/intersection.h"
#include "pathtracer/camera.h"
#include "scene/scene.h"
using CGL::SceneObjects::Scene;

#include "scene/environment_light.h"
using CGL::SceneObjects::EnvironmentLight;

using CGL::SceneObjects::BVHCuda;

#include "util/reservoir.h"

#include "scene/light.h"
using CGL::SceneObjects::CudaLight;
using CGL::SceneObjects::CudaLightBundle;
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
        void set_frame_size(size_t width, size_t height);

        void write_to_framebuffer(ImageBuffer& framebuffer, size_t x0, size_t y0, size_t x1, size_t y1);

        /**
         * If the pathtracer is in READY, delete all internal data, transition to INIT.
         */
        void clear();

        Vector3D zero_bounce_radiance(const Ray& r, const SceneObjects::CudaIntersection& isect);
        Vector3D one_bounce_radiance(const Ray& r, const SceneObjects::CudaIntersection& isect);
        Vector3D at_least_one_bounce_radiance(const Ray& r, const SceneObjects::CudaIntersection& isect);

        // ReSTIR GI //
        std::vector<Sample> initialSampleBuffer;
        std::vector<Reservoir> temporalReservoirBuffer;
        std::vector<Reservoir> spatialReservoirBuffer;
        void temporal_resampling(size_t x, size_t y);
        void spatial_resampling(size_t x, size_t y);
        void render_final_sample(size_t x, size_t y);
    
        /**
         * Trace a camera ray given by the pixel coordinate.
         */
        void raytrace_pixel(size_t x, size_t y);

        // Integrator sampling settings //

        size_t max_ray_depth; ///< maximum allowed ray depth (applies to all rays)
        size_t isAccumBounces; ///< number of bounces to accumulate
        size_t ns_aa;         ///< number of camera rays in one pixel (along one axis)
        size_t ns_area_light; ///< number samples per area light source
        size_t ns_diff;       ///< number of samples - diffuse surfaces
        size_t ns_glsy;       ///< number of samples - glossy surfaces
        size_t ns_refr;       ///< number of samples - refractive surfaces

        // Components //

        BVHCuda* bvh;                 ///< BVH accelerator aggregate
        EnvironmentLight* envLight;    ///< environment map
        Sampler2D* gridSampler;        ///< samples unit grid
        Sampler3D* hemisphereSampler;  ///< samples unit hemisphere
        HDRImageBuffer sampleBuffer;   ///< sample buffer
        Timer timer;                   ///< performance test timer

        Scene* scene;         ///< current scene
        Camera* camera;       ///< current camera

        // Lights
        CudaLight *lights; 
        CudaLightBundle *light_data ;
        size_t num_lights;
    };

}  // namespace CGL

#endif  // CGL_RAYTRACER_H
