cmake_minimum_required(VERSION 3.8)
project(pathtracer LANGUAGES CXX CUDA) 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

#-------------------------------------------------------------------------------
# Build options
#-------------------------------------------------------------------------------
option(BUILD_DEBUG     "Build with debug settings"    OFF)

set(BUILD_DEBUG ${BUILD_DEBUG} CACHE BOOL "Build debug" FORCE)

if (BUILD_DEBUG)
  set(CMAKE_BUILD_TYPE Debug)
endif()

#-------------------------------------------------------------------------------
# Set target
#-------------------------------------------------------------------------------
set(APPLICATION_SOURCE

    # Scene Object & Structure
    src/scene/bvh.cpp
    src/scene/bvh.cu
    src/scene/primitive.cu

    # Pathtracer
    src/scene/camera.cpp
    src/scene/camera.cu
    src/scene/bsdf.cu
    src/target/pathtracer.cu

    # MeshEdit
    src/util/matrix3x3.cpp
    src/util/matrix3x3.cu
    src/util/matrix4x4.cpp
    src/util/reservoir.cu
    src/util/vector2D.cpp
    src/util/vector4D.cpp
    src/util/quaternion.cpp

    src/host/raytraced_renderer.cpp
    src/host/raytraced_renderer.cu
    src/scene/ray.cu
    src/scene/texture.cu

    # misc
    src/util/lodepng.cpp

    # Application
    src/host/application.cpp

    src/scene/light.cu
)

set(APPLICATION_HEADERS
    # Scene Object & Structure
    src/scene/bbox.h
    src/scene/bvh.h
    src/scene/light.h
    src/scene/primitive.h
    # MeshEdit
    src/util/gpu_rand.h
    src/util/image.h
    src/util/matrix3x3.h
    src/util/matrix4x4.h
    src/util/reservoir.h
    src/util/vector2D.h
    src/util/vector3D.h
    src/util/vector4D.h
    # Pathtracer
    src/scene/bsdf.h
    src/scene/camera.h
    src/target/intersection.h
    src/target/pathtracer.h
    src/scene/ray.h
    src/host/raytraced_renderer.h
    src/scene/texture.h
    # misc
    src/util/lodepng.h 
)

add_executable(pathtracer ${APPLICATION_SOURCE} ${APPLICATION_HEADERS})
target_include_directories(pathtracer PUBLIC src)

#-------------------------------------------------------------------------------
# Find dependencies
#-------------------------------------------------------------------------------
target_include_directories(pathtracer PUBLIC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

# Install settings
set(CMAKE_INSTALL_PREFIX "${Assignment1_SOURCE_DIR}/")