cmake_minimum_required(VERSION 3.8)
project(pathtracer LANGUAGES CXX CUDA) 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

#-------------------------------------------------------------------------------
# Build options
#-------------------------------------------------------------------------------
option(BUILD_DEBUG     "Build with debug settings"    OFF)
option(BUILD_DOCS      "Build documentation"          OFF)


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
    src/scene/bbox.cu
    src/scene/primitive.cu

    # Pathtracer
    src/pathtracer/camera.cpp
    src/pathtracer/camera.cu
    src/pathtracer/bsdf.cu
    src/pathtracer/pathtracer.cpp
    src/pathtracer/pathtracer.cu

    # MeshEdit
    src/util/matrix3x3.cpp
    src/util/matrix3x3.cu
    src/util/matrix4x4.cpp
    src/util/reservoir.cu
    src/util/vector2D.cpp
    src/util/vector3D.cpp
    src/util/vector4D.cpp
    src/util/quaternion.cpp

    src/pathtracer/raytraced_renderer.cpp
    src/pathtracer/raytraced_renderer.cu
    src/pathtracer/ray.cu
    src/pathtracer/texture.cu

    # misc
    src/util/lodepng.cpp

    # Application
    src/application/application.cpp
    src/application/main.cpp

    src/scene/light.cu
)

set(APPLICATION_HEADERS
    # Collada Parser
    src/scene/camera_info.h
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
    src/pathtracer/bsdf.h
    src/pathtracer/camera.h
    src/pathtracer/intersection.h
    src/pathtracer/pathtracer.h
    src/pathtracer/ray.h
    src/pathtracer/raytraced_renderer.h
    src/pathtracer/texture.h
    # misc
    src/util/lodepng.h
    # Application
    src/application/application.h
)

add_executable(pathtracer ${APPLICATION_SOURCE} ${APPLICATION_HEADERS})
target_include_directories(pathtracer PUBLIC src)

#-------------------------------------------------------------------------------
# Find dependencies
#-------------------------------------------------------------------------------
target_include_directories(pathtracer PUBLIC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)

target_link_libraries(pathtracer
    PUBLIC
)

#-------------------------------------------------------------------------------
# Add subdirectories
#-------------------------------------------------------------------------------

# build documentation
if(BUILD_DOCS)
  find_package(DOXYGEN)
  if(DOXYGEN_FOUND AND BUILD_DOCS)
    add_subdirectory(docs)
  endif()
endif()

# Install settings
set(CMAKE_INSTALL_PREFIX "${Assignment1_SOURCE_DIR}/")
