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

    # Collada Parser
    src/scene/collada/collada.cpp

    # Dynamic Scene
    src/scene/gl_scene/mesh.cpp
    src/scene/gl_scene/scene.cpp

    # MeshEdit
    src/util/halfEdgeMesh.cpp
    src/util/matrix3x3.cpp
    src/util/matrix3x3.cu
    src/util/matrix4x4.cpp
    src/util/reservoir.cu
    src/util/tinyxml2.cpp
    src/util/vector2D.cpp
    src/util/vector3D.cpp
    src/util/vector4D.cpp

    src/pathtracer/advanced_bsdf.cpp
    src/pathtracer/raytraced_renderer.cpp
    src/pathtracer/raytraced_renderer.cu
    src/pathtracer/ray.cu


    # misc
    src/util/lodepng.cpp
    src/util/image.cu

    # Application
    src/application/application.cpp
    src/application/main.cpp
)

set(APPLICATION_HEADERS
    # Collada Parser
    src/scene/collada/camera_info.h
    src/scene/collada/collada_info.h
    src/scene/collada/collada.h
    src/scene/collada/light_info.h
    src/scene/collada/material_info.h
    src/scene/collada/polymesh_info.h
    src/scene/collada/sphere_info.h
    # Dynamic Scene
    src/scene/gl_scene/mesh.h
    src/scene/gl_scene/scene.h
    # Scene Object & Structure
    src/scene/bbox.h
    src/scene/bvh.h
    src/scene/light.h
    src/scene/primitive.h
    # MeshEdit
    src/util/gpu_rand.h
    src/util/halfEdgeMesh.h
    src/util/image.h
    src/util/matrix3x3.h
    src/util/matrix4x4.h
    src/util/reservoir.h
    src/util/vector2D.h
    src/util/vector3D.h
    src/util/vector4D.h

    # tinygltf
    src/util/json.hpp
    src/util/stb_image.h
    src/util/stb_image_write.h
    src/util/tiny_gltf.h

    # Pathtracer
    src/pathtracer/bsdf.h
    src/pathtracer/camera.h
    src/pathtracer/intersection.h
    src/pathtracer/pathtracer.h
    src/pathtracer/ray.h
    src/pathtracer/raytraced_renderer.h
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
    OpenGL::GL
    OpenGL::GLU
    pthread
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
