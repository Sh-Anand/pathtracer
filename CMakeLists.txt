cmake_minimum_required(VERSION 3.8)
project(Assignment3)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
set(APPLICATION_3_1_SOURCE

    # Scene Object & Structure
    src/scene/sphere.cpp
    src/scene/triangle.cpp
    src/scene/light.cpp
    src/scene/bvh.cpp
    src/scene/bbox.cpp

    # Pathtracer
    src/pathtracer/camera.cpp
    src/pathtracer/bsdf.cpp
    src/pathtracer/pathtracer.cpp
)

set(APPLICATION_3_2_SOURCE
    src/scene/object.cpp

    # Collada Parser
    src/scene/collada/collada.cpp
    src/scene/collada/camera_info.cpp
    src/scene/collada/light_info.cpp
    src/scene/collada/sphere_info.cpp
    src/scene/collada/polymesh_info.cpp
    src/scene/collada/material_info.cpp

    # Dynamic Scene
    src/scene/gl_scene/mesh.cpp
    src/scene/gl_scene/scene.cpp
    src/scene/gl_scene/sphere.cpp

    # MeshEdit
    src/util/halfEdgeMesh.cpp

    src/pathtracer/sampler.cpp
    src/pathtracer/advanced_bsdf.cpp
    src/scene/environment_light.cpp
    src/pathtracer/camera_lens.cpp
    src/pathtracer/raytraced_renderer.cpp

    # misc
    src/util/lodepng.cpp

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
    src/scene/gl_scene/ambient_light.h
    src/scene/gl_scene/area_light.h
    src/scene/gl_scene/directional_light.h
    src/scene/gl_scene/environment_light.h
    src/scene/gl_scene/material.h
    src/scene/gl_scene/mesh.h
    src/scene/gl_scene/point_light.h
    src/scene/gl_scene/scene.h
    src/scene/gl_scene/sphere.h
    src/scene/gl_scene/spot_light.h
    # Scene Object & Structure
    src/scene/aggregate.h
    src/scene/bbox.h
    src/scene/bvh.h
    src/scene/environment_light.h
    src/scene/light.h
    src/scene/object.h
    src/scene/primitive.h
    src/scene/scene.h
    src/scene/sphere.h
    src/scene/triangle.h
    # MeshEdit
    src/util/halfEdgeMesh.h
    src/util/image.h
    src/util/mutablePriorityQueue.h
    src/util/random_util.h
    src/util/work_queue.h
    # Pathtracer
    src/pathtracer/bsdf.h
    src/pathtracer/camera.h
    src/pathtracer/intersection.h
    src/pathtracer/pathtracer.h
    src/pathtracer/ray.h
    src/pathtracer/raytraced_renderer.h
    src/pathtracer/sampler.h
    # misc
    src/util/lodepng.h
    # Application
    src/application/application.h
    src/application/renderer.h
)

if (WIN32)
    list(APPEND APPLICATION_3_1_SOURCE src/util/win32/getopt.c)
endif()

if (WIN32 OR APPLE)
  add_library(pt31 STATIC ${APPLICATION_3_1_SOURCE} ${APPLICATION_HEADERS})
else()
  add_library(pt31 SHARED ${APPLICATION_3_1_SOURCE} ${APPLICATION_HEADERS})
endif()

target_include_directories(pt31 PUBLIC src)

add_executable(pathtracer ${APPLICATION_3_2_SOURCE} ${APPLICATION_HEADERS})
target_include_directories(pathtracer PUBLIC src)

target_link_libraries(pt31 PUBLIC CGL)
target_link_libraries(pathtracer PUBLIC pt31)

set(CGL_INCLUDE_DIRS CGL/include)

#-------------------------------------------------------------------------------
# Find dependencies
#-------------------------------------------------------------------------------
add_subdirectory(CGL)
target_include_directories(pt31 PUBLIC ${CGL_INCLUDE_DIRS})
target_include_directories(pathtracer PUBLIC ${CGL_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CGL_CXX_FLAGS}")

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
