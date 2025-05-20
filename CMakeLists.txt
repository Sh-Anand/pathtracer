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
  # switch to Debug build
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)

  # C++ debug flags: symbols, no opt, extra warnings
  set(CMAKE_CXX_FLAGS_DEBUG
      "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall -Wextra -Wpedantic")

  # CUDA debug flags: device debug info, line info
  set(CMAKE_CUDA_FLAGS_DEBUG
      "${CMAKE_CUDA_FLAGS_DEBUG} -G -lineinfo")
endif()

#-------------------------------------------------------------------------------
# Set target
#-------------------------------------------------------------------------------
set(APPLICATION_SOURCE
    # Scene Object & Structure
    src/scene/bvh.cpp
    src/scene/bvh.cu

    # Pathtracer
    src/scene/camera.cpp

    src/host/raytraced_renderer.cpp
    src/host/raytraced_renderer.cu

    # Application
    src/host/application.cpp
)

set(APPLICATION_HEADERS
    src/scene/bvh.h
    src/scene/light.h
    src/scene/geometry.h
    src/util/gpu_rand.h
    src/util/matrix.h
    src/util/reservoir.h
    src/util/vector.h
    src/scene/camera.h
    src/scene/material.h
    src/target/pathtracer.h
    src/host/raytraced_renderer.h
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
