cmake_minimum_required(VERSION 3.8)
project(pathtracer LANGUAGES CXX CUDA) 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

set(APPLICATION_SOURCE
    src/scene/bvh.cpp
    src/scene/bvh.cu
    src/host/raytraced_renderer.cpp
    src/host/raytraced_renderer.cu
    src/host/application.cpp
)

find_package(assimp REQUIRED)
find_package(OpenEXR REQUIRED)

add_executable(pathtracer ${APPLICATION_SOURCE})
target_include_directories(pathtracer PUBLIC src)
target_link_libraries(pathtracer
    assimp::assimp
    OpenEXR::OpenEXR)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
