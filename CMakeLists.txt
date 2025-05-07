cmake_minimum_required(VERSION 3.8)
project(pathtracer LANGUAGES CXX CUDA)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

option(BUILD_DEBUG "Build with debug settings" OFF)

if (BUILD_DEBUG)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# automatically grab every .cpp/.cu under src/
file(GLOB_RECURSE
  PATHTRACER_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu"
)

add_executable(pathtracer ${PATHTRACER_SOURCES})
target_include_directories(pathtracer PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")

