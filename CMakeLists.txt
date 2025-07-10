cmake_minimum_required(VERSION 3.14)       # FetchContent needs ≥3.11
project(pathtracer LANGUAGES CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_DEBUG "Build with debug settings" OFF)
if (BUILD_DEBUG)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
  set(CMAKE_CXX_FLAGS_DEBUG
      "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall -Wextra -Wpedantic")
endif()

set(APPLICATION_SOURCE
    src/host/application.cpp
)

#  Assimp: find it or fetch-&-build it
find_package(assimp CONFIG QUIET)

if (NOT assimp_FOUND)
  message(STATUS "Assimp not found – downloading and building it ...")

  include(FetchContent)

  # Options to slim down and make static
  set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "" FORCE)
  set(ASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT ON CACHE BOOL "" FORCE)
  set(ASSIMP_BUILD_ALL_EXPORTERS_BY_DEFAULT OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)   # force static .a

  FetchContent_Declare(
    assimp
    GIT_REPOSITORY https://github.com/assimp/assimp.git
    GIT_TAG        v5.3.1
  )
  FetchContent_MakeAvailable(assimp)

  if (NOT TARGET assimp::assimp)
    add_library(assimp::assimp ALIAS assimp)
  endif()
endif()

add_executable(baker ${APPLICATION_SOURCE})
target_include_directories(baker PUBLIC src)
target_link_libraries(baker PRIVATE assimp::assimp)
