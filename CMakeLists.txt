cmake_minimum_required(VERSION 3.10)
project(OakTree VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(OpenMP REQUIRED)
find_package (Eigen3 3.4 REQUIRED NO_MODULE)
find_package(Python 3.9 COMPONENTS Interpreter Development.Module REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/nanobind)
add_library(octree "src/octree.cpp")
add_library(io "src/io.cpp")
add_executable(demo "src/demo.cpp")
nanobind_add_module(oaktree "src/oaktree.cpp")

set_target_properties(octree PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(io PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(octree PRIVATE OpenMP::OpenMP_CXX Eigen3::Eigen)
target_link_libraries(demo PUBLIC octree io Eigen3::Eigen)
target_link_libraries(oaktree PRIVATE octree io)
include_directories("${PROJECT_SOURCE_DIR}/src" "${PROJECT_SOURCE_DIR}/external")