cmake_minimum_required(VERSION 3.10)
project(OakTree VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3")

find_package(OpenMP REQUIRED)
find_package (Eigen3 3.4 REQUIRED NO_MODULE)

add_library(octree "src/octree.cpp")
add_library(io "src/io.cpp")
add_executable(demo "src/demo.cpp")

target_link_libraries(octree PRIVATE OpenMP::OpenMP_CXX Eigen3::Eigen)
target_link_libraries(demo PUBLIC octree io Eigen3::Eigen)
include_directories("${PROJECT_SOURCE_DIR}/src" "${PROJECT_SOURCE_DIR}/external")