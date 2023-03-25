cmake_minimum_required(VERSION 3.10)
project(OakTree VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenMP REQUIRED)

add_library(octree "src/octree.cpp")
add_library(io "src/io.cpp")
add_executable(demo "src/demo.cpp")

target_link_libraries(octree PRIVATE OpenMP::OpenMP_CXX)
target_link_libraries(demo PUBLIC octree io)
include_directories("${PROJECT_SOURCE_DIR}/src" "${PROJECT_SOURCE_DIR}/external")