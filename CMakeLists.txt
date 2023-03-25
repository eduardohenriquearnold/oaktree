cmake_minimum_required(VERSION 3.10)
project(OakTree VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(Octree "src/octree.cpp")
add_library(IO "src/io.cpp")
add_executable(Demo "src/demo.cpp")

target_link_libraries(Demo PUBLIC Octree IO)
include_directories("${PROJECT_SOURCE_DIR}/src" "${PROJECT_SOURCE_DIR}/external")