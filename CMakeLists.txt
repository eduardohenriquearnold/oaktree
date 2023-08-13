cmake_minimum_required(VERSION 3.15)
project(OakTree VERSION 0.0.1 LANGUAGES CXX)

# Warn if the user invokes CMake directly
if (NOT SKBUILD)
  message(WARNING "\
  This CMake file is meant to be executed using 'scikit-build-core'.
  Running it directly will almost certainly not produce the desired
  result. If you are a user trying to install this package, use the
  command below, which will install all necessary build dependencies,
  compile the package in an isolated environment, and then install it.
  =====================================================================
   $ pip install .
  =====================================================================
  If you are a software developer, and this is your own package, then
  it is usually much more efficient to install the build dependencies
  in your environment once and use the following command that avoids
  a costly creation of a new virtual environment at every compilation:
  =====================================================================
   $ pip install nanobind scikit-build-core[pyproject]
   $ pip install --no-build-isolation -ve .
  =====================================================================
  You may optionally add -Ceditable.rebuild=true to auto-rebuild when
  the package is imported. Otherwise, you need to rerun the above
  after editing C++ files.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(OpenMP REQUIRED)
find_package(Python 3.9 
  REQUIRED COMPONENTS Interpreter Development.Module
  OPTIONAL_COMPONENTS Development.SABIModule)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/nanobind)

# c++ library and nanobind to module
add_library(node "src/node.cpp")
nanobind_add_module(_oaktree STABLE_ABI "src/oaktree.cpp")

# testing binary
enable_testing()
add_executable(cpp_test "src/test.cpp")
add_test(
  NAME cpp_test
  COMMAND $<TARGET_FILE:cpp_test>
  )

# set target properties
set_target_properties(node PROPERTIES POSITION_INDEPENDENT_CODE ON)

# linking
target_link_libraries(node PRIVATE OpenMP::OpenMP_CXX)
target_link_libraries(_oaktree PRIVATE node)
target_link_libraries(cpp_test PRIVATE node)

include_directories("${PROJECT_SOURCE_DIR}/src" "${PROJECT_SOURCE_DIR}/external" "${PROJECT_SOURCE_DIR}/external/eigen" "${PROJECT_SOURCE_DIR}/external/cereal/include")

install(TARGETS _oaktree LIBRARY DESTINATION oaktree)