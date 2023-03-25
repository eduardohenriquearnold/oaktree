#pragma once
#include <vector>
#include <random>

#include <linalg.h>
using namespace linalg::ostream_overloads;
using double3 = linalg::aliases::double3;
using double3x3 = linalg::aliases::double3x3;
using double4x4 = linalg::aliases::double4x4;

std::vector<double3> random_pointcloud(unsigned int num_points, double3 length);
std::vector<double3> random_pointcloud(unsigned int num_points, double length);