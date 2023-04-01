#pragma once
#include <vector>
#include <random>

#include <eigen3/Eigen/Dense>
using double3 = Eigen::Vector3d;
using double3x3 = Eigen::Matrix<double, 3, 3>;
using double4x4 = Eigen::Matrix<double, 4, 4>;

std::vector<double3> random_pointcloud(unsigned int num_points, double3 length);
std::vector<double3> random_pointcloud(unsigned int num_points, double length);