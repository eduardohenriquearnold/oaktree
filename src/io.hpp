#pragma once
#include <random>
#include <eigen3/Eigen/Dense>

Eigen::MatrixX3d random_pointcloud(unsigned int num_points, Eigen::Vector3d length);
Eigen::MatrixX3d rrandom_pointcloud(unsigned int num_points, double length);