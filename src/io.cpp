#include "io.hpp"

std::vector<double3> random_pointcloud(unsigned int num_points, double3 length)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution_x(-length[0] / 2, length[0] / 2);
    std::uniform_real_distribution<double> distribution_y(-length[1] / 2, length[1] / 2);
    std::uniform_real_distribution<double> distribution_z(-length[2] / 2, length[2] / 2);

    std::vector<double3> points(num_points, double3());
    for (double3 &point: points)
        point = double3(distribution_x(generator), distribution_y(generator), distribution_z(generator));

    return points;
}

std::vector<double3> random_pointcloud(unsigned int num_points, double length)
{
    return random_pointcloud(num_points, double3(length, length, length));
}