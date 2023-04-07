#include "io.hpp"

Eigen::MatrixX3d random_pointcloud(unsigned int num_points, Eigen::Vector3d length)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution_x(-length[0] / 2, length[0] / 2);
    std::uniform_real_distribution<double> distribution_y(-length[1] / 2, length[1] / 2);
    std::uniform_real_distribution<double> distribution_z(-length[2] / 2, length[2] / 2);
    
    Eigen::MatrixX3d points(num_points, 3);
    for (auto point: points.rowwise())
    {
        point[0] = distribution_x(generator);
        point[1] = distribution_y(generator);
        point[2] = distribution_z(generator);
    }

    return points;
}

Eigen::MatrixX3d random_pointcloud(unsigned int num_points, double length)
{
    return random_pointcloud(num_points, Eigen::Vector3d(length, length, length));
}