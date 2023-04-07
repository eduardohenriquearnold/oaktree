#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <memory>
#include <stdexcept>

#include <eigen3/Eigen/Dense>
using double3 = Eigen::Vector3d;
using double3x3 = Eigen::Matrix<double, 3, 3>;
using double4x4 = Eigen::Matrix<double, 4, 4>;
using doubleX3 = Eigen::MatrixX3d;

//CImg Docs https://cimg.eu/reference/structcimg__library_1_1CImg.html
#define cimg_display 0
#include <CImg.h>
using CImg = cimg_library::CImg<double>;

class OctreeNode
{
public:
    OctreeNode(double3 v0, double3 v1) : vert0(v0), vert1(v1){};
    OctreeNode(unsigned int max_points_per_node, const doubleX3 &given_points, const doubleX3 &given_points_rgb=doubleX3());

    void stats() const;
    void test() const;
    std::pair<CImg, CImg> render(const double3x3 &K, const double4x4 &cam2world, std::pair<int, int> image_hw);

// private:
    // Opposite extreme vertices (specify position and extent of AABB)
    double3 vert0;
    double3 vert1;

    // Points (used only for leaf nodes)
    doubleX3 points;
    doubleX3 points_rgb;

    // Children (used only for non-leaf nodes)
    std::vector<OctreeNode> children;

    void update_vertices();
    void split(unsigned int max_points_per_node);
    double ray_intersection(const double3 &origin, const double3 &dir) const;
    std::pair<double, double3> ray_cast(const double3 &origin, const double3 &dir, const double &radius_pixel) const;
};