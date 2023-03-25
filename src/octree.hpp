#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <memory>
#include <random>

#include <linalg.h>
using namespace linalg::ostream_overloads;
using double3 = linalg::aliases::double3;
using double3x3 = linalg::aliases::double3x3;
using double4x4 = linalg::aliases::double4x4;

//CImg Docs https://cimg.eu/reference/structcimg__library_1_1CImg.html
#define cimg_display 0
#include <CImg.h>
using CImg = cimg_library::CImg<double>;

class OctreeNode
{
public:
    // Opposite extreme vertices (specify position and extent of AABB)
    double3 vert0;
    double3 vert1;

    // Point indices (for leaf nodes)
    std::vector<size_t> index;

    std::vector<OctreeNode> children;

    OctreeNode(double3 v0, double3 v1) : vert0(v0), vert1(v1){};
    OctreeNode(std::vector<double3> points, unsigned int max_points_per_node);

    void split(std::vector<double3> points, unsigned int max_points_per_node);
    double ray_intersection(const double3 &origin, const double3 &dir) const;
    std::pair<double, size_t> ray_cast(const double3 &origin, const double3 &dir, const double &radius_pixel, const std::vector<double3> &points) const;
    CImg render_depth(const std::vector<double3> points, const double3x3 K, const double4x4 cam2world, std::pair<int, int> image_hw);

    void stats();
    void test(std::vector<double3> points);
    void update_vertices(std::vector<double3> points);
};