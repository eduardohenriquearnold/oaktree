#pragma once
#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <list>
#include <queue>
#include <memory>
#include <tuple>
#include <stdexcept>

#include <Eigen/Dense>
using double3 = Eigen::Vector3d;
using float3= Eigen::Vector3f;
using double3x3 = Eigen::Matrix<double, 3, 3>;
using double4x4 = Eigen::Matrix<double, 4, 4>;
using doubleX3 = Eigen::MatrixX3d;
using floatX3 = Eigen::MatrixX3f;
using doubleX = Eigen::VectorXf;

#include "io.hpp"

class Node
{
public:
    Node() {};
    Node(std::filesystem::path path);
    Node(double3 v0, double3 v1) : vert0(v0), vert1(v1) {};
    Node(unsigned int max_points_per_node, const doubleX3& given_points, const floatX3& given_points_rgb = floatX3());

    template <class Archive>
    void serialize(Archive& ar) { ar(vert0, vert1, points, points_rgb, children); };
    void save(std::filesystem::path path);
    std::tuple<int, int, int> len() const;
    void test() const;
    void update_vertices();
    void split(unsigned int max_points_per_node);
    double ray_intersection(const double3& origin, const double3& dir) const;
    std::pair<double, float3> ray_cast(const double3& origin, const double3& dir, const double& radius_pixel) const;
    ImageTensor render(const double3x3& K, const double4x4& cam2world, std::pair<int, int> image_hw, uint pixel_dilation = 4) const;

    // Opposite extreme vertices (specify position and extent of AABB)
    double3 vert0;
    double3 vert1;

    // Points (used only for leaf nodes)
    std::list<double3> points;
    std::list<float3> points_rgb;

    // Children (used only for non-leaf nodes)
    std::vector<Node> children;
};