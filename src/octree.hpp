#pragma once
#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <queue>
#include <memory>
#include <stdexcept>

#include <eigen3/Eigen/Dense>
using double3 = Eigen::Vector3d;
using double3x3 = Eigen::Matrix<double, 3, 3>;
using double4x4 = Eigen::Matrix<double, 4, 4>;
using doubleX3 = Eigen::MatrixX3d;
using doubleX = Eigen::VectorXf;

#include "io.hpp"

class ImageTensor
{
    public:
        ImageTensor(uint height_, uint width_, uint channels_): height(height_), width(width_), channels(channels_), pixels(height_ * width_ * channels_, 0){};
        ImageTensor(const ImageTensor& other):  height(other.height), width(other.width), channels(other.channels), pixels(other.pixels){};
        ImageTensor(){};
        float& operator()(uint i, uint j, uint k)
        {
            return pixels[i * channels * width + j * channels + k];
        };
        ImageTensor& operator=(const ImageTensor& other)
        {
            height = other.height;
            width = other.width;
            channels = other.channels;
            pixels = other.pixels;
            return *this;
        };

        std::vector<float> pixels;
        uint height, width, channels;
};


class OctreeNode
{
public:
    OctreeNode(){};
    OctreeNode(std::filesystem::path path);
    OctreeNode(double3 v0, double3 v1) : vert0(v0), vert1(v1){};
    OctreeNode(unsigned int max_points_per_node, const doubleX3 &given_points, const doubleX3 &given_points_rgb=doubleX3());

    void stats() const;
    void test() const;
    ImageTensor render(const double3x3 &K, const double4x4 &cam2world, std::pair<int, int> image_hw);
    template<class Archive> void serialize(Archive &ar){ar(vert0, vert1, points, points_rgb, children);};
    void save(std::filesystem::path path);

private:
    // Opposite extreme vertices (specify position and extent of AABB)
    double3 vert0;
    double3 vert1;

    // Points (used only for leaf nodes)
    std::vector<double3> points;
    std::vector<double3> points_rgb;

    // Children (used only for non-leaf nodes)
    std::vector<OctreeNode> children;

    void update_vertices();
    void split(unsigned int max_points_per_node);
    double ray_intersection(const double3 &origin, const double3 &dir) const;
    std::pair<double, double3> ray_cast(const double3 &origin, const double3 &dir, const double &radius_pixel) const;
};