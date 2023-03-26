#include <chrono>
#include "octree.hpp"
#include "io.hpp"

struct profiler
{
    std::string name;
    std::chrono::high_resolution_clock::time_point p;
    profiler(std::string const &n) :
        name(n), p(std::chrono::high_resolution_clock::now()) { }
    ~profiler()
    {
        using dura = std::chrono::duration<double>;
        auto d = std::chrono::high_resolution_clock::now() - p;
        std::cout << name << ": "
            << std::chrono::duration_cast<dura>(d).count()
            << std::endl;
    }
};
#define PROFILE_BLOCK(pbn) profiler _pfinstance(pbn)

int main()
{
    // create random point cloud and color components
    std::vector<double3> pcl = random_pointcloud(100000, double3(3., 0.5, 2.));
    std::vector<double3> rgb = std::vector<double3>(pcl);
    for (double3 &color : rgb)
        color = (normalize(color) + 1)/2;

    // create and test octree
    OctreeNode root(100, pcl, rgb);
    root.stats();
    root.test();
    std::cout << "Finished creating octree" << std::endl;

    // set extrinsics/intrinsics
    double3 center(-1.5, 1.5, -1.5);
    // double4x4 pose = translation_matrix(center);
    double4x4 pose = inverse(linalg::lookat_matrix<double>(center, double3(0, 0, 0), double3(0,-1,0), linalg::pos_z));
    double3x3 K {{500, 0, 0}, {0, 500, 0}, {270, 360, 1}};
    std::pair<int, int> image_hw = std::make_pair(720, 540);

    // render depth/rgb image
    CImg depth, color;
    {
        PROFILE_BLOCK("Render");
        auto res = root.render(K, pose, image_hw);
        depth = res.first;
        color = res.second;
    }
    std::cout << "Finished rendering" << std::endl;
    std::cout << depth.min() << " " << depth.max() << std::endl;

    // save result to file
    depth.normalize(0, 255);
    depth.save("depth.bmp");
    color.save("color.bmp");
    std::cout << "Saved render" << std::endl;
}