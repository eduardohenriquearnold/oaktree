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
    std::vector<double3> pcl = random_pointcloud(100000, double3(3., 0.5, 2.));
    OctreeNode root(pcl, 100);
    root.stats();
    root.test(pcl);
    std::cout << "Finished creating octree" << std::endl;

    double3 center(-0.5, 0., -0.5);
    double4x4 pose = translation_matrix(center);
    // double4x4 pose = inverse(linalg::lookat_matrix<double>(center, double3(0, 0, 0), double3(0,-1,0), linalg::pos_z));
    double3x3 K {{500, 0, 0}, {0, 500, 0}, {270, 360, 1}};
    std::pair<int, int> image_hw = std::make_pair(720, 540);

    CImg depth;
    {
        PROFILE_BLOCK("Render");
        depth = root.render_depth(K, pose, image_hw);
    }
    std::cout << "Finished rendering" << std::endl;
    std::cout << depth.min() << " " << depth.max() << std::endl;

    depth.normalize(0, 255);
    depth.save("depth.bmp");
    std::cout << "Saved depth map" << std::endl;
}