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

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar,4,4> lookAt(Derived const & eye, Derived const & center, Derived const & up){
    typedef Eigen::Matrix<typename Derived::Scalar,4,4> Matrix4;
    typedef Eigen::Matrix<typename Derived::Scalar,3,1> Vector3;
    Vector3 z = (center - eye).normalized();
    Vector3 x = up.cross(z).normalized();
    Vector3 y = z.cross(x);

    Matrix4 mat = Matrix4::Zero();
    mat.col(0) << x, 0;
    mat.col(1) << y, 0;
    mat.col(2) << z, 0;
    mat.col(3) << eye, 1;
    return mat;
}

int main()
{
    // create random point cloud and color components
    std::vector<double3> pcl = random_pointcloud(100000, double3(3., 0.5, 2.));
    std::vector<double3> rgb = std::vector<double3>(pcl);
    for (double3 &color : rgb)
        color = (color.normalized().array() + 1)/2;

    // create and test octree
    OctreeNode root(100, pcl, rgb);
    root.stats();
    root.test();
    std::cout << "Finished creating octree" << std::endl;

    // set extrinsics/intrinsics
    std::pair<int, int> image_hw = std::make_pair(720, 540);
    double3 eye(-1.5, 1.5, -1.5);
    double4x4 pose = lookAt<double3>(eye, double3(0, 0, 0), double3(0,-1,0));
    double3x3 K;
    K << 500, 0  , 270, 
           0, 500, 360,
           0,   0,   1;
    
    std::cout << K << std::endl;
    std::cout << pose << std::endl;

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