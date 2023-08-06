#include <chrono>
#include "octree.hpp"

//CImg Docs https://cimg.eu/reference/structcimg__library_1_1CImg.html
#define cimg_display 0
#include <CImg.h>
using CImg = cimg_library::CImg<double>;

struct profiler
{
    std::string name;
    std::chrono::high_resolution_clock::time_point p;
    profiler(std::string const& n) :
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
Eigen::Matrix<typename Derived::Scalar, 4, 4> lookAt(Derived const& eye, Derived const& center, Derived const& up) {
    typedef Eigen::Matrix<typename Derived::Scalar, 4, 4> Matrix4;
    typedef Eigen::Matrix<typename Derived::Scalar, 3, 1> Vector3;
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

std::pair<CImg, CImg> convert_eigen_cimg(ImageTensor& rendered, std::pair<int, int> image_hw)
{
    CImg depth(image_hw.second, image_hw.first);
    CImg rgb(image_hw.second, image_hw.first, 1, 3);

    for (int i = 0; i < image_hw.first; i++)
        for (int j = 0; j < image_hw.second; j++)
        {
            rgb(j, i, 0) = 255 * rendered(i, j, 0);
            rgb(j, i, 1) = 255 * rendered(i, j, 1);
            rgb(j, i, 2) = 255 * rendered(i, j, 2);
            depth(j, i) = rendered(i, j, 3);
        }

    return std::make_pair(depth, rgb);
}

int main()
{
    std::filesystem::path path("octree.bin");

    if (!std::filesystem::exists(path))
    {
        // create random point cloud and color components
        doubleX3 pcl = random_pointcloud(1000000, double3(3., 0.5, 2.));
        doubleX3 rgb = doubleX3(pcl);
        for (auto color : rgb.rowwise())
            color = (color.normalized().array() + 1) / 2;

        // create and test octree
        OctreeNode root(100, pcl, rgb);
        std::cout << "Finished creating octree" << std::endl;

        root.save(path);
        std::cout << "Saved octree" << std::endl;
        return 0;
    }

    OctreeNode root(path);
    std::cout << "Loaded octree" << std::endl;
    root.stats();
    root.test();

    // set extrinsics/intrinsics
    std::pair<int, int> image_hw = std::make_pair(720, 540);
    double3 eye(-8.5, 7.5, 2.5);
    double4x4 pose = lookAt<double3>(eye, double3(0, 0, 0), double3(0, 0, 1));
    double3x3 K;
    K << 500, 0, 270,
        0, 500, 360,
        0, 0, 1;

    // render depth/rgb image
    ImageTensor rendered;
    {
        PROFILE_BLOCK("Render");
        rendered = root.render(K, pose, image_hw);
    }
    CImg depth, color;
    auto res = convert_eigen_cimg(rendered, image_hw);
    depth = res.first;
    color = res.second;
    std::cout << "Finished rendering" << std::endl;
    std::cout << depth.min() << " " << depth.max() << std::endl;

    // save result to file
    depth.normalize(0, 255);
    depth.save("depth.bmp");
    color.save("color.bmp");
    std::cout << "Saved render" << std::endl;
}