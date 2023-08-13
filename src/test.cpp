#ifdef NDEBUG
# undef NDEBUG
#endif

#include <cassert>
#include <cmath>
#include <iostream>

#include "node.hpp"

// Define a small value for floating-point comparison
const double EPSILON = 1e-6;

// Define a helper function to check if two doubles are approximately equal
bool isApproximatelyEqual(double a, double b, double epsilon = EPSILON)
{
    return std::abs(a - b) < epsilon;
}

// Ensures ray intersection for any direction (try 100 random ones) for origin inside node
void test_ray_intersection_within_node()
{
    Node octree(-1 * double3::Ones(), double3::Ones());
    double3 origin(0.0, 0.0, 0.0);


    for (int i = 0; i < 100; i++)
    {
        double3 direction = double3::Random().normalized();
        double intersection = octree.ray_intersection(origin, direction);
        assert(isApproximatelyEqual(intersection, 1e-5));
    }
}

void test_ray_intersection_outside_node()
{
    Node octree(-1 * double3::Ones(), double3::Ones());
    double3 direction(-1.0, 0.0, 0.0);
    double origin_x = 15.6;
    double3 origin(origin_x, 0.0, 0.0);

    double intersection = octree.ray_intersection(origin, direction);
    assert(isApproximatelyEqual(intersection, origin_x - 1.0));
}


void test_create_save_load()
{
    unsigned int npts = 500000;
    std::filesystem::path path("/tmp/octree-test.bin");

    // create octree with random points
    doubleX3 given_points = doubleX3::Random(npts, 3) * 100.;
    doubleX3 given_points_rgb = (doubleX3::Random(npts, 3).array() + 1.) / 2.;
    Node node(1000, given_points, given_points_rgb);

    // assert node created with success
    node.test();
    std::tuple<int, int, int> len = node.len();
    assert(std::get<2>(len) == npts);

    // save
    node.save(path);

    // load
    Node node2(path);

    // check test still pass
    node2.test();

    // check size is still the same
    std::tuple<int, int, int> len2 = node2.len();
    assert(len == len2);
}


void test_render_plane()
{
    // set up camera
    double4x4 pose = double4x4::Identity();
    double3x3 K;
    K << 5., 0., 5.,
         0., 5., 5.,
         0., 0., 1.;
    std::pair<int, int> image_hw(10, 10);
    
    // set up point cloud as plane at distance of 1m from camera
    unsigned int npts = 3205400;
    doubleX3 given_points = doubleX3::Random(npts, 3);
    given_points.col(2).setOnes();

    // create node
    Node node(1000, given_points);

    // render
    auto rendered = node.render(K, pose, image_hw, 4);

    // assert all pixels have depth one (or very close to that)
    for (int i=0; i<image_hw.first; i++)
        for (int j=0; j<image_hw.second; j++)
            assert(isApproximatelyEqual(rendered(i, j, 3), 1., 0.03));
}


int main()
{
    std::cout << "Running Tests..." << std::endl;

    test_ray_intersection_within_node();
    test_ray_intersection_outside_node();
    test_create_save_load();
    test_render_plane();

    std::cout << "All tests passed!" << std::endl;

    return 0;
}