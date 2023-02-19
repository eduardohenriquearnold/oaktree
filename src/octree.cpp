#include <iostream>
#include <vector>
#include <memory>
#include <random>

#include <linalg.h>
using namespace linalg::ostream_overloads;
using float3 = linalg::aliases::float3;

class OctreeNode
{
public:
    // Opposite extreme vertices (specify position and extent of AABB)
    float3 vert0;
    float3 vert1;

    // Point indices (for leaf nodes)
    std::vector<unsigned int> index;

    // Pointers to children
    OctreeNode *child0;
    OctreeNode *child1;
    OctreeNode *child2;
    OctreeNode *child3;
    OctreeNode *child4;
    OctreeNode *child5;
    OctreeNode *child6;
    OctreeNode *child7;
};

std::vector<float3> random_pointcloud(unsigned int num_points, float spread)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(-spread, spread);

    std::vector<float3> points;
    for (int i = 0; i < num_points; i++)
    {
        float3 point{distribution(generator), distribution(generator), distribution(generator)};
        points.push_back(point);
    }
    return points;
}

OctreeNode *create_octree(std::vector<float3> points, unsigned int max_points_per_node)
{
    return new OctreeNode();
}

int main()
{
    OctreeNode a;
    std::vector<float3> pcl = random_pointcloud(100, 0.5f);
    std::cout << a.vert0.x << std::endl;
    std::cout << pcl[0] << std::endl;
    std::cout << "Done" << std::endl;
}
