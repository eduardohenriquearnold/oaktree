#include <iostream>
#include <vector>
#include <memory>
#include <random>

#include <linalg.h>
using namespace linalg::ostream_overloads;
using double3 = linalg::aliases::double3;

class OctreeNode
{
public:
    // Opposite extreme vertices (specify position and extent of AABB)
    double3 vert0;
    double3 vert1;

    // Point indices (for leaf nodes)
    std::vector<unsigned int> index;

    // Pointers to children
    std::vector<OctreeNode *> children;

    OctreeNode(double3 v0, double3 v1) : vert0(v0), vert1(v1){};
    void erase();
};

void OctreeNode::erase()
{
    for (OctreeNode *child : children)
    {
        child->erase();
        free(child);
    }
};

std::vector<double3> random_pointcloud(unsigned int num_points, float spread)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-spread, spread);

    std::vector<double3> points;
    for (int i = 0; i < num_points; i++)
    {
        double3 point{distribution(generator), distribution(generator), distribution(generator)};
        points.push_back(point);
    }
    return points;
}

void split_node(OctreeNode *node, unsigned int max_points_per_node, std::vector<double3> points)
{
    if (node->index.size() <= max_points_per_node)
        return;

    // TODO complete for-loop that creates all children with correct extents
    double3 hsize = 0.5 * (node->vert1 - node->vert0);
    for (int i = 0; i < 8; i++)
    {
        double3 delta = double3() * hsize;
    }

    double3 node_center = 0.5 * (node->vert0 + node->vert1);
    for (unsigned int i = 0; i < node->index.size(); i++)
    {
        double3 point = points[node->index[i]] - node_center;
        int children_id = (point.x > 0 ? 1 : 0) + (point.y > 0 ? 1 : 0) * 2 + (point.z > 0 ? 1 : 0) * 4;
        node->children[children_id]->index.push_back(i);
    }
    node->index.clear();

    // Recursively prune child without points and split child with points
    for (auto it = node->children.begin(); it != node->children.end(); it++)
        if ((*it)->index.size() == 0)
        {
            (*it)->erase();
            node->children.erase(it--);
        }
        else
            split_node(*it, max_points_per_node, points);
}

OctreeNode *create_octree(std::vector<double3> points, unsigned int max_points_per_node)
{
    // TODO: Get extent of points
    OctreeNode *root = new OctreeNode(double3(-1., -1., -1.), double3(1., 1., 1.));

    for (unsigned int i = 0; i < points.size(); i++)
        root->index.push_back(i);

    split_node(root, max_points_per_node, points);
    return root;
}

int main()
{
    std::vector<double3> pcl = random_pointcloud(100, 0.5f);
    std::cout << pcl[0] << std::endl;
}
