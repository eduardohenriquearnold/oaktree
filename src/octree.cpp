#include <iostream>
#include <vector>
#include <queue>
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
    std::vector<size_t> index;

    // Pointers to children
    std::vector<OctreeNode *> children;

    OctreeNode(double3 v0, double3 v1) : vert0(v0), vert1(v1){};
    ~OctreeNode();
};

OctreeNode::~OctreeNode()
{
    for (OctreeNode *child : children)
    {
        delete child;
        free(child);
    }
};

std::vector<double3> random_pointcloud(unsigned int num_points, double length)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-length / 2, length / 2);

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

    // Create all 8 children
    double3 half_size = 0.5 * (node->vert1 - node->vert0);
    for (int i = 0; i < 8; i++)
    {
        int x = i % 2;
        int y = (i / 2) % 2;
        int z = (i / 4) % 2;
        double3 vert0 = double3(x, y, z) * half_size + node->vert0;
        double3 vert1 = node->vert1 - double3(1 - x, 1 - y, 1 - z) * half_size;
        node->children.push_back(new OctreeNode(vert0, vert1));
    }

    // Distribute points into children
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
            delete (*it);
            node->children.erase(it--);
        }
        else
            split_node(*it, max_points_per_node, points);
}

OctreeNode *create_octree(std::vector<double3> points, unsigned int max_points_per_node)
{
    // TODO: Get extent of points
    OctreeNode *root = new OctreeNode(double3(-1., -1., -1.), double3(1., 1., 1.));

    for (size_t i = 0; i < points.size(); i++)
        root->index.push_back(i);

    split_node(root, max_points_per_node, points);
    return root;
}

void octree_stats(OctreeNode *root)
{
    int num_points = 0;
    int num_nodes = 0;
    int max_level = 0;
    std::queue<OctreeNode *> q;
    std::queue<unsigned int> level;
    q.push(root);
    level.push(0);

    OctreeNode *current;
    unsigned int current_level;
    while (!q.empty())
    {
        current = q.front();
        q.pop();
        current_level = level.front();
        level.pop();

        num_nodes++;
        num_points += current->index.size();
        if (current_level > max_level)
            max_level = current_level;

        for (OctreeNode *child : current->children)
        {
            q.push(child);
            level.push(current_level + 1);
        }
    }

    std::cout << "Octree: " << num_nodes << " nodes. " << max_level << " levels. " << num_points << " points." << std::endl;
}

int main()
{
    std::vector<double3> pcl = random_pointcloud(100000, 2.);
    OctreeNode *root = create_octree(pcl, 100);
    octree_stats(root);
}
