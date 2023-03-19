#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <memory>
#include <random>

#include <linalg.h>
using namespace linalg::ostream_overloads;
using double3 = linalg::aliases::double3;
using double3x3 = linalg::aliases::double3x3;
using double4x4 = linalg::aliases::double4x4;

//CImg Docs https://cimg.eu/reference/structcimg__library_1_1CImg.html
#define cimg_display 0
#include <CImg.h>
using CImg = cimg_library::CImg<double>;

class OctreeNode
{
public:
    // Opposite extreme vertices (specify position and extent of AABB)
    double3 vert0;
    double3 vert1;

    // Point indices (for leaf nodes)
    std::vector<size_t> index;

    std::vector<OctreeNode> children;

    OctreeNode(double3 v0, double3 v1) : vert0(v0), vert1(v1){};
    OctreeNode(std::vector<double3> points, unsigned int max_points_per_node);

    void split(std::vector<double3> points, unsigned int max_points_per_node);
    double ray_intersection(const double3 &origin, const double3 &dir) const;
    std::pair<double, size_t> ray_cast(const double3 &origin, const double3 &dir, const double &radius_pixel, const std::vector<double3> &points) const;

    void stats();
    void test(std::vector<double3> points);
    void update_vertices(std::vector<double3> points);
};

std::vector<double3> random_pointcloud(unsigned int num_points, double length)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-length / 2, length / 2);

    std::vector<double3> points(num_points, double3());
    for (double3 &point: points)
        point = double3(distribution(generator), distribution(generator), distribution(generator));

    return points;
}

void OctreeNode::update_vertices(std::vector<double3> points)
{
    if (index.size() == 0)
        return;

    double3 min_vert = points[index[0]];
    double3 max_vert = points[index[0]];
    for (size_t &point_idx : index)
    {
        double3 &point = points[point_idx];
        min_vert = min(min_vert, point);
        max_vert = max(max_vert, point);
    }

    vert0 = min_vert;
    vert1 = max_vert;
}

void OctreeNode::split(std::vector<double3> points, unsigned int max_points_per_node)
{
    // Set bounds based on current points
    update_vertices(points);

    if (index.size() <= max_points_per_node)
        return;

    // Create all 8 children with bounds derived from parent
    double3 half_size = 0.5 * (vert1 - vert0);
    for (int i = 0; i < 8; i++)
    {
        int x = i % 2;
        int y = (i / 2) % 2;
        int z = (i / 4) % 2;
        double3 v0 = double3(x, y, z) * half_size + vert0;
        double3 v1 = vert1 - double3(1 - x, 1 - y, 1 - z) * half_size;
        children.push_back(OctreeNode(v0, v1));
    }

    // Distribute points into children
    double3 node_center = 0.5 * (vert0 + vert1);
    for (size_t point_index : index)
    {
        double3 point = points[point_index] - node_center;
        int children_id = (point.x > 0 ? 1 : 0) + (point.y > 0 ? 1 : 0) * 2 + (point.z > 0 ? 1 : 0) * 4;
        children[children_id].index.push_back(point_index);
    }
    index.clear();

    // Prune child without points and recursively split child with points
    for (auto it = children.begin(); it != children.end(); it++)
        if (it->index.size() == 0)
            children.erase(it--);
        else
            it->split(points, max_points_per_node);
}

OctreeNode::OctreeNode(std::vector<double3> points, unsigned int max_points_per_node)
{
    for (size_t i = 0; i < points.size(); i++)
        index.push_back(i);

    split(points, max_points_per_node);
}

// Show stats of the Octree
void OctreeNode::stats()
{
    int num_points = 0;
    int num_nodes = 0;
    int max_level = 0;
    std::queue<OctreeNode *> q;
    std::queue<unsigned int> level;
    q.push(this);
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

        for (OctreeNode &child : current->children)
        {
            q.push(&child);
            level.push(current_level + 1);
        }
    }

    std::cout << "Octree: " << num_nodes << " nodes. " << max_level << " levels. " << num_points << " points." << std::endl;
}

// Test that all leaf nodes have points within their bounds
void OctreeNode::test(std::vector<double3> points)
{
    std::queue<OctreeNode *> q;
    q.push(this);

    OctreeNode *current;
    while (!q.empty())
    {
        current = q.front();
        q.pop();

        for (OctreeNode &child : current->children)
            q.push(&child);

        for (size_t idx : current->index)
        {
            double3 pt = points[idx];
            if ((pt[0] < current->vert0[0]) || (pt[0] > current->vert1[0]) || (pt[1] < current->vert0[1]) || (pt[1] > current->vert1[1]) || (pt[2] < current->vert0[2]) || (pt[2] > current->vert1[2]))
            {
                std::cout << "Found error for point index " << idx << std::endl;
                return;
            }
        }
    }
    std::cout << "Octree test passed." << std::endl;
}

// Return shortest ray distance that intersects node, return -1 if no intersection
double OctreeNode::ray_intersection(const double3 &origin, const double3 &dir) const
{
    // Based on https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html
    double3 tvert0 = (vert0 - origin) / dir;
    double3 tvert1 = (vert1 - origin) / dir;

    double3 tmin = min(tvert0, tvert1);
    double3 tmax = max(tvert0, tvert1);

    double tmin_all = maxelem(tmin);
    double tmax_all = minelem(tmax);

    if ((tmin_all > tmax.y) || (tmin.y > tmax_all) || (tmin_all > tmax.z) || (tmin.z > tmax_all))
        return -1;

    return tmin_all;
}

// Cast ray at Octree, finds closest point (along ray direction) that projects to pixel
// Returns pair:
// - double t: distance to point, along the ray. -1 if no intersection
// - size_t point_index: index of point in original point cloud
// Assumes dir (direction) has unit length
std::pair<double, size_t> OctreeNode::ray_cast(const double3 &origin, const double3 &dir, const double &radius_pixel, const std::vector<double3> &points) const
{
    // Priority queue contains octree nodes to visit next (ordered ascending from ray intersection distance)
    using queue_type = std::pair<double, const OctreeNode *>;
    std::priority_queue<queue_type, std::vector<queue_type>, std::greater<queue_type>> node_queue;
    node_queue.push(std::make_pair(0, this));

    // Other queue contains list of points inside the cone, ordered by distance along ray
    double shortest_ray_distance = 1e10;
    size_t best_idx = -1;

    while(!node_queue.empty())
    {
        const OctreeNode *current = node_queue.top().second;
        node_queue.pop();

        // Check if points fall within cone, return closest one along ray
        for (const size_t &idx : current->index)
        {
            double3 point = points[idx] - origin;
            double distance_along_ray = dot(point, dir);
            double radius_point = distance(point, distance_along_ray * dir);
            double radius_cone = radius_pixel * distance_along_ray;
            if (radius_point <= radius_cone && distance_along_ray < shortest_ray_distance)
            {
                shortest_ray_distance = distance_along_ray;
                best_idx = idx;
            }
        }
        if (best_idx != -1)
            return std::make_pair(shortest_ray_distance, best_idx);

        // Add children which intersect rays to the queue (ordered by who intersects first)
        for (const OctreeNode &child : current->children){
            double t = child.ray_intersection(origin, dir);
            if (t > 0)
                node_queue.push(std::make_pair(t, &child));
        }
    }

    return std::make_pair(-1, 0);
}

CImg render_depth(const OctreeNode &node, const std::vector<double3> points, const double3x3 K, const double4x4 cam2world, std::pair<int, int> image_hw)
{
    // Create depth map, CImg uses column major storage, so depth[j, i], for j column, i row
    CImg depth(image_hw.second, image_hw.first);

    // Compute radius pixel
    double3x3 Kinv = inverse(K);
    double radius_pixel = length(mul(Kinv, double3(0.5, 0.5, 0)));

    // Get origin and rotation matrix (cam2world)
    double3 origin(cam2world[3][0], cam2world[3][1], cam2world[3][2]);
    double3x3 rotmat {{cam2world[0][0], cam2world[0][1], cam2world[0][2]}, {cam2world[1][0], cam2world[1][1], cam2world[1][2]}, {cam2world[2][0], cam2world[2][1], cam2world[2][2]}};

    // Cast rays
    #pragma omp parallel for schedule(dynamic,1) collapse(2)
    for (int i=0; i<image_hw.first; i++)
        for (int j=0; j<image_hw.second; j++)
        {
            double3 uv_hom(j+0.5, i+0.5, 1);
            double3 unproj = normalize(mul(Kinv, uv_hom));
            double3 ray_world = mul(rotmat, unproj);
            auto res = node.ray_cast(origin, ray_world, radius_pixel, points);
            depth(j, i) = res.first;
        }
    
    return depth;
}

#include <chrono>
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
    std::vector<double3> pcl = random_pointcloud(100000, 2.);
    OctreeNode root(pcl, 100);
    root.stats();
    root.test(pcl);
    std::cout << "Finished creating octree" << std::endl;

    double4x4 pose = translation_matrix(double3(0, 0, -5));
    // double3 center(-10, 4, -10);
    // double4x4 pose = inverse(linalg::lookat_matrix<double>(center, double3(0, 0, 0), double3(0,-1,0), linalg::pos_z));
    double3x3 K {{500, 0, 0}, {0, 500, 0}, {270, 360, 1}};
    std::pair<int, int> image_hw = std::make_pair(720, 540);

    CImg depth;
    {
        PROFILE_BLOCK("Render");
        depth = render_depth(root, pcl, K, pose, image_hw);
    }
    std::cout << "Finished rendering" << std::endl;
    std::cout << depth.min() << " " << depth.max() << std::endl;

    depth.normalize(0, 255);
    depth.save("depth.bmp");
    std::cout << "Saved depth map" << std::endl;
}
