#include "octree.hpp"

void OctreeNode::update_vertices()
{
    if (points.size() == 0)
        return;

    double3 min_vert = points[0];
    double3 max_vert = points[0];
    for (const double3 &point : points)
    {
        min_vert = min_vert.cwiseMin(point);
        max_vert = max_vert.cwiseMax(point);
    }

    vert0 = min_vert;
    vert1 = max_vert;
}

void OctreeNode::split(unsigned int max_points_per_node)
{
    // Set bounds based on current points
    update_vertices();

    if (points.size() <= max_points_per_node)
        return;

    // Create all 8 children with bounds derived from parent
    double3 half_size = 0.5 * (vert1 - vert0);
    for (int i = 0; i < 8; i++)
    {
        int x = i % 2;
        int y = (i / 2) % 2;
        int z = (i / 4) % 2;
        double3 v0 = double3(x, y, z).cwiseProduct(half_size) + vert0;
        double3 v1 = vert1 - double3(1 - x, 1 - y, 1 - z).cwiseProduct(half_size);
        children.push_back(OctreeNode(v0, v1));
    }

    // Distribute points into children
    double3 node_center = 0.5 * (vert0 + vert1);
    for (size_t idx=0; idx<points.size(); idx++)
    {
        double3 point = points[idx] - node_center;
        int children_id = (point.x() > 0 ? 1 : 0) + (point.y() > 0 ? 1 : 0) * 2 + (point.z() > 0 ? 1 : 0) * 4;
        children[children_id].points.push_back(points[idx]);

        if (!points_rgb.empty())
            children[children_id].points_rgb.push_back(points_rgb[idx]);
    }
    points.clear();
    points_rgb.clear();

    // Prune children without points
    std::remove_if(children.begin(), children.end(), [](OctreeNode &child){return child.points.empty();});

    // Recursively split children points
    #pragma omp parallel for schedule(dynamic,1)
    for (OctreeNode &child: children)
            child.split(max_points_per_node);
}

OctreeNode::OctreeNode(unsigned int max_points_per_node, const doubleX3 &given_points, const doubleX3 &given_points_rgb)
{
    if (given_points_rgb.rows() && (given_points.rows() != given_points_rgb.rows()))
        throw std::range_error("Expected points_rgb to be empty or match points size");

    for (auto point: given_points.rowwise())
        points.push_back(double3(point.transpose()));

    for (auto point_rgb: given_points_rgb.rowwise())
        points_rgb.push_back(double3(point_rgb.transpose()));

    split(max_points_per_node);
}

// Show stats of the Octree
void OctreeNode::stats() const
{
    int num_points = 0;
    int num_nodes = 0;
    int max_level = 0;
    std::queue<const OctreeNode *> q;
    std::queue<unsigned int> level;
    q.push(this);
    level.push(0);

    const OctreeNode *current;
    unsigned int current_level;
    while (!q.empty())
    {
        current = q.front();
        q.pop();
        current_level = level.front();
        level.pop();

        num_nodes++;
        num_points += current->points.size();
        if (current_level > max_level)
            max_level = current_level;

        for (const OctreeNode &child : current->children)
        {
            q.push(&child);
            level.push(current_level + 1);
        }
    }

    std::cout << "Octree: " << num_nodes << " nodes. " << max_level << " levels. " << num_points << " points." << std::endl;
}

// Test that all leaf nodes have points within their bounds
void OctreeNode::test() const
{
    std::queue<const OctreeNode *> q;
    q.push(this);

    const OctreeNode *current;
    while (!q.empty())
    {
        current = q.front();
        q.pop();

        for (const OctreeNode &child : current->children)
            q.push(&child);

        for (const double3 &pt : current->points)
            if ((pt[0] < current->vert0[0]) || (pt[0] > current->vert1[0]) || (pt[1] < current->vert0[1]) || (pt[1] > current->vert1[1]) || (pt[2] < current->vert0[2]) || (pt[2] > current->vert1[2]))
                throw std::logic_error("Found point outside node bounds!");
    }
    std::cout << "Octree test passed." << std::endl;
}

// Return shortest ray distance that intersects node, return -1 if no intersection
double OctreeNode::ray_intersection(const double3 &origin, const double3 &dir) const
{
    // Based on https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html
    double3 tvert0 = (vert0 - origin).cwiseQuotient(dir);
    double3 tvert1 = (vert1 - origin).cwiseQuotient(dir);

    double3 tmin = tvert0.cwiseMin(tvert1);
    double3 tmax = tvert0.cwiseMax(tvert1);

    double tmin_all = tmin.maxCoeff();
    double tmax_all = tmax.minCoeff();

    if ((tmin_all > tmax.y()) || (tmin.y() > tmax_all) || (tmin_all > tmax.z()) || (tmin.z() > tmax_all))
        return -1;

    return tmin_all;
}

// Cast ray at Octree, finds closest point (along ray direction) that projects to pixel
// Returns pair:
// - double t: distance to point, along the ray. -1 if no intersection
// - double3 point_rgb: rgb of selected point. (0, 0, 0) if no intersection
// Assumes dir (direction) has unit length
std::pair<double, double3> OctreeNode::ray_cast(const double3 &origin, const double3 &dir, const double &radius_pixel) const
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
        for (size_t idx = 0; idx < current->points.size(); idx++)
        {
            double3 point = current->points[idx] - origin;
            double distance_along_ray = point.dot(dir);
            double radius_point = (point - distance_along_ray * dir).norm();
            double radius_cone = radius_pixel * distance_along_ray;
            if (radius_point <= radius_cone && distance_along_ray < shortest_ray_distance)
            {
                shortest_ray_distance = distance_along_ray;
                best_idx = idx;
            }
        }
        if (best_idx != -1)
        {
            double3 rgb;
            if (!current->points_rgb.empty())
                rgb = current->points_rgb[best_idx];
            return std::make_pair(shortest_ray_distance, rgb);
        }

        // Add children which intersect rays to the queue (ordered by who intersects first)
        for (const OctreeNode &child : current->children){
            double t = child.ray_intersection(origin, dir);
            if (t > 0)
                node_queue.push(std::make_pair(t, &child));
        }
    }

    return std::make_pair(-1, double3::Zero());
}

// Returns pair (depthmap, RGB) 
std::pair<CImg,CImg> OctreeNode::render(const double3x3& K, const double4x4& cam2world, std::pair<int, int> image_hw)
{
    // Create depth map, CImg uses column major storage, so depth[j, i], for j column, i row
    CImg depth(image_hw.second, image_hw.first);
    CImg rgb(image_hw.second, image_hw.first, 1, 3);

    // Compute radius pixel
    double3x3 Kinv = K.inverse();
    const int pixel_dilation = 4;
    double radius_pixel = pixel_dilation * (Kinv * double3(0.5, 0.5, 0)).norm();

    // Get origin and rotation matrix (cam2world)
    double3 origin = cam2world(Eigen::seq(0, 2), 3);
    double3x3 rotmat = cam2world(Eigen::seq(0, 2), Eigen::seq(0, 2));

    // Cast rays
    #pragma omp parallel for schedule(dynamic,1) collapse(2)
    for (int i=0; i<image_hw.first; i++)
        for (int j=0; j<image_hw.second; j++)
        {
            double3 uv_hom(j+0.5, i+0.5, 1);
            double3 unproj = (Kinv * uv_hom).normalized();
            double3 ray_world = rotmat * unproj;
            auto res = ray_cast(origin, ray_world, radius_pixel);
            depth(j, i) = res.first;
            rgb(j, i, 0) = 255*res.second[0];
            rgb(j, i, 1) = 255*res.second[1];
            rgb(j, i, 2) = 255*res.second[2];
        }
    
    return std::make_pair(depth, rgb);
}