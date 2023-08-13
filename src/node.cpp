#include "node.hpp"

void Node::update_vertices()
{
    if (points.size() <= 1)
        return;

    double3 min_vert = points[0];
    double3 max_vert = points[0];
    for (const double3& point : points)
    {
        min_vert = min_vert.cwiseMin(point);
        max_vert = max_vert.cwiseMax(point);
    }

    vert0 = min_vert;
    vert1 = max_vert;
}

void Node::split(unsigned int max_points_per_node)
{
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
        children.push_back(Node(v0, v1));
    }

    // Distribute points into children
    double3 node_center = 0.5 * (vert0 + vert1);
    for (size_t idx = 0; idx < points.size(); idx++)
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
    children.erase(std::remove_if(children.begin(), children.end(), [](Node& child) {return child.points.empty();}), children.end());

    // Recursively split children points
#pragma omp parallel for schedule(dynamic,1)
    for (Node& child : children)
        child.split(max_points_per_node);
}

Node::Node(unsigned int max_points_per_node, const doubleX3& given_points, const doubleX3& given_points_rgb)
{
    if (given_points_rgb.rows() && (given_points.rows() != given_points_rgb.rows()))
        throw std::range_error("Expected points_rgb to be empty or match points size");

    for (auto point : given_points.rowwise())
        points.push_back(double3(point.transpose()));

    for (auto point_rgb : given_points_rgb.rowwise())
        points_rgb.push_back(double3(point_rgb.transpose()));

    update_vertices();
    split(max_points_per_node);
}

Node::Node(std::filesystem::path path)
{
    std::ifstream input_stream(path, std::ios::binary);
    cereal::BinaryInputArchive archive(input_stream);
    archive(*this);
}

void Node::save(std::filesystem::path path)
{
    std::ofstream output_stream(path, std::ios::binary);
    cereal::BinaryOutputArchive archive(output_stream);
    archive(*this);

}

// Return size properties of full as a 3 int tuple containing (total #nodes, total #levels, total #points)
// Recursively traverses full Octree to get these numbers
std::tuple<int, int, int> Node::len() const
{
    int num_points = 0;
    int num_nodes = 0;
    int max_level = 0;
    std::queue<const Node*> q;
    std::queue<unsigned int> level;
    q.push(this);
    level.push(0);

    const Node* current;
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

        for (const Node& child : current->children)
        {
            q.push(&child);
            level.push(current_level + 1);
        }
    }

    return std::make_tuple(num_nodes, max_level, num_points);
}

// Test that all leaf nodes have points within their bounds
// If fails, throw exception
void Node::test() const
{
    std::queue<const Node*> q;
    q.push(this);

    const Node* current;
    while (!q.empty())
    {
        current = q.front();
        q.pop();

        if (current->points.empty() == current->children.empty())
            throw std::logic_error("Node must either have children or have points!");

        for (const Node& child : current->children)
            q.push(&child);

        for (const double3& pt : current->points)
            if ((pt[0] < current->vert0[0]) || (pt[0] > current->vert1[0]) || (pt[1] < current->vert0[1]) || (pt[1] > current->vert1[1]) || (pt[2] < current->vert0[2]) || (pt[2] > current->vert1[2]))
                throw std::logic_error("Found point outside node bounds!");
    }
}

// Return shortest ray-cone distance that intersects node, return -1 if no intersection
double Node::ray_intersection(const double3& origin, const double3& dir) const
{
    // Based on https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html
    double3 tvert0 = (vert0 - origin).cwiseQuotient(dir);
    double3 tvert1 = (vert1 - origin).cwiseQuotient(dir);

    double3 tmin = tvert0.cwiseMin(tvert1);
    double3 tmax = tvert0.cwiseMax(tvert1);

    double tmin_all = tmin.maxCoeff();
    double tmax_all = tmax.minCoeff();

    // Principal ray does not intersect, return -1.
    if ((tmax_all < 0.) || (tmin_all > tmax_all))
        return -1;

    // If tmin is negative we're inside the node, return small positive value (negative values indicate no intersection)
    if (tmin_all < 0)
        return 1e-5;

    return tmin_all;
}

// Cast ray at Octree, finds closest point (along ray direction) that projects to pixel
// Returns pair:
// - double t: distance to point, along the RAY. -1 if no intersection
// - double3 point_rgb: rgb of selected point. (0, 0, 0) if no intersection
// Assumes dir (direction) has unit length
std::pair<double, double3> Node::ray_cast(const double3& origin, const double3& dir, const double& radius_pixel) const
{
    // Priority queue contains octree nodes to visit next (ordered ascending from ray intersection distance)
    using queue_type = std::pair<double, const Node*>;
    std::priority_queue<queue_type, std::vector<queue_type>, std::greater<queue_type>> node_queue;
    node_queue.push(std::make_pair(0, this));

    double shortest_ray_distance = 1e10;
    size_t best_idx = -1;

    while (!node_queue.empty())
    {
        const Node* current = node_queue.top().second;
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
            double3 rgb = double3::Zero();
            if (!current->points_rgb.empty())
                rgb = current->points_rgb[best_idx];
            return std::make_pair(shortest_ray_distance, rgb);
        }

        // Add children which intersect rays to the queue (ordered by who intersects first)
        for (const Node& child : current->children)
        {
            double t = child.ray_intersection(origin, dir);
            if (t > 0)
                node_queue.push(std::make_pair(t, &child));
        }
    }

    return std::make_pair(-1, double3::Zero());
}


ImageTensor Node::render(const double3x3& K, const double4x4& cam2world, std::pair<int, int> image_hw, uint pixel_dilation)
{
    // Create data storage for image (stored as 4 channel: r, g, b, depth) with shape (H, W, 4), row-major
    ImageTensor image(image_hw.first, image_hw.second, 4);

    // Compute radius pixel
    double3x3 Kinv = K.inverse();
    double radius_pixel = pixel_dilation * (Kinv * double3(0.5, 0.5, 0)).norm();

    // Get origin and rotation matrix (cam2world)
    double3 origin = cam2world(Eigen::seq(0, 2), 3);
    double3x3 rotmat = cam2world(Eigen::seq(0, 2), Eigen::seq(0, 2));

    // Cast rays
#pragma omp parallel for schedule(dynamic,1) collapse(2)
    for (int i = 0; i < image_hw.first; i++)
        for (int j = 0; j < image_hw.second; j++)
        {
            double3 uv_hom(j + 0.5, i + 0.5, 1);
            double3 unproj = (Kinv * uv_hom).normalized();
            double3 ray_world = rotmat * unproj;
            auto res = ray_cast(origin, ray_world, radius_pixel);
            image(i, j, 0) = res.second[0];
            image(i, j, 1) = res.second[1];
            image(i, j, 2) = res.second[2];
            // depth is corresponds to distance on Z axis (forward axis), NOT the distance along the ray
            image(i, j, 3) = res.first * unproj(2);
        }

    return image;
}