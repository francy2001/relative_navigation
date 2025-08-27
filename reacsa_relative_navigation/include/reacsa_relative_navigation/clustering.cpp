#include "clustering.h"


template<typename PointT> std::vector<PolarPoint> toPolar(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    /*
    * Converts Cartesian coordinates of a point cloud to Polar coordinates.
    * Each point is represented by its radius (r), azimuthal angle (theta),
    * and polar angle (phi).
    * Why? This is a useful way to give more importance to points closer to the sensor, so to the depth component of the point
    */

    std::vector<PolarPoint> polarCloud;
    polarCloud.reserve(cloud->size());

    for (const auto& point : cloud->points) {
        PolarPoint p;
        p.r = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        p.theta = std::atan2(point.y, point.x);
        p.phi = std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y));
        p.idx = &point - &cloud->points[0];  // Get the index of the point
        polarCloud.push_back(p);
    }
    return polarCloud;
}

template<typename PointT> pcl::PointCloud<PointT> toCartesian(typename std::vector<PolarPoint> polar_cloud)
{
    pcl::PointCloud<PointT> cloud;
    cloud.reserve(polar_cloud.size());

    for (const auto& p : polar_cloud) {
        PointT point;
        point.x = p.r * std::cos(p.theta) * std::sin(p.phi);
        point.y = p.r * std::sin(p.theta) * std::sin(p.phi);
        point.z = p.r * std::cos(p.phi);
        cloud.push_back(point);
    }
    return cloud;
}

void proximity(const std::vector<std::vector<float>>& points, int idx, std::vector<int>& cluster, std::vector<int>& processed, KdTree* tree, float distanceTol)
{
    processed.push_back(idx);
    cluster.push_back(idx);

    std::vector<int> nearby = tree->search(points[idx], distanceTol);
    for (int id : nearby)
        if (std::find(processed.begin(), processed.end(), id) == processed.end())
            proximity(points, id, cluster, processed, tree, distanceTol);
}

void proximity_iterative(const std::vector<std::array<float,3>>& points, int start_idx, std::vector<int>& cluster, std::vector<char>& processed, KdTree* tree,
                                        float distanceTol, std::vector<int>& nearby_buffer)    // buffer riutilizzabile
{
    /*  
    * Iterative implementation of the proximity function for Euclidean clustering.
    * This function finds all points in the same cluster as the point at start_idx
    * using a stack to avoid recursion.
    */

    std::vector<int> stack;
    stack.reserve(256);
    stack.push_back(start_idx);
    processed[start_idx] = 1;

    while (!stack.empty())
    {
        // Pop the next point from the stack
        int current = stack.back(); stack.pop_back();
        cluster.push_back(current);

        // looks for nearby points
        nearby_buffer = tree->search({points[current][0], points[current][1], points[current][2]}, distanceTol);

        for (int id : nearby_buffer)
        {
            if (!processed[id])
            {
                processed[id] = 1;
                stack.push_back(id);
            }
        }
    }
}


std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    std::vector<int> processed;

    for (int i = 0; i < int(points.size()); ++i)
        if (std::find(processed.begin(), processed.end(), i) == processed.end())
        {
            std::vector<int> cluster;
            proximity(points, i, cluster, processed, tree, distanceTol);
            clusters.push_back(cluster);
        }

    return clusters;
}

std::vector<std::vector<int>> euclideanClusterOptimized(const std::vector<std::array<float,3>>& points, KdTree* tree, float distanceTol)
{
    /*  
    * Optimized implementation of Euclidean clustering using an iterative approach.
    */

    std::vector<std::vector<int>> clusters;
    size_t n = points.size();
    clusters.reserve(64);

    std::vector<char> processed(n, 0);         // flag O(1)
    std::vector<int> nearby_buffer;            // buffer riutilizzato
    nearby_buffer.reserve(64);

    for (size_t i = 0; i < n; ++i)
    {
        if (processed[i]) continue;

        std::vector<int> cluster;
        cluster.reserve(256);                  // heuristica

        proximity_iterative(points, (int)i, cluster, processed, tree, distanceTol, nearby_buffer);
        clusters.push_back(std::move(cluster));
    }

    return clusters;
}
