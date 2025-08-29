#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <rclcpp/rclcpp.hpp>                       // Nodo ROS2
#include <sensor_msgs/msg/laser_scan.hpp>          // Messaggi LaserScan
#include <sensor_msgs/msg/point_cloud2.hpp>        // Messaggi PointCloud2
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/io/pcd_io.h>              // solo se leggi/salvi PCD (debug, test)
#include <pcl/common/common.h>          // utile per min/max e manipolazioni generiche
#include <pcl/filters/voxel_grid.h>     // voxel grid filtering
#include <pcl/filters/crop_box.h>       // region of interest (ROI)
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "kdtree.h"


struct PolarPoint {
    float r;      // distance from the sensor: r = sqrt(x^2 + y^2 + z^2)
    float theta;  // azimuth: arctan2(y,x)
    float phi;    // elevation: arctan2(z, sqrt(x^2 + y^2))
    int idx;      // original index in the cloud
};

struct Box
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
    Eigen::Quaternionf orientation; // orientation of the box (for PCA)
};

// KdTree* tree_ = nullptr; // KdTree for clustering

template<typename PointT> std::vector<PolarPoint> toPolar(
    typename pcl::PointCloud<PointT>::Ptr cloud
);

template<typename PointT> pcl::PointCloud<PointT> toCartesian(
    typename std::vector<PolarPoint> polar_cloud
);

void proximity(
    const std::vector<std::vector<float>>& points, 
    int idx, 
    std::vector<int>& cluster, 
    std::vector<int>& processed, 
    KdTree* tree, 
    float distanceTol
);

void proximity_iterative(
    const std::vector<std::array<float,3>>& points,
    int start_idx,
    std::vector<int>& cluster,
    std::vector<char>& processed,
    KdTree* tree,
    float distanceTol,
    std::vector<int>& nearby_buffer
);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points,
                                                    KdTree* tree,
                                                    float distanceTol
);

std::vector<std::vector<int>> euclideanClusterOptimized(const std::vector<std::array<float,3>>& points, 
                                                        KdTree* tree, 
                                                        float distanceTol
);


#endif