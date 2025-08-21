#ifndef LIDAR_POSE_ESTIMATION_H
#define LIDAR_POSE_ESTIMATION_H

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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "kdtree.h"

struct PolarPoint {
    float r;      // distanza dal sensore: r = sqrt(x^2 + y^2 + z^2)
    float theta;  // azimuth: arctan2(y,x)
    float phi;    // elevation: arctan2(z, sqrt(x^2 + y^2))
    int idx;      // indice originale nel cloud
};

struct Box
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};

using namespace std::chrono_literals;

class LidarListener : public rclcpp::Node
{
public:
    LidarListener();

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    template<typename PointT> typename pcl::PointCloud<PointT>::Ptr filterPointCloud(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        float filterRes
    );
    template<typename PointT> std::unordered_set<int> Ransac3d(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        int maxIterations, 
        float distanceTol
    );
    template<typename PointT> std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        int maxIterations, 
        float distanceThreshold
    );
    template<typename PointT> std::vector<PolarPoint> toPolar(
        typename pcl::PointCloud<PointT>::Ptr cloud
    );
    template<typename PointT> std::unordered_set<int> Ransac2D(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        int maxIterations, 
        float distanceTol
    );
    void proximity(
        const std::vector<std::vector<float>>& points, 
        int idx, 
        std::vector<int>& cluster, 
        std::vector<int>& processed, 
        KdTree* tree, 
        float distanceTol
    );
    std::vector<std::vector<int>> euclideanCluster(
       const std::vector<std::vector<float>>& points,
       KdTree* tree,
       float distanceTol
    );
    template<typename PointT> std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(
        typename pcl::PointCloud<PointT>::Ptr obsCloud, 
        float clusterTolerance, 
        int minSize, 
        int maxSize
    );

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;

};

#endif // LIDAR_POSE_ESTIMATION_H
