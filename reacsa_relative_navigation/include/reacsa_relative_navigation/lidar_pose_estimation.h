#ifndef LIDAR_POSE_ESTIMATION_H
#define LIDAR_POSE_ESTIMATION_H

#include <rclcpp/rclcpp.hpp>                       // Nodo ROS2
#include <sensor_msgs/msg/laser_scan.hpp>          // Messaggi LaserScan
#include <sensor_msgs/msg/point_cloud2.hpp>        // Messaggi PointCloud2
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "pcl/io/pcd_io.h"              // solo se leggi/salvi PCD (debug, test)
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
#include "visualization_msgs/msg/marker.hpp"

#include "clustering.h"
#include "clustering.cpp"
#include "tracking.h"
#include "tracking.cpp"

using namespace std::chrono_literals;

class LidarListener : public rclcpp::Node
{
public:
    LidarListener();
    ~LidarListener();

private:
    KdTree* tree_ = nullptr; // KdTree for clustering
    KalmanFilter2D kf = KalmanFilter2D(0.1, 1e-3, 1e-1);  // dt=0.1s, rumore processo, rumore misura
    bool kf_initialized = false;
    Box* target_box = nullptr;
    std::array<Box, 10> previous_boxes = {};

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    
    void tfStaticCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

    template<typename PointT> typename pcl::PointCloud<PointT>::Ptr filterPointCloud(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        float filterRes
    );
    
    template<typename PointT> std::unordered_set<int> Ransac3d(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        int maxIterations, 
        float distanceTolX,
        float distanceTolY,
        float distanceTolZ
    );
    
    template<typename PointT> std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        int maxIterations, 
        std::array<float, 3> distanceThreshold
    );
    
    template<typename PointT> std::unordered_set<int> Ransac2D(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        int maxIterations, 
        float distanceTol
    );

    template<typename PointT> std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(
        typename pcl::PointCloud<PointT>::Ptr obsCloud, 
        float clusterTolerance, 
        int minSize, 
        int maxSize
    );
    
    template<typename PointT> std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusteringPCL(
        typename pcl::PointCloud<PointT>::Ptr obsCloud, 
        float clusterTolerance, 
        int minSize, 
        int maxSize
    );
    
    template<typename PointT> std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusteringOptimized(typename pcl::PointCloud<PointT>::Ptr obsCloud,
                                                                                                    float clusterTolerance,
                                                                                                    int minSize,
                                                                                                    int maxSize
    );

    template<typename PointT> Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
    template<typename PointT> Box BoundingBoxPCA(typename pcl::PointCloud<PointT>::Ptr cluster);

    Eigen::Vector3f getBoxCentre(const Box &box);
    Box reconstructBox(const Eigen::Vector3f &centroid, const Box &box);
    Box trackBox(const Box &measured_box);
    Box trackBoxNoKF(const Box &measured_box);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_estimation_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

#endif // LIDAR_POSE_ESTIMATION_H
