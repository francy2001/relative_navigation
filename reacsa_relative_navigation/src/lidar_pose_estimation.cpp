/*
* Lidar Pose Estimation Node
* This node subscribes to LaserScan and PointCloud2 messages from a lidar sensor,
* filters the point cloud using voxel grid filtering, and applies a custom RANSAC
* algorithm to segment planes from the point cloud.

* Values to Tune:
* - Voxel size for filtering
* - Distance threshold for RANSAC plane segmentation
*/


#include "lidar_pose_estimation.h"

template<typename PointT> typename pcl::PointCloud<PointT>::Ptr  LidarListener::filterPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes)
{
    // Time segmentation process
    auto startime = std::chrono::steady_clock::now();

    // Implement voxel grid filtering here
    RCLCPP_INFO(this->get_logger(), "[Medium method]Filtering PointCloud2 data...");

    // Real application of Voxel Grid Filtering: dividing the space into cubes of side filterRes
    // and replacing all points in each cube with their centroid
    // This reduces the number of points in the cloud, making it easier to process
    // and speeding up the pose estimation process
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloud_filtered);

    // Filtering data to have data just between [-135,135] degrees
    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);
    for(const auto& point : cloud_filtered->points)
    {
        // Computing the orizzontal angle of the point
        // and checking if it is within the range [-135, 135] degrees
        float angle = std::atan2(point.y, point.x); // Angle in radians: [-pi, pi]
        // Normalizing the angle to be in the range [-3/4 pi, 3/4 pi]
        while(angle > M_PI) angle -= 2 * M_PI;
        while(angle < -M_PI) angle += 2 * M_PI;

        if(std::abs(angle) <= 3.0f * M_PI / 4.0f) // [-3/4 pi, 3/4 pi] --> [-135, 135] degrees
        {
            // If the point is within the range, add it to the cloud_region
            cloud_region->points.push_back(point);
        }else{
            RCLCPP_DEBUG(this->get_logger(), "Point out of range: x=%.2f, y=%.2f, angle=%.2f", point.x, point.y, angle);
        }
    }
    cloud_region->width = static_cast<uint32_t>(cloud_region->points.size());
    cloud_region->height = 1; // Unorganized point cloud
    cloud_region->is_dense = true;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startime);
    RCLCPP_INFO(this->get_logger(), "filtering took %ld milliseconds", elapsedTime.count());

    return cloud_region;
}

template<typename PointT> std::unordered_set<int> LidarListener::Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    /*
    * Implementation of the 3D RANSAC algorithm for plane segmentation in a point cloud.
    * The algorithm iteratively selects three random points to define a candidate plane,
    * computes the distance of all points in the cloud to this plane, and classifies as
    * inliers those within a given distance tolerance (distanceTol). After running for
    * a specified number of iterations (maxIterations), it returns the set of point indices
    * that best fit the plane with the largest number of inliers.
    */

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // Creation of a plane for each iteration
    // A plane is defined by the equation Ax + By + Cz + D = 0
    for (int i = 0; i < maxIterations; i++)
    {
        std::unordered_set<int> inliers;

        // Randomly pick three unique points 
        // Why? With 3 points we can define a plane in 3D space
        int ind1, ind2, ind3;
        do { ind1 = rand() % cloud->points.size(); } while(false);
        do { ind2 = rand() % cloud->points.size(); } while(ind2 == ind1);
        do { ind3 = rand() % cloud->points.size(); } while(ind3 == ind1 || ind3 == ind2);

        auto p1 = cloud->points[ind1];
        auto p2 = cloud->points[ind2];
        auto p3 = cloud->points[ind3];

        // Define the plane using the three points
        float A = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
        float B = (p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
        float C = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
        float D = -(A*p1.x + B*p1.y + C*p1.z);

        // Check if the plane is valid (so that the points are not collinear)
        float denom = std::sqrt(A*A + B*B + C*C);
        if (denom == 0) continue; // denom = 0 implies that the three points are collinear => infinite number of planes passing through them

        // Once computed the plane, let's compute the distance of each point to the plane
        // and check if it is within the distanceTol. If so, we add the point to the inliers set
        // Real segmentation phase
        for (int idx = 0; idx < int(cloud->points.size()); idx++)
        {
            auto pt = cloud->points[idx];
            float dist = std::fabs(A*pt.x + B*pt.y + C*pt.z + D) / denom;

            if (dist < distanceTol)
                inliers.insert(idx);
        }
        // If the number of inliers is greater than the current best, update the result
        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }

    return inliersResult;
}

template<typename PointT> std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> LidarListener::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();

    // Using custom RANSAC implementation to segment the plane
    std::unordered_set<int> inliersResult = Ransac3d<PointT>(cloud, maxIterations, distanceThreshold);

    // Create two point clouds: one for the plane and one for the obstacles
    // The plane cloud will contain the points that are inliers to the plane
    // The obstacles cloud will contain the points that are not inliers to the plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obsCloud(new pcl::PointCloud<PointT>());

    std::ofstream outfile("planeCloud_log.txt");
    std::ofstream outfile2("obstacleCloud_log.txt");
    if (!outfile.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open planeCloud_log.txt for writing");
    }
    if (!outfile2.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open obstacleCloud_log.txt for writing");
    }

    for (int i = 0; i < int(cloud->points.size()); i++)
    {
        PointT point = cloud->points[i];
        if (inliersResult.count(i)){
            planeCloud->points.push_back(point);
            //RCLCPP_INFO(this->get_logger(), "Point[%d] is an inlier: x=%.2f, y=%.2f, z=%.2f", i, point.x, point.y, point.z);
            outfile << "Point[" << i << "] is an inlier: x=" << point.x << ", y=" << point.y << ", z=" << point.z << "\n";
        }else{
            obsCloud->points.push_back(point);
            //RCLCPP_INFO(this->get_logger(), "Point[%d] is an obstacle: x=%.2f, y=%.2f, z=%.2f", i, point.x, point.y, point.z);
            outfile2 << "Point[" << i << "] is an obstacle: x=" << point.x << ", y=" << point.y << ", z=" << point.z << "\n";
        }
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    RCLCPP_INFO(this->get_logger(), "Custom RANSAC plane segmentation took %ld milliseconds", elapsedTime.count());
    
    return std::make_pair(obsCloud, planeCloud);
}

LidarListener::LidarListener() : rclcpp::Node("lidar_pose_estimation")
{
    // Subscribe to the topic publishing LaserScan data from the lidar
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/reacsa/scan", 
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
        std::bind(&LidarListener::laserScanCallback, this, std::placeholders::_1)
    );
    // Subscribe to the topic publishing PointCloud2 data from the lidar
    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/reacsa/velodyne_points", 
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&LidarListener::pointCloudCallback, this, std::placeholders::_1)
    );

    // Publishing the segmented point clouds
    plane_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/reacsa/plane_points", 
        10
    );
    obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/reacsa/obstacle_points", 
        10
    );

    RCLCPP_INFO(this->get_logger(), "LidarListener initialized: listening to /scan and /velodyne_points...");
}

void LidarListener::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "LaserScan received: angle_min=%.2f, angle_max=%.2f, ranges.size=%zu",
    msg->angle_min, msg->angle_max, msg->ranges.size());
    
    for (size_t i = 0; i < std::min(msg->ranges.size(), size_t(5)); ++i)
    RCLCPP_INFO(this->get_logger(), "range[%zu]=%.2f", i, msg->ranges[i]);
}

void LidarListener::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "PointCloud2 received: width=%u, height=%u, point_step=%u",
    msg->width, msg->height, msg->point_step);
    
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    int count = 0;
    for (; iter_x != iter_x.end() && count < 5; ++iter_x, ++iter_y, ++iter_z, ++count)
    {
        RCLCPP_INFO(this->get_logger(), "Point[%d]: x=%.2f, y=%.2f, z=%.2f", count, *iter_x, *iter_y, *iter_z);
    }
    
    // Filtering points using Voxel Grid Filtering
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *cloud);
    
    // --- Test filtering [TODO: PARAMETERS TO TUNE]---
    float voxelSize = 0.01f; // 1 cm voxel grid
    //Eigen::Vector4f minPoint(-10.0, -10.0, -2.0, 1.0); // box min corner
    //Eigen::Vector4f maxPoint(10.0, 10.0, 3.0, 1.0);    // box max corner
    
    auto cloud_filtered = filterPointCloud<pcl::PointXYZ>(cloud, voxelSize);
    
    RCLCPP_INFO(this->get_logger(), "Cloud size before filtering: %zu points", cloud->points.size());
    RCLCPP_INFO(this->get_logger(), "Filtered cloud size: %zu points", cloud_filtered->points.size());
    
    // Print first few points
    for (size_t i = 0; i < std::min<size_t>(5, cloud_filtered->points.size()); ++i)
    {
        const auto& p = cloud_filtered->points[i];
        RCLCPP_INFO(this->get_logger(), "Filtered Point[%zu]: x=%.2f, y=%.2f, z=%.2f", i, p.x, p.y, p.z);
    }
    // --- End of filtering ---
    // --- Test RANSAC Plane Segmentation [TODO: PARAMETERS TO TUNE] ---
    int maxIterations = 200; // Number of iterations for RANSAC
    float distanceThreshold = 0.2f; // Distance threshold for inliers
    auto [obsCloud, planeCloud] = SegmentPlane<pcl::PointXYZ>(cloud_filtered, maxIterations, distanceThreshold);
    // Publish the segmented point clouds
    sensor_msgs::msg::PointCloud2 plane_msg;
    sensor_msgs::msg::PointCloud2 obstacle_msg;
    pcl::toROSMsg(*planeCloud, plane_msg);
    pcl::toROSMsg(*obsCloud, obstacle_msg);
    plane_msg.header = msg->header;
    obstacle_msg.header = msg->header;
    plane_pub_->publish(plane_msg);
    obstacle_pub_->publish(obstacle_msg);

    // Log the size of the segmented clouds
    RCLCPP_INFO(this->get_logger(), "Segmented Plane Cloud size: %zu points", planeCloud->points.size());
    RCLCPP_INFO(this->get_logger(), "Segmented Obstacles Cloud size: %zu points", obsCloud->points.size());
    // --- End of RANSAC Plane Segmentation ---
    
}



int main(int argc, char const* argv[])
{
    rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LidarListener>());
	rclcpp::shutdown();
	return 0;
}
