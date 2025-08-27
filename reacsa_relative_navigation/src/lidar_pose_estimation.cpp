/*
* Lidar Pose Estimation Node
* This node subscribes to LaserScan and PointCloud2 messages from a lidar sensor,
* filters the point cloud using voxel grid filtering, and applies a custom RANSAC
* algorithm to segment planes from the point cloud.

* Values to Tune:
* - Voxel size for filtering
* - Distance threshold for RANSAC plane segmentation
* - Number of iterations for RANSAC
* - Distance tolerance for clustering
* - Minimum and maximum cluster sizes
*/


#include "lidar_pose_estimation.h"


template<typename PointT> typename pcl::PointCloud<PointT>::Ptr  LidarListener::filterPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes)
{
    /*
    * Implementation of Voxel Grid Filtering and Angle Based Filtering for downsampling a point cloud.
    */

    // Time segmentation process
    auto startime = std::chrono::steady_clock::now();

    // Implement voxel grid filtering here
    RCLCPP_DEBUG(this->get_logger(), "[Medium method]Filtering PointCloud2 data...");

    // Real application of Voxel Grid Filtering: dividing the space into cubes of side filterRes
    // and replacing all points in each cube with their centroid
    // This reduces the number of points in the cloud, making it easier to process
    // and speeding up the pose estimation process
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloud_filtered);

    // Filtering data to have data just between [-135,135] degrees [to tune based on which range of degrees we want to consider]
    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);
    for(const auto& point : cloud_filtered->points)
    {
        RCLCPP_DEBUG(this->get_logger(), "Point: x=%.2f, y=%.2f, z=%.2f", point.x, point.y, point.z);
        // Computing the horizontal angle of the point
        // and checking if it is within the range [-135, 135] degrees
        float angle = std::atan2(point.y, point.x); // Angle in radians: [-pi, pi]
        RCLCPP_DEBUG(this->get_logger(), "Point angle: %.2f radians", angle);

        if(std::abs(angle) <= M_PI ) // [-3/8 pi, 3/8 pi] --> [-67.5, 67.5] degrees
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
    RCLCPP_DEBUG(this->get_logger(), "filtering took %ld milliseconds", elapsedTime.count());

    return cloud_region;
}

template<typename PointT> std::unordered_set<int> LidarListener::Ransac2D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    std::unordered_set<int> inliers;
    srand(time(NULL));

    // For max iterations 
    int i = 0;
    double line = 0.0;
    double dist = 0.0;
    double den = 0.0;
    int ind1, ind2 = 0;

    while (i < maxIterations) {

        // clear this every iteration
        inliers.clear();

        // ensure same indices are not picked
        while (true) {
            ind1 = rand() % cloud->points.size();
            ind2 = rand() % cloud->points.size();
            if (ind1 != ind2)
                break;
        }

        // points to construct the line
        pcl::PointXYZ p1 = cloud->points[ind1];
        pcl::PointXYZ p2 = cloud->points[ind2];

        for (int ind = 0; ind < int(cloud->points.size()); ind++) {
            double x = cloud->points[ind].x;
            double y = cloud->points[ind].y;
            line = (p1.y - p2.y) * x + (p2.x - p1.x) * y + (p1.x * p2.y - p2.x * p1.y);
            den = sqrt(pow((p1.y - p2.y), 2) + pow((p2.x - p1.x), 2));
            dist = fabs(line / den);
            RCLCPP_DEBUG(this->get_logger(), "distance %f", dist);
            if (dist < distanceTol) {
                inliers.insert(ind);
            }
        }
        // return the set with maximum number of inliers
        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
        i++;
    }
    return inliersResult;
}

template<typename PointT> std::unordered_set<int> LidarListener::Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTolX, float distanceTolY, float distanceTolZ)
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
        // normal unit vector components
        float nx = A / denom;
        float ny = B / denom;
        float nz = C / denom;

        for (int idx = 0; idx < int(cloud->points.size()); idx++)
        {
            auto pt = cloud->points[idx];
            float signed_dist = (A*pt.x + B*pt.y + C*pt.z + D) / denom;

            // component-wise distances along x,y,z (absolute values)
            float dx = std::fabs(signed_dist * nx);
            float dy = std::fabs(signed_dist * ny);
            float dz = std::fabs(signed_dist * nz);

            if (dx < distanceTolX && dy < distanceTolY && dz < distanceTolZ)
                inliers.insert(idx);
        }
        // If the number of inliers is greater than the current best, update the result
        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }

    return inliersResult;
}

template<typename PointT> std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> LidarListener::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, std::array<float, 3> distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();

    // Using polar coordinates
    std::vector<PolarPoint> polarCoords = toPolar<pcl::PointXYZ>(cloud);
    typename pcl::PointCloud<PointT>::Ptr polarCloud(new pcl::PointCloud<PointT>());
    for (const auto& p : polarCoords) {
        PointT point;
        point.x = p.r;
        point.y = p.theta;
        point.z = p.phi;
        polarCloud->points.push_back(point);
    }

    // Using custom RANSAC implementation to segment the plane
    // std::unordered_set<int> inliersResult = Ransac3d<PointT>(polarCloud, maxIterations, distanceThreshold[0], distanceThreshold[1], distanceThreshold[2]);
    std::unordered_set<int> inliersResult = Ransac3d<PointT>(cloud, maxIterations, distanceThreshold[0], distanceThreshold[1], distanceThreshold[2]);
    
    // Create two point clouds: one for the plane and one for the obstacles
    // The plane cloud will contain the points that are inliers to the plane
    // The obstacles cloud will contain the points that are not inliers to the plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obsCloud(new pcl::PointCloud<PointT>());

    // Save the point cloud to a text file for debugging
    // std::ofstream outfile("point_cloud.txt");
    // if (!outfile.is_open()) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to open point_cloud.txt for writing");
    // }

    for (int i = 0; i < int(cloud->points.size()); i++)
    {
        PointT point = cloud->points[i];
        if (inliersResult.count(i)){
            planeCloud->points.push_back(point);
            RCLCPP_DEBUG(this->get_logger(), "Point[%d] is an inlier: x=%.2f, y=%.2f, z=%.2f", i, point.x, point.y, point.z);
        }else{
            obsCloud->points.push_back(point);
            RCLCPP_DEBUG(this->get_logger(), "Point[%d] is an obstacle: x=%.2f, y=%.2f, z=%.2f", i, point.x, point.y, point.z);
        }
        // outfile << "x=" << polarCloud->points[i].x << ", y=" << polarCloud->points[i].y << ", z=" << polarCloud->points[i].z << "\n";
    }

    // outfile << "End of point cloud data\n";
    // outfile.close();
    
    RCLCPP_DEBUG(this->get_logger(), "Segmented Plane Cloud size: %zu points", planeCloud->points.size());
    RCLCPP_DEBUG(this->get_logger(), "Segmented Obstacles Cloud size: %zu points", obsCloud->points.size());

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    RCLCPP_DEBUG(this->get_logger(), "Custom RANSAC plane segmentation took %ld milliseconds", elapsedTime.count());
    
    return std::make_pair(obsCloud, planeCloud);
}

// 3D Eucledian Clustering Implementation (custom)
template<typename PointT> std::vector<typename pcl::PointCloud<PointT>::Ptr> LidarListener::Clustering(typename pcl::PointCloud<PointT>::Ptr obsCloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    // Convert PCL point cloud to vector of points for custom KD-tree
    std::vector<std::vector<float>> points;
    for (int i = 0; i < int(obsCloud->points.size()); i++)
    {
        std::vector<float> point = {obsCloud->points[i].x, obsCloud->points[i].y, obsCloud->points[i].z};
        points.push_back(point);
    }
    
    // Create custom KD-tree (from your quiz implementation)
    KdTree* tree = new KdTree;
    
    for (int i = 0; i < int(points.size()); i++)
        tree->insert(points[i], i);
    
    // Use your custom euclidean clustering function
    std::vector<std::vector<int>> cluster_indices = euclideanCluster(points, tree, clusterTolerance);
    

    // Convert clusters back to PCL format
    for (std::vector<int> cluster : cluster_indices)
    {
        if (int(cluster.size()) >= minSize && int(cluster.size()) <= maxSize)
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
            
            for (int index : cluster)
                cloud_cluster->points.push_back(obsCloud->points[index]);

            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            
            clusters.push_back(cloud_cluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    RCLCPP_INFO(this->get_logger(), "Clustering took %ld milliseconds and found %zu clusters", elapsedTime.count(), clusters.size());
    return clusters;
}

template<typename PointT> std::vector<typename pcl::PointCloud<PointT>::Ptr> LidarListener::ClusteringPCL(typename pcl::PointCloud<PointT>::Ptr obsCloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    if (obsCloud->empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Obs cloud empty");
        return clusters;
    }

    // Use PCL KdTree (bulk build)
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(obsCloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(obsCloud);
    ec.extract(cluster_indices);

    // Convert indices to pointclouds
    for (const auto& indices : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        cloud_cluster->points.reserve(indices.indices.size());
        for (int idx : indices.indices)
            cloud_cluster->points.push_back(obsCloud->points[idx]);
        cloud_cluster->width = static_cast<uint32_t>(cloud_cluster->points.size());
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    RCLCPP_INFO(this->get_logger(), "Clustering took %ld ms and found %zu clusters",
                elapsedTime.count(), clusters.size());
    return clusters;
}

template<typename PointT> std::vector<typename pcl::PointCloud<PointT>::Ptr> LidarListener::ClusteringOptimized(typename pcl::PointCloud<PointT>::Ptr obsCloud,
                                                                                                                float clusterTolerance,
                                                                                                                int minSize,
                                                                                                                int maxSize)
{
    /*
    * Optimized clustering implementation using a custom KD-tree and iterative proximity function.
    */

    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = {};
    if (obsCloud->empty()) return clusters;

    // Conversion to compact version of points
    std::vector<std::array<float,3>> points;
    points.reserve(obsCloud->points.size());
    for (const auto &p : obsCloud->points)
        points.push_back({p.x, p.y, p.z});

    // Define KD-Tree as a part of the class to do not rebuild it every time (less memory occupancy)
    if (this->tree_) {
        delete this->tree_;
        this->tree_ = nullptr;
    }
    this->tree_ = new KdTree();

    // Insert point into the tree
    for (size_t i = 0; i < points.size(); ++i)
        this->tree_->insert({points[i][0], points[i][1], points[i][2]}, (int)i);
    RCLCPP_DEBUG(this->get_logger(), "Built tree with %zu points", points.size());

    // Optimized clustering
    auto idx_clusters = euclideanClusterOptimized(points, this->tree_, clusterTolerance);

    for (const auto &idxs : idx_clusters)
    {
        if ((int)idxs.size() < minSize || (int)idxs.size() > maxSize) continue;
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        cloud_cluster->points.reserve(idxs.size());
        for (int idx : idxs)
            cloud_cluster->points.push_back(obsCloud->points[idx]);
        cloud_cluster->width = static_cast<uint32_t>(cloud_cluster->points.size());
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    RCLCPP_DEBUG(this->get_logger(), "Clustering took %ld ms and found %zu clusters", elapsed.count(), clusters.size());
    return clusters;
}

template<typename PointT> Box LidarListener::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

Eigen::Vector3f LidarListener::getBoxCentre(const Box &box) {
    return Eigen::Vector3f(
        (box.x_min + box.x_max) / 2.0f,
        (box.y_min + box.y_max) / 2.0f,
        (box.z_min + box.z_max) / 2.0f
    );
}

Box LidarListener::reconstructBox(const Eigen::Vector3f &centroid, const Box &measured_box)
{
    static float smoothed_w = 0.0f;
    static float smoothed_h = 0.0f;
    static float smoothed_d = 0.0f;

    // dimensioni misurate dalla box corrente
    float measured_w = measured_box.x_max - measured_box.x_min;
    float measured_h = measured_box.y_max - measured_box.y_min;
    float measured_d = measured_box.z_max - measured_box.z_min;

    // smoothing esponenziale
    float alpha = 0.8f; // quanto dare peso allo storico (alto = più stabile)
    smoothed_w = alpha * smoothed_w + (1 - alpha) * measured_w;
    smoothed_h = alpha * smoothed_h + (1 - alpha) * measured_h;
    smoothed_d = alpha * smoothed_d + (1 - alpha) * measured_d;

    // costruisci nuova box centrata sul centroide stimato
    Box new_box;
    new_box.x_min = centroid.x() - smoothed_w / 2.0f;
    new_box.x_max = centroid.x() + smoothed_w / 2.0f;
    new_box.y_min = centroid.y() - smoothed_h / 2.0f;
    new_box.y_max = centroid.y() + smoothed_h / 2.0f;
    new_box.z_min = centroid.z() - smoothed_d / 2.0f;
    new_box.z_max = centroid.z() + smoothed_d / 2.0f;

    return new_box;
}

Box LidarListener::trackBox(const Box &measured_box)
{
    Eigen::Vector3f measured_centroid = getBoxCentre(measured_box);

    if (!kf_initialized) {
        // Initialize Kalman Filter with the first measurement
        kf.init(measured_centroid.x(), measured_centroid.y());
        kf_initialized = true;
        RCLCPP_INFO(this->get_logger(), "Kalman Filter initialized at x=%.2f, y=%.2f, z=%.2f",
                    measured_centroid.x(), measured_centroid.y(), measured_centroid.z());
    } else {
        // Predict the next state
        kf.predict();
        // Update with the new measurement
        kf.update(measured_centroid.x(), measured_centroid.y());
        RCLCPP_DEBUG(this->get_logger(), "Kalman Filter updated state: x=%.2f, y=%.2f",
                     kf.getX(), kf.getY());
    }

    Eigen::Vector3f est_center(kf.getX(), kf.getY(), measured_centroid.z());
    // Reconstruct the box around the estimated centroid
    Box tracked_box = reconstructBox(est_center, measured_box);
    return tracked_box;
}

Box LidarListener::trackBoxNoKF(const Box &measured_box)
{
    Eigen::Vector3f centroid = getBoxCentre(measured_box);

    if(!target_box || previous_boxes.empty()){
        RCLCPP_DEBUG(this->get_logger(), "Initial centroid position: (%.2f, %.2f, %.2f)", centroid.x(), centroid.y(), centroid.z());
        target_box = new Box(measured_box);  // salva copia completa, incluso orientation
        previous_boxes[previous_boxes.size()-1] = measured_box;
        return measured_box;
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Previous centroid position: (%.2f, %.2f, %.2f)", getBoxCentre(*target_box).x(),
                     getBoxCentre(*target_box).y(), getBoxCentre(*target_box).z());

        std::array<bool, 10> outlier{}; 
        for(size_t i = 0; i < previous_boxes.size(); ++i) {
            Eigen::Vector3f diff = centroid - getBoxCentre(previous_boxes[i]);
            float dist = diff.norm();
            RCLCPP_DEBUG(this->get_logger(), "Current centroid: (%.2f, %.2f, %.2f), movement distance: %.2f",
                         centroid.x(), centroid.y(), centroid.z(), dist);
            if(dist > 2.5){ // se il movimento è troppo grande, consideralo outlier
                RCLCPP_DEBUG(this->get_logger(), "Movement over threshold, keeping previous position.");
                outlier[i] = true;
            }
        }

        // Se la maggioranza delle ultime misure è outlier, mantieni la box precedente
        if (std::count(outlier.begin(), outlier.end(), true) > 7) {
            return *target_box;
        } else {
            // Aggiorna box target
            target_box->x_min = measured_box.x_min;
            target_box->x_max = measured_box.x_max;
            target_box->y_min = measured_box.y_min;
            target_box->y_max = measured_box.y_max;
            target_box->z_min = measured_box.z_min;
            target_box->z_max = measured_box.z_max;
            target_box->orientation = measured_box.orientation; // aggiorna anche orientamento

            // Aggiorna storico
            previous_boxes[previous_boxes.size()-1] = measured_box;

            return *target_box;
        }
    }
}


template<typename PointT> Box LidarListener::BoundingBoxPCA(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    Box box;

    if (cluster->empty()) return box;

    // Computing Centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);

    // Computing Covariance Matrix
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cluster, centroid, covariance);

    // 3. PCA -> eigenvectors and eigenvalues of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covariance);
    Eigen::Matrix3f eigVectors = eig.eigenvectors(); 

    // 4. Trasforma punti nel frame PCA
    Eigen::Matrix4f pcaTransform = Eigen::Matrix4f::Identity();
    pcaTransform.block<3,3>(0,0) = eigVectors.transpose(); // ruota nel frame PCA
    pcaTransform.block<3,1>(0,3) = -eigVectors.transpose() * centroid.head<3>(); // trasla

    pcl::PointCloud<PointT> cloudPCA;
    pcl::transformPointCloud(*cluster, cloudPCA, pcaTransform);

    // 5. Bounding box nel frame PCA
    PointT minPt, maxPt;
    pcl::getMinMax3D(cloudPCA, minPt, maxPt);

    // 6. Centro della box in frame PCA
    Eigen::Vector3f meanPCA = 0.5f * (minPt.getVector3fMap() + maxPt.getVector3fMap());

    // 7. Trasforma il centro della box nel frame originale
    Eigen::Vector3f centerWorld = eigVectors * meanPCA + centroid.head<3>();

    // 8. Riempie la struct Box
    box.x_min = centerWorld.x() - 0.5f * (maxPt.x - minPt.x);
    box.x_max = centerWorld.x() + 0.5f * (maxPt.x - minPt.x);
    box.y_min = centerWorld.y() - 0.5f * (maxPt.y - minPt.y);
    box.y_max = centerWorld.y() + 0.5f * (maxPt.y - minPt.y);
    box.z_min = centerWorld.z() - 0.5f * (maxPt.z - minPt.z);
    box.z_max = centerWorld.z() + 0.5f * (maxPt.z - minPt.z);

    // Salviamo l'orientamento della box
    box.orientation = Eigen::Quaternionf(eigVectors); // aggiungi campo orientation a Box

    return box;
}


LidarListener::~LidarListener() {
    if (tree_) {
        delete tree_;
        tree_ = nullptr;
    }
}


LidarListener::LidarListener() : rclcpp::Node("lidar_pose_estimation")
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::Duration(std::chrono::seconds(10)));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS qos_static(10);
    qos_static.transient_local();
    qos_static.reliable();

    tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/reacsa/tf_static",
        qos_static,
        std::bind(&LidarListener::tfStaticCallback, this, std::placeholders::_1));

    rclcpp::QoS qos_tf(100);
    qos_tf.best_effort(); // se il publisher è best_effort
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/reacsa/tf", qos_tf,
        std::bind(&LidarListener::tfCallback, this, std::placeholders::_1));

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
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/reacsa/obstacle_markers",
        10
    );

    pose_estimation_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/reacsa/lidar_pose_estimation",
        10
    );


    RCLCPP_INFO(this->get_logger(), "LidarListener initialized: listening to /scan and /velodyne_points...");
}

void LidarListener::tfStaticCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received /reacsa/tf_static with %zu transforms", msg->transforms.size());
    for (const auto &t : msg->transforms) {
        tf_buffer_->setTransform(t, "reacsa_tf_bridge", true);
    }
}

void LidarListener::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received /reacsa/tf with %zu transforms", msg->transforms.size());
    for (const auto &t : msg->transforms) {
        tf_buffer_->setTransform(t, "reacsa_tf_bridge", false);
    }
}


void LidarListener::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "LaserScan received: angle_min=%.2f, angle_max=%.2f, ranges.size=%zu",
    msg->angle_min, msg->angle_max, msg->ranges.size());
    
    for (size_t i = 0; i < std::min(msg->ranges.size(), size_t(5)); ++i)
    RCLCPP_DEBUG(this->get_logger(), "range[%zu]=%.2f", i, msg->ranges[i]);
}

void LidarListener::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "PointCloud2 received: width=%u, height=%u, point_step=%u", msg->width, msg->height, msg->point_step);

    sensor_msgs::msg::PointCloud2 obstacle_msg;
    sensor_msgs::msg::PointCloud2 plane_msg;
    visualization_msgs::msg::Marker marker;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    int count = 0;
    for (; iter_x != iter_x.end() && count < 5; ++iter_x, ++iter_y, ++iter_z, ++count)
    {
        RCLCPP_DEBUG(this->get_logger(), "Point[%d]: x=%.2f, y=%.2f, z=%.2f", count, *iter_x, *iter_y, *iter_z);
    }
    
    // Transform From the lidar frame to the world frame
    RCLCPP_DEBUG(get_logger(), "Frames in buffer:\n%s", tf_buffer_->allFramesAsString().c_str());
    std::string target_frame = "world";
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_->lookupTransform(
            target_frame,            // frame di destinazione
            msg->header.frame_id,    // frame sorgente (lidar)
            msg->header.stamp,       // timestamp "safe"
            rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
        return;
    }


    sensor_msgs::msg::PointCloud2 cloud_transformed;
    tf2::doTransform(*msg, cloud_transformed, transform_stamped);

    // Filtering points using Voxel Grid Filtering
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(cloud_transformed, *cloud);    
    float voxelSize = 0.01f; // 1 cm voxel grid    
    auto cloud_filtered = filterPointCloud<pcl::PointXYZ>(cloud, voxelSize);
    
    RCLCPP_INFO(this->get_logger(), "Cloud size before filtering: %zu points", cloud->points.size());
    RCLCPP_INFO(this->get_logger(), "Filtered cloud size: %zu points", cloud_filtered->points.size());

    // Print first few points
    for (size_t i = 0; i < std::min<size_t>(5, cloud_filtered->points.size()); ++i)
    {
        const auto& p = cloud_filtered->points[i];
        RCLCPP_DEBUG(this->get_logger(), "Filtered Point[%zu]: x=%.2f, y=%.2f, z=%.2f", i, p.x, p.y, p.z);
    }

    // RANSAC Plane Segmentation
    int maxIterations = 500; // Number of iterations for RANSAC
    std::array<float,3> distanceThreshold = {0.15f, 0.15f, 0.15f}; // Distance threshold for inliers
    auto [obsCloud, planeCloud] = SegmentPlane<pcl::PointXYZ>(cloud_filtered, maxIterations, distanceThreshold);
    pcl::toROSMsg(*planeCloud, plane_msg);
    plane_msg.header = msg->header;
    plane_pub_->publish(plane_msg);
    RCLCPP_INFO(this->get_logger(), "Segmented Plane Cloud size: %zu points", planeCloud->points.size());
    RCLCPP_INFO(this->get_logger(), "Segmented Obstacles Cloud size: %zu points", obsCloud->points.size());

    // KD-Tree Clustering and Euclidean Clustering
    float clusterTolerance = 0.5f; // Distance tolerance for clustering
    int minSize = 10; // Minimum size of a cluster
    int maxSize = 5000; // Maximum size of a cluster
    auto clusters = ClusteringOptimized<pcl::PointXYZ>(obsCloud, clusterTolerance, minSize, maxSize);
    auto largestCluster = typename pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    RCLCPP_INFO(this->get_logger(), "Number of clusters found: %zu", clusters.size());
    // Let's take the one with the largest size as the main obstacle
    if(clusters.empty()){
        // If no clusters found, publish empty message
        RCLCPP_DEBUG(this->get_logger(), "No clusters found!");
        obstacle_msg.header = msg->header;
        obstacle_msg.height = 1;
        obstacle_msg.width = 0;
        obstacle_msg.is_bigendian = false;
        obstacle_msg.is_dense = true;
        obstacle_msg.point_step = 0;
        obstacle_msg.row_step = 0;
        obstacle_msg.fields.clear();
        obstacle_msg.data.clear();
        obstacle_pub_->publish(obstacle_msg);
    }else{
        // Find the largest cluster and publish it
        auto pointer = std::max_element(clusters.begin(), clusters.end(),
        [](const typename pcl::PointCloud<pcl::PointXYZ>::Ptr& a, const typename pcl::PointCloud<pcl::PointXYZ>::Ptr& b) {
            return a->points.size() < b->points.size();
        });
        largestCluster = *pointer;
        pcl::toROSMsg(*largestCluster, obstacle_msg);
        obstacle_msg.header = msg->header;
        obstacle_msg.header.frame_id = target_frame;
        obstacle_pub_->publish(obstacle_msg);
    }

    // Draw bounding box around the largest cluster
    if (!largestCluster->points.empty()) {
        // Box box = BoundingBox<pcl::PointXYZ>(largestCluster);
        Box box = BoundingBoxPCA<pcl::PointXYZ>(largestCluster);
        Box tracked_box = trackBoxNoKF(box);

        // Publish the pose estimation of the lidar
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.header.frame_id = target_frame;
        pose_msg.pose.position.x = getBoxCentre(tracked_box).x();
        pose_msg.pose.position.y = getBoxCentre(tracked_box).y();
        pose_msg.pose.position.z = getBoxCentre(tracked_box).z();
        pose_msg.pose.orientation.x = tracked_box.orientation.x();
        pose_msg.pose.orientation.y = tracked_box.orientation.y();
        pose_msg.pose.orientation.z = tracked_box.orientation.z();
        pose_msg.pose.orientation.w = tracked_box.orientation.w();
        pose_estimation_pub_->publish(pose_msg);

        // Publish the bounding box as a visualization marker
        marker.header.frame_id = target_frame;
        marker.header.stamp = this->now();
        marker.ns = "obstacle_bounding_box";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        Eigen::Vector3f center = getBoxCentre(tracked_box);
        marker.pose.position.x = center.x();
        marker.pose.position.y = center.y();
        marker.pose.position.z = center.z();
        marker.pose.orientation.x = tracked_box.orientation.x();
        marker.pose.orientation.y = tracked_box.orientation.y();
        marker.pose.orientation.z = tracked_box.orientation.z();
        marker.pose.orientation.w = tracked_box.orientation.w();
        marker.scale.x = tracked_box.x_max - tracked_box.x_min;
        marker.scale.y = tracked_box.y_max - tracked_box.y_min;
        marker.scale.z = tracked_box.z_max - tracked_box.z_min;
        marker.color.a = 0.5; // Transparency
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.3); // Marker lasts for 0.1 seconds
        marker_pub_->publish(marker);

    } else {
        RCLCPP_INFO(this->get_logger(), "Largest cluster is empty, no bounding box to compute.");
    }
}

int main(int argc, char const* argv[])
{
    rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LidarListener>());
	rclcpp::shutdown();
	return 0;
}
