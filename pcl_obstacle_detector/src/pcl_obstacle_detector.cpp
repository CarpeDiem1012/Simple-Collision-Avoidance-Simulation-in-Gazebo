#include "pcl_obstacle_detector.h"

// Define the constructor of the ObstacleDetector class
CObstacleDetector::CObstacleDetector()
{
  // Define the Subscriber to subscribe the point cloud data
  pclSub =
      nh.subscribe("/point_cloud", 100, &CObstacleDetector::pclCallback, this);

  // Define the publisher to publish the detection messages
  pclPub = nh.advertise<vision_msgs::Detection3DArray>(
      "/pcl_obstacle_detector_node/detections", 100);

  // Load Parameters
  ros::param::get("distance_threshold", distance_threshold_);
  ros::param::get("cluster_tolerance", cluster_tolerance_);
  ros::param::get("min_cluster_size", min_cluster_size_);
  ros::param::get("max_cluster_size", max_cluster_size_);

  // Print the initialization message
  ROS_INFO("Obstacle Detector Initialized...");
}

// Define the callback function
void CObstacleDetector::pclCallback(
    const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // Initiate the pointers for point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Transform the point cloud from the message
  pcl::fromROSMsg(*msg, *cloud_in);

  // Obtain the filtered point cloud without ground plane
  cloud_filtered = GroundPlaneRemoval(cloud_in);

  // Publish the converted clusters
  // Cluster extraction will be performed within clusterPublisher
  clusterPublisher(msg, cloud_filtered);
}

// Define the function of removing ground plane from point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr CObstacleDetector::GroundPlaneRemoval(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in)
{
  // Initiate the pointers for segment model
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold_);

  // Perform the model segmentation
  seg.setInputCloud(cloud_in);
  seg.segment(*inliers, *coefficients);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Extract the inliers
  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setNegative(true); // Remove the ground plane
  extract.filter(*cloud_filtered);

  /*ROS_INFO_STREAM("Point cloud after filtering has " << cloud_filtered->size()
                                                     << " data points.");*/
  return cloud_filtered;
}

// Define the function to extract the cluster from filtered point cloud
std::vector<pcl::PointIndices> CObstacleDetector::clusterExtraction(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
{
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);

  // Create a vector to store cluster
  std::vector<pcl::PointIndices> cluster_indices;

  // Perform the cluster extraction
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  ROS_INFO_STREAM("[LiDAR]:Detected " << cluster_indices.size()
                                      << " obstacles.");
  return cluster_indices;
}

// Define the function to publisih the clusters
void CObstacleDetector::clusterPublisher(
    const sensor_msgs::PointCloud2ConstPtr &msg,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
{
  // Obtain the extracted clusters from point cloud
  std::vector<pcl::PointIndices> cluster_indices =
      clusterExtraction(cloud_filtered);

  // Define the vision_msgs
  vision_msgs::Detection3DArray det_array;
  // Pass the header
  det_array.header = msg->header;
  // Define each element into Dectection3D
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto &idx : it->indices)
    {
      cloud_cluster->push_back((*cloud_filtered)[idx]);
    }

    Eigen::Vector4f centroid; // Declare the centroid matrix
    pcl::PointXYZ min_pt;     // Declare the point with min_distance
    pcl::PointXYZ max_pt;     // Declare the point with max_distance

    pcl::compute3DCentroid(*cloud_cluster, centroid);
    pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

    // Define the individual cluster message Detection3D
    vision_msgs::Detection3D det_;
    det_.header = msg->header;
    det_.bbox.center.position.x = centroid[0];
    det_.bbox.center.position.y = centroid[1];
    det_.bbox.center.position.z = centroid[2];
    det_.bbox.size.x = max_pt.x - min_pt.x;
    det_.bbox.size.y = max_pt.y - min_pt.y;
    det_.bbox.size.z = max_pt.z - min_pt.z;

    // Add the Detection3D det_ to the Array
    det_array.detections.push_back(det_);
  }
  // Publish the Detection3DArray message
  pclPub.publish(det_array);
}
