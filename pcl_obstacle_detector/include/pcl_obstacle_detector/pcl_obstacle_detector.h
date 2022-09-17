#ifndef PCL_OBSTACLE_DETECTOR_H_
#define PCL_OBSTACLE_DETECTOR_H_

#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3DArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

class CObstacleDetector
{
public:
    CObstacleDetector(); // Declare the constructor

    // Declare the callback function of Subcriber
    void pclCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    // Declare the function of removing ground plane from point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr GroundPlaneRemoval(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in);

    // Declare the function to extract the cluster from filtered point cloud
    std::vector<pcl::PointIndices> clusterExtraction(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered);

    // Declare the function to publish the clusters
    void clusterPublisher(const sensor_msgs::PointCloud2ConstPtr &msg,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered);

    ~CObstacleDetector() {}

private:
    ros::NodeHandle nh;     // ROS node Handle
    ros::Subscriber pclSub; // Subscriber for point cloud data
    ros::Publisher pclPub;  // Publisher for detections

    // Parameters
    double distance_threshold_;
    double cluster_tolerance_;
    double min_cluster_size_;
    double max_cluster_size_;
};

#endif