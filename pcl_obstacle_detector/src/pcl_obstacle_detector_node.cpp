#include "pcl_obstacle_detector.h"

int main(int argc, char** argv) {
  // Initiate the ROS node
  ros::init(argc, argv, "pcl_obstacle_detector_node");

  // Initiate the class for ObstacleDetector
  CObstacleDetector ObstacleDetector;

  // Wait for the point cloud data  input
  ros::spin();

  return 0;
}