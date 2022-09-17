#ifndef CONTROL_BARREL_WORLD_H_
#define CONTROL_BARREL_WORLD_H_

#include <ros/ros.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection3DArray.h>
#include <cmath>
#include <vector>
#include <prius_msgs/Control.h>

// Declare the class about Prius Controller
class CPriusControl {
 public:
  CPriusControl();  // Declare the constructor

  // Declare the callback function of opencv_person_detector
  void personDetectorCallback(const vision_msgs::Detection2DArray& msg);

  // Declare the callback function of pcl_obstacle_detector
  void obstacleDetectorCallback(const vision_msgs::Detection3DArray& msg);

  // Declare the function to publish the control message
  void controlPublish();

  // Declare the function to get the closest y-value of the obstacle
  float getYValue(const std::vector<vision_msgs::Detection3D>& detections);

  ~CPriusControl() {}

 private:
  ros::NodeHandle nh;  // ROS node Handle
  ros::Subscriber personDetectorSub;
  ros::Subscriber obstacleDetectorSub;
  ros::Publisher controlPub;

  // Parameters
  bool personDetector_;    // Check if Prius should brake
  bool obstacleDetector_;  // Check if filtered set is empty
  float steer_;            // Check where the Prius should steer
};
#endif
