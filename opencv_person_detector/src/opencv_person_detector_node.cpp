#include "opencv_person_detector.h"

int main(int argc, char** argv) {
  // Initiate the ROS node
  ros::init(argc, argv, "opencv_person_detector_node");

  // Initiate the class for PersonDetector
  CPersonDetector PersonDetector;

  // Wait for the image input
  ros::spin();

  return 0;
}