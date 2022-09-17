#ifndef OPENCV_PERSON_DETECTOR_H_
#define OPENCV_PERSON_DETECTOR_H_

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <vision_msgs/Detection2DArray.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <vector>

class CPersonDetector {
 public:
  CPersonDetector();  // Declare the constructor

  // Declare callback function
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // Declare the HOG detection function
  void imageDetection(cv::Mat& img);

  // Declare the detection publisher function
  void detectionsPublisher(const sensor_msgs::ImageConstPtr& msg);

  ~CPersonDetector() {}

 private:
  ros::NodeHandle nh;                    // ROS node Handle
  image_transport::Subscriber imageSub;  // Subscriber for camera image
  image_transport::Publisher imagePub;   // Publisher for visualization
  ros::Publisher msgPub;                 // Publisher for detections
  std::vector<cv::Rect> detections_;     // Rectangles of the detected objects

  // Declare the parameters
  double hit_threshold_;
  double stride_value_;
  cv::Size win_stride_;
};

#endif
