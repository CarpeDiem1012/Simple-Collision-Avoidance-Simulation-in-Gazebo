#include <opencv_person_detector.h>

// Define the constructor of the PersonDetector class
CPersonDetector::CPersonDetector() {
  // Create an ImageTransport instance, initializing it with NodeHandle
  image_transport::ImageTransport it(nh);

  // Define an Image Subscriber to subscribe the camera image
  imageSub = it.subscribe("/prius/front_camera/image_raw", 1,
                          &CPersonDetector::imageCallback, this);

  // Define the Image Publisher to publish the visualization of detections
  imagePub = it.advertise("/opencv_person_detector_node/visual", 1);

  // Define the publisher to publish the detection messages
  msgPub = nh.advertise<vision_msgs::Detection2DArray>(
      "/opencv_person_detector_node/detections", 10);

  // Define the parameters
  ros::param::get("hit_threshold_", hit_threshold_);
  ros::param::get("stride_value_", stride_value_);
  win_stride_ = cv::Size(stride_value_, stride_value_);

  // Print the initialization message
  ROS_INFO("Person Detector Initialized...");
}

// Define the callback function
void CPersonDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  // Convert the ROS image message to an OpenCV image
  cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

  // Use HOG Detector to draw the person in the image
  imageDetection(img);

  // Converted the message from the image
  sensor_msgs::ImagePtr msg_ =
      cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
  // Publish the visualization of detections to Rviz
  imagePub.publish(msg_);

  // Publish the detections to vision_msgs
  detectionsPublisher(msg);
}

// Define the HOG Person Detection function
void CPersonDetector::imageDetection(cv::Mat &img) {
  // Detect the person in the images
  cv::HOGDescriptor hog;
  hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
  hog.detectMultiScale(img, detections_, hit_threshold_, win_stride_);

  // Draw the rectangles on the images
  int num_detect = detections_.size();
  for (size_t i = 0; i < num_detect; i++) {
    cv::Rect r = detections_[i];
    cv::rectangle(img, r.tl(), r.br(), cv::Scalar(0, 255, 0), 3);
  }

  // Print the detection result
  ROS_INFO("[CAMERA]:Detected %d persons", num_detect);
}

// Define the detectionsPublisher function
void CPersonDetector::detectionsPublisher(
    const sensor_msgs::ImageConstPtr &msg) {
  // Declare the Detection2DArray message
  vision_msgs::Detection2DArray det_array;

  det_array.header = msg->header;  // Define the header of det_array

  // Define the detections of det_array
  for (size_t i = 0; i < detections_.size(); i++) {
    vision_msgs::Detection2D det_;
    det_.header = msg->header;
    // Define the bbox members of dec_ from cv::Rect
    cv::Rect r = detections_[i];
    det_.bbox.size_x = r.width;
    det_.bbox.size_y = r.height;
    det_.bbox.center.x = r.x + r.width / 2.0;
    det_.bbox.center.y = r.y + r.height / 2.0;

    det_array.detections.push_back(det_);
  }
  msgPub.publish(det_array);  // Publish the detections
}