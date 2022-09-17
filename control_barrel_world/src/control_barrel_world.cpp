#include "control_barrel_world.h"

// Define the constructor of the PriusControl class
CPriusControl::CPriusControl()
{
  // Define the Subscriber to subscribe the person detections
  personDetectorSub =
      nh.subscribe("/opencv_person_detector_node/detections", 10,
                   &CPriusControl::personDetectorCallback, this);

  // Define the Subscriber to subscribe the obstacle detections
  obstacleDetectorSub =
      nh.subscribe("/pcl_obstacle_detector_node/detections", 10,
                   &CPriusControl::obstacleDetectorCallback, this);

  // Define the publisher to publish the control messages
  controlPub = nh.advertise<prius_msgs::Control>("/prius", 10);

  // Initiate the parameters
  personDetector_ = false;
  obstacleDetector_ = false;
  steer_ = 0;

  // Print the initialization message
  ROS_INFO("Controller Initialized...");
}

// Define the callback function of pcl_obstacle_detector
void CPriusControl::obstacleDetectorCallback(
    const vision_msgs::Detection3DArray &msg)
{
  // Declare the vector to store the obstacle within range
  std::vector<vision_msgs::Detection3D> detections_in_range;

  // Store all obstacles within range to the vector
  for (size_t i = 0; i < msg.detections.size(); i++)
  {
    geometry_msgs::Pose center = msg.detections[i].bbox.center;
    float center_x = center.position.x;
    float center_y = center.position.y;
    float center_dist = sqrt(pow(center_x, 2) + pow(center_y, 2));
    if (center_x > 0 && center_dist <= 4)
    {
      detections_in_range.push_back(msg.detections[i]);
    }
  }

  // Define the state parameters
  if (detections_in_range.size() == 0)
  {
    obstacleDetector_ = false; // Empty detections
  }
  else
  {
    obstacleDetector_ = true; // Filtered set has detections
    float closest_y = getYValue(detections_in_range);
    if (closest_y > 0)
    {
      steer_ = -1; // Turn right
    }
    else
    {
      steer_ = 1; // Trun left
    }
  }
}

// Define the function to get the closest y-value of the obstacle
float CPriusControl::getYValue(
    const std::vector<vision_msgs::Detection3D> &detections_in_range)
{
  float closest_y_;
  float closest_dist = 4.0;

  // Iterate each obstacle to get the closest Y-value
  for (size_t i = 0; i < detections_in_range.size(); i++)
  {
    geometry_msgs::Pose center_ = detections_in_range[i].bbox.center;
    float center_x_ = center_.position.x;
    float center_y_ = center_.position.y;
    float center_dist_ = sqrt(pow(center_x_, 2) + pow(center_y_, 2));
    // Update the closest obstacle
    if (closest_dist > center_dist_)
    {
      closest_dist = center_dist_;
      closest_y_ = center_y_;
    }
  }
  ROS_INFO_STREAM("[CONTROLLER]:The closest obstacle is " << closest_dist
                                                          << " away.");
  return closest_y_;
}

// Define the callback function of opencv_person_detector
void CPriusControl::personDetectorCallback(
    const vision_msgs::Detection2DArray &msg)
{
  // Iterate each person to determine if Prius should brake
  for (size_t i = 0; i < msg.detections.size(); i++)
  {
    float size_x = msg.detections[i].bbox.size_x;
    float size_y = msg.detections[i].bbox.size_y;
    if (size_x * size_y > 25000)
    {
      personDetector_ = true;
      break;
    }
    else
    {
      personDetector_ = false;
    }
  }
}

void CPriusControl::controlPublish()
{
  // Declare the control message
  prius_msgs::Control ctl;

  // Define the control algorithm
  if (personDetector_ == true)
  {
    // Define the braking situation
    ctl.brake = 1;
    ctl.steer = 0;
    ctl.throttle = 0;
    ctl.shift_gears = 0;
    ROS_INFO("[CONTROLLER]:Detected close pedestrian. BRAKE!\n");
  }
  else
  {
    if (obstacleDetector_ == false)
    {
      // Define the situation with no detection
      ctl.steer = 0;
      ctl.throttle = 1;
      ctl.shift_gears = 2;
      ROS_INFO("[CONTROLLER]:GO FORWARD!\n");
    }
    else
    {
      // Define the situation with steering
      ctl.steer = steer_;
      ctl.throttle = 1;
      ctl.shift_gears = 2;
      ROS_INFO("[CONTROLLER]:STEERING!\n");
    }
  }
  controlPub.publish(ctl); // Publish the control message
}
