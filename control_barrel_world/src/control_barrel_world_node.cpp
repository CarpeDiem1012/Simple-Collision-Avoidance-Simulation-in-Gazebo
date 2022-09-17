#include "control_barrel_world.h"

int main(int argc, char** argv) {
  // Initiate the ROS node
  ros::init(argc, argv, "control_barrel_world_node");

  // Initiate the class for PriusControl
  CPriusControl PriusControl;

  ros::Rate rate(30);

  while (ros::ok()) {
    // Publsih the control message
    PriusControl.controlPublish();

    ros::spinOnce();  // Wait for detections callback

    rate.sleep();
  }

  return 0;
}