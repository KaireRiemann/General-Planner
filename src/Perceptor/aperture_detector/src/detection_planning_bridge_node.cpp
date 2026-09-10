#include "detector/detection_planning_bridge.hpp"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection_planning_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  detector::DetectionPlanningBridge bridge(nh, pnh);
  ros::spin();
  return 0;
}
