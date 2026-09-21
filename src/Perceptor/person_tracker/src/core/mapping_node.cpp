#include <ros/ros.h>
#include <iostream>
#include "core/mapping_manager.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "person_tracker");
  try {
    person_tracker::MappingRos node;
    ros::spin();
  } catch (const std::exception & error) {
    ROS_FATAL("Tracker initialization failed: %s", error.what());
    std::cerr << "Tracker initialization failed: " << error.what() << std::endl;
    return 1;
  }
  return 0;
}
