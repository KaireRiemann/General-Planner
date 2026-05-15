#include "ros/ros.h"

#define BACKWARD_HAS_DW 1
#include "utils/backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}

#include "task_planner/ros1_task_planner.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "task_planner");
    ros::NodeHandle nh("~");

    task_planner::TaskPlanner task_planner(nh);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}
