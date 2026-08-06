#include <general_core/planner_runtime/planner_command_gateway.hpp>
#include <general_core/planner_runtime/planner_supervisor.hpp>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_runtime_node");
  ros::NodeHandle nh("~");
  general_planner::planner_runtime::PlannerCommandGateway gateway(nh);
  general_planner::planner_runtime::PlannerSupervisor supervisor(nh, gateway);
  ros::spin();
  return 0;
}
