#pragma once

#include <string>

#include <general_core/planning_semantics.hpp>

namespace ros_interface {

struct RosAdapterContract {
    std::string adapter_name{"ros_adapter"};
    std::string odom_topic;
    std::string cloud_topic;
    std::string target_topic;
    std::string command_topic;
    std::string trajectory_topic;
    general_planner::architecture::MissionMode mission{
            general_planner::architecture::MissionMode::IDLE};
};

} // namespace ros_interface
