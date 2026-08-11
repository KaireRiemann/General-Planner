#include <general_core/planner_runtime/planner_command_gateway.hpp>
#include <general_core/planner_runtime/global_map_runtime.hpp>
#include <general_core/planner_runtime/planner_supervisor.hpp>

// This must precede the HighSpeedExp headers.  A legacy HighSpeedExp header
// exports `using namespace fast_planner`, whose template Trajectory otherwise
// collides with geometry_utils::Trajectory while the ROS1 FSM visualizer is
// parsed in this combined translation unit.
#include <ros_interface/ros1/fsm_ros1.hpp>

#include <general_core/exploration/exploration_utils/frontier_manager/frontier_manager.h>
#include <general_core/exploration/exploration_utils/lidar_map/lidar_map.h>
#include <general_core/exploration/exploration_utils/pointcloud_topo/graph.h>
#include <general_core/exploration/exploration_utils/pointcloud_topo/parallel_bubble_astar.h>
#include <general_core/exploration/highspeed/dynamic_bounding_box_selector.h>
#include <general_core/exploration/highspeed/fast_exploration_fsm.h>
#include <general_core/exploration/highspeed/fast_exploration_manager.h>
#include <general_core/exploration/highspeed/planner_manager.h>
#include <ros/ros.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_runtime_node");
  ros::NodeHandle nh("~");
  using general_planner::planner_runtime::GlobalMapRuntime;

  std::string global_map_config;
  std::string navigation_config;
  std::string navigation_command_topic{"/planning/navigation/pos_cmd"};
  nh.param<std::string>("global_map_config", global_map_config, "");
  nh.param<std::string>("navigation_config", navigation_config, "");
  nh.param<std::string>("navigation_cmd_topic", navigation_command_topic,
                        navigation_command_topic);
  if (global_map_config.empty() || navigation_config.empty()) {
    ROS_FATAL("M2 planner_runtime_node requires ~global_map_config and "
              "~navigation_config");
    return 2;
  }

  try {
    auto global_map_runtime = std::make_shared<GlobalMapRuntime>();
    global_map_runtime->init(nh, global_map_config);

    // Keep LIO/Bubble/frontier task structures local to exploration, but make
    // the LIO evidence and ROG safety map world-lifetime resources.
    auto lio_interface = std::make_shared<fast_planner::LIOInterface>();
    auto parallel_path_finder =
        std::make_shared<ParallelBubbleAstar>();
    auto frontier_manager = std::make_shared<FrontierManager>();
    auto bubble_graph = std::make_shared<TopoGraph>();
    auto exploration_planner =
        std::make_shared<fast_planner::FastPlannerManager>();
    auto exploration_manager =
        std::make_shared<fast_planner::FastExplorationManager>();
    auto exploration_fsm = std::make_shared<fast_planner::FastExplorationFSM>();

    fast_planner::DynamicBoundingBoxSelector bbox_selector;
    bbox_selector.init(nh);
    bool dynamic_box_selected = false;
    Eigen::Vector3f selected_min;
    Eigen::Vector3f selected_max;
    if (bbox_selector.enabled()) {
      if (!bbox_selector.waitForSelection(selected_min, selected_max)) {
        if (!ros::ok()) {
          return 0;
        }
        ROS_FATAL("[M2 runtime] dynamic exploration bounding-box selection failed");
        return 3;
      }
      dynamic_box_selected = true;
      nh.setParam("box_num", 1);
      nh.setParam("box_0/down", std::vector<double>{
          selected_min.x(), selected_min.y(), selected_min.z()});
      nh.setParam("box_0/up", std::vector<double>{
          selected_max.x(), selected_max.y(), selected_max.z()});
    }

    lio_interface->init(nh);
    if (dynamic_box_selected &&
        !lio_interface->setSingleExplorationBox(selected_min, selected_max)) {
      ROS_FATAL("[M2 runtime] selected exploration bounding box is invalid");
      return 3;
    }
    bubble_graph->init(nh, lio_interface, parallel_path_finder);
    parallel_path_finder->init(nh, lio_interface);
    exploration_planner->initPlanModules(
        nh, parallel_path_finder, bubble_graph,
        global_map_runtime->mapManager());
    frontier_manager->init(nh, lio_interface, bubble_graph);
    exploration_manager->initialize(nh, frontier_manager, exploration_planner);
    exploration_fsm->init(nh, exploration_manager, true);

    global_map_runtime->attachLioMap(lio_interface);
    global_map_runtime->addOdomConsumer(
        [exploration_fsm](const nav_msgs::OdometryConstPtr &odom) {
          exploration_fsm->ingestOdometry(odom);
        });
    global_map_runtime->addCloudConsumer(
        [exploration_fsm](const sensor_msgs::PointCloud2ConstPtr &cloud,
                          const nav_msgs::OdometryConstPtr &odom,
                          const general_planner::MapManager::UpdateSnapshot &update) {
          exploration_fsm->ingestCloudOdom(cloud, odom);
          // The local exploration modules use this only as a revision fence;
          // the actual ROG update already happened in GlobalMapRuntime.
          (void)update;
        });

    auto navigation_fsm = std::make_shared<fsm::FsmRos1>();
    navigation_fsm->init(nh, navigation_config,
                         global_map_runtime->mapManager(),
                         navigation_command_topic);

    general_planner::planner_runtime::PlannerCommandGateway gateway(nh);
    general_planner::planner_runtime::PlannerSupervisor supervisor(
        nh, gateway,
        [global_map_runtime]() { return global_map_runtime->status(); });

    ROS_INFO("[M2 runtime] composed exploration + state2state adapters share "
             "one GlobalMapRuntime; serial handover must remain disabled");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();
    spinner.stop();
  } catch (const std::exception &error) {
    ROS_FATAL_STREAM("[M2 runtime] initialization failed: " << error.what());
    return 1;
  }
  return 0;
}
