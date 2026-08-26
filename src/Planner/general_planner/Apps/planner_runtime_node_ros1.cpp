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
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/topic.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/**
 * The HighSpeedExp frontend predates the composed runtime and requires a
 * finite index domain for its frontier key packing and Bubble-Topo regions.
 * A destination task must not, however, require operators to measure and
 * configure the whole scene before clicking a goal.  This bootstrap creates a
 * sufficiently large *internal capacity domain* around the first valid
 * odometry pose.  It is not a coverage boundary: target-directed ranking,
 * known-free commit checks, and the ROG map still decide where the vehicle may
 * travel.
 *
 * It deliberately runs before LIO, FrontierManager and TopoGraph are created;
 * those structures cache the domain origin/dimensions and cannot safely be
 * resized after startup.  Coverage deployments can disable it and retain the
 * explicit multi-box profile in their YAML.
 */
bool configureAutomaticTargetWorkspace(ros::NodeHandle &nh) {
  bool enabled = false;
  nh.param("target_exploration/auto_workspace/enabled", enabled, enabled);
  if (!enabled) {
    return false;
  }

  double half_extent_xy = 120.0;
  double below_odom = 2.0;
  double above_odom = 5.0;
  double min_xy_extent = 20.0;
  double min_z_extent = 2.0;
  double odom_wait_timeout = 5.0;
  nh.param("target_exploration/auto_workspace/half_extent_xy", half_extent_xy,
           half_extent_xy);
  nh.param("target_exploration/auto_workspace/below_odom", below_odom,
           below_odom);
  nh.param("target_exploration/auto_workspace/above_odom", above_odom,
           above_odom);
  nh.param("target_exploration/auto_workspace/min_xy_extent", min_xy_extent,
           min_xy_extent);
  nh.param("target_exploration/auto_workspace/min_z_extent", min_z_extent,
           min_z_extent);
  nh.param("target_exploration/auto_workspace/odom_wait_timeout",
           odom_wait_timeout, odom_wait_timeout);

  half_extent_xy = std::clamp(half_extent_xy, 10.0, 500.0);
  below_odom = std::clamp(below_odom, 0.5, 100.0);
  above_odom = std::clamp(above_odom, 0.5, 100.0);
  min_xy_extent = std::clamp(min_xy_extent, 1.0, 2.0 * half_extent_xy);
  min_z_extent = std::clamp(min_z_extent, 0.5, below_odom + above_odom);
  odom_wait_timeout = std::clamp(odom_wait_timeout, 0.0, 30.0);

  std::string odom_topic{"/lidar_slam/odom"};
  nh.param<std::string>("odometry_topic", odom_topic, odom_topic);
  const auto odom = ros::topic::waitForMessage<nav_msgs::Odometry>(
      odom_topic, nh, ros::Duration(odom_wait_timeout));

  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  bool valid_odom = false;
  if (odom) {
    center = Eigen::Vector3d(odom->pose.pose.position.x,
                             odom->pose.pose.position.y,
                             odom->pose.pose.position.z);
    valid_odom = center.allFinite();
  }
  if (!valid_odom) {
    ROS_WARN_STREAM(
        "[target workspace] no valid odometry on " << odom_topic << " within "
        << odom_wait_timeout << "s; bootstrap around world origin. "
        "The first target must lie within the configured automatic capacity.");
    center.setZero();
  }

  const double half_xy = std::max(half_extent_xy, 0.5 * min_xy_extent);
  double minimum_z = center.z() - below_odom;
  double maximum_z = center.z() + above_odom;
  if (maximum_z - minimum_z < min_z_extent) {
    const double padding = 0.5 * (min_z_extent - (maximum_z - minimum_z));
    minimum_z -= padding;
    maximum_z += padding;
  }
  const Eigen::Vector3d minimum(center.x() - half_xy, center.y() - half_xy,
                                minimum_z);
  const Eigen::Vector3d maximum(center.x() + half_xy, center.y() + half_xy,
                                maximum_z);
  nh.setParam("box_num", 1);
  nh.setParam("box_0/down", std::vector<double>{minimum.x(), minimum.y(),
                                                   minimum.z()});
  nh.setParam("box_0/up", std::vector<double>{maximum.x(), maximum.y(),
                                                 maximum.z()});
  ROS_INFO_STREAM("[target workspace] automatic capacity "
                  << (valid_odom ? "centered at odometry" : "centered at origin")
                  << " center=[" << center.transpose() << "] xy_half_extent="
                  << half_xy << " z=[" << minimum.z() << ", " << maximum.z()
                  << "]; no scene bounding box is required");
  return true;
}

}  // namespace

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
    // Keep the execution domains independent.  In particular, a large PCL
    // fusion, an exploration-frontier rebuild, or a stuck state2state
    // frontend/optimizer must never delay supervisor handover, navigation
    // control, or the state2state command source.
    ros::CallbackQueue world_callback_queue;
    ros::CallbackQueue navigation_callback_queue;
    ros::CallbackQueue navigation_command_callback_queue;
    ros::CallbackQueue navigation_replan_callback_queue;
    ros::CallbackQueue supervisor_callback_queue;
    ros::CallbackQueue gateway_callback_queue;
    ros::NodeHandle world_nh(nh);
    ros::NodeHandle navigation_nh(nh);
    ros::NodeHandle navigation_command_nh(nh);
    ros::NodeHandle navigation_replan_nh(nh);
    ros::NodeHandle supervisor_nh(nh);
    ros::NodeHandle gateway_nh(nh);
    world_nh.setCallbackQueue(&world_callback_queue);
    navigation_nh.setCallbackQueue(&navigation_callback_queue);
    navigation_command_nh.setCallbackQueue(&navigation_command_callback_queue);
    navigation_replan_nh.setCallbackQueue(&navigation_replan_callback_queue);
    supervisor_nh.setCallbackQueue(&supervisor_callback_queue);
    gateway_nh.setCallbackQueue(&gateway_callback_queue);

    auto global_map_runtime = std::make_shared<GlobalMapRuntime>();
    global_map_runtime->init(world_nh, global_map_config);

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

    // Dynamic RViz corner selection is retained for coverage missions.  A
    // target-exploration launch normally chooses the automatic workspace below
    // instead, so its first interaction remains a single destination click.
    const bool automatic_workspace = configureAutomaticTargetWorkspace(nh);
    fast_planner::DynamicBoundingBoxSelector bbox_selector;
    bbox_selector.init(nh);
    bool dynamic_box_selected = false;
    Eigen::Vector3f selected_min;
    Eigen::Vector3f selected_max;
    if (bbox_selector.enabled() && automatic_workspace) {
      ROS_FATAL("[M2 runtime] automatic target workspace and dynamic bounding "
                "box selection cannot be enabled together");
      return 3;
    }
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

    lio_interface->init(world_nh);
    if (dynamic_box_selected &&
        !lio_interface->setSingleExplorationBox(selected_min, selected_max)) {
      ROS_FATAL("[M2 runtime] selected exploration bounding box is invalid");
      return 3;
    }
    bubble_graph->init(world_nh, lio_interface, parallel_path_finder);
    parallel_path_finder->init(world_nh, lio_interface);
    exploration_planner->initPlanModules(
        world_nh, parallel_path_finder, bubble_graph,
        global_map_runtime->mapManager());
    frontier_manager->init(world_nh, lio_interface, bubble_graph);
    exploration_manager->initialize(world_nh, frontier_manager,
                                    exploration_planner);
    exploration_fsm->init(world_nh, exploration_manager, true);

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
    navigation_fsm->init(navigation_nh, navigation_config,
                         global_map_runtime->mapManager(),
                         navigation_command_topic,
                         navigation_command_nh,
                         navigation_replan_nh);

    // The gateway is the last safety boundary and the supervisor owns task
    // handover.  Neither is allowed to share the map/exploration queue.
    general_planner::planner_runtime::PlannerCommandGateway gateway(gateway_nh);
    general_planner::planner_runtime::PlannerSupervisor supervisor(
        supervisor_nh, gateway,
        [global_map_runtime]() { return global_map_runtime->status(); });

    ROS_INFO("[M2 runtime] composed exploration + state2state adapters share "
             "one GlobalMapRuntime; queues=world,navigation,nav_command,"
             "nav_replan,supervisor,gateway (one serial thread each)");
    ros::AsyncSpinner world_spinner(1, &world_callback_queue);
    ros::AsyncSpinner navigation_spinner(1, &navigation_callback_queue);
    ros::AsyncSpinner navigation_command_spinner(
        1, &navigation_command_callback_queue);
    ros::AsyncSpinner navigation_replan_spinner(
        1, &navigation_replan_callback_queue);
    ros::AsyncSpinner supervisor_spinner(1, &supervisor_callback_queue);
    ros::AsyncSpinner gateway_spinner(1, &gateway_callback_queue);
    world_spinner.start();
    navigation_spinner.start();
    navigation_command_spinner.start();
    navigation_replan_spinner.start();
    supervisor_spinner.start();
    gateway_spinner.start();
    ros::waitForShutdown();
    gateway_spinner.stop();
    supervisor_spinner.stop();
    navigation_replan_spinner.stop();
    navigation_command_spinner.stop();
    navigation_spinner.stop();
    world_spinner.stop();
  } catch (const std::exception &error) {
    ROS_FATAL_STREAM("[M2 runtime] initialization failed: " << error.what());
    return 1;
  }
  return 0;
}
