/*** 
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-10-12 11:33:14
 * @LastEditTime: 2025-03-12 17:02:18
 * @Description: 
 * @
 * @Copyright (c) 2025 by ning-zelin, All Rights Reserved. 
 */


#include <general_core/exploration/highspeed/fast_exploration_fsm.h>
#include <general_core/exploration/highspeed/fast_exploration_manager.h>
#include <general_core/exploration/highspeed/dynamic_bounding_box_selector.h>
#include <general_core/exploration/exploration_utils/frontier_manager/frontier_manager.h>
#include <general_core/exploration/highspeed/planner_manager.h>
#include <ros/ros.h>
#include <vector>

namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char **argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");
  LIOInterface::Ptr lio_interface = std::make_shared<LIOInterface>();
  ParallelBubbleAstar::Ptr parallel_path_finder = std::make_shared<ParallelBubbleAstar>();
  FrontierManager::Ptr frontier_manager = std::make_shared<FrontierManager>();
  TopoGraph::Ptr graph = std::make_shared<TopoGraph>();
  FastPlannerManager::Ptr planner_manager = std::make_shared<FastPlannerManager>();
  FastExplorationManager::Ptr explore_manager = std::make_shared<FastExplorationManager>();
  FastExplorationFSM expl_fsm;

  DynamicBoundingBoxSelector bbox_selector;
  bbox_selector.init(nh);
  bool dynamic_box_selected = false;
  Eigen::Vector3f selected_min;
  Eigen::Vector3f selected_max;
  //阻塞等待动态搜索范围
  if (bbox_selector.enabled()) {
    if (bbox_selector.waitForSelection(selected_min, selected_max)) {
      dynamic_box_selected = true;
      const std::vector<double> min_param = {
          selected_min.x(), selected_min.y(), selected_min.z()};
      const std::vector<double> max_param = {
          selected_max.x(), selected_max.y(), selected_max.z()};
      // Override the private box parameters before LIOInterface reads them.
      // Consequently dynamic mode never even initializes from box_0/box_1.
      nh.setParam("box_num", 1);
      nh.setParam("box_0/down", min_param);
      nh.setParam("box_0/up", max_param);
    } else {
      if (!ros::ok()) {
        return 0;
      }
      ROS_FATAL("[dynamic bbox] selection failed; fixed YAML bounds are "
                "disabled while dynamic selection is enabled");
      return 3;
    }
  }
  //再依次初始化所有模块，注意初始化顺序，LIOInterface必须最先初始化
  lio_interface->init(nh);
  if (dynamic_box_selected) {
    if (!lio_interface->setSingleExplorationBox(selected_min, selected_max)) {
      ROS_FATAL("[dynamic bbox] selected box could not be applied");
      return 2;
    }
    ROS_INFO_STREAM(
        "[dynamic bbox] applied before LIO/graph/frontier/planner initialization"
        << " min=[" << selected_min.transpose() << "] max=["
        << selected_max.transpose() << "]");
  }
  graph->init(nh, lio_interface, parallel_path_finder);
  parallel_path_finder->init(nh, lio_interface);
  planner_manager->initPlanModules(nh, parallel_path_finder, graph);
  frontier_manager->init(nh, lio_interface, graph);
  explore_manager->initialize(nh, frontier_manager, planner_manager);
  expl_fsm.init(nh, explore_manager);

  // Startup must not depend on simulated time.  With /use_sim_time=true and
  // no /clock publisher, ros::Duration::sleep() blocks before ros::spin() and
  // no odometry or point-cloud callback can ever run.
  ros::WallDuration(1.0).sleep();
  ros::spin();
  ros::shutdown();
  return 0;
}
