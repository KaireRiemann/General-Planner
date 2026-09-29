#pragma once
#include <epicon_native/path_searching/bubble_astar.h>
#include <epicon_native/pointcloud_topo/graph_visualizer.hpp>
#include <tf/tf.h>
namespace epicon_native { namespace fast_planner {
// Only the point-cloud search context needed by upstream exploration. Actual
// trajectories are optimized by General Planner's ExplorationTrajOpt.
struct GcopterConfig {
  double maxTiltAngle{0.5}, gravAcc{9.8}, maxVelMag{2.0}, yaw_max_vel{1.2};
};
class FastPlannerManager {
public:
  using Ptr = std::shared_ptr<FastPlannerManager>;
  std::shared_ptr<GcopterConfig> gcopter_config_{new GcopterConfig};
  struct { double curr_yaw_{0.0}, end_yaw_{0.0}; } local_data_;
  LIOInterface::Ptr lidar_map_interface_;
  TopoGraph::Ptr topo_graph_;
  ParallelBubbleAstar::Ptr parallel_path_finder_;
  BubbleAstar::Ptr bubble_path_finder_;
  FastSearcher::Ptr fast_searcher_;
  GraphVisualizer::Ptr graph_visualizer_;
};
}}
