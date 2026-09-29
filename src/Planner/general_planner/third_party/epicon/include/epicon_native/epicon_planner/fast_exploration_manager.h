// Imported from EPICON; namespace and runtime integration adapted for General Planner.
#pragma once

#include <Eigen/Eigen>
#include <epicon_native/frontier_manager/frontier_manager.h>
#include <memory>
#include <omp.h>
#include <opencv2/opencv.hpp>
#include <epicon_native/pointcloud_topo/graph.h>
#include <epicon_native/plan_manage/planner_manager.h>
#include <ros/ros.h>
#include <vector>

namespace epicon_native {
using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class EDTEnvironment;
class SDFMap;
class FastPlannerManager;
struct ExplorationParam;
struct ExplorationData;

enum EXPL_RESULT { NO_FRONTIER, FAIL, START_FAIL, SUCCEED, BLOCKED };

class FastExplorationManager {
public:
  typedef shared_ptr<FastExplorationManager> Ptr;
  FastExplorationManager();
  ~FastExplorationManager();
  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  ros::Timer frontier_timer_;
  FrontierManager::Ptr frontier_manager_ptr_;
  double goal_yaw{0.0};
  double last_goal_cost_{5e3};

  shared_ptr<FastPlannerManager> planner_manager_;
  // ViewpointForest::Ptr vps_forest_;
  double getPathCost(TopoNode::Ptr &n1, Eigen::Vector3d v1, float yaw1, TopoNode::Ptr &n2, float yaw2);
  double getPathCostWithoutTopo(TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1, TopoNode::Ptr &n2, float &yaw2);
  void initialize(ros::NodeHandle &nh, FrontierManager::Ptr frt_manager,
                  FastPlannerManager::Ptr planner_manager);
  int planGlobalPath(const Vector3d &pos, const Vector3d &vel, std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &unkownpoints_,
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &activepoints_, double hybrid_search_radius, double unknown_penalty_factor);
  void solveLHK(Eigen::MatrixXd &cost_mat, vector<int> &indices, bool skip_first = false, bool skip_last = false, int result_id_offset = 1);

  void surfaceFrtCalllback(const ros::TimerEvent &e);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void updateGoalNode();

  void calculateCostMatrix(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, Eigen::MatrixXd &cost_matrix,
    std::vector<TopoNode::Ptr> &unkownpoints, std::vector<TopoNode::Ptr> &activepoints, double hybrid_search_radius, double unknown_penalty_factor);

  void removeUnreachableNodes(std::vector<TopoNode::Ptr> &node_points);

  std::vector<int> dijkstra(std::vector<std::vector<double>> &graph, int start, int end);

  void refineLocalTourHGrid(Eigen::Vector3d &cur_vel, float cur_yaw, TopoNode::Ptr &next_pos, std::vector<TopoNode::Ptr> &viewpoints, 
    std::vector<double> &distances, std::vector<TopoNode::Ptr> &refined_viewpoints, std::vector<double> &refined_distances);

  Eigen::Vector3f last_node = Eigen::Vector3f::Zero();
};

} // namespace fast_planner


} // namespace epicon_native
