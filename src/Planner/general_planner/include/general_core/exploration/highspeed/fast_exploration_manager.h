/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-21 21:31:51
 * @LastEditTime: 2024-03-10 11:39:33
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */

#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <Eigen/Eigen>
#include <general_core/exploration/exploration_utils/frontier_manager/frontier_manager.h>
#include <general_core/exploration/exploration_utils/coverage_guidance/coverage_guidance_manager.h>
#include <memory>
#include <omp.h>
#include <opencv2/opencv.hpp>
#include <general_core/exploration/exploration_utils/pointcloud_topo/graph.h>
#include <general_core/exploration/highspeed/planner_manager.h>
#include <ros/ros.h>
#include <vector>
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

enum EXPL_RESULT { NO_FRONTIER, FAIL, START_FAIL, SUCCEED };

class FastExplorationManager {
public:
  typedef shared_ptr<FastExplorationManager> Ptr;
  FastExplorationManager();
  ~FastExplorationManager();
  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  ros::Timer frontier_timer_;
  FrontierManager::Ptr frontier_manager_ptr_;
  double goal_yaw;
  bool last_plan_empty_frontier_{false};
  bool last_plan_no_reachable_{false};
  bool last_plan_requires_reorientation_{false};
  bool high_speed_mode_active_{false};

  shared_ptr<FastPlannerManager> planner_manager_;
  CoverageGuidanceManager::Ptr coverage_guidance_;
  // ViewpointForest::Ptr vps_forest_;
  double getPathCost(TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1, TopoNode::Ptr &n2, float &yaw2);
  EdgeSafetyCost getPathEdgeCost(TopoNode::Ptr &n1,
                                 const Eigen::Vector3d &v1,
                                 float yaw1,
                                 TopoNode::Ptr &n2,
                                 float yaw2);
  double getPathCostWithoutTopo(TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1, TopoNode::Ptr &n2, float &yaw2);
  void initialize(ros::NodeHandle &nh, FrontierManager::Ptr frt_manager,
                  FastPlannerManager::Ptr planner_manager);
  int planGlobalPath(const Vector3d &pos, const Vector3d &vel);
  void updateCoverageGuidance(const Vector3d &pos);
  bool coverageGuidanceBlocksFinish() const;
  void deferCurrentGoalAfterPlanningFailure();
  int selectStableGoalIndex(const vector<TopoNode::Ptr> &viewpoints,
                            const vector<double> &distance_odom2vp,
                            int candidate_idx,
                            const Eigen::Vector3d &vel);
  void solveTour(Eigen::MatrixXd &cost_mat, vector<int> &indices);
  bool solveLKH(const Eigen::MatrixXd &cost_mat, vector<int> &indices);
  void solveFallbackTour(const Eigen::MatrixXd &cost_mat,
                         vector<int> &indices) const;

  void surfaceFrtCalllback(const ros::TimerEvent &e);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void updateGoalNode();

private:
  struct DeferredGoal {
    int cluster_id{-1};
    Eigen::Vector3f position{Eigen::Vector3f::Zero()};
    ros::Time until;
  };

  std::uint64_t coverage_map_version_{0};
  vector<DeferredGoal> deferred_goals_;

  double failedGoalPenalty(const TopoNode::Ptr &viewpoint) const;
};

} // namespace fast_planner

#endif
