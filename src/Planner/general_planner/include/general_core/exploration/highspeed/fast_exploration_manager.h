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
#include <limits>
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

struct CoverageFinishStatus {
  bool guard_enabled{false};
  bool plan_valid{false};
  bool plateau_reached{false};
  bool targets_exhausted{false};
  int observed_voxels{0};
  int valid_voxels{0};
  int actionable_targets{0};
  int eligible_targets{0};
  int cooling_targets{0};
  int exhausted_targets{0};
  double coverage_ratio{0.0};
  double plateau_duration{0.0};
  double next_retry_duration{0.0};

  bool ready() const {
    return !guard_enabled ||
           (plan_valid && plateau_reached && targets_exhausted);
  }
};

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
  CoverageFinishStatus coverageFinishStatus();
  void deferCurrentGoalAfterPlanningFailure();
  bool completeActiveCoverageGoalIfReached(const Vector3d &pos);
  bool hasActiveCoverageRecoveryGoal() const {
    return has_active_coverage_goal_;
  }
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

  struct DeferredCoverageGoal {
    std::uint64_t stable_id{0};
    Eigen::Vector3d approach{Eigen::Vector3d::Zero()};
    Eigen::Vector3d unknown_position{Eigen::Vector3d::Zero()};
    int voxel_count{0};
    ros::Time until;
    int no_gain_attempts{0};
    int failure_attempts{0};
    bool exhausted{false};
  };

  struct NormalGoalProgress {
    bool valid{false};
    int cluster_id{-1};
    Eigen::Vector3f goal{Eigen::Vector3f::Zero()};
    double best_route_cost{std::numeric_limits<double>::infinity()};
    double best_goal_distance{std::numeric_limits<double>::infinity()};
    ros::Time last_progress_time;
    ros::Time last_sample_time;
    int samples{0};
  };

  enum class CoverageRecoveryOutcome {
    REACHED,
    TIMEOUT,
    TRAJECTORY_FAILURE,
    UNSAFE,
    DISCONNECTED,
    OCCLUDED,
    FRONTIER_RESUMED
  };

  std::uint64_t coverage_map_version_{0};
  vector<DeferredGoal> deferred_goals_;
  vector<DeferredCoverageGoal> deferred_coverage_goals_;
  bool has_active_coverage_goal_{false};
  CoverageTarget active_coverage_target_;
  ros::Time active_coverage_goal_start_;
  int active_coverage_observed_voxels_{-1};
  int coverage_finish_progress_observed_voxels_{-1};
  ros::Time coverage_finish_last_progress_time_;
  double coverage_recovery_cooldown_{45.0};
  double coverage_recovery_timeout_{25.0};
  double coverage_recovery_match_radius_{1.8};
  double coverage_recovery_reached_radius_{1.0};
  double coverage_finish_plateau_duration_{20.0};
  int coverage_finish_min_progress_voxels_{12};
  int coverage_recovery_min_gain_voxels_{12};
  int coverage_recovery_max_no_gain_attempts_{2};
  int coverage_recovery_max_failure_attempts_{2};
  bool coverage_terminal_retry_enable_{true};
  double coverage_terminal_retry_interval_{2.0};
  bool coverage_executable_candidate_enable_{true};
  int coverage_executable_candidate_max_count_{8};
  int coverage_executable_empty_min_count_{4};
  int coverage_executable_empty_count_{0};
  double coverage_executable_empty_min_duration_{1.5};
  ros::Time coverage_executable_empty_since_;
  bool force_coverage_fallback_once_{false};
  double coverage_executable_candidate_max_speed_{0.50};
  bool coverage_moving_handoff_enable_{false};
  double coverage_route_rank_weight_{0.15};
  bool coverage_floor_priority_enable_{false};
  double coverage_floor_priority_min_z_{3.8};
  int coverage_floor_transition_rank_window_{4};
  double coverage_executable_candidate_bonus_{3.0};
  NormalGoalProgress normal_goal_progress_;
  bool frontier_progress_watchdog_enable_{true};
  double frontier_progress_timeout_{12.0};
  double frontier_progress_min_cost_drop_{0.75};
  double frontier_progress_min_distance_drop_{0.75};

  double failedGoalPenalty(const TopoNode::Ptr &viewpoint) const;
  bool updateNormalGoalProgress(const TopoNode::Ptr &viewpoint,
                                double route_cost,
                                const Eigen::Vector3d &robot_position);
  void resetNormalGoalProgress();
  bool coverageRecoveryDeferred(const CoverageTarget &target,
                                const ros::Time &now) const;
  bool coverageRecoveryExhausted(const CoverageTarget &target) const;
  bool coverageRecoveryCooling(const CoverageTarget &target,
                               const ros::Time &now,
                               double *remaining = nullptr) const;
  bool coverageTerminalRetryReady(const CoverageTarget &target,
                                  const ros::Time &now,
                                  double *retry_after = nullptr) const;
  void deferCoverageRecovery(const CoverageTarget &target,
                             CoverageRecoveryOutcome outcome);
  void pruneDeferredCoverageGoals(const ros::Time &now);
  int latestCoverageObservedVoxels() const;
  static const char *coverageRecoveryOutcomeName(
      CoverageRecoveryOutcome outcome);
};

} // namespace fast_planner

#endif
