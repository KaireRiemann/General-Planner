#pragma once

#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "traj_opt/config.hpp"
#include "traj_opt/minco/minco_trajectory.hpp"
#include "traj_opt/costfunctional/temporalcosts/linear_time_cost.hpp"
#include "traj_opt/costfunctional/spatialmap/polytope_spatial_map.hpp"
#include "traj_opt/costfunctional/temporalmap/quad_inv_time_map.hpp"
#include "traj_opt/costfunctional_manager/exp_integal_cost_manager.hpp"
#include "traj_opt/costfunctional_manager/backup_integal_cost_manager.hpp"
#include "traj_opt/costfunctional_manager/esdf_integral_cost_manager.hpp"
#include "traj_opt/tracking_perching_traj_opt.hpp"
#include "traj_opt/swarm_traj.hpp"

#include <data_structure/base/polytope.h>
#include <data_structure/base/trajectory.h>
#include <ros_interface/ros_interface.hpp>
#include <general_core/map_manager.hpp>
#include <utils/header/type_utils.hpp>
#include <utils/optimization/lbfgs.h>

namespace traj_opt
{
constexpr int TRAJ_DIM = 3;
constexpr int SNAP_TRAJ_S = 4;
constexpr int SNAP_TRAJ_ORDER = 2 * SNAP_TRAJ_S - 1;
constexpr int YAW_TRAJ_S = 2;
constexpr int YAW_TRAJ_ORDER = 2 * YAW_TRAJ_S - 1;

using SnapTraj = minco::MINCO_S4<TRAJ_DIM>;
using SnapBoundaryState = typename SnapTraj::BoundaryState;
using YawTraj = minco::MINCO_S2<1>;
using YawBoundaryState = typename YawTraj::BoundaryState;

using geometry_utils::Polytope;
using geometry_utils::PolytopeVec;
using geometry_utils::Trajectory;
using super_utils::Mat3Df;
using super_utils::PolyhedronH;
using super_utils::PolyhedronV;
using super_utils::PolyhedraH;
using super_utils::PolyhedraV;
using super_utils::StatePVAJ;
using super_utils::Vec3f;
using super_utils::Vec4f;
using super_utils::VecDf;
using super_utils::VecDi;
using super_utils::vec_E;
using super_utils::vec_Vec3f;


class YawTrajOpt
{
public:
  using Ptr = std::shared_ptr<YawTrajOpt>;

  explicit YawTrajOpt(const double &yaw_dot_max);

  void getYawTimeAllocation(const double &duration, VecDf &times) const;

  static void getYawWaypointAllocation(const Vec4f &init_state,
                                       Vec4f &goal_state,
                                       VecDf &way_pts,
                                       VecDf &times,
                                       const Trajectory &pos_traj);

  bool optimize(const Vec4f &istate_in,
                const Vec4f &gstate_in,
                const Trajectory &pos_traj,
                Trajectory &out_traj,
                const int &order = 3,
                const bool &free_start = false,
                const bool &free_goal = true);

private:
  static Trajectory toGeometryTrajectory(const YawTraj &traj);
  static YawBoundaryState toBoundaryState(const Vec4f &state);

  bool free_goal_{false};
  double yaw_dot_max_{10.0};
};

class ESDFTrajOpt
{
public:
  using Ptr = std::shared_ptr<ESDFTrajOpt>;

  ESDFTrajOpt(const traj_opt::Config &cfg,
              const ros_interface::RosInterface::Ptr &ros_ptr);
  ~ESDFTrajOpt();

  void setMapManager(const general_planner::MapManager::Ptr &map_manager);
  void setSafeDistance(double safe_distance);
  void setWeight(double weight);
  void setSwarmConfig(const SwarmPenaltyConfig &config);
  void setSwarmTrajectories(const SwarmTrajectoriesConstPtr &trajectories);
  void setSwarmCurrentWallTime(double wall_time);

  bool optimize(const StatePVAJ &headPVAJ,
                const StatePVAJ &tailPVAJ,
                const vec_E<Vec3f> &guide_path,
                const std::vector<double> &guide_t,
                Trajectory &out_traj);

private:
  struct OptimizationVariables
  {
    double rho{0.0};
    bool block_energy_cost{false};
    double smooth_eps{0.0};
    int integral_res{1};
    int iter_num{0};
    VecDf times;
    Mat3Df points;
    VecDf magnitude_bounds;
    VecDf penalty_weights;
    VecDf penalty_log;
    flatness::FlatnessMap quadrotor_flatness;
    StatePVAJ head_pvaj;
    StatePVAJ tail_pvaj;
    double safe_distance{0.5};
    double weight_esdf{1.0};
    double weight_guide{0.0};
    double weight_guide_integral{0.0};
    double max_violation{0.0};
    Mat3Df guide_points;
    vec_E<Vec3f> guide_path;
  };

  struct ValidationReport
  {
    bool valid{false};
    std::string reason{"UNSET"};
    double duration{0.0};
    double max_vel{0.0};
    double max_acc{0.0};
    double min_esdf_dist{std::numeric_limits<double>::infinity()};
    double time{0.0};
    Vec3f position{Vec3f::Zero()};
    int grid_type{-1};
  };

  static double costFunctional(void *ptr, const VecDf &x, VecDf &g);
  double evaluateCurrentCost(const VecDf &x, VecDf &g);
  double optimize(Trajectory &traj, double rel_cost_tol);
  void decodeOptimizationVector(const VecDf &x, VecDf &times, Mat3Df &inner) const;
  void logValidationReport(const std::string &stage,
                           const ValidationReport &report,
                           double cost = 0.0) const;
  static std::string validationReportToString(const ValidationReport &report);
  bool initializeFromGuide(const vec_E<Vec3f> &guide_path,
                           const std::vector<double> &guide_t);
  ValidationReport validateTrajectoryDetailed(const Trajectory &traj) const;
  bool validateTrajectory(const Trajectory &traj) const;

  static Trajectory toGeometryTrajectory(const SnapTraj &traj);
  static SnapBoundaryState toSnapBoundary(const StatePVAJ &state);

  traj_opt::Config cfg_;
  ros_interface::RosInterface::Ptr ros_ptr_;
  general_planner::MapManager::Ptr map_manager_;
  SnapTraj minco_traj_;
  temporal_map::QuadInvTimeMap time_map_;
  cost_functional::LinearTimeCost linear_time_cost_;
  cost_functional_manager::ESDFIntegralCostManager esdf_cost_manager_;
  SwarmPenaltyConfig swarm_config_;
  SwarmTrajectoriesConstPtr swarm_trajs_;
  double swarm_current_wall_time_{0.0};
  OptimizationVariables opt_vars_;
  mutable std::ofstream esdf_debug_log_;
};

class ExpTrajOpt
{
public:
  using Ptr = std::shared_ptr<ExpTrajOpt>;

  ExpTrajOpt(const traj_opt::Config &cfg,
             const ros_interface::RosInterface::Ptr &ros_ptr);
  ~ExpTrajOpt();

  bool optimize(const StatePVAJ &headPVAJ,
                const StatePVAJ &tailPVAJ,
                PolytopeVec &sfcs,
                Trajectory &out_traj);

  bool optimize(const StatePVAJ &headPVAJ,
                const StatePVAJ &tailPVAJ,
                const vec_E<Vec3f> &guide_path,
                const std::vector<double> &guide_t,
                PolytopeVec &sfcs,
                Trajectory &out_traj);

  bool optimize(const StatePVAJ &headPVAJ,
                const StatePVAJ &tailPVAJ,
                PolytopeVec &sfcs,
                const vec_Vec3f &init_ps,
                const VecDf &init_ts,
                Trajectory &out_traj);

  void getInitValue(VecDf &ts, vec_Vec3f &ps) const
  {
    ts = opt_vars_.init_ts;
    ps = opt_vars_.init_ps;
  }

  void setSwarmConfig(const SwarmPenaltyConfig &config);
  void setSwarmTrajectories(const SwarmTrajectoriesConstPtr &trajectories);
  void setSwarmCurrentWallTime(double wall_time);

private:
  struct OptimizationVariables
  {
    double rho{0.0};
    int iter_num{0};
    int pos_constraint_type{0};
    bool block_energy_cost{false};
    double smooth_eps{0.0};
    int integral_res{1};
    flatness::FlatnessMap quadrotor_flatness;

    bool default_init{true};
    bool given_init_ts_and_ps{false};
    int piece_num{0};
    Mat3Df points;
    VecDf times;
    VecDf magnitude_bounds;
    VecDf penalty_weights;

    PolyhedraV v_polytopes;
    PolyhedraH h_polytopes;
    PolyhedraH h_overlap_polytopes;
    Mat3Df init_path;
    VecDf init_ts;
    vec_Vec3f init_ps;
    Mat3Df waypoint_attractor;
    VecDf waypoint_attractor_dead_d;

    VecDi v_poly_idx;
    VecDi h_poly_idx;

    StatePVAJ head_pvaj;
    StatePVAJ tail_pvaj;
    vec_E<Vec3f> guide_path;
    std::vector<double> guide_t;

    VecDf penalty_log;
  };

  static double costFunctional(void *ptr, const VecDf &x, VecDf &g);

  bool processCorridor();
  bool processCorridorWithGuideTraj();
  void defaultInitialization();
  bool setupProblemAndCheck();
  double optimize(Trajectory &traj, double rel_cost_tol);
  double evaluateCurrentCost(const VecDf &x, VecDf &g);
  bool loadCorridors(PolytopeVec &sfcs);

  static Trajectory toGeometryTrajectory(const SnapTraj &traj);
  static SnapBoundaryState toSnapBoundary(const StatePVAJ &state);

private:
  traj_opt::Config cfg_;
  std::ofstream failed_traj_log_;
  std::ofstream penalty_log_;
  ros_interface::RosInterface::Ptr ros_ptr_;

  SnapTraj minco_traj_;
  temporal_map::QuadInvTimeMap time_map_;
  spatial_map::PolytopeSpatialMap spatial_map_;
  cost_functional::LinearTimeCost linear_time_cost_;
  cost_functional_manager::ExpIntegralCostManager exp_cost_manager_;
  SwarmPenaltyConfig swarm_config_;
  SwarmTrajectoriesConstPtr swarm_trajs_;
  double swarm_current_wall_time_{0.0};
  OptimizationVariables opt_vars_;
};

class BackupTrajOpt
{
public:
  using Ptr = std::shared_ptr<BackupTrajOpt>;

  explicit BackupTrajOpt(const traj_opt::Config &cfg,
                         const ros_interface::RosInterface::Ptr &ros_ptr);
  ~BackupTrajOpt();

  bool checkTrajMagnitudeBound(Trajectory &out_traj);

  bool optimize(const Trajectory &exp_traj,
                const double &t_0,
                const double &t_e,
                const double &heu_ts,
                const VecDf &heu_end_pt,
                double &heu_dur,
                const Polytope &sfc,
                Trajectory &out_traj,
                double &out_ts,
                const bool &debug = false);

  bool optimize(const Trajectory &exp_traj,
                const double &t_0,
                const double &t_e,
                const double &heu_ts,
                const Polytope &sfc,
                const VecDf &init_t_vec,
                const vec_Vec3f &init_ps,
                Trajectory &out_traj,
                double &out_ts);

  void getInitValue(double &ts, VecDf &times, vec_Vec3f &ps) const
  {
    ts = opt_vars_.init_ts;
    times = opt_vars_.init_t_vec;
    ps = opt_vars_.init_ps;
  }

private:
  struct OptimizationVariables
  {
    double rho{0.0};
    double weight_ts{0.0};
    int iter_num{0};
    int pos_constraint_type{0};
    bool block_energy_cost{false};
    bool uniform_time_en{false};
    double smooth_eps{0.0};
    int integral_res{1};
    int piece_num{1};
    flatness::FlatnessMap quadrotor_flatness;

    VecDf times;
    VecDf total_time;
    Mat3Df points;
    VecDf magnitude_bounds;
    VecDf penalty_weights;

    PolyhedronH h_polytope;
    PolyhedronV v_polytope;

    Trajectory exp_traj;
    StatePVAJ head_pvaj;
    StatePVAJ tail_pvaj;
    double min_ts{0.0};
    double max_ts{0.0};
    double ts{0.0};

    bool given_init_ts_and_ps{false};
    double given_init_ts{0.0};
    VecDf given_init_t_vec;
    vec_Vec3f given_init_ps;

    double init_ts{0.0};
    VecDf init_t_vec;
    vec_Vec3f init_ps;

    VecDf penalty_log;
  };

  static double costFunctional(void *ptr, const VecDf &x, VecDf &g);

  bool processCorridor();
  bool setupProblemAndCheck();
  double optimize(Trajectory &traj, double rel_cost_tol);
  double evaluateCurrentCost(const VecDf &x, VecDf &g);
  void decodeDecisionVector(const VecDf &x,
                            VecDf &times,
                            Mat3Df &points,
                            double &ts) const;
  VecDf encodeDecisionVector() const;

  static double logisticInterval(double lo, double hi, double eta);
  static double logisticIntervalGrad(double lo, double hi, double eta);
  static Trajectory toGeometryTrajectory(const SnapTraj &traj);
  static SnapBoundaryState toSnapBoundary(const StatePVAJ &state);

private:
  traj_opt::Config cfg_;
  ros_interface::RosInterface::Ptr ros_ptr_;
  std::ofstream failed_traj_log_;
  std::ofstream penalty_log_;

  SnapTraj minco_traj_;
  temporal_map::QuadInvTimeMap time_map_;
  spatial_map::PolytopeSpatialMap spatial_map_;
  cost_functional_manager::BackupIntegralCostManager backup_cost_manager_;
  OptimizationVariables opt_vars_;
};

class TrajManager
{
public:
  using Ptr = std::shared_ptr<TrajManager>;

  TrajManager(const traj_opt::Config &exp_cfg,
              const traj_opt::Config &esdf_cfg,
              const traj_opt::Config &backup_cfg,
              double yaw_dot_max,
              double esdf_safe_distance,
              const ros_interface::RosInterface::Ptr &ros_ptr,
              const general_planner::MapManager::Ptr &map_manager);

  ExpTrajOpt::Ptr exp() const { return exp_traj_opt_; }
  ESDFTrajOpt::Ptr esdf() const { return esdf_traj_opt_; }
  BackupTrajOpt::Ptr backup() const { return backup_traj_opt_; }
  YawTrajOpt::Ptr yaw() const { return yaw_traj_opt_; }
  TrackingJerkTrajOpt::Ptr trackingJerk() const { return tracking_jerk_traj_opt_; }
  TrackingSnapTrajOpt::Ptr trackingSnap() const { return tracking_snap_traj_opt_; }
  PerchingSnapTrajOpt::Ptr perchingSnap() const { return perching_snap_traj_opt_; }

  void setMapManager(const general_planner::MapManager::Ptr &map_manager);
  void setESDFSafeDistance(double safe_distance);
  void setSwarmConfig(const SwarmPenaltyConfig &config);
  void setSwarmTrajectories(const SwarmTrajectoriesConstPtr &trajectories);
  void setSwarmCurrentWallTime(double wall_time);

private:
  ExpTrajOpt::Ptr exp_traj_opt_;
  ESDFTrajOpt::Ptr esdf_traj_opt_;
  BackupTrajOpt::Ptr backup_traj_opt_;
  YawTrajOpt::Ptr yaw_traj_opt_;
  TrackingJerkTrajOpt::Ptr tracking_jerk_traj_opt_;
  TrackingSnapTrajOpt::Ptr tracking_snap_traj_opt_;
  PerchingSnapTrajOpt::Ptr perching_snap_traj_opt_;
};
} // namespace traj_opt
