#pragma once

#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "traj_opt/config.hpp"
#include "traj_opt/minco/minco_trajectory.hpp"
#include "traj_opt/minco/minco_optimizer.hpp"
#include "traj_opt/costfunctional/temporalcosts/linear_time_cost.hpp"
#include "traj_opt/costfunctional/spatialmap/polytope_spatial_map.hpp"
#include "traj_opt/costfunctional/temporalmap/quad_inv_time_map.hpp"
#include "traj_opt/costfunctional_manager/exploration_cost_manager.hpp"
#include "traj_opt/costfunctional_manager/exp_integal_cost_manager.hpp"
#include "traj_opt/costfunctional_manager/exp_convex_cost_manager.hpp"
#include "traj_opt/costfunctional_manager/exp_packed_corrector_cost_manager.hpp"
#include "traj_opt/convex_hull/constraint_pack.hpp"
#include "traj_opt/costfunctional_manager/backup_integal_cost_manager.hpp"
#include "traj_opt/costfunctional_manager/esdf_integral_cost_manager.hpp"
#include "traj_opt/costfunctional_manager/plain_integral_cost_manager.hpp"
#include "traj_opt/tracking_perching_traj_opt.hpp"
#include "traj_opt/swarm_traj.hpp"

#include <data_structure/base/polytope.h>
#include <data_structure/base/trajectory.h>
#include <ros_interface/ros_interface.hpp>
#include <map_manager/map_manager.hpp>
#include <utils/header/type_utils.hpp>
#include <utils/optimization/lbfgs.h>
#include <utils/optimization/fast_lbfgs.hpp>

namespace path_search
{
class Astar;
}

namespace traj_opt
{
constexpr int TRAJ_DIM = 3;
constexpr int SNAP_TRAJ_S = 4;
constexpr int SNAP_TRAJ_ORDER = 2 * SNAP_TRAJ_S - 1;
constexpr int YAW_TRAJ_S = 2;
constexpr int YAW_TRAJ_ORDER = 2 * YAW_TRAJ_S - 1;

using SnapTraj = minco::MINCO_S4<TRAJ_DIM>;
using SnapBoundaryState = typename SnapTraj::BoundaryState;
using SnapOptimizer = minco::MINCOOptimizer<TRAJ_DIM,
                                            SNAP_TRAJ_S,
                                            temporal_map::QuadInvTimeMap,
                                            spatial_map::PolytopeSpatialMap>;
using YawTraj = minco::MINCO_S2<1>;
using YawBoundaryState = typename YawTraj::BoundaryState;

using geometry_utils::Polytope;
using geometry_utils::PolytopeVec;
using geometry_utils::Trajectory;
using general_utils::Mat3Df;
using general_utils::PolyhedronH;
using general_utils::PolyhedronV;
using general_utils::PolyhedraH;
using general_utils::PolyhedraV;
using general_utils::StatePVAJ;
using general_utils::Vec3f;
using general_utils::Vec4f;
using general_utils::VecDf;
using general_utils::VecDi;
using general_utils::vec_E;
using general_utils::vec_Vec3f;


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
                                       const Trajectory &pos_traj,
                                       double yaw_dot_max);

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
  void setShortcutGuide(bool shortcut_guide);
  void setLabel(const std::string &label);
  void setSwarmConfig(const SwarmPenaltyConfig &config);
  void setSwarmTrajectories(const SwarmTrajectoriesConstPtr &trajectories);
  void setSwarmCurrentWallTime(double wall_time);

	  bool optimize(const StatePVAJ &headPVAJ,
	                const StatePVAJ &tailPVAJ,
	                const vec_E<Vec3f> &guide_path,
	                const std::vector<double> &guide_t,
	                Trajectory &out_traj);

	  void getInitValue(VecDf &ts, vec_Vec3f &ps) const
	  {
	    ts = opt_vars_.times;
	    ps.clear();
	    ps.reserve(static_cast<std::size_t>(opt_vars_.points.cols()));
	    for (int i = 0; i < opt_vars_.points.cols(); ++i)
	    {
	      ps.emplace_back(opt_vars_.points.col(i));
	    }
	  }

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
    double weight_guide_vel_integral{0.0};
    double max_violation{0.0};
    Mat3Df guide_points;
    vec_E<Vec3f> guide_path;
    vec_E<Vec3f> guide_velocities;
    bool shortcut_guide{true};
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
  double evaluateMincoCost(const VecDf &x, VecDf &g);
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
  SnapOptimizer optimizer_;
  temporal_map::QuadInvTimeMap time_map_;
  cost_functional::LinearTimeCost linear_time_cost_;
  cost_functional_manager::ESDFIntegralCostManager esdf_cost_manager_;
  SwarmPenaltyConfig swarm_config_;
  SwarmTrajectoriesConstPtr swarm_trajs_;
  double swarm_current_wall_time_{0.0};
  OptimizationVariables opt_vars_;
  mutable std::ofstream esdf_debug_log_;
  std::string label_{"ESDFTrajOpt"};
};

class PlainTrajOpt
{
public:
  using Ptr = std::shared_ptr<PlainTrajOpt>;

  PlainTrajOpt(const traj_opt::Config &cfg,
               const ros_interface::RosInterface::Ptr &ros_ptr);
  ~PlainTrajOpt();

  void setMapManager(const general_planner::MapManager::Ptr &map_manager);
  void setLocalAstar(const std::shared_ptr<path_search::Astar> &astar);
  void setSafeDistance(double safe_distance);
  void setShortcutGuide(bool shortcut_guide);
  void setSwarmConfig(const SwarmPenaltyConfig &config);
  void setSwarmTrajectories(const SwarmTrajectoriesConstPtr &trajectories);
  void setSwarmCurrentWallTime(double wall_time);

	  bool optimize(const StatePVAJ &headPVAJ,
	                const StatePVAJ &tailPVAJ,
	                const vec_E<Vec3f> &guide_path,
	                const std::vector<double> &guide_t,
	                Trajectory &out_traj);

	  void getInitValue(VecDf &ts, vec_Vec3f &ps) const
	  {
	    ts = opt_vars_.times;
	    ps.clear();
	    ps.reserve(static_cast<std::size_t>(opt_vars_.points.cols()));
	    for (int i = 0; i < opt_vars_.points.cols(); ++i)
	    {
	      ps.emplace_back(opt_vars_.points.col(i));
	    }
	  }

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
    double weight_pv{1.0};
    double weight_guide{0.0};
    double weight_guide_integral{0.0};
    double weight_guide_vel_integral{0.0};
    double weight_guide_tube{0.0};
    double guide_tube_radius{0.0};
    double guide_tube_radius_sqr{0.0};
    double guide_tube_violation{0.0};
    double max_violation{0.0};
    Mat3Df guide_points;
    vec_E<Vec3f> guide_path;
    vec_E<Vec3f> guide_velocities;
    cost_functional_manager::PlainPVPairBuckets pv_pairs;
    int pv_samples_per_piece{0};
    int local_astar_segments{0};
    int local_astar_success{0};
    int local_astar_pairs{0};
    int fallback_pv_pairs{0};
    bool shortcut_guide{true};
  };

  struct ValidationReport
  {
    bool valid{false};
    std::string reason{"UNSET"};
    double duration{0.0};
    double max_vel{0.0};
    double max_acc{0.0};
    double min_clearance{std::numeric_limits<double>::infinity()};
    double time{0.0};
    Vec3f position{Vec3f::Zero()};
    int grid_type{-1};
  };

  static double costFunctional(void *ptr, const VecDf &x, VecDf &g);
  double evaluateMincoCost(const VecDf &x, VecDf &g);
  double optimize(Trajectory &traj, double rel_cost_tol);
  void decodeOptimizationVector(const VecDf &x, VecDf &times, Mat3Df &inner) const;
  void logValidationReport(const std::string &stage,
                           const ValidationReport &report,
                           double cost = 0.0) const;
  static std::string validationReportToString(const ValidationReport &report);
  static int reboundProgress(void *ptr,
                             const VecDf &x,
                             const VecDf &g,
                             double fx,
                             double step,
                             int k,
                             int ls);
  bool initializeFromGuide(const vec_E<Vec3f> &guide_path,
                           const std::vector<double> &guide_t);
  bool findPVPairForPoint(const Vec3f &query,
                          const Vec3f &reference,
                          Vec3f &base_point,
                          Vec3f &direction) const;
  bool plainSampleNeedsPVPair(const Vec3f &position) const;
  bool buildPVPairFromLocalPath(const std::vector<Vec3f> &sample_positions,
                                int sample_idx,
                                const vec_E<Vec3f> &local_path,
                                Vec3f &base_point,
                                Vec3f &direction) const;
  void generateLocalAstarPVPairs(const std::vector<Vec3f> &sample_positions,
                                 std::vector<unsigned char> &pv_filled,
                                 int &active_pv_pairs);
  void resetPVPairBuckets(int sample_count);
  bool appendPVPair(int sample_idx,
                    const Vec3f &base_point,
                    const Vec3f &direction,
                    std::vector<unsigned char> &pv_filled,
                    int &active_pv_pairs);
  void collectCurrentTrajectorySamples(std::vector<Vec3f> &sample_positions) const;
  bool sampleNeedsNewPVPair(int sample_idx,
                            const Vec3f &position) const;
  bool maybeRefreshPVPairsForRebound(const VecDf &x, int iteration);
  int refreshPVPairsFromCurrentTrajectory();
  bool plainSampleOccupied(const Vec3f &position) const;
  ValidationReport validateTrajectoryDetailed(const Trajectory &traj) const;
  bool validateTrajectory(const Trajectory &traj) const;

  static Trajectory toGeometryTrajectory(const SnapTraj &traj);
  static SnapBoundaryState toSnapBoundary(const StatePVAJ &state);

  traj_opt::Config cfg_;
  ros_interface::RosInterface::Ptr ros_ptr_;
  general_planner::MapManager::Ptr map_manager_;
  std::shared_ptr<path_search::Astar> local_astar_;
  SnapTraj minco_traj_;
  SnapOptimizer optimizer_;
  temporal_map::QuadInvTimeMap time_map_;
  cost_functional::LinearTimeCost linear_time_cost_;
  cost_functional_manager::PlainIntegralCostManager plain_cost_manager_;
  SwarmPenaltyConfig swarm_config_;
  SwarmTrajectoriesConstPtr swarm_trajs_;
  double swarm_current_wall_time_{0.0};
  OptimizationVariables opt_vars_;
  ValidationReport last_opt_report_;
  mutable std::ofstream plain_debug_log_;
  std::string label_{"PlainTrajOpt"};
};

class ExpTrajOpt
{
public:
  using Ptr = std::shared_ptr<ExpTrajOpt>;

  struct TimingReport
  {
    std::string mode{"not_run"};
    std::size_t evaluations{0};
    std::size_t iterations{0};
    std::size_t line_search_evaluations{0};
    std::size_t max_line_search_evaluations{0};
    double accepted_step_sum{0.0};
    double min_accepted_step{0.0};
    std::size_t polynomial_pieces{0};
    std::size_t dense_nodes_per_evaluation{0};
    std::size_t hull_control_checks_per_evaluation{0};
    std::size_t scalar_constraint_checks{0};
    std::size_t alm_constraints{0};
    std::size_t alm_outer_iterations{0};
    std::size_t alm_inner_solves{0};
    std::size_t alm_topology_changes{0};
    std::size_t adaptive_coarse_segments{0};
    std::size_t adaptive_fine_segments{0};
    double alm_max_violation{0.0};
    // Full ALM: PHR certificate. Phase-2: oracle continuous_feasible.
    bool alm_certified{false};
    // Phase-2 observability (also mirrored into alm_* counters when active).
    bool phase2_triggered{false};
    std::size_t phase2_packed_constraints{0};
    bool jerk_certificate_enabled{false};
    std::size_t trust_region_rejections{0};
    double trust_region_ratio{0.0};
    double actual_reduction{0.0};
    double predicted_reduction{0.0};
    double alm_warm_start_seconds{0.0};
    double dense_integral_seconds{0.0};
    double control_point_seconds{0.0};
    double hull_transform_seconds{0.0};
    double hull_hodograph_seconds{0.0};
    double hull_position_residual_seconds{0.0};
    double hull_derivative_residual_seconds{0.0};
    double hull_reverse_hodograph_seconds{0.0};
    double hull_backward_add_seconds{0.0};
    double hull_discrete_attractor_seconds{0.0};
    double minco_evaluation_seconds{0.0};
    double optimization_seconds{0.0};
    double dense_share_of_minco_evaluation{0.0};
    double control_point_share_of_minco_evaluation{0.0};
    double dense_share_of_optimization{0.0};
    bool fast_stop_satisfied{false};
    std::size_t fast_stop_iteration{0};
    bool fast_fallback_used{false};
    std::size_t fast_stop_candidate_checks{0};
    std::size_t fast_stop_cost_passes{0};
    std::size_t fast_stop_decision_step_passes{0};
    std::size_t fast_stop_penalty_change_passes{0};
    std::size_t fast_stop_physical_time_passes{0};
    std::size_t fast_stop_waypoint_passes{0};
    std::size_t fast_stop_gradient_passes{0};
    std::size_t fast_stop_violation_passes{0};
    std::size_t fast_stop_nonstall_passes{0};
    std::size_t fast_stop_base_rule_passes{0};
    std::size_t fast_stop_guarded_rule_passes{0};
    // 0 disabled/no cache, 1 selected, 2 invalid cache,
    // 3 endpoint mismatch, 4 overlap clipping failure,
    // 5 invalid waypoint shift, 6 encoding failure, 7 objective rejection.
    int warm_start_status{0};
    bool warm_start_attempted{false};
    bool warm_start_accepted{false};
    bool warm_start_topology_resampled{false};
    std::size_t warm_start_comparison_evaluations{0};
    double warm_start_seconds{0.0};
    double warm_start_baseline_cost{0.0};
    double warm_start_candidate_cost{0.0};
    double warm_start_baseline_gradient{0.0};
    double warm_start_candidate_gradient{0.0};
    double warm_start_baseline_penalty{0.0};
    double warm_start_candidate_penalty{0.0};
    double warm_start_max_waypoint_shift{0.0};
    bool continuous_feasible{false};
    bool robustly_certified{false};
    bool has_certified_incumbent{false};
    double max_normalized_violation{0.0};
    double min_position_margin{0.0};
    double primal_residual{0.0};
    double dual_residual{0.0};
    double complementarity_residual{0.0};
    double stationarity_residual{0.0};
  };

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

  const TimingReport &lastTimingReport() const
  {
    return last_timing_report_;
  }

  const TimingReport &cumulativeTimingReport() const
  {
    return cumulative_timing_report_;
  }

  const SolverQualityReport &lastQualityReport() const
  {
    return last_quality_report_;
  }

private:
  struct OptimizationVariables
  {
    double rho{0.0};
    int iter_num{0};
    std::size_t lbfgs_iterations{0};
    std::size_t line_search_evaluations{0};
    std::size_t max_line_search_evaluations{0};
    double accepted_step_sum{0.0};
    double min_accepted_step{0.0};
    bool lbfgs_fast_enabled{false};
    int lbfgs_mem_size{32};
    bool lbfgs_step_bound_enabled{true};
    double lbfgs_time_ratio_min{0.5};
    double lbfgs_time_ratio_max{2.0};
    int lbfgs_fast_window{5};
    int lbfgs_fast_min_iterations{20};
    int lbfgs_fast_consecutive{2};
    double lbfgs_fast_rel_cost{1.0e-4};
    double lbfgs_fast_rel_step{2.0e-3};
    double lbfgs_fast_rel_penalty{5.0e-3};
    bool lbfgs_fast_phase0_guards_en{false};
    double lbfgs_fast_rel_time{2.0e-2};
    double lbfgs_fast_rel_waypoint{2.0e-2};
    double lbfgs_fast_scaled_grad{5.0e-2};
    double lbfgs_fast_min_step{1.0e-8};
    double lbfgs_fast_penalty_tol{1.0e-2};
    int lbfgs_fast_small_step_limit{3};
    bool lbfgs_warm_start_enabled{false};
    double lbfgs_warm_start_max_endpoint_shift{3.0};
    double lbfgs_warm_start_max_waypoint_shift{2.0};
    double lbfgs_warm_start_duration_blend{0.75};
    double lbfgs_warm_start_cost_ratio{0.75};
    double lbfgs_warm_start_gradient_ratio{1.0};
    double lbfgs_warm_start_penalty_ratio{1.0};
    double guide_initial_time_scale{1.0};
    bool fast_stop_satisfied{false};
    std::size_t fast_stop_iteration{0};
    bool fast_fallback_used{false};
    int warm_start_status{0};
    bool warm_start_attempted{false};
    bool warm_start_accepted{false};
    bool warm_start_topology_resampled{false};
    std::size_t warm_start_comparison_evaluations{0};
    double warm_start_seconds{0.0};
    double warm_start_baseline_cost{0.0};
    double warm_start_candidate_cost{0.0};
    double warm_start_baseline_gradient{0.0};
    double warm_start_candidate_gradient{0.0};
    double warm_start_baseline_penalty{0.0};
    double warm_start_candidate_penalty{0.0};
    double warm_start_max_waypoint_shift{0.0};
    bool has_certified_incumbent{false};
    VecDf certified_incumbent_x;
    double certified_incumbent_cost{std::numeric_limits<double>::infinity()};
    ContinuousCertificateReport last_certificate{};
    SolverQualityReport quality{};
    int pos_constraint_type{0};
    bool block_energy_cost{false};
    double smooth_eps{0.0};
    int integral_res{1};
    bool convex_hull_enabled{false};
    traj_opt::convex_hull::Basis convex_hull_basis{
        traj_opt::convex_hull::Basis::Bezier};
    static constexpr int convex_hull_subdivision_depth{2};
    bool convex_hull_require_certification{true};
    bool convex_hull_phase2_objective_active{false};
    bool phase2_triggered{false};
    std::size_t phase2_packed_constraints{0};
    std::size_t stage_a_fine_segments{0};
    int convex_hull_phase2_top_k{32};
    int convex_hull_phase2_max_constraints{64};
    int convex_hull_phase2_max_outer{4};
    double convex_hull_phase2_inner_tol_init{1.0e-2};
    double convex_hull_phase2_inner_tol_final{1.0e-4};
    bool convex_hull_phase2_append_en{true};
    bool convex_hull_phase2_multiplier_reuse_en{true};
    bool convex_hull_flatness_enabled{false};
    std::size_t trust_region_rejections{0};
    double trust_region_ratio{0.0};
    double actual_reduction{0.0};
    double predicted_reduction{0.0};
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
    double weight_guide_integral{0.0};
    double guide_integral_violation{0.0};
    double guide_path_tube_radius{0.0};
    double guide_path_z_tube_radius{0.0};
    double guide_path_huber_delta{0.0};
    bool guide_path_time_gradient_en{false};
    double guide_path_cost_log{0.0};
    double guide_path_max_abs_time_grad{0.0};
    int guide_path_out_of_time_range_samples{0};
    double weight_guide_z_tube{0.0};
    double guide_z_tube_radius{0.0};
    double guide_z_tube_violation{0.0};

    VecDf penalty_log;
  };

  static double costFunctional(void *ptr, const VecDf &x, VecDf &g);
  static double stepBoundFunctional(void *ptr,
                                    const VecDf &x,
                                    const VecDf &direction);
  static math_utils::FastLbfgs::PhysicalSnapshot fastLbfgsSnapshot(
      void *ptr, const Eigen::VectorXd &x);

  void configureFastLbfgs(double rel_cost_tol,
                          bool early_stop_enabled,
                          int decision_dim);
  void syncFastLbfgsReport();
  bool buildWarmStartCandidate(VecDf &candidate);
  void updateWarmStartCache(const Trajectory &traj);

  bool processCorridor();
  bool processCorridorWithGuideTraj();
  void defaultInitialization();
  bool setupProblemAndCheck();
  double optimize(Trajectory &traj, double rel_cost_tol);
  double evaluateMincoCost(const VecDf &x, VecDf &g);
  bool loadCorridors(PolytopeVec &sfcs);

  static Trajectory toGeometryTrajectory(const SnapTraj &traj);
  static SnapBoundaryState toSnapBoundary(const StatePVAJ &state);

private:
  traj_opt::Config cfg_;
  std::ofstream failed_traj_log_;
  std::ofstream penalty_log_;
  ros_interface::RosInterface::Ptr ros_ptr_;

  SnapOptimizer optimizer_;
  temporal_map::QuadInvTimeMap time_map_;
  spatial_map::PolytopeSpatialMap spatial_map_;
  cost_functional::LinearTimeCost linear_time_cost_;
  cost_functional_manager::ExpIntegralCostManager exp_cost_manager_;
  cost_functional_manager::ExpConvexCostManager exp_convex_cost_manager_;
  cost_functional_manager::ExpPackedCorrectorCostManager
      exp_packed_corrector_cost_manager_;
  math_utils::FastLbfgs fast_lbfgs_;
  SwarmPenaltyConfig swarm_config_;
  SwarmTrajectoriesConstPtr swarm_trajs_;
  double swarm_current_wall_time_{0.0};
  OptimizationVariables opt_vars_;
  TimingReport last_timing_report_;
  TimingReport cumulative_timing_report_;
  SolverQualityReport last_quality_report_;

  struct WarmStartCache
  {
    bool valid{false};
    int piece_num{0};
    VecDf durations;
    Mat3Df inner_points;
    Vec3f head{Vec3f::Zero()};
    Vec3f tail{Vec3f::Zero()};
    Trajectory trajectory;
  };
  WarmStartCache warm_start_cache_;

  // Cross-replan Phase-2 multiplier / topology cache.
  std::uint64_t phase2_cached_signature_{0};
  traj_opt::convex_hull::PackedConstraintSet phase2_cached_pack_{};
  Eigen::VectorXd phase2_cached_multipliers_;
  double phase2_cached_penalty_{0.0};

  void maybeUpdateCertifiedIncumbent(const VecDf &x, double cost);
  void fillPostSolveQualityReport(const VecDf &x, const VecDf &grad);
  bool runPhase2PackedCorrection(VecDf &x,
                                 double &min_cost,
                                 double rel_cost_tol,
                                 int lbfgs_mem_size,
                                 std::size_t &outer_iterations,
                                 std::size_t &inner_solves,
                                 std::size_t &topology_changes,
                                 double &max_violation,
                                 bool &correction_certified);
};

class ExplorationTrajOpt
{
public:
  using Ptr = std::shared_ptr<ExplorationTrajOpt>;

  ExplorationTrajOpt(const traj_opt::Config &cfg,
                     const ros_interface::RosInterface::Ptr &ros_ptr);
  ~ExplorationTrajOpt();

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
                const vec_E<Vec3f> &guide_path,
                const std::vector<double> &guide_t,
                PolytopeVec &sfcs,
                const VecDf &piece_velocity_bounds,
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
    VecDf piece_velocity_bounds;
    VecDf magnitude_bounds;
    VecDf penalty_weights;
    bool use_piece_velocity_bounds{false};

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
    double weight_guide_integral{0.0};
    double guide_integral_violation{0.0};
    double guide_path_tube_radius{0.0};
    double guide_path_z_tube_radius{0.0};
    double guide_path_huber_delta{0.0};
    bool guide_path_time_gradient_en{false};
    double guide_path_cost_log{0.0};
    double guide_path_max_abs_time_grad{0.0};
    int guide_path_out_of_time_range_samples{0};
    double weight_guide_z_tube{0.0};
    double guide_z_tube_radius{0.0};
    double guide_z_tube_violation{0.0};

    VecDf penalty_log;
  };

  static double costFunctional(void *ptr, const VecDf &x, VecDf &g);

  bool processCorridor();
  bool processCorridorWithGuideTraj();
  void defaultInitialization();
  bool setupProblemAndCheck();
  void clearPieceVelocityBounds();
  void setPieceVelocityBounds(const VecDf &piece_velocity_bounds);
  void normalizePieceVelocityBounds();
  double optimize(Trajectory &traj, double rel_cost_tol);
  double evaluateMincoCost(const VecDf &x, VecDf &g);
  bool loadCorridors(PolytopeVec &sfcs);

  static Trajectory toGeometryTrajectory(const SnapTraj &traj);
  static SnapBoundaryState toSnapBoundary(const StatePVAJ &state);

private:
  traj_opt::Config cfg_;
  std::ofstream failed_traj_log_;
  std::ofstream penalty_log_;
  ros_interface::RosInterface::Ptr ros_ptr_;

  SnapOptimizer optimizer_;
  temporal_map::QuadInvTimeMap time_map_;
  spatial_map::PolytopeSpatialMap spatial_map_;
  cost_functional::LinearTimeCost linear_time_cost_;
  cost_functional_manager::ExplorationCostManager exploration_cost_manager_;
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
    double weight_guide_z_tube{0.0};
    double guide_z_tube_radius{0.0};
    double guide_z_tube_violation{0.0};

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
  double evaluateMincoCost(const VecDf &x, VecDf &g);
  static Trajectory toGeometryTrajectory(const SnapTraj &traj);
  static SnapBoundaryState toSnapBoundary(const StatePVAJ &state);

private:
  traj_opt::Config cfg_;
  ros_interface::RosInterface::Ptr ros_ptr_;
  std::ofstream failed_traj_log_;
  std::ofstream penalty_log_;

  SnapOptimizer optimizer_;
  minco::BackupBoundaryMapping<TRAJ_DIM, SNAP_TRAJ_S> backup_boundary_mapping_;
  temporal_map::QuadInvTimeMap time_map_;
  spatial_map::PolytopeSpatialMap spatial_map_;
  cost_functional::LinearTimeCost linear_time_cost_;
  cost_functional_manager::BackupIntegralCostManager backup_cost_manager_;
  OptimizationVariables opt_vars_;
};

class TrajManager
{
public:
  using Ptr = std::shared_ptr<TrajManager>;

  TrajManager(const traj_opt::Config &exp_cfg,
              const traj_opt::Config &esdf_cfg,
              const traj_opt::Config &plain_cfg,
              const traj_opt::Config &backup_cfg,
              double yaw_dot_max,
              double esdf_safe_distance,
              const ros_interface::RosInterface::Ptr &ros_ptr,
              const general_planner::MapManager::Ptr &map_manager);

  ExpTrajOpt::Ptr exp() const { return exp_traj_opt_; }
  ESDFTrajOpt::Ptr esdf() const { return esdf_traj_opt_; }
  PlainTrajOpt::Ptr plain() const { return plain_traj_opt_; }
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
  PlainTrajOpt::Ptr plain_traj_opt_;
  BackupTrajOpt::Ptr backup_traj_opt_;
  YawTrajOpt::Ptr yaw_traj_opt_;
  TrackingJerkTrajOpt::Ptr tracking_jerk_traj_opt_;
  TrackingSnapTrajOpt::Ptr tracking_snap_traj_opt_;
  PerchingSnapTrajOpt::Ptr perching_snap_traj_opt_;
};
} // namespace traj_opt
