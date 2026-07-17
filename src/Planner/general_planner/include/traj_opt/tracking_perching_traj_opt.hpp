#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "data_structure/base/polytope.h"
#include "data_structure/base/trajectory.h"
#include <map_manager/map_manager.hpp>
#include "traj_opt/config.hpp"
#include "traj_opt/minco/boundary_mapping.hpp"
#include "traj_opt/perching_surface_state.hpp"
#include "utils/header/type_utils.hpp"

namespace ros_interface
{
class RosInterface;
}

namespace traj_opt
{

struct DynamicTargetState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double t{0.0};
  general_utils::Vec3f position{general_utils::Vec3f::Zero()};
  general_utils::Vec3f velocity{general_utils::Vec3f::Zero()};
  general_utils::Vec3f acceleration{general_utils::Vec3f::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
};

using DynamicTargetStates = general_utils::vec_E<DynamicTargetState>;

struct TrackingVisibleRegion
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double t{0.0};
  general_utils::Vec3f target_position{general_utils::Vec3f::Zero()};
  general_utils::Vec3f visible_point{general_utils::Vec3f::Zero()};
  double theta{3.14159265358979323846};
  double confidence{0.0};
  bool valid{false};
};

struct TrackingProblem
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  general_utils::StatePVAJ head_pvaj{general_utils::StatePVAJ::Zero()};
  general_utils::StatePVAJ tail_pvaj{general_utils::StatePVAJ::Zero()};
  Eigen::Matrix<double, 1, 2> head_yaw{Eigen::Matrix<double, 1, 2>::Zero()};
  Eigen::Matrix<double, 1, 2> tail_yaw{Eigen::Matrix<double, 1, 2>::Zero()};

  general_utils::vec_E<general_utils::Vec3f> guide_path;
  std::vector<double> guide_t;
  geometry_utils::PolytopeVec sfcs;
  general_utils::vec_E<general_utils::Vec3f> viewpoints;
  std::vector<double> target_sample_times;
  DynamicTargetStates target_prediction;
  general_utils::vec_E<TrackingVisibleRegion> visible_regions;

  double safe_distance{0.45};
  double tracking_distance{3.0};
  double distance_tolerance{0.8};
  double height_offset{0.8};
  double height_tolerance{0.6};

  double od_h_lower{2.2};
  double od_h_upper{3.8};
  double od_v_lower{0.2};
  double od_v_upper{1.4};
  double weight_od_near{20.0};
  double weight_od_far{5.0};
  double weight_od_vertical{8.0};
  double weight_oa{5.0};
  double weight_oe{1.0};
  double weight_relative_velocity{1.0};
  double weight_tangent_velocity{5.0};
  double weight_viewpoint_attractor{50.0};
  double weight_visible_region{3.0};
  double weight_fov{20.0};
  double weight_target_forward{15.0};

  double weight_tracking{5.0};
  double weight_visibility{1.0};
  double visibility_safe_distance{0.25};
  double visibility_cone_ratio{0.12};
  double visibility_angle_clearance{0.08726646259971647};
  bool adaptive_occlusion_enable{true};
  double adaptive_occlusion_activation_distance{0.25};
  double adaptive_occlusion_max_weight_scale{12.0};
  double adaptive_occlusion_od_far_weight_scale{4.0};
  double adaptive_occlusion_distance_upper_scale{0.65};
  double adaptive_occlusion_min_horizontal_upper{0.85};
  double fov_horizontal{1.5707963267948966};
  double fov_vertical{1.0471975511965976};
  double fov_range{4.0};
  double fov_range_margin{0.05};
  double fov_front_margin{0.05};
  double target_front_margin{0.15};
  double target_motion_speed_threshold{0.25};
  double joint_sample_dt{0.05};
  bool dense_joint_sample_enable{true};
  int visibility_samples{5};
  bool use_esdf_visibility{true};
  bool use_visible_region{true};
  bool reacquire_mode{false};
  bool static_tracking_mode{false};

  int piece_num{0};
  double min_piece_duration{0.12};
  double min_total_duration{0.0};
  double time_lower_bound_weight{0.0};
  bool use_corridor{false};
};

struct PerchingInitialGuess
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid{false};
  double total_time{0.0};
  Eigen::Vector2d nu{Eigen::Vector2d::Zero()};
  double tau_f{0.0};
  general_utils::vec_E<general_utils::Vec3f> guide_path;
  std::vector<double> guide_t;
};

struct PerchingProblem
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  general_utils::StatePVAJ head_pvaj{general_utils::StatePVAJ::Zero()};
  general_utils::StatePVAJ nominal_tail_pvaj{general_utils::StatePVAJ::Zero()};
  general_utils::vec_E<general_utils::Vec3f> guide_path;
  std::vector<double> guide_t;
  PerchingSurfaceState surface;

  minco::PerchingSemanticConfig terminal;
  bool use_terminal_config{false};
  bool use_initial_guess{false};
  PerchingInitialGuess initial_guess;

  bool use_tracking_warm_start{false};
  double init_total_time{0.0};
  Eigen::Vector2d init_nu{Eigen::Vector2d::Zero()};
  double init_tau_f{0.0};
  general_utils::vec_E<general_utils::Vec3f> warm_start_guide_path;
  std::vector<double> warm_start_guide_t;
  Eigen::Matrix<double, 1, 2> warm_start_head_yaw{Eigen::Matrix<double, 1, 2>::Zero()};

  double safe_distance{0.45};
  double robot_l{0.28};
  double platform_radius{0.35};
  double robot_radius{0.25};
  double platform_clearance{0.05};
  double platform_collision_activation_distance{1.2};
  double weight_platform_collision{8.0};
  double weight_visual_alignment{1.0};
  double visual_min_distance{0.2};
  double visual_activation_distance{3.0};
  double visual_fx{1.0};
  double visual_fy{1.0};
  double relative_z_min{0.1};
  double relative_z_max{3.0};
  double weight_relative_height{1.0};

  int piece_num{0};
  double min_piece_duration{0.12};
  double min_total_duration{0.0};
  double max_total_duration{-1.0};
  double time_lower_bound_weight{0.0};
  double time_upper_bound_weight{0.0};
  double duration_seed{0.0};
  double duration_seed_weight{0.0};
};

struct DynamicTakeoffProblem
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  general_utils::StatePVAJ nominal_head_pvaj{general_utils::StatePVAJ::Zero()};
  general_utils::StatePVAJ tail_pvaj{general_utils::StatePVAJ::Zero()};

  PerchingSurfaceState surface;
  minco::TakeoffBoundaryConfig boundary;
  bool use_head_mapping{true};

  general_utils::vec_E<general_utils::Vec3f> guide_path;
  std::vector<double> guide_t;

  double release_contact_time{0.20};
  double platform_clearance_after_release{0.05};
  double escape_distance{1.0};
  double escape_height{0.8};
  double safe_distance{0.35};

  double platform_radius{0.35};
  double robot_radius{0.25};
  double robot_l{0.28};
  double platform_clearance{0.05};
  double platform_collision_activation_distance{1.0};

  int piece_num{3};
  double min_duration{0.6};
  double max_duration{3.0};
  double reference_speed{1.5};

  double weight_platform_collision{1.0};
  double weight_relative_height{0.0};
  double relative_z_min{-0.2};
  double relative_z_max{3.0};
};

class TrackingJerkTrajOpt
{
public:
  using Ptr = std::shared_ptr<TrackingJerkTrajOpt>;

  TrackingJerkTrajOpt(const traj_opt::Config &cfg,
                      const std::shared_ptr<ros_interface::RosInterface> &ros_ptr);

  void setMapManager(const general_planner::MapManager::Ptr &map_manager);
  void setSafeDistance(double safe_distance);

  bool optimize(const TrackingProblem &problem,
                geometry_utils::Trajectory &out_traj,
                geometry_utils::Trajectory *out_yaw_traj = nullptr);

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

class TrackingSnapTrajOpt
{
public:
  using Ptr = std::shared_ptr<TrackingSnapTrajOpt>;

  TrackingSnapTrajOpt(const traj_opt::Config &cfg,
                      const std::shared_ptr<ros_interface::RosInterface> &ros_ptr);

  void setMapManager(const general_planner::MapManager::Ptr &map_manager);
  void setSafeDistance(double safe_distance);

  bool optimize(const TrackingProblem &problem,
                geometry_utils::Trajectory &out_traj,
                geometry_utils::Trajectory *out_yaw_traj = nullptr);

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

class PerchingSnapTrajOpt
{
public:
  using Ptr = std::shared_ptr<PerchingSnapTrajOpt>;

  PerchingSnapTrajOpt(const traj_opt::Config &cfg,
                      const std::shared_ptr<ros_interface::RosInterface> &ros_ptr);

  void setMapManager(const general_planner::MapManager::Ptr &map_manager);
  void setSafeDistance(double safe_distance);

  bool optimize(const PerchingProblem &problem,
                geometry_utils::Trajectory &out_traj);

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

class DynamicTakeoffSnapTrajOpt
{
public:
  using Ptr = std::shared_ptr<DynamicTakeoffSnapTrajOpt>;

  DynamicTakeoffSnapTrajOpt(const traj_opt::Config &cfg,
                            const std::shared_ptr<ros_interface::RosInterface> &ros_ptr);

  void setMapManager(const general_planner::MapManager::Ptr &map_manager);
  void setSafeDistance(double safe_distance);

  bool optimize(const DynamicTakeoffProblem &problem,
                geometry_utils::Trajectory &out_traj);

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

} // namespace traj_opt
