#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "data_structure/base/polytope.h"
#include "data_structure/base/trajectory.h"
#include <map_manager/map_manager.hpp>
#include "traj_opt/config.hpp"
#include "traj_opt/tracking_traj_opt.hpp"
#include "traj_opt/minco/boundary_mapping.hpp"
#include "traj_opt/perching_surface_state.hpp"
#include "utils/header/type_utils.hpp"

namespace ros_interface
{
class RosInterface;
}

namespace traj_opt
{

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
