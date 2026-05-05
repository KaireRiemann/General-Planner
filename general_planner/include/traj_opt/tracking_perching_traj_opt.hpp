#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "data_structure/base/trajectory.h"
#include "general_core/map_manager.hpp"
#include "traj_opt/config.hpp"
#include "traj_opt/minco/terminal_mapping.hpp"
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
  super_utils::Vec3f position{super_utils::Vec3f::Zero()};
  super_utils::Vec3f velocity{super_utils::Vec3f::Zero()};
  super_utils::Vec3f acceleration{super_utils::Vec3f::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
};

using DynamicTargetStates = super_utils::vec_E<DynamicTargetState>;

struct TrackingProblem
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  super_utils::StatePVAJ head_pvaj{super_utils::StatePVAJ::Zero()};
  super_utils::StatePVAJ tail_pvaj{super_utils::StatePVAJ::Zero()};
  super_utils::vec_E<super_utils::Vec3f> guide_path;
  std::vector<double> guide_t;
  DynamicTargetStates target_prediction;

  double safe_distance{0.45};
  double tracking_distance{3.0};
  double distance_tolerance{0.8};
  double height_offset{0.8};
  double height_tolerance{0.6};

  double weight_tracking{5.0};
  double weight_visibility{1.0};
  double visibility_safe_distance{0.25};
  double visibility_cone_ratio{0.12};
  int visibility_samples{5};
  bool use_esdf_visibility{true};

  int piece_num{0};
  double min_piece_duration{0.12};
  double min_total_duration{0.0};
  double time_lower_bound_weight{0.0};
};

struct PerchingSurfaceState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double t{0.0};
  super_utils::Vec3f position{super_utils::Vec3f::Zero()};
  super_utils::Vec3f velocity{super_utils::Vec3f::Zero()};
  super_utils::Vec3f acceleration{super_utils::Vec3f::Zero()};
  super_utils::Vec3f surface_x{super_utils::Vec3f::UnitX()};
  super_utils::Vec3f surface_y{super_utils::Vec3f::UnitY()};
  super_utils::Vec3f surface_z{super_utils::Vec3f::UnitZ()};
  double yaw{0.0};
  double yaw_rate{0.0};
};

struct PerchingProblem
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  super_utils::StatePVAJ head_pvaj{super_utils::StatePVAJ::Zero()};
  super_utils::StatePVAJ nominal_tail_pvaj{super_utils::StatePVAJ::Zero()};
  super_utils::vec_E<super_utils::Vec3f> guide_path;
  std::vector<double> guide_t;
  PerchingSurfaceState surface;

  minco::PerchingSemanticConfig terminal;
  bool use_terminal_config{false};

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

  int piece_num{0};
  double min_piece_duration{0.12};
  double min_total_duration{0.0};
  double time_lower_bound_weight{0.0};
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
                geometry_utils::Trajectory &out_traj);

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
                geometry_utils::Trajectory &out_traj);

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

} // namespace traj_opt
