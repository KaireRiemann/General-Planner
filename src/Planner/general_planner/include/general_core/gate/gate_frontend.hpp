#pragma once
#include <aperture_detector/ApertureObservation.h>
#include <traj_opt/se3_aggressive_traj_opt.hpp>
#include <functional>

namespace general_planner::gate {
struct Config {
  std::string validation_policy{"corridor_and_map"};
  double body_radius{0.165}, body_height{0.10}, margin{0.03};
  double wall_depth{0.30}, exit_distance{1.5}, max_distance{5.0};
  double corridor_half_width{1.0}, corridor_half_height{0.8}, piece_length{0.6};
  double speed{2.0}, thrust_min{4.0}, thrust_max{19.0}, body_rate{4.0}, tilt{0.78};
  double weight_time{5.0}, weight_corridor{2.5e6};
  double observation_timeout{20.0}, observation_max_age{2.0}, planning_timeout{8.0};
  double odom_timeout{0.3}, finish_timeout{5.0}, finish_radius{0.15};
  double feedback_max_delay{0.0};
  double tracking_error{0.5}, validation_dt{0.02}, mass{1.168};
  int stable_observations{2};
  bool require_known_free{true}, restore_start_height{false};
  double tunnel_half_depth{0.0}, overlap_slack{0.05};
  double weight_velocity{1e5}, weight_body_rate{1e5}, weight_tilt{1e5}, weight_thrust{1e5};
  double smooth_eps{.01}, relative_cost_tolerance{1e-6};
  double center_tolerance{.02}, normal_tolerance{.05}, area_tolerance{.15};
  double near_gate_lateral_error{.025}, max_spatial_step{.01}, completion_hold_time{.5};
  double map_anchor_search_radius{.08}, map_anchor_search_step{.01};
  int integral_steps{24}, max_iterations{400};
};
struct Plan {
  traj_opt::SE3AggressiveProblem problem;
  geometry_utils::Trajectory trajectory;
  Eigen::Vector3d center, normal;
  double wall_half_depth{0.15};
  std::string frame;
};
using BodyClear = std::function<bool(const Eigen::Vector3d &, const Eigen::Matrix3d &,
                                     const Eigen::Vector3d &)>;
// Pure geometry builder. It consumes an observed convex aperture, never clears map cells.
bool buildProblem(const aperture_detector::ApertureObservation &observation,
                  const Eigen::Vector3d &start, double yaw, const Config &config,
                  Plan &plan, std::string &reason);
bool validateSample(const Plan &plan, double time, const Config &config,
                    const BodyClear &body_clear, std::string &reason);
bool validateTrajectory(const Plan &plan, const Config &config,
                        const BodyClear &body_clear, std::string &reason);
}  // namespace general_planner::gate

namespace general_planner::gate {
// Intersect the observed tunnel with supporting halfspaces of occupied voxel
// boxes at the wall. This makes the optimizer see the same frame as validation.
// No map cells are cleared; an infeasible intersection is rejected.
bool constrainTunnelToMap(Plan &plan, const Config &cfg,
                          const general_utils::vec_E<general_utils::Vec3f> &cells,
                          double resolution, std::string &reason);
}
