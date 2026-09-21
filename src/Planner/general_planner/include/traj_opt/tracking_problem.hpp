#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>
#include <Eigen/Core>
#include "data_structure/base/polytope.h"
#include "data_structure/base/trajectory.h"
#include "utils/header/type_utils.hpp"

namespace traj_opt {

struct DynamicTargetState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double t{0.0};
  general_utils::Vec3f position{general_utils::Vec3f::Zero()};
  general_utils::Vec3f velocity{general_utils::Vec3f::Zero()};
  general_utils::Vec3f acceleration{general_utils::Vec3f::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
  // Absolute ROS/simulation time at which t=0. NaN supports legacy callers.
  double reference_time{std::numeric_limits<double>::quiet_NaN()};
};

using DynamicTargetStates = general_utils::vec_E<DynamicTargetState>;

inline DynamicTargetState sampleTrackingTarget(const DynamicTargetStates &prediction, double t) {
  if (prediction.empty()) return {};
  if (t <= prediction.front().t) return prediction.front();
  if (t >= prediction.back().t) {
    auto out = prediction.back();
    const double dt = t - out.t;
    out.position += dt*out.velocity;
    out.acceleration.setZero();
    out.yaw += dt*out.yaw_rate;
    out.t = t;
    return out;
  }
  const auto right = std::lower_bound(prediction.begin(), prediction.end(), t,
      [](const DynamicTargetState &state, double query) { return state.t < query; });
  const auto &left = *(right - 1);
  const double interval = std::max(1.e-9, right->t - left.t);
  const double alpha = (t - left.t) / interval;
  DynamicTargetState out;
  out.t = t;
  out.reference_time = left.reference_time;
  out.position = left.position + alpha * (right->position - left.position);
  out.velocity = left.velocity + alpha * (right->velocity - left.velocity);
  out.acceleration = left.acceleration + alpha * (right->acceleration - left.acceleration);
  out.yaw = left.yaw + alpha * std::remainder(right->yaw - left.yaw, 2.0*M_PI);
  out.yaw_rate = left.yaw_rate + alpha * (right->yaw_rate - left.yaw_rate);
  return out;
}

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

struct TrackingSolveReport {
  int status{0};
  int iterations{0};
  double elapsed_ms{0.0};
  double cost{0.0};
  double gradient_inf{0.0};
  bool budget_exhausted{false};
  bool used_feasible_iterate{false};
  bool warm_start_used{false};
};

struct TrackingProblem {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  general_utils::StatePVAJ head_pvaj{general_utils::StatePVAJ::Zero()};
  general_utils::StatePVAJ tail_pvaj{general_utils::StatePVAJ::Zero()};
  Eigen::Matrix<double,1,2> head_yaw{Eigen::Matrix<double,1,2>::Zero()};
  double head_yaw_acceleration{0.0};
  double max_yaw_rate{1.2};
  double max_yaw_acceleration{2.0};
  general_utils::vec_Vec3f guide_path;
  std::vector<double> guide_t;
  geometry_utils::PolytopeVec sfcs;
  general_utils::vec_Vec3f viewpoints;
  std::vector<double> target_sample_times;
  DynamicTargetStates target_prediction;
  general_utils::vec_E<TrackingVisibleRegion> visible_regions;
  double tracking_distance{3.0};
  double distance_tolerance{0.3};
  double height_offset{1.0};
  double height_tolerance{0.3};
  double visibility_angle_clearance{0.4};
  double weight_tracking{1000.0};
  double weight_visible_region{10000.0};
  bool use_visible_region{true};
  bool use_corridor{false};
  double corridor_clearance{0.2};
  double min_piece_duration{0.01};
  double min_total_duration{3.0};
  double trusted_horizon{0.0};
  double solve_budget_seconds{0.08};
  int max_iterations{1000};
  std::function<bool()> should_stop;
  std::function<bool(const geometry_utils::Trajectory &, const geometry_utils::Trajectory &)> candidate_feasible;
  TrackingSolveReport *solve_report{nullptr};
};
} // namespace traj_opt
