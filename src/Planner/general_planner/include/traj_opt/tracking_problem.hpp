#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>
#include <Eigen/Core>
#include "data_structure/base/polytope.h"
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
    out.position += dt * out.velocity;
    out.yaw += dt * out.yaw_rate;
    out.acceleration.setZero();
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
  out.yaw = left.yaw + alpha * (right->yaw - left.yaw);
  out.yaw_rate = left.yaw_rate + alpha * (right->yaw_rate - left.yaw_rate);
  return out;
}

// Exact time derivative of the piecewise position reference used by the cost.
inline general_utils::Vec3f trackingTargetPositionDerivative(
    const DynamicTargetStates &prediction, double t) {
  if (prediction.empty() || t < prediction.front().t) return general_utils::Vec3f::Zero();
  if (t >= prediction.back().t) return prediction.back().velocity;
  const auto right = std::upper_bound(prediction.begin(), prediction.end(), t,
      [](double query, const DynamicTargetState &state) { return query < state.t; });
  const auto &left = *(right - 1);
  return (right->position - left.position) / std::max(1.e-9, right->t - left.t);
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

struct TrackingProblem
{
  double max_yaw_rate{1.2};
  double max_yaw_acceleration{2.0};
  double head_yaw_acceleration{0.0};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  general_utils::StatePVAJ head_pvaj{general_utils::StatePVAJ::Zero()};
  general_utils::StatePVAJ tail_pvaj{general_utils::StatePVAJ::Zero()};
  Eigen::Matrix<double, 1, 2> head_yaw{Eigen::Matrix<double, 1, 2>::Zero()};

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
  double weight_visible_region{3.0};
  double weight_fov{20.0};

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
  double fov_front_margin{0.05};
  Eigen::Matrix3d camera_rotation{(Eigen::Matrix3d() << 0,0,1,-1,0,0,0,-1,0).finished()};
  general_utils::Vec3f camera_translation{general_utils::Vec3f::Zero()};
  double target_half_height{0.7};
  double target_half_width{0.5};
  double joint_sample_dt{0.05};
  bool dense_joint_sample_enable{true};
  int visibility_samples{5};
  bool use_esdf_visibility{true};
  bool use_visible_region{true};
  bool reacquire_mode{false};

  int piece_num{0};
  double min_piece_duration{0.12};
  double min_total_duration{0.0};
  bool use_corridor{false};
  // Absolute execution epoch is carried by target_prediction.reference_time.
  // Trusted samples and explicitly extrapolated samples keep separate horizons.
  double trusted_horizon{0.0};
  double max_total_duration{4.0};
  double closing_gain{0.6};
  double max_closing_speed{2.0};
  double solve_budget_seconds{0.06};
  int max_iterations{100};
  std::function<bool()> should_stop;
};


} // namespace traj_opt
