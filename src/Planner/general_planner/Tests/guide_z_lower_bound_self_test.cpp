#include <traj_opt/costfunctional/spatialcosts/guide_path_consistency_penalty.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

int main()
{
  using Vec3 = Eigen::Vector3d;
  const std::vector<Vec3> guide{{0.0, 0.0, 1.5}, {2.0, 0.0, 1.5}};
  const std::vector<double> guide_t{0.0, 2.0};
  auto require = [](bool condition, const char *message) {
    if (!condition) {
      std::cerr << "guide_z_lower_bound_self_test: " << message << std::endl;
      return false;
    }
    return true;
  };

  auto evaluate = [&](double z, Vec3 &gradient) {
    double grad_time = 0.0;
    double max_lateral_violation = 0.0;
    double cost_log = 0.0;
    double max_time_grad = 0.0;
    int out_of_range = 0;
    double max_z_lower_violation = 0.0;
    gradient.setZero();
    const double cost = cost_functional::accumulateGuidePathConsistencyPenalty(
        &guide,
        &guide_t,
        1.0,
        Vec3{1.0, 0.0, z},
        /*lateral_weight=*/0.0,
        /*lateral_tube_radius=*/0.0,
        /*legacy_vertical_tube_radius=*/0.0,
        /*huber_delta=*/0.4,
        /*enable_time_gradient=*/false,
        gradient,
        grad_time,
        /*z_lower_weight=*/100.0,
        /*z_lower_tolerance=*/0.05,
        &max_lateral_violation,
        &cost_log,
        &max_time_grad,
        &out_of_range,
        &max_z_lower_violation);
    if (!require(std::abs(cost - cost_log) < 1.0e-12,
                 "cost log does not match the z-lower cost") ||
        !require(max_lateral_violation == 0.0,
                 "z-only evaluation created a lateral violation") ||
        !require(max_time_grad == 0.0,
                 "time gradient must stay disabled") ||
        !require(out_of_range == 0, "guide sample unexpectedly out of range")) {
      return std::make_pair(-1.0, -1.0);
    }
    return std::make_pair(cost, max_z_lower_violation);
  };

  Vec3 below_grad;
  const auto [below_cost, below_violation] = evaluate(1.40, below_grad);
  if (!require(below_cost > 0.0, "low-z sample must have positive cost") ||
      !require(below_violation > 0.049 && below_violation < 0.051,
               "low-z shortfall is incorrect")) {
    return 1;
  }
  // Gradient descent on this negative gradient raises z.
  if (!require(below_grad.z() < 0.0, "low-z gradient must raise z") ||
      !require(std::abs(below_grad.x()) < 1.0e-12,
               "low-z gradient changed x") ||
      !require(std::abs(below_grad.y()) < 1.0e-12,
               "low-z gradient changed y")) {
    return 1;
  }

  Vec3 boundary_grad;
  const auto [boundary_cost, boundary_violation] = evaluate(1.45, boundary_grad);
  if (!require(std::abs(boundary_cost) < 1.0e-12,
               "tolerance boundary must not have a cost") ||
      !require(std::abs(boundary_violation) < 1.0e-12,
               "tolerance boundary must not violate") ||
      !require(boundary_grad.norm() < 1.0e-12,
               "tolerance boundary must not have a gradient")) {
    return 1;
  }

  Vec3 climb_grad;
  const auto [climb_cost, climb_violation] = evaluate(1.90, climb_grad);
  if (!require(std::abs(climb_cost) < 1.0e-12,
               "upward deviation must not be penalized") ||
      !require(std::abs(climb_violation) < 1.0e-12,
               "upward deviation must not violate") ||
      !require(climb_grad.norm() < 1.0e-12,
               "upward deviation must not have a gradient")) {
    return 1;
  }

  // The optimizer may be behind a climbing frontend guide in time while still
  // being spatially feasible.  The soft floor must stay at the supplied
  // mission-height reference rather than force an impossible time-matched
  // ascent.
  const std::vector<Vec3> climbing_guide{{0.0, 0.0, 1.5},
                                         {0.5, 0.0, 2.6},
                                         {2.0, 0.0, 2.6}};
  const std::vector<double> climbing_times{0.0, 0.2, 1.0};
  Vec3 delayed_climb_grad = Vec3::Zero();
  double delayed_grad_time = 0.0;
  double delayed_z_violation = 0.0;
  const double delayed_climb_cost =
      cost_functional::accumulateGuidePathConsistencyPenalty(
          &climbing_guide,
          &climbing_times,
          /*query_time=*/0.2,
          /*position=*/Vec3{0.5, 0.0, 1.5},
          /*lateral_weight=*/0.0,
          /*lateral_tube_radius=*/0.0,
          /*legacy_vertical_tube_radius=*/0.0,
          /*huber_delta=*/0.4,
          /*enable_time_gradient=*/false,
          delayed_climb_grad,
          delayed_grad_time,
          /*z_lower_weight=*/100.0,
          /*z_lower_tolerance=*/0.05,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
          &delayed_z_violation,
          /*z_floor_reference=*/1.5);
  if (!require(std::abs(delayed_climb_cost) < 1.0e-12,
               "a delayed climb was compared against guide time instead of the z floor") ||
      !require(std::abs(delayed_z_violation) < 1.0e-12,
               "a delayed climb produced a spurious z-floor violation") ||
      !require(delayed_climb_grad.norm() < 1.0e-12,
               "a delayed climb produced a spurious z-floor gradient")) {
    return 1;
  }

  Vec3 drift_grad = Vec3::Zero();
  double drift_grad_time = 0.0;
  double drift_violation = 0.0;
  const double drift_cost = cost_functional::accumulateGuidePathConsistencyPenalty(
      &climbing_guide,
      &climbing_times,
      /*query_time=*/0.2,
      /*position=*/Vec3{0.5, 0.0, 1.40},
      /*lateral_weight=*/0.0,
      /*lateral_tube_radius=*/0.0,
      /*legacy_vertical_tube_radius=*/0.0,
      /*huber_delta=*/0.4,
      /*enable_time_gradient=*/false,
      drift_grad,
      drift_grad_time,
      /*z_lower_weight=*/100.0,
      /*z_lower_tolerance=*/0.05,
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      &drift_violation,
      /*z_floor_reference=*/1.5);
  if (!require(drift_cost > 0.0 && drift_violation > 0.049 && drift_violation < 0.051,
               "the mission-height floor did not penalize downward drift") ||
      !require(drift_grad.z() < 0.0,
               "the mission-height floor did not pull a downward drift upward")) {
    return 1;
  }

  std::cout << "guide_z_lower_bound_self_test: PASS" << std::endl;
  return 0;
}
