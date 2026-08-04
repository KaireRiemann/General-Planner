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

  std::cout << "guide_z_lower_bound_self_test: PASS" << std::endl;
  return 0;
}
