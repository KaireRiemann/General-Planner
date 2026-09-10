#include "traj_opt/se3_aggressive_traj_opt.hpp"
#include "traj_opt/costfunctional/temporalmap/quad_inv_time_map.hpp"
#include <iostream>
#include <stdexcept>

namespace traj_opt {
struct SE3SpatialMapTestAccess {
  static void require(bool value, const char *message) {
    if (!value) throw std::runtime_error(message);
  }
  static Eigen::Matrix<double, 6, Eigen::Dynamic> box(double lo, double hi) {
    Eigen::Matrix<double, 6, Eigen::Dynamic> h(6, 6);
    h.setZero();
    for (int axis = 0; axis < 3; ++axis) {
      h(axis, 2 * axis) = 1;
      h(axis, 2 * axis + 1) = -1;
      h(axis + 3, 2 * axis) = axis == 0 ? hi : 2;
      h(axis + 3, 2 * axis + 1) = axis == 0 ? lo : -2;
    }
    return h;
  }
  static void run() {
    Config cfg{};
    cfg.grav = 9.81;
    cfg.integral_reso = 8;
    cfg.smooth_eps = 0.01;
    cfg.opt_accuracy = 1e-5;
    SE3AggressiveTrajOpt opt(cfg, nullptr);
    SE3AggressiveProblem p;
    p.piece_num = 3;
    p.hpolys = {box(-2, 0.5), box(-0.5, 2)};
    p.hpolys[1].conservativeResize(6, 7);
    p.hpolys[1].col(6) << 0, 1, 1, 0, 0, 0;  // clipped box: six vertices
    p.piece_to_corridor = {0, 0, 1};
    p.head_pvaj(0, 0) = -1.5;
    p.tail_pvaj(0, 0) = 1.5;
    p.max_tilt = 0.10;
    p.weight_tilt = 3.7;
    p.weight_body_rate = 0;
    p.weight_thrust = 0;
    p.weight_vel = 0;
    p.weight_corridor = 0;
    require(opt.initialize(p), "valid overlap rejected");
    require(opt.spatial_map_.getUnconstrainedDim(1) == 8, "box spatial dimension is not vertex count");
    require(opt.spatial_map_.getUnconstrainedDim(2) == 6, "mixed polytope dimensions not preserved");
    int size = p.piece_num;
    for (int i = 1; i < p.piece_num; ++i) size += opt.spatial_map_.getUnconstrainedDim(i);
    Eigen::VectorXd x(size);
    temporal_map::QuadInvTimeMap time_map;
    for (int i = 0; i < p.piece_num; ++i) x(i) = time_map.toTau(1.2 + 0.1 * i);
    // Arbitrary unconstrained variables must remain in the designated cell/intersection.
    for (int i = p.piece_num; i < size; ++i) x(i) = 0.25 + 0.02 * (i % 7);
    Eigen::VectorXd times;
    Eigen::Matrix3Xd inner;
    opt.decodeOptimizationVector(x, times, inner);
    require(inner(0, 0) >= -2 && inner(0, 0) <= 0.5, "same-cell point outside corridor");
    require(inner(0, 1) >= -0.5 && inner(0, 1) <= 0.5, "transition point outside intersection");
    for (int i = 1; i < p.piece_num; ++i) {
      const auto encoded = opt.spatial_map_.toUnconstrained(inner.col(i - 1), i);
      require((opt.spatial_map_.toPhysical(encoded, i) - inner.col(i - 1)).norm() < 1e-4,
              "spatial map inverse round-trip failed");
    }
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(size);
    const double cost = opt.evaluateCurrentCost(x, grad);
    require(std::isfinite(cost) && grad.allFinite(), "non-finite objective");
    for (int i = 0; i < size; ++i) {
      const double eps = 1e-6;
      Eigen::VectorXd plus = x, minus = x, scratch = grad;
      plus(i) += eps; minus(i) -= eps;
      const double numeric = (opt.evaluateCurrentCost(plus, scratch) - opt.evaluateCurrentCost(minus, scratch)) / (2 * eps);
      require(std::abs(numeric - grad(i)) < 2e-4 * std::max(1.0, std::abs(numeric)), "mapped MINCO gradient mismatch");
    }
    geometry_utils::Trajectory trajectory;
    require(opt.optimize(p, trajectory), "mapped solver failed");
    double knot_time = 0.0;
    for (int i = 0; i < p.piece_num - 1; ++i) {
      knot_time += trajectory[i].getDuration();
      const auto knot = trajectory.getPos(knot_time);
      require(knot.x() >= (i == 0 ? -2.0 : -0.5) - 1e-5 && knot.x() <= 0.5 + 1e-5,
              "optimized knot escaped mapped region");
    }
    auto canceled = p; canceled.should_stop = [] { return true; };
    geometry_utils::Trajectory canceled_trajectory;
    require(!opt.optimize(canceled, canceled_trajectory) && canceled_trajectory.empty(), "canceled solve committed trajectory");
    auto bad = p;
    bad.hpolys[1] = box(0.8, 2);
    require(!opt.initialize(bad), "disjoint corridors accepted");
    bad = p; bad.piece_to_corridor = {0, 1, 0};
    require(!opt.initialize(bad), "reversed corridor sequence accepted");
    bad = p; bad.hpolys.insert(bad.hpolys.begin() + 1, box(-0.4, 0.4));
    bad.piece_to_corridor = {0, 0, 2};
    require(!opt.initialize(bad), "skipped corridor accepted");
    bad = p; bad.head_pvaj(0, 0) = -3;
    require(!opt.initialize(bad), "outside start accepted");
    bad = p; bad.hpolys[0](0, 0) = NAN;
    require(!opt.initialize(bad), "non-finite planes accepted");
    p.use_corridor = false;
    require(opt.initialize(p), "identity fallback rejected");
    require(opt.spatial_map_.getUnconstrainedDim(1) == 3, "identity map was not reset");
    std::cout << "SE3 spatial map: containment, overlap, full gradient, invalid input and reset passed\n";
  }
};
}
int main() {
  try { traj_opt::SE3SpatialMapTestAccess::run(); }
  catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
  return 0;
}
