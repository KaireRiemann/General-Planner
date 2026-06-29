#include "traj_opt/se3_aggressive_traj_opt.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "traj_opt/costfunctional/penalty_utils.hpp"
#include "traj_opt/costfunctional/temporalmap/quad_inv_time_map.hpp"
#include "traj_opt/costfunctional_manager/se3_aggressive_cost_manager.hpp"
#include "traj_opt/flatness/se3_flatness_map.hpp"
#include "utils/optimization/lbfgs.h"

using general_utils::Mat3Df;
using general_utils::StatePVAJ;
using general_utils::Vec3f;
using general_utils::VecDf;

namespace traj_opt {
namespace {

std::vector<double> polylineArcLengths(const general_utils::vec_E<Vec3f> &path) {
  std::vector<double> arc(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    arc[i] = arc[i - 1] + (path[i] - path[i - 1]).norm();
  }
  return arc;
}

Vec3f interpolateByArc(const general_utils::vec_E<Vec3f> &path,
                       const std::vector<double> &arc,
                       double target_arc) {
  if (path.empty()) {
    return Vec3f::Zero();
  }
  if (path.size() == 1 || target_arc <= 0.0) {
    return path.front();
  }
  if (target_arc >= arc.back()) {
    return path.back();
  }
  const auto upper = std::lower_bound(arc.begin(), arc.end(), target_arc);
  const int idx = std::max(1, static_cast<int>(std::distance(arc.begin(), upper)));
  const double left = arc[static_cast<std::size_t>(idx - 1)];
  const double right = arc[static_cast<std::size_t>(idx)];
  const double ratio = (target_arc - left) / std::max(1.0e-8, right - left);
  return path[static_cast<std::size_t>(idx - 1)] +
         ratio * (path[static_cast<std::size_t>(idx)] - path[static_cast<std::size_t>(idx - 1)]);
}

} // namespace

SE3AggressiveTrajOpt::SE3AggressiveTrajOpt(const traj_opt::Config &cfg,
                                           const ros_interface::RosInterface::Ptr &ros_ptr)
    : cfg_(cfg), ros_ptr_(ros_ptr) {}

void SE3AggressiveTrajOpt::setMapManager(const general_planner::MapManager::Ptr &map_manager) {
  map_manager_ = map_manager;
}

SE3AggressiveTrajOpt::JerkTraj::BoundaryState
SE3AggressiveTrajOpt::toJerkBoundary(const StatePVAJ &state) {
  JerkTraj::BoundaryState out;
  out.col(0) = state.col(0);
  out.col(1) = state.col(1);
  out.col(2) = state.col(2);
  return out;
}

geometry_utils::Trajectory SE3AggressiveTrajOpt::toGeometryTrajectory(const JerkTraj &traj) {
  geometry_utils::Trajectory out;
  const auto &durations = traj.getDurations();
  out.reserve(static_cast<int>(durations.size()));
  for (int i = 0; i < durations.size(); ++i) {
    Eigen::MatrixXd piece_coeff = Eigen::MatrixXd::Zero(3, 8);
    piece_coeff.rightCols(JerkTraj::COEFF_NUM) = traj.getPieceCoeffMat(i);
    out.emplace_back(durations(i), piece_coeff);
  }
  return out;
}

bool SE3AggressiveTrajOpt::initialize(const SE3AggressiveProblem &problem) {
  opt_vars_ = OptimizationVariables();
  opt_vars_.problem = problem;
  opt_vars_.piece_num = std::max(1, problem.piece_num);
  opt_vars_.integral_res = std::max(1, cfg_.integral_reso);
  opt_vars_.times.resize(opt_vars_.piece_num);
  opt_vars_.inner_points.resize(3, std::max(0, opt_vars_.piece_num - 1));
  opt_vars_.penalty_log.resize(5);
  opt_vars_.penalty_log.setZero();

  general_utils::vec_E<Vec3f> path = problem.guide_path;
  if (path.size() < 2) {
    path.clear();
    path.emplace_back(problem.head_pvaj.col(0));
    path.emplace_back(problem.tail_pvaj.col(0));
  }
  if ((path.front() - problem.head_pvaj.col(0)).norm() > 1.0e-4) {
    path.insert(path.begin(), problem.head_pvaj.col(0));
  }
  if ((path.back() - problem.tail_pvaj.col(0)).norm() > 1.0e-4) {
    path.emplace_back(problem.tail_pvaj.col(0));
  }

  const std::vector<double> arc = polylineArcLengths(path);
  const double length = std::max(1.0e-6, arc.back());
  double total_duration = length / std::max(1.0e-3, problem.reference_speed);
  total_duration = std::clamp(total_duration,
                              std::max(1.0e-3, problem.min_duration),
                              std::max(problem.min_duration, problem.max_duration));
  opt_vars_.times.setConstant(total_duration / static_cast<double>(opt_vars_.piece_num));

  for (int i = 1; i < opt_vars_.piece_num; ++i) {
    const double target_arc =
        length * static_cast<double>(i) / static_cast<double>(opt_vars_.piece_num);
    opt_vars_.inner_points.col(i - 1) = interpolateByArc(path, arc, target_arc);
  }
  return opt_vars_.times.allFinite() && opt_vars_.times.minCoeff() > 1.0e-6;
}

void SE3AggressiveTrajOpt::decodeOptimizationVector(
    const Eigen::VectorXd &x,
    Eigen::VectorXd &times,
    Eigen::Matrix<double, 3, Eigen::Dynamic> &inner) const {
  temporal_map::QuadInvTimeMap time_map;
  times.resize(opt_vars_.piece_num);
  for (int i = 0; i < opt_vars_.piece_num; ++i) {
    times(i) = time_map.toTime(x(i));
  }

  inner.resize(3, std::max(0, opt_vars_.piece_num - 1));
  int offset = opt_vars_.piece_num;
  for (int i = 0; i < opt_vars_.piece_num - 1; ++i) {
    inner.col(i) = x.segment<3>(offset);
    offset += 3;
  }
}

double SE3AggressiveTrajOpt::costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &g) {
  return reinterpret_cast<SE3AggressiveTrajOpt *>(ptr)->evaluateCurrentCost(x, g);
}

double SE3AggressiveTrajOpt::evaluateCurrentCost(const Eigen::VectorXd &x,
                                                 Eigen::VectorXd &g) {
  temporal_map::QuadInvTimeMap time_map;
  const SE3AggressiveProblem &problem = opt_vars_.problem;
  ++opt_vars_.iter_num;
  g.setZero();

  VecDf times;
  Mat3Df inner;
  decodeOptimizationVector(x, times, inner);
  if (!times.allFinite() || times.minCoeff() <= 1.0e-6 || !inner.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }

  if (!minco_traj_.generate(inner,
                            toJerkBoundary(problem.head_pvaj),
                            toJerkBoundary(problem.tail_pvaj),
                            times)) {
    return std::numeric_limits<double>::infinity();
  }

  double cost = 0.0;
  JerkTraj::CoeffMat gdC(minco_traj_.getCoefficients().rows(), 3);
  VecDf gdT(opt_vars_.piece_num);
  gdC.setZero();
  gdT.setZero();

  double energy = 0.0;
  JerkTraj::CoeffMat gdC_energy;
  VecDf gdT_energy;
  minco_traj_.getEnergyPartialGradByCoeffs(energy, gdC_energy);
  minco_traj_.getEnergyPartialGradByTimes(gdT_energy);
  cost += energy;
  gdC += gdC_energy;
  gdT += gdT_energy;
  opt_vars_.penalty_log(0) = energy;

  const double total_duration = times.sum();
  if (problem.weight_time > 0.0) {
    cost += problem.weight_time * total_duration;
    gdT.array() += problem.weight_time;

    const double duration_bound_weight = 100.0 * problem.weight_time;
    double penalty = 0.0;
    double d_penalty = 0.0;
    if (cost_functional::positivePartCubic(problem.min_duration - total_duration,
                                           penalty,
                                           d_penalty)) {
      cost += duration_bound_weight * penalty;
      gdT.array() -= duration_bound_weight * d_penalty;
    }
    if (cost_functional::positivePartCubic(total_duration - problem.max_duration,
                                           penalty,
                                           d_penalty)) {
      cost += duration_bound_weight * penalty;
      gdT.array() += duration_bound_weight * d_penalty;
    }
  }

  cost_functional_manager::SE3AggressiveCostManager cost_manager;
  cost_manager.reset(cfg_, problem, map_manager_);

  const auto &coeffs = minco_traj_.getCoefficients();
  double seg_start = 0.0;
  opt_vars_.max_vel = 0.0;
  opt_vars_.max_thrust = 0.0;
  opt_vars_.max_body_rate = 0.0;
  opt_vars_.max_corridor_violation = 0.0;
  for (int i = 0; i < opt_vars_.piece_num; ++i) {
    const double T = times(i);
    const double inv_K = 1.0 / static_cast<double>(opt_vars_.integral_res);
    const double step = T * inv_K;
    const int base = i * JerkTraj::COEFF_NUM;
    const auto coeff_block = coeffs.template block<JerkTraj::COEFF_NUM, 3>(base, 0);

    for (int k = 0; k <= opt_vars_.integral_res; ++k) {
      const double alpha = static_cast<double>(k) * inv_K;
      const double t = alpha * T;
      const double node = (k == 0 || k == opt_vars_.integral_res) ? 0.5 : 1.0;
      const double common = node * step;

      JerkTraj::BasisRow bp, bv, ba, bj, bs;
      JerkTraj::computeBasisFunctions(t, bp, bv, ba, bj, bs);
      const JerkTraj::BasisRow b5 = JerkTraj::derivativeBasis(5, t);

      const Vec3f p = coeff_block.transpose() * bp.transpose();
      const Vec3f v = coeff_block.transpose() * bv.transpose();
      const Vec3f a = coeff_block.transpose() * ba.transpose();
      const Vec3f j = coeff_block.transpose() * bj.transpose();
      const Vec3f s = coeff_block.transpose() * bs.transpose();
      const Vec3f crackle = coeff_block.transpose() * b5.transpose();

      Vec3f gp = Vec3f::Zero();
      Vec3f gv = Vec3f::Zero();
      Vec3f ga = Vec3f::Zero();
      Vec3f gj = Vec3f::Zero();
      Vec3f gs = Vec3f::Zero();
      double gyaw = 0.0;
      double gyaw_rate = 0.0;
      double gt = 0.0;

      const double sample_yaw =
          problem.use_yaw ? problem.yaw : (problem.yaw_heading_to_velocity ? NAN : 0.0);
      const double sample_yaw_rate = problem.use_yaw ? problem.yaw_rate : 0.0;
      const double sample_cost = cost_manager.evaluateIntegral(i * (opt_vars_.integral_res + 1) + k,
                                                               t,
                                                               seg_start + t,
                                                               i,
                                                               k,
                                                               p,
                                                               v,
                                                               a,
                                                               j,
                                                               s,
                                                               sample_yaw,
                                                               sample_yaw_rate,
                                                               gp,
                                                               gv,
                                                               ga,
                                                               gj,
                                                               gs,
                                                               gyaw,
                                                               gyaw_rate,
                                                               gt);
      (void)gyaw;
      (void)gyaw_rate;

      cost += common * sample_cost;
      gdC.template block<JerkTraj::COEFF_NUM, 3>(base, 0).noalias() +=
          (bp.transpose() * gp.transpose() +
           bv.transpose() * gv.transpose() +
           ba.transpose() * ga.transpose() +
           bj.transpose() * gj.transpose() +
           bs.transpose() * gs.transpose()) *
          common;

      gdT(i) += node * inv_K * sample_cost;
      gdT(i) += (gp.dot(v) +
                 gv.dot(a) +
                 ga.dot(j) +
                 gj.dot(s) +
                 gs.dot(crackle)) *
                alpha * common;
      if (std::abs(gt) > 1.0e-12) {
        if (i > 0) {
          gdT.head(i).array() += gt * common;
        }
        gdT(i) += gt * alpha * common;
      }

      opt_vars_.max_vel = std::max(opt_vars_.max_vel, v.norm());
      opt_vars_.max_thrust =
          std::max(opt_vars_.max_thrust, (a + cfg_.grav * Vec3f::UnitZ()).norm());
      SE3FlatnessMap flatness;
      flatness.setYawMode(problem.use_yaw, problem.yaw_heading_to_velocity);
      SE3FlatnessOutput flat;
      if (flatness.forward(v, a, j, s, sample_yaw, sample_yaw_rate, cfg_.grav, flat)) {
        opt_vars_.max_body_rate =
            std::max(opt_vars_.max_body_rate, flat.omega.norm());
      }
    }
    seg_start += T;
  }

  const auto grad_result = minco_traj_.propagateGradFull(gdC, gdT);
  for (int i = 0; i < opt_vars_.piece_num; ++i) {
    g(i) += time_map.backward(x(i), times(i), grad_result.grad_by_times(i));
  }

  int offset = opt_vars_.piece_num;
  for (int i = 0; i < opt_vars_.piece_num - 1; ++i) {
    g.segment<3>(offset) += grad_result.grad_by_points.col(i);
    offset += 3;
  }

  opt_vars_.penalty_log.tail(4) = cost_manager.getPenaltyLog();
  opt_vars_.max_corridor_violation = cost_manager.getMaxCorridorViolation();
  return cost;
}

double SE3AggressiveTrajOpt::optimizeInternal(geometry_utils::Trajectory &traj,
                                              double rel_cost_tol) {
  temporal_map::QuadInvTimeMap time_map;
  Eigen::VectorXd x(opt_vars_.piece_num + 3 * std::max(0, opt_vars_.piece_num - 1));
  for (int i = 0; i < opt_vars_.piece_num; ++i) {
    x(i) = time_map.toTau(opt_vars_.times(i));
  }
  int offset = opt_vars_.piece_num;
  for (int i = 0; i < opt_vars_.piece_num - 1; ++i) {
    x.segment<3>(offset) = opt_vars_.inner_points.col(i);
    offset += 3;
  }

  opt_vars_.iter_num = 0;
  double min_cost = 0.0;
  math_utils::lbfgs::lbfgs_parameter_t params;
  params.mem_size = 128;
  params.past = 3;
  params.min_step = 1.0e-32;
  params.g_epsilon = 0.0;
  params.delta = rel_cost_tol;

  const int ret = math_utils::lbfgs::lbfgs_optimize(x,
                                                   min_cost,
                                                   &SE3AggressiveTrajOpt::costFunctional,
                                                   nullptr,
                                                   nullptr,
                                                   this,
                                                   params);
  const bool recoverable_ret =
      ret == math_utils::lbfgs::LBFGSERR_MAXIMUMITERATION ||
      ret == math_utils::lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
      ret == math_utils::lbfgs::LBFGSERR_MINIMUMSTEP ||
      ret == math_utils::lbfgs::LBFGSERR_WIDTHTOOSMALL;
  if (ret < 0 && !recoverable_ret) {
    traj.clear();
    std::cout << " -- [SE3AggressiveTrajOpt] SE3_OPT_FAILED reason="
              << math_utils::lbfgs::lbfgs_strerror(ret) << std::endl;
    return std::numeric_limits<double>::infinity();
  }
  if (recoverable_ret) {
    std::cout << " -- [SE3AggressiveTrajOpt] SE3_OPT_RECOVERABLE reason="
              << math_utils::lbfgs::lbfgs_strerror(ret) << std::endl;
  }

  Eigen::VectorXd grad = Eigen::VectorXd::Zero(x.size());
  min_cost = evaluateCurrentCost(x, grad);
  traj = toGeometryTrajectory(minco_traj_);
  if (ros_ptr_) {
    traj.start_WT = ros_ptr_->getSimTime();
  }
  return min_cost;
}

bool SE3AggressiveTrajOpt::optimize(const SE3AggressiveProblem &problem,
                                    geometry_utils::Trajectory &out_traj) {
  if (!initialize(problem)) {
    std::cout << " -- [SE3AggressiveTrajOpt] SE3_OPT_FAILED reason=INIT_FAILED" << std::endl;
    return false;
  }

  std::cout << " -- [SE3AggressiveTrajOpt] SE3_OPT_START pieces=" << opt_vars_.piece_num
            << ", T=" << opt_vars_.times.sum() << std::endl;
  out_traj.clear();
  const double rel_tol = cfg_.opt_accuracy > 0.0 ? cfg_.opt_accuracy : 1.0e-5;
  const double cost = optimizeInternal(out_traj, rel_tol);
  if (!std::isfinite(cost) || out_traj.empty()) {
    return false;
  }

  std::cout << " -- [SE3AggressiveTrajOpt] SE3_OPT_SUCCESS duration="
            << out_traj.getTotalDuration()
            << ", max_vel=" << opt_vars_.max_vel
            << ", max_thrust=" << opt_vars_.max_thrust
            << ", max_body_rate=" << opt_vars_.max_body_rate
            << ", corridor_violation=" << opt_vars_.max_corridor_violation
            << std::endl;
  return true;
}

} // namespace traj_opt
