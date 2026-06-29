#include "traj_opt/tracking_perching_traj_opt.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>

#include "ros_interface/ros_interface.hpp"
#include "traj_opt/costfunctional/penalty_utils.hpp"
#include "traj_opt/costfunctional/spatialcosts/acceleration_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/angular_rate_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/esdf_distance_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/flatness_state.hpp"
#include "traj_opt/costfunctional/spatialcosts/jerk_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/polytope_position_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/thrust_band_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/velocity_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialmap/polytope_spatial_map.hpp"
#include "traj_opt/costfunctional/temporalcosts/linear_time_cost.hpp"
#include "traj_opt/costfunctional/temporalmap/quad_inv_time_map.hpp"
#include "traj_opt/costfunctional_manager/perching_cost_manager.hpp"
#include "traj_opt/costfunctional_manager/takeoff_cost_manager.hpp"
#include "traj_opt/costfunctional_manager/tracking_cost_manager.hpp"
#include "traj_opt/minco/minco_optimizer.hpp"
#include "utils/geometry/geometry_utils.h"
#include "utils/optimization/lbfgs.h"

namespace traj_opt
{
namespace
{

using geometry_utils::Trajectory;
using general_utils::Mat3Df;
using general_utils::StatePVAJ;
using general_utils::Vec3f;
using general_utils::VecDf;

constexpr double kTiny = 1.0e-9;

struct R3IdentitySpatialMap
{
  using VectorType = Eigen::Vector3d;

  int getUnconstrainedDim(int) const
  {
    return 3;
  }

  VectorType toPhysical(const Eigen::VectorXd &xi, int) const
  {
    VectorType out = VectorType::Zero();
    if (xi.size() >= 3)
    {
      out = xi.head<3>();
    }
    return out;
  }

  Eigen::VectorXd toUnconstrained(const VectorType &p, int) const
  {
    Eigen::VectorXd xi(3);
    xi = p;
    return xi;
  }

  Eigen::VectorXd backwardGrad(const Eigen::VectorXd &, const VectorType &grad_p, int) const
  {
    Eigen::VectorXd grad_xi(3);
    grad_xi = grad_p;
    return grad_xi;
  }

  void addNormPenalty(const Eigen::VectorXd &, double &, Eigen::VectorXd &) const
  {
  }
};

struct TaskTimeCost
{
  double linear_weight{0.0};
  double min_piece_duration{0.0};
  double min_total_duration{0.0};
  double max_total_duration{-1.0};
  double upper_bound_weight{0.0};
  double lower_bound_weight{0.0};
  double duration_seed{0.0};
  double duration_seed_weight{0.0};
  double smooth_eps{0.01};

  double operator()(const std::vector<double> &Ts, Eigen::VectorXd &grad) const
  {
    double cost = 0.0;
    double total_t = 0.0;
    for (std::size_t i = 0; i < Ts.size(); ++i)
    {
      total_t += Ts[i];
      cost += linear_weight * Ts[i];
      grad(static_cast<Eigen::Index>(i)) += linear_weight;

      double f = 0.0;
      double df = 0.0;
      if (lower_bound_weight > 0.0 &&
          min_piece_duration > 0.0 &&
          cost_functional::smoothedL1(min_piece_duration - Ts[i],
                                      smooth_eps,
                                      f,
                                      df))
      {
        cost += lower_bound_weight * f;
        grad(static_cast<Eigen::Index>(i)) -= lower_bound_weight * df;
      }
    }

    double f = 0.0;
    double df = 0.0;
    if (lower_bound_weight > 0.0 &&
        min_total_duration > 0.0 &&
        cost_functional::smoothedL1(min_total_duration - total_t,
                                    smooth_eps,
                                    f,
                                    df))
    {
      cost += lower_bound_weight * f;
      for (Eigen::Index i = 0; i < grad.size(); ++i)
      {
        grad(i) -= lower_bound_weight * df;
      }
    }
    if (upper_bound_weight > 0.0 &&
        max_total_duration > 0.0 &&
        cost_functional::smoothedL1(total_t - max_total_duration,
                                    smooth_eps,
                                    f,
                                    df))
    {
      cost += upper_bound_weight * f;
      for (Eigen::Index i = 0; i < grad.size(); ++i)
      {
        grad(i) += upper_bound_weight * df;
      }
    }
    if (duration_seed_weight > 0.0 && duration_seed > 0.0)
    {
      const double err = total_t - duration_seed;
      cost += duration_seed_weight * err * err;
      for (Eigen::Index i = 0; i < grad.size(); ++i)
      {
        grad(i) += 2.0 * duration_seed_weight * err;
      }
    }
    return cost;
  }
};

struct TaskTimeMap
{
  static constexpr double kMinTime = 1.0e-4;

  void setUpperBound(const double upper_bound)
  {
    upper_bound_ =
        std::isfinite(upper_bound) && upper_bound > kMinTime
            ? upper_bound
            : std::numeric_limits<double>::infinity();
  }

  bool bounded() const
  {
    return std::isfinite(upper_bound_);
  }

  double toTime(const double tau) const
  {
    if (!bounded())
    {
      return unbounded_.toTime(tau);
    }

    const double z = std::clamp(tau, -60.0, 60.0);
    const double sigmoid = 1.0 / (1.0 + std::exp(-z));
    return kMinTime + (upper_bound_ - kMinTime) * sigmoid;
  }

  double toTau(const double T) const
  {
    if (!bounded())
    {
      return unbounded_.toTau(T);
    }

    const double span = upper_bound_ - kMinTime;
    const double ratio = std::clamp((T - kMinTime) / span,
                                    1.0e-6,
                                    1.0 - 1.0e-6);
    return std::log(ratio / (1.0 - ratio));
  }

  double backward(const double tau, const double T, const double gradT) const
  {
    if (!bounded())
    {
      return unbounded_.backward(tau, T, gradT);
    }

    (void)tau;
    const double span = upper_bound_ - kMinTime;
    const double ratio = std::clamp((T - kMinTime) / span, 0.0, 1.0);
    return gradT * span * ratio * (1.0 - ratio);
  }

private:
  temporal_map::QuadInvTimeMap unbounded_;
  double upper_bound_{std::numeric_limits<double>::infinity()};
};

template <int S>
using TaskOptimizer = minco::MINCOOptimizer<3, S, TaskTimeMap, R3IdentitySpatialMap>;

template <int S>
using TaskTraj = typename TaskOptimizer<S>::TrajType;

template <int S>
using TaskBoundaryState = typename TaskTraj<S>::BoundaryState;

double clampPositive(double value, double fallback)
{
  if (!std::isfinite(value) || value <= 0.0)
  {
    return fallback;
  }
  return value;
}

Vec3f normalizedOr(const Vec3f &v, const Vec3f &fallback)
{
  if (!v.allFinite() || v.norm() < 1.0e-6)
  {
    return fallback;
  }
  return v.normalized();
}

double pathLength(const general_utils::vec_E<Vec3f> &path)
{
  double length = 0.0;
  for (int i = 1; i < static_cast<int>(path.size()); ++i)
  {
    length += (path[i] - path[i - 1]).norm();
  }
  return length;
}

void normalizeTrackingHPoly(spatial_map::PolyhedronH &poly)
{
  if (poly.rows() == 0)
  {
    return;
  }
  Eigen::ArrayXd norms = poly.leftCols<3>().rowwise().norm();
  norms = norms.max(1.0e-12);
  poly.array().colwise() /= norms;
}

general_utils::vec_E<Vec3f> sanitizeGuide(const general_utils::vec_E<Vec3f> &guide_path,
                                        const Vec3f &start,
                                        const Vec3f &goal)
{
  general_utils::vec_E<Vec3f> out;
  out.reserve(std::max<std::size_t>(guide_path.size(), 2));
  out.emplace_back(start);
  for (const auto &p : guide_path)
  {
    if (!p.allFinite())
    {
      continue;
    }
    if ((p - out.back()).norm() > 1.0e-4)
    {
      out.emplace_back(p);
    }
  }
  if ((goal - out.back()).norm() > 1.0e-4)
  {
    out.emplace_back(goal);
  }
  if (out.size() == 1)
  {
    out.emplace_back(goal);
  }
  return out;
}

Vec3f interpolateByArc(const general_utils::vec_E<Vec3f> &path,
                       const std::vector<double> &arc,
                       double s)
{
  if (path.empty())
  {
    return Vec3f::Zero();
  }
  if (path.size() == 1 || s <= 0.0)
  {
    return path.front();
  }
  if (s >= arc.back())
  {
    return path.back();
  }

  const auto it = std::lower_bound(arc.begin(), arc.end(), s);
  const int idx = static_cast<int>(std::distance(arc.begin(), it));
  const double left = arc[static_cast<std::size_t>(idx - 1)];
  const double right = arc[static_cast<std::size_t>(idx)];
  const double alpha = (s - left) / std::max(kTiny, right - left);
  return path[static_cast<std::size_t>(idx - 1)] +
         alpha * (path[static_cast<std::size_t>(idx)] - path[static_cast<std::size_t>(idx - 1)]);
}

double estimateDuration(double length,
                        double start_speed,
                        double end_speed,
                        double max_vel,
                        double max_acc)
{
  if (length < 1.0e-6)
  {
    return 0.2;
  }

  max_vel = std::max(0.2, max_vel);
  max_acc = std::max(0.2, max_acc);
  start_speed = std::clamp(start_speed, 0.0, max_vel);
  end_speed = std::clamp(end_speed, 0.0, max_vel);

  const double acc_len = std::max(0.0, (max_vel * max_vel - start_speed * start_speed) / (2.0 * max_acc));
  const double dec_len = std::max(0.0, (max_vel * max_vel - end_speed * end_speed) / (2.0 * max_acc));
  if (length > acc_len + dec_len)
  {
    return (max_vel - start_speed) / max_acc +
           (max_vel - end_speed) / max_acc +
           (length - acc_len - dec_len) / max_vel;
  }

  const double peak_sq = std::max(0.0, 0.5 * (start_speed * start_speed + end_speed * end_speed) +
                                           max_acc * length);
  const double peak = std::sqrt(peak_sq);
  return std::max(0.0, (peak - start_speed) / max_acc) +
         std::max(0.0, (peak - end_speed) / max_acc);
}

template <int S>
TaskBoundaryState<S> toBoundaryState(const StatePVAJ &state)
{
  TaskBoundaryState<S> out;
  out.setZero();
  for (int i = 0; i < S && i < state.cols(); ++i)
  {
    out.col(i) = state.col(i);
  }
  return out;
}

template <int S>
Trajectory toGeometryTrajectory(const TaskTraj<S> &traj)
{
  Trajectory out;
  const auto &durations = traj.getDurations();
  out.reserve(static_cast<int>(durations.size()));
  for (int i = 0; i < durations.size(); ++i)
  {
    out.emplace_back(durations(i), traj.getPieceCoeffMat(i));
  }
  return out;
}

template <int DIM, int S>
Trajectory toGeometryTrajectoryGeneric(const minco::MINCOTrajectory<DIM, S> &traj)
{
  Trajectory out;
  const auto &durations = traj.getDurations();
  out.reserve(static_cast<int>(durations.size()));
  for (int i = 0; i < durations.size(); ++i)
  {
    out.emplace_back(durations(i), traj.getPieceCoeffMat(i));
  }
  return out;
}

template <int S>
bool prepareInitialState(const traj_opt::Config &cfg,
                         const StatePVAJ &head,
                         const StatePVAJ &tail,
                         const general_utils::vec_E<Vec3f> &guide_path,
                         int requested_piece_num,
                         double min_piece_duration,
                         std::vector<double> &times,
                         typename TaskOptimizer<S>::WaypointsType &waypoints)
{
  const auto path = sanitizeGuide(guide_path, head.col(0), tail.col(0));
  const double length = std::max(pathLength(path), (tail.col(0) - head.col(0)).norm());
  if (length < 1.0e-5)
  {
    return false;
  }

  std::vector<double> arc(path.size(), 0.0);
  for (int i = 1; i < static_cast<int>(path.size()); ++i)
  {
    arc[static_cast<std::size_t>(i)] =
        arc[static_cast<std::size_t>(i - 1)] + (path[i] - path[i - 1]).norm();
  }

  const double max_vel = clampPositive(cfg.max_vel, 2.0);
  const double max_acc = clampPositive(cfg.max_acc, 2.0);
  const double segment_length = std::max(0.6, 0.45 * max_vel);
  int piece_num = requested_piece_num > 0 ? requested_piece_num : cfg.piece_num;
  if (piece_num <= 0)
  {
    piece_num = static_cast<int>(std::ceil(length / segment_length));
  }
  piece_num = std::clamp(piece_num, 1, 32);

  const double duration = std::max(static_cast<double>(piece_num) * std::max(0.05, min_piece_duration),
                                   estimateDuration(length,
                                                    head.col(1).norm(),
                                                    tail.col(1).norm(),
                                                    max_vel,
                                                    max_acc));
  times.assign(static_cast<std::size_t>(piece_num), duration / static_cast<double>(piece_num));

  waypoints.resize(piece_num + 1, 3);
  for (int i = 0; i <= piece_num; ++i)
  {
    const double s = length * static_cast<double>(i) / static_cast<double>(piece_num);
    waypoints.row(i) = interpolateByArc(path, arc, s).transpose();
  }
  waypoints.row(0) = head.col(0).transpose();
  waypoints.row(piece_num) = tail.col(0).transpose();
  return true;
}

template <int S>
bool prepareTimedInitialState(const traj_opt::Config &cfg,
                              const StatePVAJ &head,
                              const StatePVAJ &tail,
                              const PerchingInitialGuess &initial_guess,
                              int requested_piece_num,
                              double min_piece_duration,
                              std::vector<double> &times,
                              typename TaskOptimizer<S>::WaypointsType &waypoints)
{
  if (!initial_guess.valid ||
      initial_guess.guide_path.size() < 2 ||
      !std::isfinite(initial_guess.total_time) ||
      initial_guess.total_time <= 0.0)
  {
    return false;
  }

  const auto path = sanitizeGuide(initial_guess.guide_path, head.col(0), tail.col(0));
  if (path.size() < 2)
  {
    return false;
  }

  std::vector<double> arc(path.size(), 0.0);
  for (int i = 1; i < static_cast<int>(path.size()); ++i)
  {
    arc[static_cast<std::size_t>(i)] =
        arc[static_cast<std::size_t>(i - 1)] + (path[i] - path[i - 1]).norm();
  }
  const double length = std::max(arc.back(), (tail.col(0) - head.col(0)).norm());
  if (length < 1.0e-5)
  {
    return false;
  }

  std::vector<double> path_t;
  bool timed_path_valid =
      initial_guess.guide_t.size() == initial_guess.guide_path.size() &&
      initial_guess.guide_t.size() >= 2;
  if (timed_path_valid)
  {
    path_t = initial_guess.guide_t;
    path_t.front() = 0.0;
    path_t.back() = initial_guess.total_time;
    for (int i = 1; i < static_cast<int>(path_t.size()); ++i)
    {
      if (!std::isfinite(path_t[static_cast<std::size_t>(i)]) ||
          path_t[static_cast<std::size_t>(i)] <= path_t[static_cast<std::size_t>(i - 1)])
      {
        timed_path_valid = false;
        break;
      }
    }
  }

  int piece_num = requested_piece_num > 0
                      ? requested_piece_num
                      : static_cast<int>(path.size()) - 1;
  piece_num = std::clamp(piece_num, 1, 32);
  const double total_time =
      std::max({initial_guess.total_time,
                static_cast<double>(piece_num) * std::max(0.05, min_piece_duration),
                static_cast<double>(piece_num) * 0.05});

  times.assign(static_cast<std::size_t>(piece_num), total_time / static_cast<double>(piece_num));
  waypoints.resize(piece_num + 1, 3);
  waypoints.row(0) = head.col(0).transpose();
  waypoints.row(piece_num) = tail.col(0).transpose();

  if (timed_path_valid && piece_num == static_cast<int>(path.size()) - 1)
  {
    for (int i = 0; i <= piece_num; ++i)
    {
      waypoints.row(i) = path[static_cast<std::size_t>(i)].transpose();
      if (i > 0)
      {
        times[static_cast<std::size_t>(i - 1)] =
            std::max(0.05,
                     path_t[static_cast<std::size_t>(i)] -
                         path_t[static_cast<std::size_t>(i - 1)]);
      }
    }
    return true;
  }

  for (int i = 1; i < piece_num; ++i)
  {
    const double s = length * static_cast<double>(i) / static_cast<double>(piece_num);
    waypoints.row(i) = interpolateByArc(path, arc, s).transpose();
  }

  (void)cfg;
  return true;
}

using BvpCoeffMat = Eigen::Matrix<double, 3, 8>;

BvpCoeffMat solveSeventhOrderBvp(const TaskBoundaryState<4> &head,
                                 const TaskBoundaryState<4> &tail,
                                 const double T)
{
  const double t1 = T;
  const double t2 = t1 * t1;
  const double t3 = t2 * t1;
  const double t4 = t2 * t2;
  const double t5 = t3 * t2;
  const double t6 = t3 * t3;
  const double t7 = t4 * t3;

  BvpCoeffMat coeff;
  coeff.col(0) = (tail.col(3) / 6.0 + head.col(3) / 6.0) * t3 +
                 (-2.0 * tail.col(2) + 2.0 * head.col(2)) * t2 +
                 (10.0 * tail.col(1) + 10.0 * head.col(1)) * t1 +
                 (-20.0 * tail.col(0) + 20.0 * head.col(0));
  coeff.col(1) = (-0.5 * tail.col(3) - head.col(3) / 1.5) * t3 +
                 (6.5 * tail.col(2) - 7.5 * head.col(2)) * t2 +
                 (-34.0 * tail.col(1) - 36.0 * head.col(1)) * t1 +
                 (70.0 * tail.col(0) - 70.0 * head.col(0));
  coeff.col(2) = (0.5 * tail.col(3) + head.col(3)) * t3 +
                 (-7.0 * tail.col(2) + 10.0 * head.col(2)) * t2 +
                 (39.0 * tail.col(1) + 45.0 * head.col(1)) * t1 +
                 (-84.0 * tail.col(0) + 84.0 * head.col(0));
  coeff.col(3) = (-tail.col(3) / 6.0 - head.col(3) / 1.5) * t3 +
                 (2.5 * tail.col(2) - 5.0 * head.col(2)) * t2 +
                 (-15.0 * tail.col(1) - 20.0 * head.col(1)) * t1 +
                 (35.0 * tail.col(0) - 35.0 * head.col(0));
  coeff.col(4) = head.col(3) / 6.0;
  coeff.col(5) = head.col(2) / 2.0;
  coeff.col(6) = head.col(1);
  coeff.col(7) = head.col(0);

  coeff.col(0) /= t7;
  coeff.col(1) /= t6;
  coeff.col(2) /= t5;
  coeff.col(3) /= t4;
  return coeff;
}

double approximateMaxOmegaFromBvp(const Trajectory &traj,
                                  const double gravity,
                                  const double dt)
{
  if (traj.empty())
  {
    return std::numeric_limits<double>::infinity();
  }
  double max_omega = 0.0;
  const double duration = traj.getTotalDuration();
  const double sample_dt = std::max(0.005, dt);
  for (double t = 0.0; t <= duration + 1.0e-9; t += sample_dt)
  {
    const double eval_t = std::min(t, duration);
    const Eigen::Vector3d acc = traj.getAcc(eval_t);
    const Eigen::Vector3d jerk = traj.getJer(eval_t);
    const Eigen::Vector3d thrust = acc + Eigen::Vector3d(0.0, 0.0, std::abs(gravity));
    const double thrust_norm = thrust.norm();
    if (!std::isfinite(thrust_norm) || thrust_norm < 1.0e-6)
    {
      return std::numeric_limits<double>::infinity();
    }
    const Eigen::Vector3d zb = thrust / thrust_norm;
    const Eigen::Matrix3d d_norm =
        (Eigen::Matrix3d::Identity() - zb * zb.transpose()) / thrust_norm;
    const double omega12 = (d_norm * jerk).norm();
    if (!std::isfinite(omega12))
    {
      return std::numeric_limits<double>::infinity();
    }
    max_omega = std::max(max_omega, omega12);
  }
  return max_omega;
}

Trajectory makeBvpTrajectory(const BvpCoeffMat &coeff,
                             const double duration)
{
  Trajectory traj;
  traj.reserve(1);
  traj.emplace_back(duration, Eigen::MatrixXd(coeff));
  return traj;
}

template <int S>
bool prepareBvpInitialState(const traj_opt::Config &cfg,
                            const StatePVAJ &head,
                            const StatePVAJ &nominal_tail,
                            const minco::BoundaryStateMappingBase<3, S> &boundary_mapping,
                            const PerchingInitialGuess &initial_guess,
                            int requested_piece_num,
                            double min_piece_duration,
                            double max_total_duration,
                            std::vector<double> &times,
                            typename TaskOptimizer<S>::WaypointsType &waypoints,
                            Eigen::VectorXd &extra_vars,
                            const std::string &log_context = "PerchingSnapTrajOpt",
                            const std::string &log_name = "PERCHING",
                            bool use_perching_extra_seed = true)
{
  static_assert(S == 4, "BVP perching initial guess is implemented for T4 MINCO.");
  if (!boundary_mapping.enabled())
  {
    return false;
  }

  int piece_num = requested_piece_num > 0 ? requested_piece_num : cfg.piece_num;
  if (piece_num <= 0)
  {
    piece_num = 3;
  }
  piece_num = std::clamp(piece_num, 1, 32);
  const bool has_time_cap =
      std::isfinite(max_total_duration) && max_total_duration > 0.0;
  const double min_total_by_piece =
      static_cast<double>(piece_num) * std::max(0.05, min_piece_duration);
  if (has_time_cap && min_total_by_piece > max_total_duration + 1.0e-6)
  {
    std::cout << " -- [" << log_context << "] " << log_name
              << "_BVP_INITIAL_GUESS_FAILED reason=time_bound_infeasible"
              << " min_total=" << min_total_by_piece
              << ", max_total_duration=" << max_total_duration
              << ", pieces=" << piece_num << std::endl;
    return false;
  }

  const int extra_dim = boundary_mapping.extraVariableDim();
  extra_vars.resize(extra_dim);
  if (extra_dim > 0)
  {
    boundary_mapping.setInitialExtraVariables(extra_vars);
    if (use_perching_extra_seed && initial_guess.valid && extra_dim >= 3)
    {
      extra_vars(0) = initial_guess.nu.x();
      extra_vars(1) = initial_guess.nu.y();
      extra_vars(2) = initial_guess.tau_f;
    }
  }

  const double distance = (nominal_tail.col(0) - head.col(0)).norm();
  const double max_vel = clampPositive(cfg.max_vel, 2.0);
  double bvp_T =
      initial_guess.valid && std::isfinite(initial_guess.total_time) && initial_guess.total_time > 0.0
          ? initial_guess.total_time
          : std::max(0.5, distance / max_vel);
  bvp_T = std::max(bvp_T, static_cast<double>(piece_num) * std::max(0.05, min_piece_duration));
  if (has_time_cap)
  {
    bvp_T = std::min(bvp_T, max_total_duration);
  }

  const TaskBoundaryState<4> head_state = toBoundaryState<4>(head);
  const TaskBoundaryState<4> nominal_tail_state = toBoundaryState<4>(nominal_tail);
  const double omega_limit =
      cfg.max_omg > 0.0 ? 1.5 * cfg.max_omg : std::numeric_limits<double>::infinity();
  const double max_bvp_T =
      has_time_cap ? max_total_duration : std::max(8.0, 3.0 * bvp_T);

  BvpCoeffMat best_coeff = BvpCoeffMat::Zero();
  TaskBoundaryState<4> best_head = head_state;
  TaskBoundaryState<4> best_tail = nominal_tail_state;
  double best_T = bvp_T;
  double best_omega = std::numeric_limits<double>::infinity();

  for (int iter = 0; iter < 12; ++iter)
  {
    Eigen::VectorXd cache_T(piece_num);
    cache_T.setConstant(bvp_T / static_cast<double>(piece_num));
    TaskBoundaryState<4> mapped_head = head_state;
    TaskBoundaryState<4> mapped_tail = nominal_tail_state;
    boundary_mapping.mapBoundaryStates(head_state,
                                       nominal_tail_state,
                                       cache_T,
                                       extra_vars,
                                       mapped_head,
                                       mapped_tail);
    const BvpCoeffMat coeff = solveSeventhOrderBvp(mapped_head, mapped_tail, bvp_T);
    const Trajectory bvp_traj = makeBvpTrajectory(coeff, bvp_T);
    const double max_omega =
        approximateMaxOmegaFromBvp(bvp_traj, cfg.grav, std::clamp(bvp_T / 100.0, 0.01, 0.05));
    if (max_omega < best_omega)
    {
      best_omega = max_omega;
      best_T = bvp_T;
      best_coeff = coeff;
      best_head = mapped_head;
      best_tail = mapped_tail;
    }
    if (max_omega <= omega_limit)
    {
      break;
    }
    const double next_T =
        std::min(max_bvp_T, bvp_T + std::max(0.25, 0.25 * bvp_T));
    if (next_T <= bvp_T + 1.0e-6)
    {
      break;
    }
    bvp_T = next_T;
    if (bvp_T >= max_bvp_T - 1.0e-6)
    {
      break;
    }
  }

  const Trajectory bvp_traj = makeBvpTrajectory(best_coeff, best_T);
  if (bvp_traj.empty() || !std::isfinite(best_omega))
  {
    return false;
  }

  times.assign(static_cast<std::size_t>(piece_num), best_T / static_cast<double>(piece_num));
  waypoints.resize(piece_num + 1, 3);
  for (int i = 0; i <= piece_num; ++i)
  {
    const double t = best_T * static_cast<double>(i) / static_cast<double>(piece_num);
    waypoints.row(i) = bvp_traj.getPos(t).transpose();
  }
  waypoints.row(0) = best_head.col(0).transpose();
  waypoints.row(piece_num) = best_tail.col(0).transpose();

  std::cout << " -- [" << log_context << "] " << log_name
            << "_BVP_INITIAL_GUESS T="
            << best_T << ", max_omega=" << best_omega
            << ", omega_limit=" << omega_limit
            << ", pieces=" << piece_num << std::endl;
  if (has_time_cap && best_omega > omega_limit &&
      best_T >= max_total_duration - 1.0e-6)
  {
    std::cout << " -- [" << log_context << "] " << log_name
              << "_BVP_INITIAL_GUESS_TIME_LIMIT_REACHED"
              << " T=" << best_T
              << ", max_total_duration=" << max_total_duration
              << ", max_omega=" << best_omega
              << ", omega_limit=" << omega_limit << std::endl;
  }
  return waypoints.allFinite();
}

template <int S, typename CostManager>
class TaskRunner
{
public:
  TaskRunner(const traj_opt::Config &cfg,
             const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
      : cfg_(cfg),
        ros_ptr_(ros_ptr)
  {
    time_cost_.linear_weight = cfg_.penna_t;
    time_cost_.smooth_eps = cfg_.smooth_eps;
    optimizer_.setTimeMap(&time_map_);
    optimizer_.setSpatialMap(&spatial_map_);
    optimizer_.setEnergyWeight(cfg_.block_energy_cost ? 0.0 : 1.0);
    optimizer_.setSamplesPerPiece(std::max(1, cfg_.integral_reso));
  }

  void setMapManager(const general_planner::MapManager::Ptr &map_manager)
  {
    map_manager_ = map_manager;
  }

  void setSafeDistance(double safe_distance)
  {
    safe_distance_ = safe_distance;
  }

protected:
  bool run(const StatePVAJ &head,
           const StatePVAJ &tail,
           const general_utils::vec_E<Vec3f> &guide_path,
           int piece_num,
           double min_piece_duration,
           double min_total_duration,
           double time_lower_bound_weight,
           double max_total_duration,
           double time_upper_bound_weight,
           double duration_seed,
           double duration_seed_weight,
           CostManager &cost_manager,
           Trajectory &out_traj,
           const minco::BoundaryStateMappingBase<3, S> *boundary_mapping = nullptr,
           const std::vector<double> *initial_times = nullptr,
           const typename TaskOptimizer<S>::WaypointsType *initial_waypoints = nullptr,
           const Eigen::VectorXd *initial_extra_vars = nullptr)
  {
    std::vector<double> times;
    typename TaskOptimizer<S>::WaypointsType waypoints;
    const bool use_explicit_initial_state =
        initial_times != nullptr &&
        initial_waypoints != nullptr &&
        static_cast<int>(initial_times->size()) > 0 &&
        initial_waypoints->rows() == static_cast<int>(initial_times->size()) + 1 &&
        initial_waypoints->cols() == 3;
    if (use_explicit_initial_state)
    {
      times = *initial_times;
      waypoints = *initial_waypoints;
      for (const double t : times)
      {
        if (!std::isfinite(t) || t <= 0.0)
        {
          return false;
        }
      }
      if (!waypoints.allFinite())
      {
        return false;
      }
    }
    else if (!prepareInitialState<S>(cfg_,
                                     head,
                                     tail,
                                     guide_path,
                                     piece_num,
                                     min_piece_duration,
                                     times,
                                     waypoints))
    {
      return false;
    }

    time_map_.setUpperBound(-1.0);
    if (max_total_duration > 0.0)
    {
      const int active_piece_num = static_cast<int>(times.size());
      if (active_piece_num <= 0)
      {
        return false;
      }

      const double min_required_duration =
          std::max(min_total_duration,
                   static_cast<double>(active_piece_num) *
                       std::max(0.0, min_piece_duration));
      if (min_required_duration > max_total_duration + 1.0e-6)
      {
        std::cout << " -- [TaskTrajOpt] time bound infeasible: min_required="
                  << min_required_duration
                  << ", max_total_duration=" << max_total_duration
                  << ", pieces=" << active_piece_num << std::endl;
        return false;
      }

      const double per_piece_upper =
          max_total_duration / static_cast<double>(active_piece_num);
      if (!std::isfinite(per_piece_upper) ||
          per_piece_upper <= TaskTimeMap::kMinTime)
      {
        return false;
      }
      time_map_.setUpperBound(per_piece_upper);
      const double init_upper =
          std::max(TaskTimeMap::kMinTime,
                   per_piece_upper * (1.0 - 1.0e-6));
      for (double &t : times)
      {
        t = std::clamp(t, TaskTimeMap::kMinTime, init_upper);
      }
    }

    if (!optimizer_.setInitState(times,
                                 waypoints,
                                 toBoundaryState<S>(head),
                                 toBoundaryState<S>(tail)))
    {
      return false;
    }

    active_cost_manager_ = &cost_manager;
    active_boundary_mapping_ = boundary_mapping;
    time_cost_.min_piece_duration = min_piece_duration;
    time_cost_.min_total_duration = min_total_duration;
    time_cost_.max_total_duration = max_total_duration;
    time_cost_.lower_bound_weight =
        time_lower_bound_weight > 0.0
            ? time_lower_bound_weight
            : std::max(100.0, std::abs(cfg_.penna_t) * 10.0);
    time_cost_.upper_bound_weight = std::max(0.0, time_upper_bound_weight);
    time_cost_.duration_seed = duration_seed;
    time_cost_.duration_seed_weight = std::max(0.0, duration_seed_weight);
    Eigen::VectorXd x =
        initial_extra_vars != nullptr
            ? optimizer_.encodeDecisionVector(times,
                                              waypoints,
                                              active_boundary_mapping_,
                                              initial_extra_vars)
            : optimizer_.generateInitialGuess(active_boundary_mapping_);
    if (x.size() == 0 || !x.allFinite())
    {
      return false;
    }

    iter_num_ = 0;
    double min_cost = 0.0;
    math_utils::lbfgs::lbfgs_parameter_t params;
    params.mem_size = 64;
    params.past = 3;
    params.min_step = 1.0e-32;
    params.g_epsilon = 0.0;
    params.delta = std::max(1.0e-8, cfg_.opt_accuracy);
    params.max_iterations = 100;
    params.max_linesearch = 32;
    const int ret =
        math_utils::lbfgs::lbfgs_optimize(x, min_cost, &TaskRunner::costFunctional, nullptr, nullptr, this, params);
    const bool recoverable =
        ret == math_utils::lbfgs::LBFGSERR_MAXIMUMITERATION ||
        ret == math_utils::lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
        ret == math_utils::lbfgs::LBFGSERR_MINIMUMSTEP ||
        ret == math_utils::lbfgs::LBFGSERR_WIDTHTOOSMALL;
    if (ret < 0 && !recoverable)
    {
      std::cout << " -- [TaskTrajOpt] optimization failed: " << math_utils::lbfgs::lbfgs_strerror(ret) << std::endl;
      return false;
    }

    Eigen::VectorXd grad = Eigen::VectorXd::Zero(x.size());
    min_cost = evaluate(x, grad);
    if (!std::isfinite(min_cost) || !grad.allFinite())
    {
      return false;
    }

    out_traj = toGeometryTrajectory<S>(optimizer_.getTrajectory());
    out_traj.start_WT = ros_ptr_ ? ros_ptr_->getSimTime() : 0.0;
    if (max_total_duration > 0.0 &&
        out_traj.getTotalDuration() > max_total_duration + 1.0e-6)
    {
      std::cout << " -- [TaskTrajOpt] optimization rejected by hard duration bound: duration="
                << out_traj.getTotalDuration()
                << ", max_total_duration=" << max_total_duration
                << std::endl;
      active_cost_manager_ = nullptr;
      active_boundary_mapping_ = nullptr;
      return false;
    }
    optimizer_.setWarmStartGuess(x);
    active_cost_manager_ = nullptr;
    active_boundary_mapping_ = nullptr;
    return !out_traj.empty();
  }

  const general_planner::MapManager::Ptr &mapManager() const
  {
    return map_manager_;
  }

  double safeDistance() const
  {
    return safe_distance_;
  }

  traj_opt::Config &mutableConfig()
  {
    return cfg_;
  }

private:
  static double costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g)
  {
    auto *runner = reinterpret_cast<TaskRunner *>(ptr);
    return runner->evaluate(x, g);
  }

  double evaluate(const Eigen::VectorXd &x, Eigen::VectorXd &g)
  {
    ++iter_num_;
    if (active_cost_manager_ == nullptr)
    {
      g.setZero();
      return std::numeric_limits<double>::infinity();
    }
    if (active_boundary_mapping_ != nullptr)
    {
      return optimizer_.evaluateWithBoundaryMapping(x,
                                                    g,
                                                    time_cost_,
                                                    *active_cost_manager_,
                                                    active_boundary_mapping_);
    }
    return optimizer_.evaluate(x, g, time_cost_, *active_cost_manager_);
  }

private:
  traj_opt::Config cfg_;
  std::shared_ptr<ros_interface::RosInterface> ros_ptr_;
  general_planner::MapManager::Ptr map_manager_;
  double safe_distance_{0.45};
  int iter_num_{0};

  TaskTimeMap time_map_;
  R3IdentitySpatialMap spatial_map_;
  TaskTimeCost time_cost_;
  TaskOptimizer<S> optimizer_;
  CostManager *active_cost_manager_{nullptr};
  const minco::BoundaryStateMappingBase<3, S> *active_boundary_mapping_{nullptr};
};

template <int S>
class TrackingRunner : public TaskRunner<S, cost_functional_manager::TrackingCostManager>
{
public:
  using Base = TaskRunner<S, cost_functional_manager::TrackingCostManager>;

  TrackingRunner(const traj_opt::Config &cfg,
                 const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
      : Base(cfg, ros_ptr)
  {
  }

  bool optimize(TrackingProblem problem, Trajectory &out_traj)
  {
    if (problem.safe_distance <= 0.0)
    {
      problem.safe_distance = Base::safeDistance();
    }
    if (problem.tail_pvaj.col(0).squaredNorm() < 1.0e-12 && !problem.guide_path.empty())
    {
      problem.tail_pvaj.col(0) = problem.guide_path.back();
    }
    cost_manager_.reset(Base::mutableConfig(),
                        Base::mapManager(),
                        problem,
                        &Base::mutableConfig().quadrotot_flatness);
    return Base::run(problem.head_pvaj,
                     problem.tail_pvaj,
                     problem.guide_path,
                     problem.piece_num,
                     problem.min_piece_duration,
                     problem.min_total_duration,
                     problem.time_lower_bound_weight,
                     -1.0,
                     0.0,
                     0.0,
                     0.0,
                     cost_manager_,
                     out_traj);
  }

private:
  cost_functional_manager::TrackingCostManager cost_manager_;
};

template <int SPos>
class JointTrackingRunner
{
public:
  using PosTraj = minco::MINCOTrajectory<3, SPos>;
  using YawTraj = minco::MINCOTrajectory<1, 2>;
  using PosBoundaryState = typename PosTraj::BoundaryState;
  using YawBoundaryState = typename YawTraj::BoundaryState;
  using PosInnerMat = typename PosTraj::InnerPointsMat;
  using YawInnerMat = typename YawTraj::InnerPointsMat;
  using PosCoeffMat = typename PosTraj::CoeffMat;
  using YawCoeffMat = typename YawTraj::CoeffMat;

  JointTrackingRunner(const traj_opt::Config &cfg,
                      const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
      : cfg_(cfg),
        ros_ptr_(ros_ptr)
  {
    time_cost_.linear_weight = cfg_.penna_t;
    time_cost_.smooth_eps = cfg_.smooth_eps;
    samples_per_piece_ = std::max(1, cfg_.integral_reso);
    pos_energy_weight_ = cfg_.block_energy_cost ? 0.0 : 1.0;
    yaw_energy_weight_ = 0.05;
  }

  void setMapManager(const general_planner::MapManager::Ptr &map_manager)
  {
    map_manager_ = map_manager;
  }

  void setSafeDistance(double safe_distance)
  {
    safe_distance_ = safe_distance;
  }

  bool optimize(TrackingProblem problem,
                Trajectory &out_traj,
                Trajectory *out_yaw_traj)
  {
    problem_ = std::move(problem);
    if (problem_.safe_distance <= 0.0)
    {
      problem_.safe_distance = safe_distance_;
    }
    if (problem_.tail_pvaj.col(0).squaredNorm() < 1.0e-12 && !problem_.guide_path.empty())
    {
      problem_.tail_pvaj.col(0) = problem_.guide_path.back();
    }
    if (problem_.viewpoints.empty() && !problem_.guide_path.empty())
    {
      problem_.viewpoints = problem_.guide_path;
      problem_.target_sample_times = problem_.guide_t;
    }

    use_corridor_ = problem_.use_corridor && !problem_.sfcs.empty();
    if (use_corridor_)
    {
      if (!setupCorridorInitialState())
      {
        return false;
      }
    }
    else if (!prepareInitialState<SPos>(cfg_,
                                        problem_.head_pvaj,
                                        problem_.tail_pvaj,
                                        problem_.guide_path,
                                        problem_.piece_num,
                                        problem_.min_piece_duration,
                                        init_times_,
                                        init_pos_waypoints_))
    {
      return false;
    }

    piece_num_ = static_cast<int>(init_times_.size());
    init_yaw_inner_.resize(1, std::max(0, piece_num_ - 1));
    setupBoundaryStatesAndYawGuess();

    Eigen::VectorXd x = makeInitialGuess();
    if (x.size() == 0 || !x.allFinite())
    {
      return false;
    }

    cost_manager_.reset(cfg_,
                        map_manager_,
                        problem_,
                        &cfg_.quadrotot_flatness);
    time_cost_.min_piece_duration = problem_.min_piece_duration;
    time_cost_.min_total_duration = problem_.min_total_duration;
    time_cost_.lower_bound_weight =
        problem_.time_lower_bound_weight > 0.0
            ? problem_.time_lower_bound_weight
            : std::max(100.0, std::abs(cfg_.penna_t) * 10.0);

    double min_cost = 0.0;
    math_utils::lbfgs::lbfgs_parameter_t params;
    params.mem_size = 64;
    params.past = 3;
    params.min_step = 1.0e-32;
    params.g_epsilon = 0.0;
    params.delta = std::max(1.0e-8, cfg_.opt_accuracy);
    params.max_iterations = 120;
    params.max_linesearch = 32;
    const int ret =
        math_utils::lbfgs::lbfgs_optimize(x, min_cost, &JointTrackingRunner::costFunctional, nullptr, nullptr, this, params);
    const bool recoverable =
        ret == math_utils::lbfgs::LBFGSERR_MAXIMUMITERATION ||
        ret == math_utils::lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
        ret == math_utils::lbfgs::LBFGSERR_MINIMUMSTEP ||
        ret == math_utils::lbfgs::LBFGSERR_WIDTHTOOSMALL;
    if (ret < 0 && !recoverable)
    {
      std::cout << " -- [TrackingJointTrajOpt] optimization failed: "
                << math_utils::lbfgs::lbfgs_strerror(ret) << std::endl;
      return false;
    }

    Eigen::VectorXd grad = Eigen::VectorXd::Zero(x.size());
    min_cost = evaluate(x, grad);
    if (!std::isfinite(min_cost) || !grad.allFinite())
    {
      return false;
    }

    out_traj = toGeometryTrajectoryGeneric(pos_traj_);
    if (use_corridor_ && !validateTrajectoryInCorridor(out_traj))
    {
      std::cout << " -- [TrackingJointTrajOpt] optimized trajectory violates tracking SFC." << std::endl;
      return false;
    }
    out_traj.start_WT = ros_ptr_ ? ros_ptr_->getSimTime() : 0.0;
    if (out_yaw_traj != nullptr)
    {
      *out_yaw_traj = toGeometryTrajectoryGeneric(yaw_traj_);
      out_yaw_traj->start_WT = out_traj.start_WT;
    }
    warm_start_ = x;
    return !out_traj.empty();
  }

private:
  static double costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g)
  {
    auto *runner = reinterpret_cast<JointTrackingRunner *>(ptr);
    return runner->evaluate(x, g);
  }

  int decisionDim() const
  {
    return piece_num_ + posDecisionDim() + std::max(0, piece_num_ - 1);
  }

  int posOffset() const
  {
    return piece_num_;
  }

  int yawOffset() const
  {
    return piece_num_ + posDecisionDim();
  }

  int posDecisionDim() const
  {
    int dim = 0;
    for (int i = 1; i < piece_num_; ++i)
    {
      dim += positionDof(i);
    }
    return dim;
  }

  int positionDof(int inner_index) const
  {
    return use_corridor_ ? corridor_spatial_map_.getUnconstrainedDim(inner_index) : 3;
  }

  Vec3f toPhysicalPosition(const Eigen::Ref<const Eigen::VectorXd> &xi,
                           int inner_index) const
  {
    return use_corridor_ ? corridor_spatial_map_.toPhysical(xi, inner_index) : xi.head<3>();
  }

  Eigen::VectorXd toUnconstrainedPosition(const Vec3f &p,
                                          int inner_index) const
  {
    if (use_corridor_)
    {
      return corridor_spatial_map_.toUnconstrained(p, inner_index);
    }
    Eigen::VectorXd xi(3);
    xi = p;
    return xi;
  }

  Eigen::VectorXd backwardPositionGrad(const Eigen::Ref<const Eigen::VectorXd> &xi,
                                       const Vec3f &grad_p,
                                       int inner_index) const
  {
    if (use_corridor_)
    {
      return corridor_spatial_map_.backwardGrad(xi, grad_p, inner_index);
    }
    Eigen::VectorXd grad_xi(3);
    grad_xi = grad_p;
    return grad_xi;
  }

  bool setupCorridorInitialState()
  {
    if (problem_.sfcs.empty())
    {
      return false;
    }

    h_polytopes_.clear();
    h_polytopes_.reserve(problem_.sfcs.size());
    for (const auto &sfc : problem_.sfcs)
    {
      spatial_map::PolyhedronH h_poly = sfc.GetPlanes();
      normalizeTrackingHPoly(h_poly);
      if (h_poly.rows() == 0 || !std::isfinite(h_poly.sum()))
      {
        return false;
      }
      h_polytopes_.push_back(h_poly);
    }

    piece_num_ = static_cast<int>(h_polytopes_.size());
    if (piece_num_ <= 0)
    {
      return false;
    }

    init_times_.assign(static_cast<std::size_t>(piece_num_), std::max(0.05, problem_.min_piece_duration));
    init_pos_waypoints_.resize(piece_num_ + 1, 3);
    init_pos_waypoints_.row(0) = problem_.head_pvaj.col(0).transpose();
    init_pos_waypoints_.row(piece_num_) = problem_.tail_pvaj.col(0).transpose();

    v_polytopes_.clear();
    v_polytopes_.reserve(std::max(1, 2 * (piece_num_ - 1) + 1));
    h_poly_idx_.resize(piece_num_);
    v_poly_idx_.resize(std::max(0, piece_num_ - 1));

    spatial_map::PolyhedronV cur_v;
    spatial_map::PolyhedronV cur_v_local;
    auto pushLocalVPoly = [&](const spatial_map::PolyhedronV &v_poly) {
      if (v_poly.cols() <= 0 || !std::isfinite(v_poly.sum()))
      {
        return false;
      }
      cur_v_local.resize(3, v_poly.cols());
      cur_v_local.col(0) = v_poly.col(0);
      if (v_poly.cols() > 1)
      {
        cur_v_local.rightCols(v_poly.cols() - 1) =
            v_poly.rightCols(v_poly.cols() - 1).colwise() - v_poly.col(0);
      }
      v_polytopes_.push_back(cur_v_local);
      return true;
    };

    std::vector<double> time_stamps(static_cast<std::size_t>(piece_num_ + 1), 0.0);
    time_stamps.front() = 0.0;
    time_stamps.back() = std::max(problem_.min_total_duration,
                                  problem_.guide_t.empty() ? 0.0 : problem_.guide_t.back());
    if (time_stamps.back() <= 0.0)
    {
      time_stamps.back() = static_cast<double>(piece_num_) * std::max(0.1, problem_.min_piece_duration);
    }

    for (int i = 0; i < piece_num_ - 1; ++i)
    {
      h_poly_idx_(i) = i;
      if (!geometry_utils::enumerateVs(h_polytopes_[static_cast<std::size_t>(i)], cur_v) ||
          !pushLocalVPoly(cur_v))
      {
        return false;
      }

      spatial_map::PolyhedronH overlap(h_polytopes_[static_cast<std::size_t>(i)].rows() +
                                           h_polytopes_[static_cast<std::size_t>(i + 1)].rows(),
                                       4);
      overlap.topRows(h_polytopes_[static_cast<std::size_t>(i)].rows()) =
          h_polytopes_[static_cast<std::size_t>(i)];
      overlap.bottomRows(h_polytopes_[static_cast<std::size_t>(i + 1)].rows()) =
          h_polytopes_[static_cast<std::size_t>(i + 1)];

      Vec3f interior = Vec3f::Zero();
      const double interior_depth = geometry_utils::findInteriorDist(overlap, interior);
      if (!std::isfinite(interior_depth) || interior_depth <= 1.0e-4)
      {
        return false;
      }
      geometry_utils::enumerateVs(overlap, interior, cur_v);
      if (!pushLocalVPoly(cur_v))
      {
        return false;
      }
      v_poly_idx_(i) = 2 * i + 1;
      init_pos_waypoints_.row(i + 1) = interior.transpose();

      double stamp = time_stamps.back() * static_cast<double>(i + 1) / static_cast<double>(piece_num_);
      if (problem_.guide_path.size() == problem_.guide_t.size() && !problem_.guide_path.empty())
      {
        double min_dist = std::numeric_limits<double>::max();
        for (int guide_id = 0; guide_id < static_cast<int>(problem_.guide_path.size()); ++guide_id)
        {
          const double dist = (problem_.guide_path[static_cast<std::size_t>(guide_id)] - interior).norm();
          if (dist < min_dist)
          {
            min_dist = dist;
            stamp = problem_.guide_t[static_cast<std::size_t>(guide_id)];
          }
        }
      }
      time_stamps[static_cast<std::size_t>(i + 1)] = std::clamp(stamp, time_stamps.front(), time_stamps.back());
    }
    h_poly_idx_(piece_num_ - 1) = piece_num_ - 1;
    if (!geometry_utils::enumerateVs(h_polytopes_.back(), cur_v) ||
        !pushLocalVPoly(cur_v))
    {
      return false;
    }

    std::sort(time_stamps.begin(), time_stamps.end());
    time_stamps.front() = 0.0;
    time_stamps.back() = std::max(time_stamps.back(),
                                  static_cast<double>(piece_num_) * std::max(0.1, problem_.min_piece_duration));
    for (int i = 0; i < piece_num_; ++i)
    {
      init_times_[static_cast<std::size_t>(i)] =
          std::max(std::max(0.05, problem_.min_piece_duration),
                   time_stamps[static_cast<std::size_t>(i + 1)] -
                       time_stamps[static_cast<std::size_t>(i)]);
    }

    corridor_spatial_map_.reset(&v_polytopes_, &v_poly_idx_, piece_num_ - 1, false);
    return true;
  }

  bool validateTrajectoryInCorridor(const Trajectory &traj) const
  {
    if (!use_corridor_ || h_polytopes_.empty() || traj.empty())
    {
      return true;
    }
    const int piece_num = std::min(traj.getPieceNum(), static_cast<int>(h_polytopes_.size()));
    const int sample_num = std::max(4, samples_per_piece_);
    for (int i = 0; i < piece_num; ++i)
    {
      const double T = traj[i].getDuration();
      for (int k = 0; k <= sample_num; ++k)
      {
        const double t = T * static_cast<double>(k) / static_cast<double>(sample_num);
        if (!geometry_utils::pointInsidePolytope(traj[i].getPos(t),
                                                 h_polytopes_[static_cast<std::size_t>(i)],
                                                 0.02))
        {
          return false;
        }
      }
    }
    return true;
  }

  Eigen::VectorXd makeInitialGuess() const
  {
    const int dim = decisionDim();
    if (warm_start_.size() == dim && warm_start_.allFinite())
    {
      return warm_start_;
    }

    Eigen::VectorXd x(dim);
    for (int i = 0; i < piece_num_; ++i)
    {
      x(i) = time_map_.toTau(init_times_[static_cast<std::size_t>(i)]);
    }

    int offset = posOffset();
    for (int i = 1; i < piece_num_; ++i)
    {
      const Eigen::VectorXd xi = toUnconstrainedPosition(init_pos_waypoints_.row(i).transpose(), i);
      x.segment(offset, xi.size()) = xi;
      offset += xi.size();
    }
    offset = yawOffset();
    for (int i = 1; i < piece_num_; ++i)
    {
      x(offset++) = init_yaw_inner_(0, i - 1);
    }
    return x;
  }

  void decodeDecision(const Eigen::Ref<const Eigen::VectorXd> &x,
                      Eigen::VectorXd &durations,
                      PosInnerMat &pos_inner,
                      YawInnerMat &yaw_inner,
                      double &cost,
                      Eigen::VectorXd &grad) const
  {
    durations.resize(piece_num_);
    for (int i = 0; i < piece_num_; ++i)
    {
      durations(i) = time_map_.toTime(x(i));
    }

    pos_inner.resize(3, std::max(0, piece_num_ - 1));
    yaw_inner.resize(1, std::max(0, piece_num_ - 1));
    int offset = posOffset();
    for (int i = 1; i < piece_num_; ++i)
    {
      const int dof = positionDof(i);
      const Eigen::VectorXd xi = x.segment(offset, dof);
      pos_inner.col(i - 1) = toPhysicalPosition(xi, i);
      if (use_corridor_)
      {
        Eigen::VectorXd grad_xi = Eigen::VectorXd::Zero(dof);
        corridor_spatial_map_.addNormPenalty(xi, cost, grad_xi);
        grad.segment(offset, dof) += grad_xi;
      }
      offset += dof;
    }
    offset = yawOffset();
    for (int i = 1; i < piece_num_; ++i)
    {
      yaw_inner(0, i - 1) = x(offset++);
    }
  }

  traj_opt::DynamicTargetState targetAt(double t) const
  {
    if (problem_.target_prediction.empty())
    {
      return {};
    }
    if (problem_.target_prediction.size() == 1 || t <= problem_.target_prediction.front().t)
    {
      return problem_.target_prediction.front();
    }
    if (t >= problem_.target_prediction.back().t)
    {
      return problem_.target_prediction.back();
    }

    const auto it = std::lower_bound(problem_.target_prediction.begin(),
                                     problem_.target_prediction.end(),
                                     t,
                                     [](const traj_opt::DynamicTargetState &state, double query_t) {
                                       return state.t < query_t;
                                     });
    const int idx = static_cast<int>(std::distance(problem_.target_prediction.begin(), it));
    const auto &left = problem_.target_prediction[static_cast<std::size_t>(idx - 1)];
    const auto &right = problem_.target_prediction[static_cast<std::size_t>(idx)];
    const double alpha = (t - left.t) / std::max(kTiny, right.t - left.t);

    traj_opt::DynamicTargetState out;
    out.t = t;
    out.position = left.position + alpha * (right.position - left.position);
    out.velocity = left.velocity + alpha * (right.velocity - left.velocity);
    out.acceleration = left.acceleration + alpha * (right.acceleration - left.acceleration);
    out.yaw = left.yaw + alpha * (right.yaw - left.yaw);
    out.yaw_rate = left.yaw_rate + alpha * (right.yaw_rate - left.yaw_rate);
    return out;
  }

  double faceYaw(const Vec3f &position, const Vec3f &target, double last_yaw) const
  {
    const Vec3f dir = target - position;
    double yaw = last_yaw;
    if (dir.head<2>().norm() > 1.0e-4)
    {
      yaw = std::atan2(dir.y(), dir.x());
      geometry_utils::normalizeNextYaw(last_yaw, yaw);
    }
    return yaw;
  }

  void setupBoundaryStatesAndYawGuess()
  {
    pos_head_state_ = toBoundaryState<SPos>(problem_.head_pvaj);
    pos_tail_state_ = toBoundaryState<SPos>(problem_.tail_pvaj);

    yaw_head_state_.setZero();
    yaw_tail_state_.setZero();
    yaw_head_state_(0, 0) = std::isfinite(problem_.head_yaw(0, 0))
                                ? problem_.head_yaw(0, 0)
                                : 0.0;
    yaw_head_state_(0, 1) = std::isfinite(problem_.head_yaw(0, 1))
                                ? problem_.head_yaw(0, 1)
                                : 0.0;

    std::vector<double> cumulative(piece_num_ + 1, 0.0);
    for (int i = 0; i < piece_num_; ++i)
    {
      cumulative[static_cast<std::size_t>(i + 1)] =
          cumulative[static_cast<std::size_t>(i)] + init_times_[static_cast<std::size_t>(i)];
    }

    double last_yaw = yaw_head_state_(0, 0);
    for (int i = 1; i < piece_num_; ++i)
    {
      const auto target = targetAt(cumulative[static_cast<std::size_t>(i)]);
      last_yaw = faceYaw(init_pos_waypoints_.row(i).transpose(), target.position, last_yaw);
      init_yaw_inner_(0, i - 1) = last_yaw;
    }

    const auto tail_target = targetAt(cumulative.back());
    double tail_yaw = faceYaw(problem_.tail_pvaj.col(0), tail_target.position, last_yaw);
    if (std::isfinite(problem_.tail_yaw(0, 0)) &&
        (std::abs(problem_.tail_yaw(0, 0)) > 1.0e-6 || problem_.tail_yaw(0, 1) != 0.0))
    {
      tail_yaw = problem_.tail_yaw(0, 0);
      geometry_utils::normalizeNextYaw(last_yaw, tail_yaw);
    }
    yaw_tail_state_(0, 0) = tail_yaw;
    yaw_tail_state_(0, 1) = std::isfinite(problem_.tail_yaw(0, 1)) ? problem_.tail_yaw(0, 1) : 0.0;
  }

  double evaluate(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::VectorXd &g)
  {
    g.setZero();
    if (x.size() != decisionDim())
    {
      return std::numeric_limits<double>::infinity();
    }

    Eigen::VectorXd durations;
    PosInnerMat pos_inner;
    YawInnerMat yaw_inner;
    double total_cost = 0.0;
    decodeDecision(x, durations, pos_inner, yaw_inner, total_cost, g);
    if ((durations.array() <= 0.0).any())
    {
      return std::numeric_limits<double>::infinity();
    }
    if (!pos_traj_.generate(pos_inner, pos_head_state_, pos_tail_state_, durations) ||
        !yaw_traj_.generate(yaw_inner, yaw_head_state_, yaw_tail_state_, durations))
    {
      return std::numeric_limits<double>::infinity();
    }

    PosCoeffMat gdC_pos = PosCoeffMat::Zero(PosTraj::COEFF_NUM * piece_num_, 3);
    YawCoeffMat gdC_yaw = YawCoeffMat::Zero(YawTraj::COEFF_NUM * piece_num_, 1);
    Eigen::VectorXd gdT_pos = Eigen::VectorXd::Zero(piece_num_);
    Eigen::VectorXd gdT_yaw = Eigen::VectorXd::Zero(piece_num_);

    if (pos_energy_weight_ > 0.0)
    {
      double energy = 0.0;
      PosCoeffMat energy_grad;
      Eigen::VectorXd time_grad;
      pos_traj_.getEnergyPartialGradByCoeffs(energy, energy_grad);
      pos_traj_.getEnergyPartialGradByTimes(time_grad);
      total_cost += pos_energy_weight_ * energy;
      gdC_pos += pos_energy_weight_ * energy_grad;
      gdT_pos += pos_energy_weight_ * time_grad;
    }
    if (yaw_energy_weight_ > 0.0)
    {
      double energy = 0.0;
      YawCoeffMat energy_grad;
      Eigen::VectorXd time_grad;
      yaw_traj_.getEnergyPartialGradByCoeffs(energy, energy_grad);
      yaw_traj_.getEnergyPartialGradByTimes(time_grad);
      total_cost += yaw_energy_weight_ * energy;
      gdC_yaw += yaw_energy_weight_ * energy_grad;
      gdT_yaw += yaw_energy_weight_ * time_grad;
    }

    std::vector<double> T_vec(durations.data(), durations.data() + durations.size());
    Eigen::VectorXd gdT_time = Eigen::VectorXd::Zero(piece_num_);
    total_cost += time_cost_(T_vec, gdT_time);
    gdT_pos += gdT_time;

    const auto &pos_coeffs = pos_traj_.getCoefficients();
    const auto &yaw_coeffs = yaw_traj_.getCoefficients();

    double seg_start_time = 0.0;
    for (int i = 0; i < piece_num_; ++i)
    {
      const double T = durations(i);
      const double inv_K = 1.0 / static_cast<double>(samples_per_piece_);
      const double dt = T * inv_K;
      const int pos_base = i * PosTraj::COEFF_NUM;
      const int yaw_base = i * YawTraj::COEFF_NUM;
      const auto pos_block = pos_coeffs.template block<PosTraj::COEFF_NUM, 3>(pos_base, 0);
      const auto yaw_block = yaw_coeffs.template block<YawTraj::COEFF_NUM, 1>(yaw_base, 0);

      for (int k = 0; k <= samples_per_piece_; ++k)
      {
        const double alpha = static_cast<double>(k) * inv_K;
        const double t_local = alpha * T;
        const double t_global = seg_start_time + t_local;
        const double trap_weight = (k == 0 || k == samples_per_piece_) ? 0.5 : 1.0;
        const double common_weight = trap_weight * dt;

        typename PosTraj::BasisRow bp, bv, ba, bj, bs;
        PosTraj::computeBasisFunctions(t_local, bp, bv, ba, bj, bs);
        Vec3f p = Vec3f::Zero();
        Vec3f v = Vec3f::Zero();
        Vec3f a = Vec3f::Zero();
        Vec3f j = Vec3f::Zero();
        Vec3f s = Vec3f::Zero();
        p.transpose().noalias() = bp * pos_block;
        v.transpose().noalias() = bv * pos_block;
        a.transpose().noalias() = ba * pos_block;
        j.transpose().noalias() = bj * pos_block;
        s.transpose().noalias() = bs * pos_block;

        Vec3f gp_integral = Vec3f::Zero();
        Vec3f gv_integral = Vec3f::Zero();
        Vec3f ga_integral = Vec3f::Zero();
        Vec3f gj_integral = Vec3f::Zero();
        double gt_integral = 0.0;
        double c_corridor = 0.0;
        if (use_corridor_ && i < static_cast<int>(h_polytopes_.size()) && cfg_.penna_pos > 0.0)
        {
          c_corridor = cost_functional::accumulatePolytopePositionPenalty(
              h_polytopes_[static_cast<std::size_t>(i)],
              p,
              cfg_.smooth_eps,
              cfg_.penna_pos,
              gp_integral);
        }
        const double c_continuous = cost_manager_.evaluateIntegral(i * samples_per_piece_ + k,
                                                                   t_local,
                                                                   t_global,
                                                                   i,
                                                                   k,
                                                                   p,
                                                                   v,
                                                                   a,
                                                                   j,
                                                                   gp_integral,
                                                                   gv_integral,
                                                                   ga_integral,
                                                                   gj_integral,
                                                                   gt_integral);
        const double c_integral = c_corridor + c_continuous;
        total_cost += c_integral * common_weight;
        gdC_pos.template block<PosTraj::COEFF_NUM, 3>(pos_base, 0).noalias() +=
            (bp.transpose() * gp_integral.transpose() +
             bv.transpose() * gv_integral.transpose() +
             ba.transpose() * ga_integral.transpose() +
             bj.transpose() * gj_integral.transpose()) *
            common_weight;
        gdT_pos(i) += c_integral * trap_weight * inv_K;
        gdT_pos(i) += (gp_integral.dot(v) + gv_integral.dot(a) +
                       ga_integral.dot(j) + gj_integral.dot(s)) *
                      alpha * common_weight;
        gdT_pos(i) += gt_integral * alpha * common_weight;
      }

      if (!problem_.dense_joint_sample_enable &&
          problem_.weight_fov > 0.0 &&
          problem_.joint_sample_dt > 0.0)
      {
        const double fov_sample_dt = std::max(0.01, problem_.joint_sample_dt);
        const int fov_samples =
            std::max(samples_per_piece_,
                     static_cast<int>(std::ceil(T / fov_sample_dt)));
        const double fov_inv_K = 1.0 / static_cast<double>(std::max(1, fov_samples));
        const double fov_dt = T * fov_inv_K;
        for (int k = 0; k <= fov_samples; ++k)
        {
          const double alpha = static_cast<double>(k) * fov_inv_K;
          const double t_local = alpha * T;
          const double t_global = seg_start_time + t_local;
          const double trap_weight = (k == 0 || k == fov_samples) ? 0.5 : 1.0;
          const double common_weight = trap_weight * fov_dt;

          typename PosTraj::BasisRow bp, bv, ba, bj, bs;
          PosTraj::computeBasisFunctions(t_local, bp, bv, ba, bj, bs);
          Vec3f p = Vec3f::Zero();
          Vec3f v = Vec3f::Zero();
          p.transpose().noalias() = bp * pos_block;
          v.transpose().noalias() = bv * pos_block;

          typename YawTraj::BasisRow ybp, ybv, yba, ybj, ybs;
          YawTraj::computeBasisFunctions(t_local, ybp, ybv, yba, ybj, ybs);
          const double yaw = (ybp * yaw_block)(0, 0);
          const double yaw_dot = (ybv * yaw_block)(0, 0);

          Vec3f gp_fov = Vec3f::Zero();
          double gyaw_fov = 0.0;
          const double c_fov =
              cost_manager_.evaluateCameraFovSample(t_global,
                                                    p,
                                                    yaw,
                                                    gp_fov,
                                                    gyaw_fov);
          total_cost += c_fov * common_weight;
          gdC_pos.template block<PosTraj::COEFF_NUM, 3>(pos_base, 0).noalias() +=
              bp.transpose() * gp_fov.transpose() * common_weight;
          gdC_yaw.template block<YawTraj::COEFF_NUM, 1>(yaw_base, 0).noalias() +=
              ybp.transpose() * Eigen::Matrix<double, 1, 1>::Constant(gyaw_fov) * common_weight;
          gdT_pos(i) += c_fov * trap_weight * fov_inv_K;
          gdT_pos(i) += gp_fov.dot(v) * alpha * common_weight;
          gdT_yaw(i) += gyaw_fov * yaw_dot * alpha * common_weight;
        }
      }
      seg_start_time += T;
    }

    const auto accumulate_discrete_visibility_sample = [&](const double sample_t) {
      if (!std::isfinite(sample_t))
      {
        return;
      }
      const double total_duration = durations.sum();
      if (sample_t < -1.0e-6 || sample_t > total_duration + 1.0e-6)
      {
        return;
      }

      const double t_global = std::clamp(sample_t, 0.0, total_duration);
      double sample_seg_start = 0.0;
      for (int i = 0; i < piece_num_; ++i)
      {
        const double T = durations(i);
        const bool in_segment =
            i == piece_num_ - 1 || t_global <= sample_seg_start + T + 1.0e-9;
        if (!in_segment)
        {
          sample_seg_start += T;
          continue;
        }

        const double t_local = std::clamp(t_global - sample_seg_start, 0.0, T);
        const int pos_base = i * PosTraj::COEFF_NUM;
        const int yaw_base = i * YawTraj::COEFF_NUM;
        const auto pos_block = pos_coeffs.template block<PosTraj::COEFF_NUM, 3>(pos_base, 0);
        const auto yaw_block = yaw_coeffs.template block<YawTraj::COEFF_NUM, 1>(yaw_base, 0);

        typename PosTraj::BasisRow bp, bv, ba, bj, bs;
        PosTraj::computeBasisFunctions(t_local, bp, bv, ba, bj, bs);
        Vec3f p = Vec3f::Zero();
        Vec3f v = Vec3f::Zero();
        Vec3f a = Vec3f::Zero();
        p.transpose().noalias() = bp * pos_block;
        v.transpose().noalias() = bv * pos_block;
        a.transpose().noalias() = ba * pos_block;

        typename YawTraj::BasisRow ybp, ybv, yba, ybj, ybs;
        YawTraj::computeBasisFunctions(t_local, ybp, ybv, yba, ybj, ybs);
        const double yaw = (ybp * yaw_block)(0, 0);
        const double yaw_dot = (ybv * yaw_block)(0, 0);
        const double yaw_acc = (yba * yaw_block)(0, 0);

        Vec3f gp = Vec3f::Zero();
        Vec3f gv = Vec3f::Zero();
        double gyaw = 0.0;
        double gyaw_dot = 0.0;
        double gt_global = 0.0;
        const double c_track = cost_manager_.evaluateJointSample(t_global,
                                                                 p,
                                                                 v,
                                                                 yaw,
                                                                 yaw_dot,
                                                                 gp,
                                                                 gv,
                                                                 gyaw,
                                                                 gyaw_dot,
                                                                 gt_global);
        total_cost += c_track;
        gdC_pos.template block<PosTraj::COEFF_NUM, 3>(pos_base, 0).noalias() +=
            bp.transpose() * gp.transpose() +
            bv.transpose() * gv.transpose();
        gdC_yaw.template block<YawTraj::COEFF_NUM, 1>(yaw_base, 0).noalias() +=
            ybp.transpose() * Eigen::Matrix<double, 1, 1>::Constant(gyaw) +
            ybv.transpose() * Eigen::Matrix<double, 1, 1>::Constant(gyaw_dot);

        const double pos_time_grad = gp.dot(v) + gv.dot(a);
        const double yaw_time_grad = gyaw * yaw_dot + gyaw_dot * yaw_acc;
        for (int time_idx = 0; time_idx < i; ++time_idx)
        {
          gdT_pos(time_idx) -= pos_time_grad;
          gdT_yaw(time_idx) -= yaw_time_grad;
        }
        break;
      }
    };

    for (const double sample_t : cost_manager_.discreteSampleTimes())
    {
      accumulate_discrete_visibility_sample(sample_t);
    }

    PosInnerMat grad_pos_points;
    YawInnerMat grad_yaw_points;
    Eigen::VectorXd grad_T_pos;
    Eigen::VectorXd grad_T_yaw;
    PosBoundaryState grad_pos_head, grad_pos_tail;
    YawBoundaryState grad_yaw_head, grad_yaw_tail;
    pos_traj_.propagateGradFull(gdC_pos,
                                gdT_pos,
                                grad_pos_points,
                                grad_T_pos,
                                grad_pos_head,
                                grad_pos_tail);
    yaw_traj_.propagateGradFull(gdC_yaw,
                                gdT_yaw,
                                grad_yaw_points,
                                grad_T_yaw,
                                grad_yaw_head,
                                grad_yaw_tail);

    const Eigen::VectorXd grad_T = grad_T_pos + grad_T_yaw;
    for (int i = 0; i < piece_num_; ++i)
    {
      g(i) += time_map_.backward(x(i), durations(i), grad_T(i));
    }
    int offset = posOffset();
    for (int i = 1; i < piece_num_; ++i)
    {
      const int dof = positionDof(i);
      g.segment(offset, dof) +=
          backwardPositionGrad(x.segment(offset, dof), grad_pos_points.col(i - 1), i);
      offset += dof;
    }
    offset = yawOffset();
    for (int i = 1; i < piece_num_; ++i)
    {
      g(offset++) += grad_yaw_points(0, i - 1);
    }

    return total_cost;
  }

private:
  traj_opt::Config cfg_;
  std::shared_ptr<ros_interface::RosInterface> ros_ptr_;
  general_planner::MapManager::Ptr map_manager_;
  double safe_distance_{0.45};
  double pos_energy_weight_{1.0};
  double yaw_energy_weight_{0.05};
  int piece_num_{0};
  int samples_per_piece_{5};
  bool use_corridor_{false};

  TrackingProblem problem_;
  std::vector<double> init_times_;
  typename TaskOptimizer<SPos>::WaypointsType init_pos_waypoints_;
  YawInnerMat init_yaw_inner_;
  Eigen::VectorXd warm_start_;
  spatial_map::PolyhedraH h_polytopes_;
  spatial_map::PolyhedraV v_polytopes_;
  Eigen::VectorXi h_poly_idx_;
  Eigen::VectorXi v_poly_idx_;
  spatial_map::PolytopeSpatialMap corridor_spatial_map_;

  temporal_map::QuadInvTimeMap time_map_;
  TaskTimeCost time_cost_;
  cost_functional_manager::TrackingCostManager cost_manager_;
  PosBoundaryState pos_head_state_;
  PosBoundaryState pos_tail_state_;
  YawBoundaryState yaw_head_state_;
  YawBoundaryState yaw_tail_state_;
  PosTraj pos_traj_;
  YawTraj yaw_traj_;
};

minco::PerchingSemanticConfig deriveBoundaryConfig(const PerchingProblem &problem,
                                                   const traj_opt::Config &cfg)
{
  if (problem.use_terminal_config)
  {
    return problem.terminal;
  }

  minco::PerchingSemanticConfig terminal;
  terminal.plate_position = problem.surface.position;
  terminal.plate_velocity = problem.surface.velocity;
  terminal.plate_acceleration = problem.surface.acceleration;
  terminal.reference_time = problem.surface.t;
  terminal.surface_x = problem.surface.surface_x;
  terminal.surface_y = problem.surface.surface_y;
  terminal.surface_z = problem.surface.surface_z;
  terminal.yaw = problem.surface.yaw;
  terminal.yaw_rate = problem.surface.yaw_rate;
  terminal.rotate_surface_with_yaw_rate = true;
  terminal.gravity = cfg.grav;
  terminal.terminal_time_seed = 0.0;

  const double projected_l =
      (problem.nominal_tail_pvaj.col(0) - problem.surface.position).dot(problem.surface.surface_z);
  terminal.robot_l = std::max(0.0, projected_l);
  terminal.v_plus = std::max(0.2, 0.35 * clampPositive(cfg.max_vel, 2.0));
  terminal.thrust_nominal = 9.81;
  terminal.thrust_range = 0.25 * terminal.thrust_nominal;
  terminal.use_dynamics_terminal_accel = true;
  terminal.pre_contact_distance = 0.4;
  terminal.terminal_relax_time = 0.35;
  terminal.weight_nu = 1.0e-2;
  terminal.weight_tau_f = 1.0e-3;
  return terminal;
}

class PerchingRunner : public TaskRunner<4, cost_functional_manager::PerchingCostManager>
{
public:
  using Base = TaskRunner<4, cost_functional_manager::PerchingCostManager>;

  PerchingRunner(const traj_opt::Config &cfg,
                 const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
      : Base(cfg, ros_ptr)
  {
  }

  bool optimize(PerchingProblem problem, Trajectory &out_traj)
  {
    if (problem.safe_distance <= 0.0)
    {
      problem.safe_distance = Base::safeDistance();
    }
    problem.surface.surface_z = normalizedOr(problem.surface.surface_z, Vec3f::UnitZ());
    problem.surface.surface_x = normalizedOr(problem.surface.surface_x, Vec3f::UnitX());
    problem.surface.surface_y =
        normalizedOr(problem.surface.surface_z.cross(problem.surface.surface_x), Vec3f::UnitY());
    problem.surface.surface_x =
        normalizedOr(problem.surface.surface_y.cross(problem.surface.surface_z), Vec3f::UnitX());

    if (problem.nominal_tail_pvaj.col(0).squaredNorm() < 1.0e-12)
    {
      problem.nominal_tail_pvaj.col(0) =
          problem.surface.position + std::max(0.2, problem.robot_radius) * problem.surface.surface_z;
    }
    if (problem.robot_l <= 0.0)
    {
      problem.robot_l =
          std::max(0.0, (problem.nominal_tail_pvaj.col(0) - problem.surface.position)
                            .dot(problem.surface.surface_z));
    }
    if (problem.guide_path.empty())
    {
      problem.guide_path.emplace_back(problem.head_pvaj.col(0));
      problem.guide_path.emplace_back(problem.nominal_tail_pvaj.col(0));
    }
    if (problem.use_tracking_warm_start &&
        problem.init_total_time > 0.0 &&
        problem.warm_start_guide_path.size() >= 2)
    {
      problem.use_initial_guess = true;
      problem.initial_guess.valid = true;
      problem.initial_guess.total_time = problem.init_total_time;
      problem.initial_guess.nu = problem.init_nu;
      problem.initial_guess.tau_f = problem.init_tau_f;
      problem.initial_guess.guide_path = problem.warm_start_guide_path;
      problem.initial_guess.guide_t = problem.warm_start_guide_t;
      problem.guide_path = problem.warm_start_guide_path;
      problem.guide_t = problem.warm_start_guide_t;
      std::cout << " -- [PerchingSnapTrajOpt] TRACKING_TO_PERCHING_WARM_START_CONSUMED"
                << " T0=" << problem.initial_guess.total_time
                << ", guide_size=" << problem.initial_guess.guide_path.size()
                << ", nu_seed=[" << problem.initial_guess.nu.x()
                << ", " << problem.initial_guess.nu.y() << "]"
                << ", tau_f_seed=" << problem.initial_guess.tau_f << std::endl;
    }

    TaskOptimizer<4>::WaypointsType initial_waypoints;
    std::vector<double> initial_times;
    Eigen::VectorXd initial_extra_vars;
    const std::vector<double> *initial_times_ptr = nullptr;
    const TaskOptimizer<4>::WaypointsType *initial_waypoints_ptr = nullptr;
    const Eigen::VectorXd *initial_extra_vars_ptr = nullptr;

    const bool boundary_mapping_enabled = problem.use_terminal_config;
    if (boundary_mapping_enabled)
    {
      boundary_mapping_.configure(deriveBoundaryConfig(problem, Base::mutableConfig()));
      std::cout << " -- [PerchingSnapTrajOpt] PERCHING_BOUNDARY_MAPPING_ENABLED"
                << std::endl;
    }

    if (boundary_mapping_enabled &&
        prepareBvpInitialState<4>(Base::mutableConfig(),
                                  problem.head_pvaj,
                                  problem.nominal_tail_pvaj,
                                  boundary_mapping_,
                                  problem.initial_guess,
                                  problem.piece_num,
                                  problem.min_piece_duration,
                                  problem.max_total_duration,
                                  initial_times,
                                  initial_waypoints,
                                  initial_extra_vars))
    {
      initial_times_ptr = &initial_times;
      initial_waypoints_ptr = &initial_waypoints;
      initial_extra_vars_ptr = &initial_extra_vars;
    }
    else if (problem.use_initial_guess &&
             prepareTimedInitialState<4>(Base::mutableConfig(),
                                         problem.head_pvaj,
                                         problem.nominal_tail_pvaj,
                                         problem.initial_guess,
                                         problem.piece_num,
                                         problem.min_piece_duration,
                                         initial_times,
                                         initial_waypoints))
    {
      initial_extra_vars.resize(minco::PerchingBoundaryMapping<3, 4>::EXTRA_DIM);
      initial_extra_vars.setZero();
      initial_extra_vars(minco::PerchingBoundaryMapping<3, 4>::IDX_NU_X) =
          problem.initial_guess.nu.x();
      initial_extra_vars(minco::PerchingBoundaryMapping<3, 4>::IDX_NU_Y) =
          problem.initial_guess.nu.y();
      initial_extra_vars(minco::PerchingBoundaryMapping<3, 4>::IDX_TAU_F) =
          problem.initial_guess.tau_f;
      initial_times_ptr = &initial_times;
      initial_waypoints_ptr = &initial_waypoints;
      initial_extra_vars_ptr = &initial_extra_vars;
      std::cout << " -- [PerchingSnapTrajOpt] PERCHING_BVP_INITIAL_GUESS_FAILED_USE_TIMED_GUIDE"
                << std::endl;
    }
    cost_manager_.reset(Base::mutableConfig(),
                        Base::mapManager(),
                        problem,
                        &Base::mutableConfig().quadrotot_flatness);
    std::cout << " -- [PerchingSnapTrajOpt] PERCHING_TIME_BOUND max_total_duration="
              << problem.max_total_duration
              << ", duration_seed=" << problem.duration_seed
              << ", upper_weight=" << problem.time_upper_bound_weight
              << ", seed_weight=" << problem.duration_seed_weight << std::endl;
    const bool ok = Base::run(problem.head_pvaj,
                              problem.nominal_tail_pvaj,
                              problem.guide_path,
                              problem.piece_num,
                              problem.min_piece_duration,
                              problem.min_total_duration,
                              problem.time_lower_bound_weight,
                              problem.max_total_duration,
                              problem.time_upper_bound_weight,
                              problem.duration_seed,
                              problem.duration_seed_weight,
                              cost_manager_,
                              out_traj,
                              boundary_mapping_enabled ? &boundary_mapping_ : nullptr,
                              initial_times_ptr,
                              initial_waypoints_ptr,
                              initial_extra_vars_ptr);
    if (ok)
    {
      std::cout << " -- [PerchingSnapTrajOpt] PERCHING_OPT_SUCCESS duration="
                << out_traj.getTotalDuration() << std::endl;
    }
    else
    {
      std::cout << " -- [PerchingSnapTrajOpt] PERCHING_OPT_FAILED" << std::endl;
    }
    return ok;
  }

private:
  cost_functional_manager::PerchingCostManager cost_manager_;
  minco::PerchingBoundaryMapping<3, 4> boundary_mapping_;
};

class TakeoffRunner : public TaskRunner<4, cost_functional_manager::TakeoffCostManager>
{
public:
  using Base = TaskRunner<4, cost_functional_manager::TakeoffCostManager>;

  TakeoffRunner(const traj_opt::Config &cfg,
                const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
      : Base(cfg, ros_ptr)
  {
  }

  bool optimize(DynamicTakeoffProblem problem, Trajectory &out_traj)
  {
    if (problem.safe_distance <= 0.0)
    {
      problem.safe_distance = Base::safeDistance();
    }
    problem.surface.surface_z = normalizedOr(problem.surface.surface_z, Vec3f::UnitZ());
    problem.surface.surface_x = normalizedOr(problem.surface.surface_x, Vec3f::UnitX());
    problem.surface.surface_y =
        normalizedOr(problem.surface.surface_z.cross(problem.surface.surface_x), Vec3f::UnitY());
    problem.surface.surface_x =
        normalizedOr(problem.surface.surface_y.cross(problem.surface.surface_z), Vec3f::UnitX());

    if (problem.nominal_head_pvaj.col(0).squaredNorm() < 1.0e-12)
    {
      problem.nominal_head_pvaj.col(0) =
          problem.surface.position + std::max(0.0, problem.robot_l) * problem.surface.surface_z;
      problem.nominal_head_pvaj.col(1) = problem.surface.velocity;
    }
    if (problem.tail_pvaj.col(0).squaredNorm() < 1.0e-12)
    {
      problem.tail_pvaj.col(0) =
          problem.nominal_head_pvaj.col(0) +
          std::max(0.5, problem.escape_distance) * problem.surface.surface_z +
          std::max(0.2, problem.escape_height) * Vec3f::UnitZ();
    }
    if (problem.guide_path.empty())
    {
      problem.guide_path.emplace_back(problem.nominal_head_pvaj.col(0));
      problem.guide_path.emplace_back(problem.tail_pvaj.col(0));
    }

    problem.boundary.surface = problem.surface;
    problem.boundary.robot_l = problem.robot_l;
    if (problem.boundary.thrust_nominal <= 0.0)
    {
      problem.boundary.thrust_nominal = std::abs(Base::mutableConfig().grav);
    }
    if (problem.boundary.gravity <= 0.0)
    {
      problem.boundary.gravity = std::abs(Base::mutableConfig().grav);
    }

    head_mapping_.configure(problem.boundary);
    const bool head_mapping_enabled = problem.use_head_mapping && head_mapping_.enabled();
    if (head_mapping_enabled)
    {
      std::cout << " -- [TakeoffSnapTrajOpt] TAKEOFF_HEAD_MAPPING_ENABLED" << std::endl;
    }

    const int piece_num =
        std::clamp(problem.piece_num > 0 ? problem.piece_num : Base::mutableConfig().piece_num,
                   1,
                   32);
    const double min_piece_duration =
        std::max(0.05, problem.min_duration / static_cast<double>(std::max(1, piece_num)));
    double duration_seed = 0.0;
    if (problem.guide_t.size() == problem.guide_path.size() &&
        !problem.guide_t.empty() &&
        std::isfinite(problem.guide_t.back()) &&
        problem.guide_t.back() > 0.0)
    {
      duration_seed = problem.guide_t.back();
    }
    else
    {
      duration_seed =
          std::max(problem.min_duration,
                   pathLength(problem.guide_path) / std::max(0.1, problem.reference_speed));
    }
    duration_seed =
        std::clamp(duration_seed,
                   std::max(problem.min_duration,
                            static_cast<double>(piece_num) * min_piece_duration),
                   std::max(problem.min_duration, problem.max_duration));

    PerchingInitialGuess initial_guess;
    initial_guess.valid = true;
    initial_guess.total_time = duration_seed;
    initial_guess.guide_path = problem.guide_path;
    initial_guess.guide_t = problem.guide_t;

    TaskOptimizer<4>::WaypointsType initial_waypoints;
    std::vector<double> initial_times;
    Eigen::VectorXd initial_extra_vars;
    const std::vector<double> *initial_times_ptr = nullptr;
    const TaskOptimizer<4>::WaypointsType *initial_waypoints_ptr = nullptr;
    const Eigen::VectorXd *initial_extra_vars_ptr = nullptr;

    if (head_mapping_enabled &&
        prepareBvpInitialState<4>(Base::mutableConfig(),
                                  problem.nominal_head_pvaj,
                                  problem.tail_pvaj,
                                  head_mapping_,
                                  initial_guess,
                                  piece_num,
                                  min_piece_duration,
                                  problem.max_duration,
                                  initial_times,
                                  initial_waypoints,
                                  initial_extra_vars,
                                  "TakeoffSnapTrajOpt",
                                  "TAKEOFF",
                                  false))
    {
      initial_times_ptr = &initial_times;
      initial_waypoints_ptr = &initial_waypoints;
      initial_extra_vars_ptr = &initial_extra_vars;
    }

    cost_manager_.reset(Base::mutableConfig(),
                        Base::mapManager(),
                        problem,
                        &Base::mutableConfig().quadrotot_flatness);
    const bool ok = Base::run(problem.nominal_head_pvaj,
                              problem.tail_pvaj,
                              problem.guide_path,
                              piece_num,
                              min_piece_duration,
                              problem.min_duration,
                              0.0,
                              problem.max_duration,
                              0.0,
                              duration_seed,
                              0.0,
                              cost_manager_,
                              out_traj,
                              head_mapping_enabled ? &head_mapping_ : nullptr,
                              initial_times_ptr,
                              initial_waypoints_ptr,
                              initial_extra_vars_ptr);
    if (ok)
    {
      std::cout << " -- [TakeoffSnapTrajOpt] TAKEOFF_OPT_SUCCESS duration="
                << out_traj.getTotalDuration() << std::endl;
    }
    else
    {
      std::cout << " -- [TakeoffSnapTrajOpt] TAKEOFF_OPT_FAILED" << std::endl;
    }
    return ok;
  }

private:
  cost_functional_manager::TakeoffCostManager cost_manager_;
  minco::TakeoffHeadBoundaryMapping<3, 4> head_mapping_;
};

} // namespace

struct TrackingJerkTrajOpt::Impl final : public JointTrackingRunner<3>
{
  Impl(const traj_opt::Config &cfg,
       const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
      : JointTrackingRunner<3>(cfg, ros_ptr)
  {
  }
};

TrackingJerkTrajOpt::TrackingJerkTrajOpt(const traj_opt::Config &cfg,
                                         const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
    : impl_(std::make_shared<Impl>(cfg, ros_ptr))
{
}

void TrackingJerkTrajOpt::setMapManager(const general_planner::MapManager::Ptr &map_manager)
{
  impl_->setMapManager(map_manager);
}

void TrackingJerkTrajOpt::setSafeDistance(double safe_distance)
{
  impl_->setSafeDistance(safe_distance);
}

bool TrackingJerkTrajOpt::optimize(const TrackingProblem &problem,
                                   Trajectory &out_traj,
                                   Trajectory *out_yaw_traj)
{
  return impl_->optimize(problem, out_traj, out_yaw_traj);
}

struct TrackingSnapTrajOpt::Impl final : public JointTrackingRunner<4>
{
  Impl(const traj_opt::Config &cfg,
       const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
      : JointTrackingRunner<4>(cfg, ros_ptr)
  {
  }
};

TrackingSnapTrajOpt::TrackingSnapTrajOpt(const traj_opt::Config &cfg,
                                         const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
    : impl_(std::make_shared<Impl>(cfg, ros_ptr))
{
}

void TrackingSnapTrajOpt::setMapManager(const general_planner::MapManager::Ptr &map_manager)
{
  impl_->setMapManager(map_manager);
}

void TrackingSnapTrajOpt::setSafeDistance(double safe_distance)
{
  impl_->setSafeDistance(safe_distance);
}

bool TrackingSnapTrajOpt::optimize(const TrackingProblem &problem,
                                   Trajectory &out_traj,
                                   Trajectory *out_yaw_traj)
{
  return impl_->optimize(problem, out_traj, out_yaw_traj);
}

struct PerchingSnapTrajOpt::Impl final : public PerchingRunner
{
  Impl(const traj_opt::Config &cfg,
       const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
      : PerchingRunner(cfg, ros_ptr)
  {
  }
};

PerchingSnapTrajOpt::PerchingSnapTrajOpt(const traj_opt::Config &cfg,
                                         const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
    : impl_(std::make_shared<Impl>(cfg, ros_ptr))
{
}

void PerchingSnapTrajOpt::setMapManager(const general_planner::MapManager::Ptr &map_manager)
{
  impl_->setMapManager(map_manager);
}

void PerchingSnapTrajOpt::setSafeDistance(double safe_distance)
{
  impl_->setSafeDistance(safe_distance);
}

bool PerchingSnapTrajOpt::optimize(const PerchingProblem &problem, Trajectory &out_traj)
{
  return impl_->optimize(problem, out_traj);
}

struct DynamicTakeoffSnapTrajOpt::Impl final : public TakeoffRunner
{
  Impl(const traj_opt::Config &cfg,
       const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
      : TakeoffRunner(cfg, ros_ptr)
  {
  }
};

DynamicTakeoffSnapTrajOpt::DynamicTakeoffSnapTrajOpt(
    const traj_opt::Config &cfg,
    const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
    : impl_(std::make_shared<Impl>(cfg, ros_ptr))
{
}

void DynamicTakeoffSnapTrajOpt::setMapManager(
    const general_planner::MapManager::Ptr &map_manager)
{
  impl_->setMapManager(map_manager);
}

void DynamicTakeoffSnapTrajOpt::setSafeDistance(double safe_distance)
{
  impl_->setSafeDistance(safe_distance);
}

bool DynamicTakeoffSnapTrajOpt::optimize(const DynamicTakeoffProblem &problem,
                                         Trajectory &out_traj)
{
  return impl_->optimize(problem, out_traj);
}

} // namespace traj_opt
