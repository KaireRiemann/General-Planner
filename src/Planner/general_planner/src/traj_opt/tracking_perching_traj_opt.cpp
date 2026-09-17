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
  static_assert(DIM > 0 && DIM <= 3, "geometry trajectories have three spatial rows");
  Trajectory out;
  const auto &durations = traj.getDurations();
  out.reserve(static_cast<int>(durations.size()));
  for (int i = 0; i < durations.size(); ++i)
  {
    // Piece evaluates Vector3d derivatives, including yaw extrema. Preserve
    // the polynomial degree but pad unused spatial axes explicitly.
    const auto raw = traj.getPieceCoeffMat(i);
    Eigen::Matrix<double, 3, Eigen::Dynamic> coeff =
        Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, raw.cols());
    coeff.template topRows<DIM>() = raw;
    out.emplace_back(durations(i), coeff);
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
