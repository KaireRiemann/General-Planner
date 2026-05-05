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
#include "traj_opt/costfunctional/spatialcosts/thrust_band_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/velocity_bound_penalty.hpp"
#include "traj_opt/costfunctional/temporalcosts/linear_time_cost.hpp"
#include "traj_opt/costfunctional/temporalmap/quad_inv_time_map.hpp"
#include "traj_opt/costfunctional_manager/perching_cost_manager.hpp"
#include "traj_opt/costfunctional_manager/tracking_cost_manager.hpp"
#include "traj_opt/minco/minco_optimizer.hpp"
#include "utils/optimization/lbfgs.h"

namespace traj_opt
{
namespace
{

using geometry_utils::Trajectory;
using super_utils::Mat3Df;
using super_utils::StatePVAJ;
using super_utils::Vec3f;
using super_utils::VecDf;

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
  double lower_bound_weight{0.0};
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
    return cost;
  }
};

template <int S>
using TaskOptimizer = minco::MINCOOptimizer<3, S, temporal_map::QuadInvTimeMap, R3IdentitySpatialMap>;

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

double pathLength(const super_utils::vec_E<Vec3f> &path)
{
  double length = 0.0;
  for (int i = 1; i < static_cast<int>(path.size()); ++i)
  {
    length += (path[i] - path[i - 1]).norm();
  }
  return length;
}

super_utils::vec_E<Vec3f> sanitizeGuide(const super_utils::vec_E<Vec3f> &guide_path,
                                        const Vec3f &start,
                                        const Vec3f &goal)
{
  super_utils::vec_E<Vec3f> out;
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

Vec3f interpolateByArc(const super_utils::vec_E<Vec3f> &path,
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

template <int S>
bool prepareInitialState(const traj_opt::Config &cfg,
                         const StatePVAJ &head,
                         const StatePVAJ &tail,
                         const super_utils::vec_E<Vec3f> &guide_path,
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
           const super_utils::vec_E<Vec3f> &guide_path,
           int piece_num,
           double min_piece_duration,
           double min_total_duration,
           double time_lower_bound_weight,
           CostManager &cost_manager,
           Trajectory &out_traj,
           const minco::TerminalMappingBase<3, S> *terminal_mapping = nullptr)
  {
    std::vector<double> times;
    typename TaskOptimizer<S>::WaypointsType waypoints;
    if (!prepareInitialState<S>(cfg_,
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

    if (!optimizer_.setInitState(times,
                                 waypoints,
                                 toBoundaryState<S>(head),
                                 toBoundaryState<S>(tail)))
    {
      return false;
    }

    active_cost_manager_ = &cost_manager;
    active_terminal_mapping_ = terminal_mapping;
    time_cost_.min_piece_duration = min_piece_duration;
    time_cost_.min_total_duration = min_total_duration;
    time_cost_.lower_bound_weight =
        time_lower_bound_weight > 0.0
            ? time_lower_bound_weight
            : std::max(100.0, std::abs(cfg_.penna_t) * 10.0);
    Eigen::VectorXd x = optimizer_.generateInitialGuess(active_terminal_mapping_);
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
    optimizer_.setWarmStartGuess(x);
    active_cost_manager_ = nullptr;
    active_terminal_mapping_ = nullptr;
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
    if (active_terminal_mapping_ != nullptr)
    {
      return optimizer_.evaluateWithTerminalMapping(x,
                                                    g,
                                                    time_cost_,
                                                    *active_cost_manager_,
                                                    active_terminal_mapping_);
    }
    return optimizer_.evaluate(x, g, time_cost_, *active_cost_manager_);
  }

private:
  traj_opt::Config cfg_;
  std::shared_ptr<ros_interface::RosInterface> ros_ptr_;
  general_planner::MapManager::Ptr map_manager_;
  double safe_distance_{0.45};
  int iter_num_{0};

  temporal_map::QuadInvTimeMap time_map_;
  R3IdentitySpatialMap spatial_map_;
  TaskTimeCost time_cost_;
  TaskOptimizer<S> optimizer_;
  CostManager *active_cost_manager_{nullptr};
  const minco::TerminalMappingBase<3, S> *active_terminal_mapping_{nullptr};
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
                     cost_manager_,
                     out_traj);
  }

private:
  cost_functional_manager::TrackingCostManager cost_manager_;
};

minco::PerchingSemanticConfig deriveTerminalConfig(const PerchingProblem &problem,
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

    terminal_mapping_.configure(deriveTerminalConfig(problem, Base::mutableConfig()));
    cost_manager_.reset(Base::mutableConfig(),
                        Base::mapManager(),
                        problem,
                        &Base::mutableConfig().quadrotot_flatness);
    return Base::run(problem.head_pvaj,
                     problem.nominal_tail_pvaj,
                     problem.guide_path,
                     problem.piece_num,
                     problem.min_piece_duration,
                     problem.min_total_duration,
                     problem.time_lower_bound_weight,
                     cost_manager_,
                     out_traj,
                     &terminal_mapping_);
  }

private:
  cost_functional_manager::PerchingCostManager cost_manager_;
  minco::PerchingTerminalMapping<3, 4> terminal_mapping_;
};

} // namespace

struct TrackingJerkTrajOpt::Impl final : public TrackingRunner<3>
{
  Impl(const traj_opt::Config &cfg,
       const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
      : TrackingRunner<3>(cfg, ros_ptr)
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

bool TrackingJerkTrajOpt::optimize(const TrackingProblem &problem, Trajectory &out_traj)
{
  return impl_->optimize(problem, out_traj);
}

struct TrackingSnapTrajOpt::Impl final : public TrackingRunner<4>
{
  Impl(const traj_opt::Config &cfg,
       const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
      : TrackingRunner<4>(cfg, ros_ptr)
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

bool TrackingSnapTrajOpt::optimize(const TrackingProblem &problem, Trajectory &out_traj)
{
  return impl_->optimize(problem, out_traj);
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

} // namespace traj_opt
