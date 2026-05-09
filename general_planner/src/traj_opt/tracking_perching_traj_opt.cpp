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
#include "traj_opt/costfunctional_manager/tracking_cost_manager.hpp"
#include "traj_opt/minco/minco_optimizer.hpp"
#include "utils/geometry/geometry_utils.h"
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

    dynamics_ = cost_functional_manager::detail::makeDynamicsPenaltyConfig(cfg_,
                                                                           map_manager_.get(),
                                                                           problem_.safe_distance,
                                                                           &cfg_.quadrotot_flatness);
    if (problem_.use_esdf_obstacle && problem_.weight_esdf_obstacle > 0.0)
    {
      dynamics_.weight_esdf = 0.0;
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

  bool interpolateViewpoint(double t, Vec3f &position, Vec3f &velocity) const
  {
    const auto &path = problem_.viewpoints;
    const auto &times = problem_.target_sample_times;
    if (path.size() < 2 || path.size() != times.size())
    {
      return false;
    }
    if (t <= times.front())
    {
      const double dt = std::max(kTiny, times[1] - times[0]);
      position = path.front();
      velocity = (path[1] - path[0]) / dt;
      return true;
    }
    if (t >= times.back())
    {
      position = path.back();
      velocity.setZero();
      return true;
    }
    const auto it = std::lower_bound(times.begin(), times.end(), t);
    const int idx = static_cast<int>(std::distance(times.begin(), it));
    const double left_t = times[static_cast<std::size_t>(idx - 1)];
    const double right_t = times[static_cast<std::size_t>(idx)];
    const double dt = std::max(kTiny, right_t - left_t);
    const double alpha = std::clamp((t - left_t) / dt, 0.0, 1.0);
    const Vec3f &left_p = path[static_cast<std::size_t>(idx - 1)];
    const Vec3f &right_p = path[static_cast<std::size_t>(idx)];
    position = left_p + alpha * (right_p - left_p);
    velocity = (right_p - left_p) / dt;
    return true;
  }

  bool interpolateVisibleRegion(double t,
                                Vec3f &visible_point,
                                Vec3f &visible_velocity,
                                double &theta,
                                double &confidence) const
  {
    const auto &regions = problem_.visible_regions;
    if (regions.empty())
    {
      return false;
    }

    auto applyRegion = [&](const traj_opt::TrackingVisibleRegion &region) {
      if (!region.valid)
      {
        return false;
      }
      visible_point = region.visible_point;
      visible_velocity.setZero();
      theta = region.theta;
      confidence = std::clamp(region.confidence, 0.0, 1.0);
      return true;
    };

    if (t <= regions.front().t)
    {
      return applyRegion(regions.front());
    }
    if (t >= regions.back().t)
    {
      return applyRegion(regions.back());
    }

    int left_idx = -1;
    int right_idx = -1;
    for (int i = 0; i < static_cast<int>(regions.size()); ++i)
    {
      if (!regions[static_cast<std::size_t>(i)].valid)
      {
        continue;
      }
      if (regions[static_cast<std::size_t>(i)].t <= t)
      {
        left_idx = i;
      }
      if (regions[static_cast<std::size_t>(i)].t >= t)
      {
        right_idx = i;
        break;
      }
    }

    if (left_idx < 0 && right_idx < 0)
    {
      return false;
    }
    if (left_idx < 0)
    {
      return applyRegion(regions[static_cast<std::size_t>(right_idx)]);
    }
    if (right_idx < 0)
    {
      return applyRegion(regions[static_cast<std::size_t>(left_idx)]);
    }
    if (left_idx == right_idx)
    {
      return applyRegion(regions[static_cast<std::size_t>(left_idx)]);
    }

    for (int i = left_idx + 1; i < right_idx; ++i)
    {
      if (!regions[static_cast<std::size_t>(i)].valid)
      {
        return false;
      }
    }
    if (right_idx - left_idx > 1)
    {
      return false;
    }

    const auto &left = regions[static_cast<std::size_t>(left_idx)];
    const auto &right = regions[static_cast<std::size_t>(right_idx)];
    if (!left.valid || !right.valid)
    {
      return false;
    }

    const double dt = std::max(kTiny, right.t - left.t);
    const double alpha = std::clamp((t - left.t) / dt, 0.0, 1.0);
    visible_point = left.visible_point + alpha * (right.visible_point - left.visible_point);
    visible_velocity = (right.visible_point - left.visible_point) / dt;
    theta = left.theta + alpha * (right.theta - left.theta);
    confidence = std::clamp(left.confidence + alpha * (right.confidence - left.confidence), 0.0, 1.0);
    return true;
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

  double addObservationDistanceCost(const Vec3f &p,
                                    const traj_opt::DynamicTargetState &target,
                                    Vec3f &grad_p,
                                    Vec3f &grad_target) const
  {
    double cost = 0.0;
    const Vec3f rel = p - target.position;
    const double h = rel.head<2>().norm();
    double f = 0.0;
    double df = 0.0;
    if (cost_functional::smoothedL1(problem_.od_h_lower - h, cfg_.smooth_eps, f, df))
    {
      cost += problem_.weight_od_near * f;
      if (h > 1.0e-6)
      {
        grad_p.head<2>() -= problem_.weight_od_near * df * rel.head<2>() / h;
      }
    }
    if (cost_functional::smoothedL1(h - problem_.od_h_upper, cfg_.smooth_eps, f, df))
    {
      cost += problem_.weight_od_far * f;
      if (h > 1.0e-6)
      {
        grad_p.head<2>() += problem_.weight_od_far * df * rel.head<2>() / h;
      }
    }

    const double z = rel.z();
    if (cost_functional::smoothedL1(problem_.od_v_lower - z, cfg_.smooth_eps, f, df))
    {
      cost += problem_.weight_od_vertical * f;
      grad_p.z() -= problem_.weight_od_vertical * df;
    }
    if (cost_functional::smoothedL1(z - problem_.od_v_upper, cfg_.smooth_eps, f, df))
    {
      cost += problem_.weight_od_vertical * f;
      grad_p.z() += problem_.weight_od_vertical * df;
    }
    grad_target -= grad_p;
    return cost;
  }

  double addObservationAngleCost(const Vec3f &p,
                                 double yaw,
                                 const traj_opt::DynamicTargetState &target,
                                 Vec3f &grad_p,
                                 Vec3f &grad_target,
                                 double &grad_yaw) const
  {
    if (problem_.weight_oa <= 0.0)
    {
      return 0.0;
    }
    const Vec3f dir = target.position - p;
    const double r2 = dir.head<2>().squaredNorm();
    if (r2 < 1.0e-8)
    {
      return 0.0;
    }
    const double desired = std::atan2(dir.y(), dir.x());
    double err = yaw - desired;
    err = std::atan2(std::sin(err), std::cos(err));
    const double grad_err = 2.0 * problem_.weight_oa * err;
    grad_yaw += grad_err;
    Vec3f local_grad_p = Vec3f::Zero();
    local_grad_p.x() += -grad_err * dir.y() / r2;
    local_grad_p.y() += grad_err * dir.x() / r2;
    grad_p += local_grad_p;
    grad_target -= local_grad_p;
    return problem_.weight_oa * err * err;
  }

  double addStabilityCost(const Vec3f &p,
                          const Vec3f &v,
                          const traj_opt::DynamicTargetState &target,
                          Vec3f &grad_p,
                          Vec3f &grad_v,
                          Vec3f &grad_target,
                          double &grad_t_global) const
  {
    (void)grad_p;
    (void)grad_target;

    double cost = 0.0;
    const Vec3f rel_v = v - target.velocity;
    if (problem_.weight_relative_velocity > 0.0)
    {
      cost += 0.5 * problem_.weight_relative_velocity * rel_v.squaredNorm();
      grad_v += problem_.weight_relative_velocity * rel_v;
      grad_t_global -= problem_.weight_relative_velocity * rel_v.dot(target.acceleration);
    }

    if (problem_.weight_tangent_velocity > 0.0)
    {
      const Vec3f rel = p - target.position;
      const double h = rel.head<2>().norm();
      if (h > 1.0e-6)
      {
        Vec3f tangent(-rel.y() / h, rel.x() / h, 0.0);
        const double tv = rel_v.dot(tangent);
        cost += 0.5 * problem_.weight_tangent_velocity * tv * tv;
        grad_v += problem_.weight_tangent_velocity * tv * tangent;
      }
    }
    return cost;
  }

  double addViewpointAttractorCost(const Vec3f &p,
                                   double t,
                                   Vec3f &grad_p,
                                   double &grad_t_global) const
  {
    if (problem_.weight_viewpoint_attractor <= 0.0)
    {
      return 0.0;
    }
    Vec3f ref = Vec3f::Zero();
    Vec3f ref_v = Vec3f::Zero();
    if (!interpolateViewpoint(t, ref, ref_v))
    {
      return 0.0;
    }
    const Vec3f diff = p - ref;
    grad_p += problem_.weight_viewpoint_attractor * diff;
    grad_t_global -= problem_.weight_viewpoint_attractor * diff.dot(ref_v);
    return 0.5 * problem_.weight_viewpoint_attractor * diff.squaredNorm();
  }

  double addVisibleRegionCost(const Vec3f &p,
                              double t,
                              const traj_opt::DynamicTargetState &target,
                              Vec3f &grad_p,
                              double &grad_t_global) const
  {
    if (!problem_.use_visible_region ||
        problem_.weight_visible_region <= 0.0 ||
        problem_.visible_regions.empty())
    {
      return 0.0;
    }

    Vec3f visible_ref = Vec3f::Zero();
    Vec3f visible_ref_v = Vec3f::Zero();
    double theta = 0.0;
    double confidence = 0.0;
    if (!interpolateVisibleRegion(t, visible_ref, visible_ref_v, theta, confidence) ||
        confidence <= 0.0)
    {
      return 0.0;
    }

    const Vec3f a = p - target.position;
    const Vec3f b = visible_ref - target.position;
    const double norm_a = a.norm();
    const double norm_b = b.norm();
    if (norm_a < 1.0e-6 || norm_b < 1.0e-6)
    {
      return 0.0;
    }

    const double theta_limit = std::max(0.0, theta - std::max(0.0, problem_.visibility_angle_clearance));
    const double cos_limit = std::cos(theta_limit);
    const double inner = a.dot(b);
    const double cos_ab = std::clamp(inner / (norm_a * norm_b), -1.0, 1.0);
    const double violation = cos_limit - cos_ab;

    double f = 0.0;
    double df = 0.0;
    if (!cost_functional::smoothedL1(violation, cfg_.smooth_eps, f, df))
    {
      return 0.0;
    }

    const Vec3f dcos_da = b / (norm_a * norm_b) -
                          inner * a / (norm_a * norm_a * norm_a * norm_b);
    const Vec3f dcos_db = a / (norm_a * norm_b) -
                          inner * b / (norm_a * norm_b * norm_b * norm_b);
    const double soft_weight = problem_.weight_visible_region * confidence;
    const Vec3f local_grad_p = -soft_weight * df * dcos_da;
    const Vec3f local_grad_center = soft_weight * df * (dcos_da + dcos_db);
    const Vec3f local_grad_visible = -soft_weight * df * dcos_db;

    grad_p += local_grad_p;
    grad_t_global += local_grad_center.dot(target.velocity) +
                     local_grad_visible.dot(visible_ref_v);
    return soft_weight * f;
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
    Eigen::VectorXd global_time_grad = Eigen::VectorXd::Zero(piece_num_);

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

        typename YawTraj::BasisRow ybp, ybv, yba, ybj, ybs;
        YawTraj::computeBasisFunctions(t_local, ybp, ybv, yba, ybj, ybs);
        const double yaw = (ybp * yaw_block)(0, 0);
        const double yaw_dot = (ybv * yaw_block)(0, 0);
        const double yaw_acc = (yba * yaw_block)(0, 0);

        Vec3f gp_integral = Vec3f::Zero();
        Vec3f gv_integral = Vec3f::Zero();
        Vec3f ga_integral = Vec3f::Zero();
        Vec3f gj_integral = Vec3f::Zero();
        double gyaw_integral = 0.0;
        double gyaw_dot_integral = 0.0;
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
        const double c_dyn = cost_functional_manager::detail::accumulateDynamicsPenalty(dynamics_,
                                                                                        p,
                                                                                        v,
                                                                                        a,
                                                                                        j,
                                                                                        yaw,
                                                                                        yaw_dot,
                                                                                        gp_integral,
                                                                                        gv_integral,
                                                                                        ga_integral,
                                                                                        gj_integral,
                                                                                        gyaw_integral,
                                                                                        gyaw_dot_integral);
        const double c_integral = c_corridor + c_dyn;
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
        gdC_yaw.template block<YawTraj::COEFF_NUM, 1>(yaw_base, 0).noalias() +=
            (ybp.transpose() * Eigen::Matrix<double, 1, 1>::Constant(gyaw_integral) +
             ybv.transpose() * Eigen::Matrix<double, 1, 1>::Constant(gyaw_dot_integral)) *
            common_weight;
        gdT_yaw(i) += (gyaw_integral * yaw_dot + gyaw_dot_integral * yaw_acc) *
                      alpha * common_weight;

        if (k > 0 || i == 0)
        {
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
          total_cost += c_track * common_weight;
          gdC_pos.template block<PosTraj::COEFF_NUM, 3>(pos_base, 0).noalias() +=
              (bp.transpose() * gp.transpose() +
               bv.transpose() * gv.transpose()) *
              common_weight;
          gdC_yaw.template block<YawTraj::COEFF_NUM, 1>(yaw_base, 0).noalias() +=
              (ybp.transpose() * Eigen::Matrix<double, 1, 1>::Constant(gyaw) +
               ybv.transpose() * Eigen::Matrix<double, 1, 1>::Constant(gyaw_dot)) *
              common_weight;
          gdT_pos(i) += c_track * trap_weight * inv_K;
          gdT_pos(i) += (gp.dot(v) + gv.dot(a)) * alpha * common_weight;
          gdT_yaw(i) += (gyaw * yaw_dot + gyaw_dot * yaw_acc) * alpha * common_weight;
          gdT_pos(i) += gt_global * alpha * common_weight;
          global_time_grad(i) += gt_global * common_weight;
        }
      }
      seg_start_time += T;
    }

    double accumulator = 0.0;
    for (int i = piece_num_ - 1; i > 0; --i)
    {
      accumulator += global_time_grad(i);
      gdT_pos(i - 1) += accumulator;
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
  cost_functional_manager::detail::DynamicsPenaltyConfig dynamics_;
  cost_functional_manager::TrackingCostManager cost_manager_;
  PosBoundaryState pos_head_state_;
  PosBoundaryState pos_tail_state_;
  YawBoundaryState yaw_head_state_;
  YawBoundaryState yaw_tail_state_;
  PosTraj pos_traj_;
  YawTraj yaw_traj_;
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

} // namespace traj_opt
