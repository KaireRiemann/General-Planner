#include "traj_opt/traj_manager.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <path_search/astar.h>
#include <utils/header/color_msg_utils.hpp>
#include <utils/optimization/optimization_utils.h>

using namespace traj_opt;
using namespace color_text;
using namespace super_utils;
using namespace math_utils;

namespace
{
using GcopterMap = optimization_utils::Gcopter<Eigen::Map<Eigen::VectorXd>>;
using GcopterConstMap = optimization_utils::Gcopter<Eigen::Map<const Eigen::VectorXd>>;

void truncateToSixDecimals(double &num)
{
  num = std::trunc(num * 1e6) / 1e6;
}

double yawDelta(const double from, const double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

double normalizeYawNear(const double reference, const double yaw)
{
  return reference + yawDelta(reference, yaw);
}

double clampYawStep(const double reference, const double yaw, const double max_delta)
{
  const double delta = yawDelta(reference, yaw);
  const double bounded_delta = std::max(-max_delta, std::min(max_delta, delta));
  return reference + bounded_delta;
}

void normalizeHPoly(PolyhedronH &poly)
{
  if (poly.rows() == 0)
  {
    return;
  }
  Eigen::ArrayXd norms = poly.leftCols<3>().rowwise().norm();
  norms = norms.max(1.0e-12);
  poly.array().colwise() /= norms;
}

std::vector<double> toStdVector(const VecDf &v)
{
  return std::vector<double>(v.data(), v.data() + v.size());
}

std::string gridTypeName(int grid_type)
{
  if (grid_type >= 0 && grid_type < static_cast<int>(super_utils::GridTypeStr.size()))
  {
    return super_utils::GridTypeStr[grid_type];
  }
  return "UNKNOWN_GRID_TYPE";
}

Mat3Df waypointsToMatrix(const StatePVAJ &head, const Mat3Df &inner, const StatePVAJ &tail)
{
  Mat3Df waypoints(3, inner.cols() + 2);
  waypoints.col(0) = head.col(0);
  for (int i = 0; i < inner.cols(); ++i)
  {
    waypoints.col(i + 1) = inner.col(i);
  }
  waypoints.rightCols(1) = tail.col(0);
  return waypoints;
}

Vec3f interpolateGuideByArc(const vec_E<Vec3f> &path,
                            const std::vector<double> &times,
                            const std::vector<double> &arc_lengths,
                            double target_arc,
                            double &target_time)
{
  if (target_arc <= 0.0)
  {
    target_time = times.front();
    return path.front();
  }
  if (target_arc >= arc_lengths.back())
  {
    target_time = times.back();
    return path.back();
  }

  const auto upper = std::lower_bound(arc_lengths.begin(), arc_lengths.end(), target_arc);
  const int idx = static_cast<int>(std::distance(arc_lengths.begin(), upper));
  const double left_arc = arc_lengths[idx - 1];
  const double right_arc = arc_lengths[idx];
  const double ratio = (target_arc - left_arc) / std::max(1.0e-6, right_arc - left_arc);
  target_time = times[idx - 1] + ratio * (times[idx] - times[idx - 1]);
  return path[idx - 1] + ratio * (path[idx] - path[idx - 1]);
}

double estimateTrapezoidalDuration(double length,
                                   double start_vel,
                                   double end_vel,
                                   double max_vel,
                                   double max_acc)
{
  if (length < 1.0e-6)
  {
    return 0.05;
  }

  max_vel = std::max(1.0e-3, max_vel);
  max_acc = std::max(1.0e-3, max_acc);
  start_vel = std::clamp(start_vel, 0.0, max_vel);
  end_vel = std::clamp(end_vel, 0.0, max_vel);

  const double acc_len = (max_vel * max_vel - start_vel * start_vel) / (2.0 * max_acc);
  const double dec_len = (max_vel * max_vel - end_vel * end_vel) / (2.0 * max_acc);
  const double critical_len = std::max(0.0, acc_len) + std::max(0.0, dec_len);
  if (length >= critical_len)
  {
    return std::max(0.0, (max_vel - start_vel) / max_acc) +
           std::max(0.0, (max_vel - end_vel) / max_acc) +
           (length - critical_len) / max_vel;
  }

  const double peak_vel_sq = std::max(0.0, 0.5 * (start_vel * start_vel +
                                                  end_vel * end_vel +
                                                  2.0 * max_acc * length));
  const double peak_vel = std::sqrt(peak_vel_sq);
  return std::max(0.0, (peak_vel - start_vel) / max_acc) +
         std::max(0.0, (peak_vel - end_vel) / max_acc);
}

struct ClosestGuidePV
{
  Vec3f position{Vec3f::Zero()};
  Vec3f velocity{Vec3f::Zero()};
  double distance{0.0};
  double arc{0.0};
  int segment_idx{-1};
  double segment_ratio{0.0};
};

ClosestGuidePV closestPVOnPolyline(const vec_E<Vec3f> &path,
                                   const vec_E<Vec3f> &velocities,
                                   const Vec3f &query)
{
  ClosestGuidePV best;
  if (path.empty())
  {
    best.position = query;
    return best;
  }
  if (path.size() == 1)
  {
    best.position = path.front();
    best.velocity = velocities.size() == path.size() ? velocities.front() : Vec3f::Zero();
    return best;
  }

  double best_sq = std::numeric_limits<double>::infinity();
  double arc_start = 0.0;
  for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i)
  {
    const Vec3f a = path[i];
    const Vec3f b = path[i + 1];
    const Vec3f ab = b - a;
    const double seg_len = ab.norm();
    const double denom = ab.squaredNorm();
    const double s = denom > 1.0e-9 ? std::clamp((query - a).dot(ab) / denom, 0.0, 1.0) : 0.0;
    const Vec3f candidate = a + s * ab;
    const double sq = (query - candidate).squaredNorm();
    if (sq < best_sq)
    {
      best_sq = sq;
      best.position = candidate;
      if (velocities.size() == path.size())
      {
        best.velocity = (1.0 - s) * velocities[i] + s * velocities[i + 1];
      }
      else
      {
        best.velocity.setZero();
      }
      best.distance = std::sqrt(std::max(0.0, sq));
      best.arc = arc_start + s * seg_len;
      best.segment_idx = i;
      best.segment_ratio = s;
    }
    arc_start += seg_len;
  }
  return best;
}

vec_E<Vec3f> estimateGuideVelocities(const vec_E<Vec3f> &path,
                                      const std::vector<double> &times,
                                      const Vec3f &start_vel,
                                      const Vec3f &end_vel,
                                      double max_vel)
{
  vec_E<Vec3f> velocities(path.size(), Vec3f::Zero());
  if (path.empty() || path.size() != times.size())
  {
    return velocities;
  }

  max_vel = std::max(1.0e-3, max_vel);
  auto clampVelocity = [max_vel](Vec3f velocity) -> Vec3f {
    if (!velocity.allFinite())
    {
      return Vec3f::Zero();
    }
    const double norm = velocity.norm();
    if (norm > max_vel)
    {
      velocity *= max_vel / std::max(1.0e-9, norm);
    }
    return velocity;
  };

  if (path.size() == 1)
  {
    velocities.front() = clampVelocity(start_vel);
    return velocities;
  }

  for (int i = 0; i < static_cast<int>(path.size()); ++i)
  {
    if (i == 0 && start_vel.norm() > 1.0e-3)
    {
      velocities[i] = clampVelocity(start_vel);
      continue;
    }
    if (i == static_cast<int>(path.size()) - 1 && end_vel.norm() > 1.0e-3)
    {
      velocities[i] = clampVelocity(end_vel);
      continue;
    }

    int left = std::max(0, i - 1);
    int right = std::min(static_cast<int>(path.size()) - 1, i + 1);
    if (left == right)
    {
      velocities[i].setZero();
      continue;
    }
    double dt = times[right] - times[left];
    if (dt <= 1.0e-4)
    {
      dt = 0.0;
      if (i > 0)
      {
        dt += std::max(0.0, times[i] - times[i - 1]);
      }
      if (i + 1 < static_cast<int>(path.size()))
      {
        dt += std::max(0.0, times[i + 1] - times[i]);
      }
    }
    velocities[i] = dt > 1.0e-4 ? clampVelocity((path[right] - path[left]) / dt) : Vec3f::Zero();
  }

  return velocities;
}
} // namespace

ExpTrajOpt::ExpTrajOpt(const traj_opt::Config &cfg,
                       const ros_interface::RosInterface::Ptr &ros_ptr)
    : cfg_(cfg), ros_ptr_(ros_ptr)
{
  if (cfg_.save_log_en)
  {
    failed_traj_log_.open(DEBUG_FILE_DIR("exp_opt_log.csv"), std::ios::out | std::ios::trunc);
    penalty_log_.open(DEBUG_FILE_DIR("exp_opt_penna.csv"), std::ios::out | std::ios::trunc);
  }

  opt_vars_.magnitude_bounds.resize(6);
  opt_vars_.penalty_weights.resize(7);
  opt_vars_.magnitude_bounds << cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk,
      cfg_.max_omg, cfg_.min_acc_thr * cfg_.mass, cfg_.max_acc_thr * cfg_.mass;
  opt_vars_.penalty_weights << cfg_.penna_pos, cfg_.penna_vel,
      cfg_.penna_acc, cfg_.penna_jerk,
      cfg_.penna_attract, cfg_.penna_omg,
      cfg_.penna_thr;
  opt_vars_.rho = cfg_.penna_t;
  opt_vars_.pos_constraint_type = cfg_.pos_constraint_type;
  opt_vars_.block_energy_cost = cfg_.block_energy_cost;
  opt_vars_.smooth_eps = cfg_.smooth_eps;
  opt_vars_.integral_res = std::max(1, cfg_.integral_reso);
  opt_vars_.quadrotor_flatness = cfg_.quadrotot_flatness;

  linear_time_cost_.weight = opt_vars_.rho;
}

ExpTrajOpt::~ExpTrajOpt()
{
  if (failed_traj_log_.is_open())
  {
    failed_traj_log_.close();
  }
  if (penalty_log_.is_open())
  {
    penalty_log_.close();
  }
}

void ExpTrajOpt::setSwarmConfig(const SwarmPenaltyConfig &config)
{
  swarm_config_ = config;
}

void ExpTrajOpt::setSwarmTrajectories(const SwarmTrajectoriesConstPtr &trajectories)
{
  swarm_trajs_ = trajectories;
}

void ExpTrajOpt::setSwarmCurrentWallTime(double wall_time)
{
  swarm_current_wall_time_ = wall_time;
}

SnapBoundaryState ExpTrajOpt::toSnapBoundary(const StatePVAJ &state)
{
  SnapBoundaryState out;
  out.col(0) = state.col(0);
  out.col(1) = state.col(1);
  out.col(2) = state.col(2);
  out.col(3) = state.col(3);
  return out;
}

Trajectory ExpTrajOpt::toGeometryTrajectory(const SnapTraj &traj)
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

bool ExpTrajOpt::processCorridor()
{
  const int size_corridor = static_cast<int>(opt_vars_.h_polytopes.size()) - 1;
  if (size_corridor < 0)
  {
    return false;
  }

  opt_vars_.v_polytopes.clear();
  opt_vars_.v_polytopes.reserve(2 * size_corridor + 1);
  opt_vars_.waypoint_attractor.resize(3, size_corridor);
  opt_vars_.waypoint_attractor_dead_d.resize(size_corridor);
  opt_vars_.h_overlap_polytopes.resize(size_corridor);

  PolyhedronH overlap;
  PolyhedronV cur_v, cur_v_local;
  for (int i = 0; i < size_corridor; ++i)
  {
    if (!geometry_utils::enumerateVs(opt_vars_.h_polytopes[i], cur_v))
    {
      std::cout << YELLOW << " -- [ExpTrajOpt] Failed to enumerate corridor vertices." << RESET << std::endl;
      return false;
    }
    cur_v_local.resize(3, cur_v.cols());
    cur_v_local.col(0) = cur_v.col(0);
    cur_v_local.rightCols(cur_v.cols() - 1) = cur_v.rightCols(cur_v.cols() - 1).colwise() - cur_v.col(0);
    opt_vars_.v_polytopes.push_back(cur_v_local);

    overlap.resize(opt_vars_.h_polytopes[i].rows() + opt_vars_.h_polytopes[i + 1].rows(), 4);
    overlap.topRows(opt_vars_.h_polytopes[i].rows()) = opt_vars_.h_polytopes[i];
    overlap.bottomRows(opt_vars_.h_polytopes[i + 1].rows()) = opt_vars_.h_polytopes[i + 1];
    opt_vars_.h_overlap_polytopes[i] = overlap;

    Vec3f interior;
    const double dis = geometry_utils::findInteriorDist(overlap, interior) / 2.0;
    if (dis < 0.0 || std::isinf(dis))
    {
      return false;
    }
    geometry_utils::enumerateVs(overlap, interior, cur_v);
    if (!std::isfinite(cur_v.sum()))
    {
      return false;
    }
    opt_vars_.waypoint_attractor.col(i) = interior;
    opt_vars_.waypoint_attractor_dead_d(i) = dis;

    cur_v_local.resize(3, cur_v.cols());
    cur_v_local.col(0) = cur_v.col(0);
    cur_v_local.rightCols(cur_v.cols() - 1) = cur_v.rightCols(cur_v.cols() - 1).colwise() - cur_v.col(0);
    opt_vars_.v_polytopes.push_back(cur_v_local);
  }

  if (!geometry_utils::enumerateVs(opt_vars_.h_polytopes.back(), cur_v))
  {
    return false;
  }
  cur_v_local.resize(3, cur_v.cols());
  cur_v_local.col(0) = cur_v.col(0);
  cur_v_local.rightCols(cur_v.cols() - 1) = cur_v.rightCols(cur_v.cols() - 1).colwise() - cur_v.col(0);
  opt_vars_.v_polytopes.push_back(cur_v_local);
  return true;
}

bool ExpTrajOpt::processCorridorWithGuideTraj()
{
  if (!processCorridor())
  {
    return false;
  }

  VecDf time_stamps(opt_vars_.waypoint_attractor.cols() + 2);
  time_stamps(0) = 0.0;
  time_stamps(time_stamps.size() - 1) = opt_vars_.guide_t.back();
  for (int j = 0; j < opt_vars_.waypoint_attractor.cols(); ++j)
  {
    double min_dis = std::numeric_limits<double>::max();
    int min_id = 0;
    for (int i = 0; i < static_cast<int>(opt_vars_.guide_path.size()); ++i)
    {
      const double dis = (opt_vars_.guide_path[i] - opt_vars_.waypoint_attractor.col(j)).norm();
      if (dis < min_dis)
      {
        min_dis = dis;
        min_id = i;
      }
    }
    opt_vars_.points.col(j) = opt_vars_.waypoint_attractor.col(j);
    time_stamps(j + 1) = opt_vars_.guide_t[min_id];
  }

  for (int i = 1; i < time_stamps.size(); ++i)
  {
    opt_vars_.times(i - 1) = std::max(0.01, time_stamps(i) - time_stamps(i - 1));
  }
  return true;
}

void ExpTrajOpt::defaultInitialization()
{
  const VecDf dis = (opt_vars_.init_path.rightCols(opt_vars_.piece_num) -
                     opt_vars_.init_path.leftCols(opt_vars_.piece_num))
                        .colwise()
                        .norm();
  opt_vars_.times = (dis.array() / std::max(1.0e-3, cfg_.max_vel)).max(0.01);
  opt_vars_.points = opt_vars_.waypoint_attractor;
}

bool ExpTrajOpt::setupProblemAndCheck()
{
  opt_vars_.piece_num = static_cast<int>(opt_vars_.h_polytopes.size());
  if (opt_vars_.piece_num <= 0)
  {
    return false;
  }
  opt_vars_.times.resize(opt_vars_.piece_num);
  opt_vars_.points.resize(3, opt_vars_.piece_num - 1);

  const bool ok = opt_vars_.default_init ? processCorridor() : processCorridorWithGuideTraj();
  if (!ok)
  {
    return false;
  }

  opt_vars_.init_path = waypointsToMatrix(opt_vars_.head_pvaj, opt_vars_.waypoint_attractor, opt_vars_.tail_pvaj);
  if (opt_vars_.default_init)
  {
    defaultInitialization();
  }
  else
  {
    opt_vars_.times *= 0.8;
  }

  if (!opt_vars_.times.allFinite() || opt_vars_.times.minCoeff() <= 1.0e-6)
  {
    return false;
  }

  opt_vars_.v_poly_idx.resize(opt_vars_.piece_num - 1);
  opt_vars_.h_poly_idx.resize(opt_vars_.piece_num);
  for (int i = 0; i < opt_vars_.piece_num; ++i)
  {
    opt_vars_.h_poly_idx(i) = i;
    if (i < opt_vars_.piece_num - 1)
    {
      opt_vars_.v_poly_idx(i) = 2 * i + 1;
    }
  }
  return true;
}

bool ExpTrajOpt::loadCorridors(PolytopeVec &sfcs)
{
  if (sfcs.empty())
  {
    std::cout << YELLOW << " -- [ExpTrajOpt] Empty SFC." << RESET << std::endl;
    return false;
  }

  if (!geometry_utils::SimplifySFC(opt_vars_.head_pvaj.col(0), opt_vars_.tail_pvaj.col(0), sfcs))
  {
    std::cout << YELLOW << " -- [ExpTrajOpt] Cannot simplify SFC." << RESET << std::endl;
    return false;
  }

  opt_vars_.h_polytopes.resize(sfcs.size());
  for (int i = 0; i < static_cast<int>(sfcs.size()); ++i)
  {
    opt_vars_.h_polytopes[i] = sfcs[i].GetPlanes();
    normalizeHPoly(opt_vars_.h_polytopes[i]);
    if (!std::isfinite(opt_vars_.h_polytopes[i].sum()))
    {
      return false;
    }
  }
  return true;
}

double ExpTrajOpt::costFunctional(void *ptr, const VecDf &x, VecDf &g)
{
  return static_cast<ExpTrajOpt *>(ptr)->evaluateCurrentCost(x, g);
}

double ExpTrajOpt::evaluateCurrentCost(const VecDf &x, VecDf &g)
{
  opt_vars_.iter_num++;
  g.setZero();

  std::vector<double> times(static_cast<std::size_t>(opt_vars_.piece_num));
  for (int i = 0; i < opt_vars_.piece_num; ++i)
  {
    times[static_cast<std::size_t>(i)] = time_map_.toTime(x(i));
  }

  Mat3Df waypoints(3, opt_vars_.piece_num + 1);
  waypoints.col(0) = opt_vars_.head_pvaj.col(0);
  waypoints.rightCols(1) = opt_vars_.tail_pvaj.col(0);
  int offset = opt_vars_.piece_num;
  for (int i = 1; i < opt_vars_.piece_num; ++i)
  {
    const int dim = spatial_map_.getUnconstrainedDim(i);
    waypoints.col(i) = spatial_map_.toPhysical(x.segment(offset, dim), i);
    offset += dim;
  }

  VecDf durations = Eigen::Map<VecDf>(times.data(), times.size());
  minco_traj_.generate(waypoints.middleCols(1, opt_vars_.piece_num - 1),
                       toSnapBoundary(opt_vars_.head_pvaj),
                       toSnapBoundary(opt_vars_.tail_pvaj),
                       durations);

  double cost = 0.0;
  typename SnapTraj::CoeffMat gdC(minco_traj_.getCoefficients().rows(), 3);
  VecDf gdT(opt_vars_.piece_num);
  gdC.setZero();
  gdT.setZero();

  if (!opt_vars_.block_energy_cost)
  {
    double energy = 0.0;
    typename SnapTraj::CoeffMat gdC_energy;
    VecDf gdT_energy;
    minco_traj_.getEnergyPartialGradByCoeffs(energy, gdC_energy);
    minco_traj_.getEnergyPartialGradByTimes(gdT_energy);
    cost += energy;
    gdC += gdC_energy;
    gdT += gdT_energy;
    opt_vars_.penalty_log(0) = energy;
  }

  exp_cost_manager_.reset(&opt_vars_.h_polytopes,
                          &opt_vars_.h_poly_idx,
                          &opt_vars_.waypoint_attractor,
                          &opt_vars_.waypoint_attractor_dead_d,
                          opt_vars_.smooth_eps,
                          opt_vars_.magnitude_bounds,
                          opt_vars_.penalty_weights,
                          &opt_vars_.quadrotor_flatness,
                          swarm_config_,
                          swarm_trajs_,
                          swarm_current_wall_time_);
  exp_cost_manager_.beginEvaluation(&times);

  const auto &coeffs = minco_traj_.getCoefficients();
  double seg_start = 0.0;
  for (int i = 0; i < opt_vars_.piece_num; ++i)
  {
    const double T = durations(i);
    const double inv_K = 1.0 / static_cast<double>(opt_vars_.integral_res);
    const double step = T * inv_K;
    const int base = i * SnapTraj::COEFF_NUM;
    const auto coeff_block = coeffs.template block<SnapTraj::COEFF_NUM, TRAJ_DIM>(base, 0);

    for (int k = 0; k <= opt_vars_.integral_res; ++k)
    {
      const double alpha = static_cast<double>(k) * inv_K;
      const double t = alpha * T;
      const double node = (k == 0 || k == opt_vars_.integral_res) ? 0.5 : 1.0;
      const double common = node * step;

      SnapTraj::BasisRow bp, bv, ba, bj, bs;
      SnapTraj::computeBasisFunctions(t, bp, bv, ba, bj, bs);
      Vec3f p = coeff_block.transpose() * bp.transpose();
      Vec3f v = coeff_block.transpose() * bv.transpose();
      Vec3f a = coeff_block.transpose() * ba.transpose();
      Vec3f j = coeff_block.transpose() * bj.transpose();
      Vec3f s = coeff_block.transpose() * bs.transpose();
      Vec3f gp = Vec3f::Zero();
      Vec3f gv = Vec3f::Zero();
      Vec3f ga = Vec3f::Zero();
      Vec3f gj = Vec3f::Zero();
      Vec3f gs = Vec3f::Zero();
      double gt = 0.0;

      const double sample_cost = exp_cost_manager_(t, seg_start + t, i, k, p, v, a, j, s, gp, gv, ga, gj, gs, gt);
      cost += common * sample_cost;
      gdC.template block<SnapTraj::COEFF_NUM, TRAJ_DIM>(base, 0).noalias() +=
          (bp.transpose() * gp.transpose() +
           bv.transpose() * gv.transpose() +
           ba.transpose() * ga.transpose() +
           bj.transpose() * gj.transpose()) * common;
      gdT(i) += node * inv_K * sample_cost;
      gdT(i) += (gp.dot(v) + gv.dot(a) + ga.dot(j) + gj.dot(s)) * alpha * common;
      if (std::abs(gt) > 1.0e-12)
      {
        if (i > 0)
        {
          gdT.head(i).array() += gt * common;
        }
        gdT(i) += gt * alpha * common;
      }
    }
    seg_start += T;
  }

  cost += linear_time_cost_(times, gdT);
  const auto grad_result = minco_traj_.propagateGradFull(gdC, gdT);

  for (int i = 0; i < opt_vars_.piece_num; ++i)
  {
    g(i) += time_map_.backward(x(i), durations(i), grad_result.grad_by_times(i));
  }

  offset = opt_vars_.piece_num;
  for (int i = 1; i < opt_vars_.piece_num; ++i)
  {
    const int dim = spatial_map_.getUnconstrainedDim(i);
    VecDf grad_xi = spatial_map_.backwardGrad(x.segment(offset, dim), grad_result.grad_by_points.col(i - 1), i);
    spatial_map_.addNormPenalty(x.segment(offset, dim), cost, grad_xi);
    g.segment(offset, dim) += grad_xi;
    offset += dim;
  }

  opt_vars_.penalty_log.tail(7) = exp_cost_manager_.getPenaltyLog().tail(7);
  return cost;
}

double ExpTrajOpt::optimize(Trajectory &traj, double rel_cost_tol)
{
  opt_vars_.penalty_log.resize(8);
  opt_vars_.penalty_log.setZero();

  if (opt_vars_.given_init_ts_and_ps)
  {
    opt_vars_.times = opt_vars_.init_ts;
    for (int i = 0; i < static_cast<int>(opt_vars_.init_ps.size()); ++i)
    {
      opt_vars_.points.col(i) = opt_vars_.init_ps[i];
    }
  }

  if (!opt_vars_.times.allFinite() || opt_vars_.times.minCoeff() < 1.0e-3)
  {
    return INFINITY;
  }

  spatial_map_.reset(&opt_vars_.v_polytopes,
                     &opt_vars_.v_poly_idx,
                     opt_vars_.piece_num - 1,
                     opt_vars_.pos_constraint_type == 1);

  const Mat3Df waypoints = waypointsToMatrix(opt_vars_.head_pvaj, opt_vars_.points, opt_vars_.tail_pvaj);
  VecDf x(opt_vars_.piece_num + [&]() {
    int dim = 0;
    for (int i = 1; i < opt_vars_.piece_num; ++i)
    {
      dim += spatial_map_.getUnconstrainedDim(i);
    }
    return dim;
  }());

  for (int i = 0; i < opt_vars_.piece_num; ++i)
  {
    x(i) = time_map_.toTau(opt_vars_.times(i));
  }
  int offset = opt_vars_.piece_num;
  for (int i = 1; i < opt_vars_.piece_num; ++i)
  {
    const VecDf xi = spatial_map_.toUnconstrained(waypoints.col(i), i);
    x.segment(offset, xi.size()) = xi;
    offset += xi.size();
  }

  opt_vars_.init_ts = opt_vars_.times;
  opt_vars_.init_ps.clear();
  for (int col = 0; col < opt_vars_.points.cols(); ++col)
  {
    opt_vars_.init_ps.emplace_back(opt_vars_.points.col(col));
  }
  for (int i = 0; i < opt_vars_.waypoint_attractor_dead_d.size(); ++i)
  {
    truncateToSixDecimals(opt_vars_.waypoint_attractor_dead_d(i));
    truncateToSixDecimals(opt_vars_.waypoint_attractor(0, i));
    truncateToSixDecimals(opt_vars_.waypoint_attractor(1, i));
    truncateToSixDecimals(opt_vars_.waypoint_attractor(2, i));
  }

  opt_vars_.iter_num = 0;
  double min_cost = 0.0;
  lbfgs::lbfgs_parameter_t params;
  params.mem_size = 256;
  params.past = 3;
  params.min_step = 1.0e-32;
  params.g_epsilon = 0.0;
  params.delta = rel_cost_tol;

  const int ret = lbfgs::lbfgs_optimize(x, min_cost, &ExpTrajOpt::costFunctional, nullptr, nullptr, this, params);

  if (cfg_.print_optimizer_log)
  {
    std::cout << " -- [ExpTrajOpt] Opt finish, iter: " << opt_vars_.iter_num << "\n"
              << "\tEnergy: " << opt_vars_.penalty_log(0) << "\n"
              << "\tPos: " << opt_vars_.penalty_log(1) << "\n"
              << "\tVel: " << opt_vars_.penalty_log(2) << "\n"
              << "\tAcc: " << opt_vars_.penalty_log(3) << "\n"
              << "\tJerk: " << opt_vars_.penalty_log(4) << "\n"
              << "\tAttract: " << opt_vars_.penalty_log(5) << "\n"
              << "\tOmg: " << opt_vars_.penalty_log(6) << "\n"
              << "\tThr: " << opt_vars_.penalty_log(7) << std::endl;
  }

  if (ret < 0)
  {
    traj.clear();
    std::cout << YELLOW << " -- [ExpTrajOpt] Optimization failed: " << lbfgs::lbfgs_strerror(ret) << RESET << std::endl;
    return INFINITY;
  }

  VecDf grad = VecDf::Zero(x.size());
  min_cost = evaluateCurrentCost(x, grad);

  traj = toGeometryTrajectory(minco_traj_);
  return min_cost;
}

bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ,
                          const StatePVAJ &tailPVAJ,
                          PolytopeVec &sfcs,
                          Trajectory &out_traj)
{
  opt_vars_.default_init = true;
  opt_vars_.given_init_ts_and_ps = false;
  opt_vars_.head_pvaj = headPVAJ;
  opt_vars_.tail_pvaj = tailPVAJ;
  opt_vars_.guide_path.clear();
  opt_vars_.guide_t.clear();
  if (!loadCorridors(sfcs) || !setupProblemAndCheck())
  {
    return false;
  }
  out_traj.clear();
  const bool success = !std::isinf(optimize(out_traj, cfg_.opt_accuracy));
  if (success)
  {
    out_traj.start_WT = ros_ptr_->getSimTime();
  }
  return success;
}

bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ,
                          const StatePVAJ &tailPVAJ,
                          const vec_E<Vec3f> &guide_path,
                          const std::vector<double> &guide_t,
                          PolytopeVec &sfcs,
                          Trajectory &out_traj)
{
  if (guide_path.size() != guide_t.size() || guide_path.empty())
  {
    return false;
  }
  opt_vars_.default_init = false;
  opt_vars_.given_init_ts_and_ps = false;
  opt_vars_.head_pvaj = headPVAJ;
  opt_vars_.tail_pvaj = tailPVAJ;
  opt_vars_.guide_path = guide_path;
  opt_vars_.guide_t = guide_t;
  if (!loadCorridors(sfcs) || !setupProblemAndCheck())
  {
    return false;
  }
  out_traj.clear();
  const bool success = !std::isinf(optimize(out_traj, cfg_.opt_accuracy));
  if (success)
  {
    out_traj.start_WT = ros_ptr_->getSimTime();
  }
  if (penalty_log_.is_open())
  {
    penalty_log_ << opt_vars_.penalty_log.transpose() << std::endl;
  }
  return success;
}

bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ,
                          const StatePVAJ &tailPVAJ,
                          PolytopeVec &sfcs,
                          const vec_Vec3f &init_ps,
                          const VecDf &init_ts,
                          Trajectory &out_traj)
{
  vec_E<Vec3f> guide_path;
  std::vector<double> guide_t;
  guide_path.emplace_back(headPVAJ.col(0));
  guide_t.emplace_back(0.0);
  double acc_t = 0.0;
  for (int i = 0; i < init_ts.size(); ++i)
  {
    if (i < static_cast<int>(init_ps.size()))
    {
      guide_path.emplace_back(init_ps[i]);
    }
    acc_t += init_ts(i);
    guide_t.emplace_back(acc_t);
  }
  if (guide_path.size() < guide_t.size())
  {
    guide_path.emplace_back(tailPVAJ.col(0));
  }

  opt_vars_.init_ts = init_ts;
  opt_vars_.init_ps = init_ps;
  opt_vars_.given_init_ts_and_ps = true;
  const bool success = optimize(headPVAJ, tailPVAJ, guide_path, guide_t, sfcs, out_traj);
  opt_vars_.given_init_ts_and_ps = false;
  return success;
}

BackupTrajOpt::BackupTrajOpt(const traj_opt::Config &cfg,
                             const ros_interface::RosInterface::Ptr &ros_ptr)
    : cfg_(cfg), ros_ptr_(ros_ptr)
{
  if (cfg_.save_log_en)
  {
    failed_traj_log_.open(DEBUG_FILE_DIR("back_opt_log.csv"), std::ios::out | std::ios::trunc);
    penalty_log_.open(DEBUG_FILE_DIR("back_opt_penna.csv"), std::ios::out | std::ios::trunc);
  }

  opt_vars_.magnitude_bounds.resize(6);
  opt_vars_.penalty_weights.resize(7);
  opt_vars_.magnitude_bounds << cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk,
      cfg_.max_omg, cfg_.min_acc_thr * cfg_.mass, cfg_.max_acc_thr * cfg_.mass;
  opt_vars_.penalty_weights << cfg_.penna_pos, cfg_.penna_vel,
      cfg_.penna_acc, cfg_.penna_jerk,
      cfg_.penna_attract, cfg_.penna_omg,
      cfg_.penna_thr;
  opt_vars_.rho = cfg_.penna_t;
  opt_vars_.weight_ts = cfg_.penna_ts;
  opt_vars_.pos_constraint_type = cfg_.pos_constraint_type;
  opt_vars_.block_energy_cost = cfg_.block_energy_cost;
  opt_vars_.uniform_time_en = cfg_.uniform_time_en;
  opt_vars_.smooth_eps = cfg_.smooth_eps;
  opt_vars_.integral_res = std::max(1, cfg_.integral_reso);
  opt_vars_.quadrotor_flatness = cfg_.quadrotot_flatness;
  opt_vars_.piece_num = std::max(1, cfg_.piece_num);
}

BackupTrajOpt::~BackupTrajOpt()
{
  if (failed_traj_log_.is_open())
  {
    failed_traj_log_.close();
  }
  if (penalty_log_.is_open())
  {
    penalty_log_.close();
  }
}

SnapBoundaryState BackupTrajOpt::toSnapBoundary(const StatePVAJ &state)
{
  SnapBoundaryState out;
  out.col(0) = state.col(0);
  out.col(1) = state.col(1);
  out.col(2) = state.col(2);
  out.col(3) = state.col(3);
  return out;
}

Trajectory BackupTrajOpt::toGeometryTrajectory(const SnapTraj &traj)
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

double BackupTrajOpt::logisticInterval(double lo, double hi, double eta)
{
  const double sigma = 1.0 / (1.0 + std::exp(-eta));
  return lo + (hi - lo) * sigma;
}

double BackupTrajOpt::logisticIntervalGrad(double lo, double hi, double eta)
{
  const double sigma = 1.0 / (1.0 + std::exp(-eta));
  return (hi - lo) * sigma * (1.0 - sigma);
}

bool BackupTrajOpt::processCorridor()
{
  PolyhedronV cur_v;
  if (!geometry_utils::enumerateVs(opt_vars_.h_polytope, cur_v))
  {
    return false;
  }
  opt_vars_.v_polytope.resize(3, cur_v.cols());
  opt_vars_.v_polytope.col(0) = cur_v.col(0);
  opt_vars_.v_polytope.rightCols(cur_v.cols() - 1) = cur_v.rightCols(cur_v.cols() - 1).colwise() - cur_v.col(0);
  return true;
}

bool BackupTrajOpt::setupProblemAndCheck()
{
  normalizeHPoly(opt_vars_.h_polytope);
  if (!processCorridor())
  {
    return false;
  }
  opt_vars_.points.resize(3, opt_vars_.piece_num);
  opt_vars_.times.resize(opt_vars_.piece_num);
  return true;
}

VecDf BackupTrajOpt::encodeDecisionVector() const
{
  spatial_map::PolytopeSpatialMap poly_map;
  poly_map.reset(&opt_vars_.v_polytope,
                 opt_vars_.piece_num,
                 opt_vars_.pos_constraint_type == 1);
  const int time_dim = opt_vars_.uniform_time_en ? 1 : opt_vars_.piece_num;
  int spatial_dim = 0;
  for (int i = 1; i <= opt_vars_.piece_num; ++i)
  {
    spatial_dim += poly_map.getUnconstrainedDim(i);
  }

  VecDf x(time_dim + spatial_dim + 1);
  if (opt_vars_.uniform_time_en)
  {
    x(0) = time_map_.toTau(opt_vars_.times.sum());
  }
  else
  {
    for (int i = 0; i < opt_vars_.piece_num; ++i)
    {
      x(i) = time_map_.toTau(opt_vars_.times(i));
    }
  }

  int offset = time_dim;
  for (int i = 1; i <= opt_vars_.piece_num; ++i)
  {
    const VecDf xi = poly_map.toUnconstrained(opt_vars_.points.col(i - 1), i);
    x.segment(offset, xi.size()) = xi;
    offset += xi.size();
  }

  const double span = std::max(1.0e-6, opt_vars_.max_ts - opt_vars_.min_ts);
  const double ratio = std::clamp((opt_vars_.ts - opt_vars_.min_ts) / span, 1.0e-6, 1.0 - 1.0e-6);
  x(offset) = std::log(ratio / (1.0 - ratio));
  return x;
}

void BackupTrajOpt::decodeDecisionVector(const VecDf &x, VecDf &times, Mat3Df &points, double &ts) const
{
  spatial_map::PolytopeSpatialMap poly_map;
  poly_map.reset(&opt_vars_.v_polytope,
                 opt_vars_.piece_num,
                 opt_vars_.pos_constraint_type == 1);
  const int time_dim = opt_vars_.uniform_time_en ? 1 : opt_vars_.piece_num;
  times.resize(opt_vars_.piece_num);
  if (opt_vars_.uniform_time_en)
  {
    times.setConstant(time_map_.toTime(x(0)) / static_cast<double>(opt_vars_.piece_num));
  }
  else
  {
    for (int i = 0; i < opt_vars_.piece_num; ++i)
    {
      times(i) = time_map_.toTime(x(i));
    }
  }

  points.resize(3, opt_vars_.piece_num);
  int offset = time_dim;
  for (int i = 1; i <= opt_vars_.piece_num; ++i)
  {
    const int dim = poly_map.getUnconstrainedDim(i);
    points.col(i - 1) = poly_map.toPhysical(x.segment(offset, dim), i);
    offset += dim;
  }
  ts = logisticInterval(opt_vars_.min_ts, opt_vars_.max_ts, x(offset));
}

double BackupTrajOpt::costFunctional(void *ptr, const VecDf &x, VecDf &g)
{
  return static_cast<BackupTrajOpt *>(ptr)->evaluateCurrentCost(x, g);
}

double BackupTrajOpt::evaluateCurrentCost(const VecDf &x, VecDf &g)
{
  opt_vars_.iter_num++;
  g.setZero();

  const int time_dim = opt_vars_.uniform_time_en ? 1 : opt_vars_.piece_num;
  spatial_map_.reset(&opt_vars_.v_polytope,
                     opt_vars_.piece_num,
                     opt_vars_.pos_constraint_type == 1);

  VecDf times;
  Mat3Df points;
  double ts = 0.0;
  decodeDecisionVector(x, times, points, ts);

  StatePVAJ head;
  opt_vars_.exp_traj.getState(ts, head);
  StatePVAJ tail = StatePVAJ::Zero();
  tail.col(0) = points.rightCols(1);

  minco_traj_.generate(points.leftCols(opt_vars_.piece_num - 1),
                       toSnapBoundary(head),
                       toSnapBoundary(tail),
                       times);

  double cost = 0.0;
  typename SnapTraj::CoeffMat gdC(minco_traj_.getCoefficients().rows(), 3);
  VecDf gdT(opt_vars_.piece_num);
  gdC.setZero();
  gdT.setZero();
  opt_vars_.penalty_log.setZero();

  if (!opt_vars_.block_energy_cost)
  {
    double energy = 0.0;
    typename SnapTraj::CoeffMat gdC_energy;
    VecDf gdT_energy;
    minco_traj_.getEnergyPartialGradByCoeffs(energy, gdC_energy);
    minco_traj_.getEnergyPartialGradByTimes(gdT_energy);
    cost += energy;
    gdC += gdC_energy;
    gdT += gdT_energy;
    opt_vars_.penalty_log(0) = energy;
  }

  backup_cost_manager_.reset(&opt_vars_.h_polytope,
                             opt_vars_.smooth_eps,
                             opt_vars_.magnitude_bounds,
                             opt_vars_.penalty_weights,
                             &opt_vars_.quadrotor_flatness);
  backup_cost_manager_.beginEvaluation();

  const auto &coeffs = minco_traj_.getCoefficients();
  for (int i = 0; i < opt_vars_.piece_num; ++i)
  {
    const double T = times(i);
    const double inv_K = 1.0 / static_cast<double>(opt_vars_.integral_res);
    const double step = T * inv_K;
    const int base = i * SnapTraj::COEFF_NUM;
    const auto coeff_block = coeffs.template block<SnapTraj::COEFF_NUM, TRAJ_DIM>(base, 0);
    for (int k = 0; k <= opt_vars_.integral_res; ++k)
    {
      const double alpha = static_cast<double>(k) * inv_K;
      const double t = alpha * T;
      const double node = (k == 0 || k == opt_vars_.integral_res) ? 0.5 : 1.0;
      const double common = node * step;
      SnapTraj::BasisRow bp, bv, ba, bj, bs;
      SnapTraj::computeBasisFunctions(t, bp, bv, ba, bj, bs);
      Vec3f p = coeff_block.transpose() * bp.transpose();
      Vec3f v = coeff_block.transpose() * bv.transpose();
      Vec3f a = coeff_block.transpose() * ba.transpose();
      Vec3f j = coeff_block.transpose() * bj.transpose();
      Vec3f s = coeff_block.transpose() * bs.transpose();
      Vec3f gp = Vec3f::Zero();
      Vec3f gv = Vec3f::Zero();
      Vec3f ga = Vec3f::Zero();
      Vec3f gj = Vec3f::Zero();
      Vec3f gs = Vec3f::Zero();
      double gt = 0.0;
      const double sample_cost = backup_cost_manager_(t, t, i, k, p, v, a, j, s, gp, gv, ga, gj, gs, gt);
      cost += common * sample_cost;
      gdC.template block<SnapTraj::COEFF_NUM, TRAJ_DIM>(base, 0).noalias() +=
          (bp.transpose() * gp.transpose() +
           bv.transpose() * gv.transpose() +
           ba.transpose() * ga.transpose() +
           bj.transpose() * gj.transpose()) * common;
      gdT(i) += node * inv_K * sample_cost;
      gdT(i) += (gp.dot(v) + gv.dot(a) + ga.dot(j) + gj.dot(s)) * alpha * common;
    }
  }

  gdT.array() += opt_vars_.rho;
  cost += opt_vars_.rho * times.sum();
  cost += opt_vars_.weight_ts * (opt_vars_.max_ts - ts);

  const auto grad_result = minco_traj_.propagateGradFull(gdC, gdT);

  if (opt_vars_.uniform_time_en)
  {
    g(0) += time_map_.backward(x(0), times.sum(), grad_result.grad_by_times.sum() / static_cast<double>(opt_vars_.piece_num));
  }
  else
  {
    for (int i = 0; i < opt_vars_.piece_num; ++i)
    {
      g(i) += time_map_.backward(x(i), times(i), grad_result.grad_by_times(i));
    }
  }

  int offset = time_dim;
  for (int i = 1; i <= opt_vars_.piece_num; ++i)
  {
    const int dim = spatial_map_.getUnconstrainedDim(i);
    Vec3f grad_p;
    if (i == opt_vars_.piece_num)
    {
      grad_p = grad_result.grad_by_tail_state.col(0);
    }
    else
    {
      grad_p = grad_result.grad_by_points.col(i - 1);
    }
    VecDf grad_xi = spatial_map_.backwardGrad(x.segment(offset, dim), grad_p, i);
    spatial_map_.addNormPenalty(x.segment(offset, dim), cost, grad_xi);
    g.segment(offset, dim) += grad_xi;
    offset += dim;
  }

  StatePVAJ exp_state_grad = grad_result.grad_by_head_state;
  const double grad_ts_from_state = exp_state_grad.col(0).dot(opt_vars_.exp_traj.getVel(ts)) +
                                    exp_state_grad.col(1).dot(opt_vars_.exp_traj.getAcc(ts)) +
                                    exp_state_grad.col(2).dot(opt_vars_.exp_traj.getJer(ts)) +
                                    exp_state_grad.col(3).dot(opt_vars_.exp_traj.getSnap(ts)) -
                                    opt_vars_.weight_ts;
  g(offset) += grad_ts_from_state * logisticIntervalGrad(opt_vars_.min_ts, opt_vars_.max_ts, x(offset));

  opt_vars_.penalty_log.tail(7) = backup_cost_manager_.getPenaltyLog().tail(7);
  opt_vars_.ts = ts;
  opt_vars_.times = times;
  opt_vars_.points = points;
  return cost;
}

double BackupTrajOpt::optimize(Trajectory &traj, double rel_cost_tol)
{
  Vec3f step = (opt_vars_.tail_pvaj.col(0) - opt_vars_.head_pvaj.col(0)) / static_cast<double>(opt_vars_.piece_num);
  for (int i = 0; i < opt_vars_.piece_num - 1; ++i)
  {
    opt_vars_.points.col(i) = opt_vars_.head_pvaj.col(0) + step * static_cast<double>(i + 1);
  }
  opt_vars_.points.rightCols(1) = opt_vars_.tail_pvaj.col(0);

  if (opt_vars_.given_init_ts_and_ps)
  {
    opt_vars_.times = opt_vars_.given_init_t_vec;
    for (int i = 0; i < static_cast<int>(opt_vars_.given_init_ps.size()); ++i)
    {
      opt_vars_.points.col(i) = opt_vars_.given_init_ps[i];
    }
    opt_vars_.ts = opt_vars_.given_init_ts;
  }

  opt_vars_.init_ts = opt_vars_.ts;
  opt_vars_.init_t_vec = opt_vars_.times;
  opt_vars_.init_ps.clear();
  for (int i = 0; i < opt_vars_.points.cols(); ++i)
  {
    opt_vars_.init_ps.emplace_back(opt_vars_.points.col(i));
  }

  VecDf x = encodeDecisionVector();
  opt_vars_.penalty_log.resize(8);
  opt_vars_.penalty_log.setZero();
  opt_vars_.iter_num = 0;

  double min_cost = 0.0;
  lbfgs::lbfgs_parameter_t params;
  params.mem_size = 256;
  params.past = 3;
  params.min_step = 1.0e-32;
  params.g_epsilon = 0.0;
  params.delta = rel_cost_tol;
  const int ret = lbfgs::lbfgs_optimize(x, min_cost, &BackupTrajOpt::costFunctional, nullptr, nullptr, this, params);

  if (ret < 0)
  {
    traj.clear();
    std::cout << YELLOW << " -- [BackupTrajOpt] Optimization failed: " << lbfgs::lbfgs_strerror(ret) << RESET << std::endl;
    return INFINITY;
  }

  VecDf grad = VecDf::Zero(x.size());
  min_cost = evaluateCurrentCost(x, grad);
  traj = toGeometryTrajectory(minco_traj_);
  return min_cost;
}

bool BackupTrajOpt::checkTrajMagnitudeBound(Trajectory &out_traj)
{
  if (out_traj.empty())
  {
    return false;
  }
  if (cfg_.penna_vel > 0 && out_traj.getMaxVelRate() > 1.2 * cfg_.max_vel)
  {
    return false;
  }
  if (cfg_.penna_acc > 0 && out_traj.getMaxAccRate() > 1.2 * cfg_.max_acc)
  {
    return false;
  }
  return true;
}

bool BackupTrajOpt::optimize(const Trajectory &exp_traj,
                             const double &t_0,
                             const double &t_e,
                             const double &heu_ts,
                             const VecDf &heu_end_pt,
                             double &heu_dur,
                             const Polytope &sfc,
                             Trajectory &out_traj,
                             double &out_ts,
                             const bool &debug)
{
  (void)debug;
  opt_vars_.h_polytope = sfc.GetPlanes();
  if (!std::isfinite(opt_vars_.h_polytope.sum()))
  {
    return false;
  }
  opt_vars_.given_init_ts_and_ps = false;
  opt_vars_.exp_traj = exp_traj;
  opt_vars_.min_ts = t_0;
  opt_vars_.max_ts = t_e;
  opt_vars_.ts = std::clamp(heu_ts, t_0 + 1.0e-4, t_e - 1.0e-4);
  opt_vars_.head_pvaj = exp_traj.getState(opt_vars_.ts);
  opt_vars_.tail_pvaj.setZero();
  opt_vars_.tail_pvaj.col(0) = heu_end_pt;
  opt_vars_.times.resize(opt_vars_.piece_num);
  opt_vars_.times.setConstant(std::max(1.0e-3, heu_dur / static_cast<double>(opt_vars_.piece_num)));

  if (!setupProblemAndCheck())
  {
    return false;
  }
  out_traj.clear();
  bool success = !std::isinf(optimize(out_traj, cfg_.opt_accuracy));
  out_ts = opt_vars_.ts;
  success = success && checkTrajMagnitudeBound(out_traj);
  if (success)
  {
    heu_dur = out_traj.getTotalDuration();
  }
  return success;
}

bool BackupTrajOpt::optimize(const Trajectory &exp_traj,
                             const double &t_0,
                             const double &t_e,
                             const double &heu_ts,
                             const Polytope &sfc,
                             const VecDf &init_t_vec,
                             const vec_Vec3f &init_ps,
                             Trajectory &out_traj,
                             double &out_ts)
{
  if (init_ps.empty() || init_t_vec.size() == 0)
  {
    return false;
  }
  double heu_dur = init_t_vec.sum();
  VecDf heu_end_pt = init_ps.back();
  opt_vars_.given_init_ts_and_ps = true;
  opt_vars_.given_init_ts = heu_ts;
  opt_vars_.given_init_t_vec = init_t_vec;
  opt_vars_.given_init_ps = init_ps;
  return optimize(exp_traj, t_0, t_e, heu_ts, heu_end_pt, heu_dur, sfc, out_traj, out_ts, false);
}

YawTrajOpt::YawTrajOpt(const double &yaw_dot_max) : yaw_dot_max_(yaw_dot_max)
{
}

void YawTrajOpt::getYawTimeAllocation(const double &duration, VecDf &times) const
{
  const double interp_dt = M_PI / std::max(1.0e-3, yaw_dot_max_);
  if (duration < interp_dt * 2.0)
  {
    times.resize(1);
    times[0] = duration;
  }
  else
  {
    const int interp_num = std::max(1, static_cast<int>(std::ceil((duration - 2.0 * interp_dt) / interp_dt)));
    const double interp_t = (duration - 2.0 * interp_dt) / static_cast<double>(interp_num);
    times.resize(2 + interp_num);
    times(0) = interp_dt;
    times(times.size() - 1) = interp_dt;
    for (int i = 0; i < interp_num; ++i)
    {
      times(i + 1) = interp_t;
    }
  }
  if (times.size() == 3 && times(1) < times(0) / 3.0)
  {
    times.resize(2);
    times.setConstant(duration / 2.0);
  }
}

void YawTrajOpt::getYawWaypointAllocation(const Vec4f &init_state,
                                          Vec4f &goal_state,
                                          VecDf &way_pts,
                                          VecDf &times,
                                          const Trajectory &pos_traj,
                                          const double yaw_dot_max)
{
  double eval_t = 0.0;
  double last_yaw = init_state(0);
  way_pts.resize(std::max(0, static_cast<int>(times.size()) - 1));
  const double pos_traj_duration = pos_traj.getTotalDuration();
  const double yaw_rate_limit = std::max(0.1, yaw_dot_max);
  constexpr double kLookBack = 0.25;
  constexpr double kMinLookAhead = 0.75;
  constexpr double kMaxLookAhead = 1.50;
  constexpr double kMinHeadingDisplacement = 0.25;
  for (int i = 0; i < way_pts.size(); ++i)
  {
    eval_t += times(i);
    double cur_yaw = last_yaw;
    const double lookahead =
        std::min(kMaxLookAhead, std::max(kMinLookAhead, static_cast<double>(times(i))));
    double t0 = std::max(0.0, eval_t - kLookBack);
    double t1 = std::min(pos_traj_duration, eval_t + lookahead);
    if (eval_t + lookahead >= pos_traj_duration)
    {
      t0 = std::max(0.0, pos_traj_duration - lookahead);
      t1 = pos_traj_duration;
    }
    if (t1 - t0 < 0.2 && pos_traj_duration > 0.2)
    {
      t0 = std::max(0.0, eval_t - 0.5 * lookahead);
      t1 = std::min(pos_traj_duration, t0 + 0.2);
    }

    const Vec3f pt_i = pos_traj.getPos(t0);
    const Vec3f pt_g = pos_traj.getPos(t1);
    const Vec3f dir = pt_g - pt_i;
    if (std::hypot(dir.x(), dir.y()) > kMinHeadingDisplacement)
    {
      cur_yaw = std::atan2(dir.y(), dir.x());
      cur_yaw = normalizeYawNear(last_yaw, cur_yaw);
      const double max_delta =
          std::max(0.25, yaw_rate_limit * std::max(0.05, static_cast<double>(times(i))) * 0.85);
      cur_yaw = clampYawStep(last_yaw, cur_yaw, max_delta);
    }
    way_pts(i) = cur_yaw;
    last_yaw = cur_yaw;
  }

  if (way_pts.size() == 0)
  {
    goal_state[0] = normalizeYawNear(init_state[0], goal_state[0]);
  }
  else
  {
    goal_state[0] = normalizeYawNear(way_pts(way_pts.size() - 1), goal_state[0]);
  }
}

YawBoundaryState YawTrajOpt::toBoundaryState(const Vec4f &state)
{
  YawBoundaryState out;
  out(0, 0) = state(0);
  out(0, 1) = state(1);
  return out;
}

Trajectory YawTrajOpt::toGeometryTrajectory(const YawTraj &traj)
{
  Trajectory out;
  const auto &durations = traj.getDurations();
  out.reserve(static_cast<int>(durations.size()));
  for (int i = 0; i < durations.size(); ++i)
  {
    const auto coeff = traj.getPieceCoeffMat(i);
    Eigen::MatrixXd piece_coeff = Eigen::MatrixXd::Zero(3, SNAP_TRAJ_ORDER + 1);
    piece_coeff.row(0).tail(YawTraj::COEFF_NUM) = coeff;
    out.emplace_back(durations(i), piece_coeff);
  }
  return out;
}

bool YawTrajOpt::optimize(const Vec4f &istate_in,
                          const Vec4f &gstate_in,
                          const Trajectory &pos_traj,
                          Trajectory &out_traj,
                          const int &order,
                          const bool &free_start,
                          const bool &free_goal)
{
  if (order != 3)
  {
    std::cout << YELLOW << " -- [YawTrajOpt] Generic MINCO yaw currently supports cubic yaw only." << RESET << std::endl;
  }

  free_goal_ = free_goal;
  Vec4f init_state = istate_in;
  Vec4f goal_state = gstate_in;
  const double pos_traj_dur = pos_traj.getTotalDuration();

  if (free_start)
  {
    Vec3f pt_i = pos_traj.getPos(0.0);
    double t_g = pos_traj_dur > 0.5 ? 0.5 : pos_traj_dur;
    Vec3f pt_g = pos_traj.getPos(t_g);
    Vec3f dir = pt_g - pt_i;
    while (dir.norm() < 0.5 && t_g < pos_traj_dur)
    {
      t_g += 0.1;
      pt_g = pos_traj.getPos(t_g);
      dir = pt_g - pt_i;
    }
    init_state(0) = std::atan2(dir.y(), dir.x());
  }

  if (free_goal_)
  {
    Vec3f pt_g = pos_traj.getPos(pos_traj_dur);
    double t_i = pos_traj_dur - 0.5 > 0.0 ? pos_traj_dur - 0.5 : 0.0;
    Vec3f pt_i = pos_traj.getPos(t_i);
    Vec3f dir = pt_g - pt_i;
    while (dir.norm() < 0.5 && t_i > 0.0)
    {
      t_i -= 0.1;
      pt_i = pos_traj.getPos(t_i);
      dir = pt_g - pt_i;
    }
    goal_state(0) = std::atan2(dir.y(), dir.x());
  }

  VecDf times;
  getYawTimeAllocation(pos_traj_dur, times);
  VecDf way_pts;
  getYawWaypointAllocation(init_state, goal_state, way_pts, times, pos_traj, yaw_dot_max_);

  Eigen::Matrix<double, 1, Eigen::Dynamic> inner(1, way_pts.size());
  for (int i = 0; i < way_pts.size(); ++i)
  {
    inner(0, i) = way_pts(i);
  }

  YawTraj yaw_traj;
  if (!yaw_traj.generate(inner, toBoundaryState(init_state), toBoundaryState(goal_state), times))
  {
    return false;
  }

  out_traj = toGeometryTrajectory(yaw_traj);
  out_traj.start_WT = pos_traj.start_WT;

  const double max_yaw_rate = out_traj.getMaxVelRate();
  if (max_yaw_rate > yaw_dot_max_ + 2.0)
  {
    std::cout << YELLOW << " -- [YawTrajOpt] Yaw rate too large, " << max_yaw_rate << RESET << std::endl;
  }
  return true;
}

ESDFTrajOpt::ESDFTrajOpt(const traj_opt::Config &cfg,
                         const ros_interface::RosInterface::Ptr &ros_ptr)
    : cfg_(cfg), ros_ptr_(ros_ptr)
{
  opt_vars_.rho = cfg_.penna_t;
  opt_vars_.block_energy_cost = cfg_.block_energy_cost;
  opt_vars_.smooth_eps = cfg_.smooth_eps;
  opt_vars_.integral_res = std::max(1, cfg_.integral_reso);
  opt_vars_.weight_esdf = cfg_.penna_pos > 0.0 ? cfg_.penna_pos : 1.0;
  opt_vars_.weight_guide = std::max(0.0, cfg_.penna_attract);
  opt_vars_.weight_guide_integral = opt_vars_.weight_guide;
  opt_vars_.weight_guide_vel_integral = std::max(0.0, cfg_.penna_guide_vel);
  opt_vars_.magnitude_bounds.resize(6);
  opt_vars_.penalty_weights.resize(7);
  opt_vars_.magnitude_bounds << cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk,
      cfg_.max_omg, cfg_.min_acc_thr * cfg_.mass, cfg_.max_acc_thr * cfg_.mass;
  opt_vars_.penalty_weights << 0.0,
      std::max(0.0, cfg_.penna_vel),
      std::max(0.0, cfg_.penna_acc),
      std::max(0.0, cfg_.penna_jerk),
      0.0,
      std::max(0.0, cfg_.penna_omg),
      std::max(0.0, cfg_.penna_thr);
  opt_vars_.penalty_log = VecDf::Zero(8);
  opt_vars_.quadrotor_flatness = cfg_.quadrotot_flatness;
  linear_time_cost_.weight = opt_vars_.rho;

  if (cfg_.save_log_en)
  {
    esdf_debug_log_.open(DEBUG_FILE_DIR("esdf_opt_debug.csv"), std::ios::out | std::ios::trunc);
    if (esdf_debug_log_.is_open())
    {
      esdf_debug_log_ << "time,stage,valid,reason,cost,duration,max_vel,max_acc,min_esdf_dist,fail_t,fail_x,fail_y,fail_z,grid_type\n";
    }
  }
}

ESDFTrajOpt::~ESDFTrajOpt()
{
  if (esdf_debug_log_.is_open())
  {
    esdf_debug_log_.close();
  }
}

void ESDFTrajOpt::setMapManager(const general_planner::MapManager::Ptr &map_manager)
{
  map_manager_ = map_manager;
}

void ESDFTrajOpt::setSafeDistance(double safe_distance)
{
  opt_vars_.safe_distance = std::max(0.0, safe_distance);
}

void ESDFTrajOpt::setWeight(double weight)
{
  opt_vars_.weight_esdf = std::max(0.0, weight);
}

void ESDFTrajOpt::setShortcutGuide(bool shortcut_guide)
{
  opt_vars_.shortcut_guide = shortcut_guide;
}

void ESDFTrajOpt::setLabel(const std::string &label)
{
  label_ = label;
}

void ESDFTrajOpt::setSwarmConfig(const SwarmPenaltyConfig &config)
{
  swarm_config_ = config;
}

void ESDFTrajOpt::setSwarmTrajectories(const SwarmTrajectoriesConstPtr &trajectories)
{
  swarm_trajs_ = trajectories;
}

void ESDFTrajOpt::setSwarmCurrentWallTime(double wall_time)
{
  swarm_current_wall_time_ = wall_time;
}

SnapBoundaryState ESDFTrajOpt::toSnapBoundary(const StatePVAJ &state)
{
  SnapBoundaryState out;
  out.col(0) = state.col(0);
  out.col(1) = state.col(1);
  out.col(2) = state.col(2);
  out.col(3) = state.col(3);
  return out;
}

Trajectory ESDFTrajOpt::toGeometryTrajectory(const SnapTraj &traj)
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

bool ESDFTrajOpt::initializeFromGuide(const vec_E<Vec3f> &guide_path,
                                      const std::vector<double> &guide_t)
{
  if (guide_path.size() < 2 || guide_path.size() != guide_t.size())
  {
    return false;
  }

  vec_E<Vec3f> filtered_path;
  std::vector<double> filtered_time;
  filtered_path.reserve(guide_path.size());
  filtered_time.reserve(guide_t.size());

  const double max_vel = std::max(1.0e-3, cfg_.max_vel);
  const double t0 = guide_t.front();
  for (int i = 0; i < static_cast<int>(guide_path.size()); ++i)
  {
    if (!guide_path[i].allFinite() || !std::isfinite(guide_t[i]))
    {
      continue;
    }

    double t = std::max(0.0, guide_t[i] - t0);
    if (!filtered_path.empty())
    {
      const double dis = (guide_path[i] - filtered_path.back()).norm();
      if (dis < 1.0e-4)
      {
        continue;
      }
      t = std::max(t, filtered_time.back() + 0.5 * dis / max_vel);
    }
    filtered_path.emplace_back(guide_path[i]);
    filtered_time.emplace_back(t);
  }

  if (filtered_path.size() < 2)
  {
    return false;
  }

  const double map_res = map_manager_ != nullptr ? map_manager_->getResolution() : 0.1;
  const int filtered_count = static_cast<int>(filtered_path.size());

  auto segmentSafeForShortcut = [&](const Vec3f &start, const Vec3f &end) {
    if (map_manager_ == nullptr)
    {
      return false;
    }
    if (!map_manager_->isLineFree(start, end, true, false))
    {
      return false;
    }

    const double len = (end - start).norm();
    const int sample_num = std::max(1, static_cast<int>(std::ceil(len / std::max(0.05, map_res))));
    const double required_dist = 0.5 * opt_vars_.safe_distance;
    for (int k = 0; k <= sample_num; ++k)
    {
      const double ratio = static_cast<double>(k) / static_cast<double>(sample_num);
      const Vec3f p = start + ratio * (end - start);
      if (!map_manager_->insideLocalMap(p))
      {
        return false;
      }
      const auto inf_type = map_manager_->getInfGridType(p);
      if (inf_type == rog_map::GridType::OCCUPIED ||
          inf_type == rog_map::GridType::OUT_OF_MAP)
      {
        return false;
      }
      if (map_manager_->hasESDF())
      {
        double dist = 0.0;
        Vec3f grad = Vec3f::Zero();
        if (map_manager_->evaluateESDF(p, dist, grad) &&
            std::isfinite(dist) &&
            dist < required_dist)
        {
          return false;
        }
      }
    }
    return true;
  };

  vec_E<Vec3f> shortcut_path;
  shortcut_path.reserve(filtered_path.size());
  if (filtered_path.size() <= 2 || map_manager_ == nullptr || !opt_vars_.shortcut_guide)
  {
    shortcut_path = filtered_path;
  }
  else
  {
    size_t anchor = 0;
    shortcut_path.emplace_back(filtered_path.front());
    while (anchor + 1 < filtered_path.size())
    {
      size_t best = anchor + 1;
      for (size_t candidate = filtered_path.size() - 1; candidate > anchor + 1; --candidate)
      {
        if (segmentSafeForShortcut(filtered_path[anchor], filtered_path[candidate]))
        {
          best = candidate;
          break;
        }
      }
      shortcut_path.emplace_back(filtered_path[best]);
      anchor = best;
    }
  }
  if (shortcut_path.size() >= 2)
  {
    filtered_path = shortcut_path;
    filtered_time.assign(filtered_path.size(), 0.0);
    for (int i = 1; i < static_cast<int>(filtered_path.size()); ++i)
    {
      const double dt = (filtered_path[i] - filtered_path[i - 1]).norm() / max_vel;
      filtered_time[i] = filtered_time[i - 1] + std::max(0.05, dt);
    }
  }
  const auto shortcut_count = static_cast<int>(filtered_path.size());
  opt_vars_.guide_path = filtered_path;
  opt_vars_.guide_velocities = estimateGuideVelocities(filtered_path,
                                                       filtered_time,
                                                       opt_vars_.head_pvaj.col(1),
                                                       opt_vars_.tail_pvaj.col(1),
                                                       max_vel);

  std::vector<double> arc_lengths(filtered_path.size(), 0.0);
  for (int i = 1; i < static_cast<int>(filtered_path.size()); ++i)
  {
    arc_lengths[i] = arc_lengths[i - 1] + (filtered_path[i] - filtered_path[i - 1]).norm();
  }
  const double total_len = arc_lengths.back();
  if (total_len < 1.0e-4)
  {
    return false;
  }

  const double target_piece_len = std::clamp(6.0 * map_res, 0.55, 0.85);
  const int exact_piece_limit = 12;
  const bool use_exact_guide = static_cast<int>(filtered_path.size()) - 1 <= exact_piece_limit;
  const int piece_num = use_exact_guide
                            ? static_cast<int>(filtered_path.size()) - 1
                            : std::clamp(static_cast<int>(std::ceil(total_len / target_piece_len)), 1, 28);

  vec_E<Vec3f> sampled_path;
  std::vector<double> sampled_time;
  sampled_path.reserve(piece_num + 1);
  sampled_time.reserve(piece_num + 1);
  if (use_exact_guide)
  {
    sampled_path = filtered_path;
    sampled_time = filtered_time;
  }
  else
  {
    for (int i = 0; i <= piece_num; ++i)
    {
      const double target_arc = total_len * static_cast<double>(i) / static_cast<double>(piece_num);
      double t = 0.0;
      sampled_path.emplace_back(interpolateGuideByArc(filtered_path, filtered_time, arc_lengths, target_arc, t));
      sampled_time.emplace_back(t);
    }
  }
  sampled_path.front() = filtered_path.front();
  sampled_path.back() = filtered_path.back();
  sampled_time.front() = filtered_time.front();
  sampled_time.back() = filtered_time.back();

  opt_vars_.points.resize(3, piece_num - 1);
  for (int i = 1; i < piece_num; ++i)
  {
    opt_vars_.points.col(i - 1) = sampled_path[i];
  }
  opt_vars_.guide_points = opt_vars_.points;

  std::vector<double> segment_lengths(piece_num, 0.0);
  double segment_length_sum = 0.0;
  for (int i = 1; i <= piece_num; ++i)
  {
    segment_lengths[i - 1] = (sampled_path[i] - sampled_path[i - 1]).norm();
    segment_length_sum += segment_lengths[i - 1];
  }
  segment_length_sum = std::max(1.0e-6, segment_length_sum);

  const double start_vel = opt_vars_.head_pvaj.col(1).norm();
  const double end_vel = opt_vars_.tail_pvaj.col(1).norm();
  const double profile_vel_ratio = std::clamp(cfg_.init_profile_vel_ratio, 0.1, 1.0);
  const double duration_scale = std::clamp(cfg_.init_duration_scale, 1.0, 3.0);
  const double profile_max_vel = std::max(1.0e-3, profile_vel_ratio * max_vel);
  const double dynamic_duration = estimateTrapezoidalDuration(segment_length_sum,
                                                              start_vel,
                                                              end_vel,
                                                              profile_max_vel,
                                                              cfg_.max_acc);
  const double cruise_duration = segment_length_sum / profile_max_vel;
  const double guide_duration = sampled_time.back() - sampled_time.front();
  double target_duration = std::max(dynamic_duration, cruise_duration);
  const double max_reasonable_guide_duration = std::max(1.0, 3.0 * target_duration);
  if (std::isfinite(guide_duration) &&
      guide_duration > 0.05 &&
      guide_duration < max_reasonable_guide_duration)
  {
    target_duration = std::max(target_duration, guide_duration);
  }
  target_duration = std::max(0.1, duration_scale * target_duration);

  opt_vars_.times.resize(piece_num);
  for (int i = 1; i <= piece_num; ++i)
  {
    const double min_dt = std::max(0.08, 0.75 * segment_lengths[i - 1] / max_vel);
    opt_vars_.times(i - 1) = std::max(min_dt,
                                      target_duration * segment_lengths[i - 1] / segment_length_sum);
  }

  if (cfg_.print_optimizer_log)
  {
    std::cout << " -- [" << label_ << "] Guide points: " << guide_path.size()
              << " -> filtered: " << filtered_count
              << " -> shortcut: " << shortcut_count
              << " -> pieces: " << piece_num
              << ", length: " << total_len
              << ", profile duration: " << target_duration
              << ", duration: " << opt_vars_.times.sum() << std::endl;
  }
  if (esdf_debug_log_.is_open())
  {
    esdf_debug_log_ << ros_ptr_->getSimTime()
                    << ",initialize,1,OK,0,"
                    << opt_vars_.times.sum() << ",0,0,inf,0,"
                    << sampled_path.front().x() << ","
                    << sampled_path.front().y() << ","
                    << sampled_path.front().z()
                    << ",raw_" << guide_path.size()
                    << "_filtered_" << filtered_count
                    << "_shortcut_" << shortcut_count
                    << "_pieces_" << piece_num
                    << std::endl;
  }
  return opt_vars_.times.allFinite() && opt_vars_.times.minCoeff() > 0.0;
}

double ESDFTrajOpt::costFunctional(void *ptr, const VecDf &x, VecDf &g)
{
  return static_cast<ESDFTrajOpt *>(ptr)->evaluateCurrentCost(x, g);
}

double ESDFTrajOpt::evaluateCurrentCost(const VecDf &x, VecDf &g)
{
  opt_vars_.iter_num++;
  g.setZero();
  const int piece_num = static_cast<int>(opt_vars_.times.size());
  VecDf times(piece_num);
  for (int i = 0; i < piece_num; ++i)
  {
    times(i) = time_map_.toTime(x(i));
  }

  Mat3Df inner(3, piece_num - 1);
  int offset = piece_num;
  for (int i = 0; i < piece_num - 1; ++i)
  {
    inner.col(i) = x.segment<3>(offset);
    offset += 3;
  }

  minco_traj_.generate(inner, toSnapBoundary(opt_vars_.head_pvaj), toSnapBoundary(opt_vars_.tail_pvaj), times);

  double cost = 0.0;
  typename SnapTraj::CoeffMat gdC(minco_traj_.getCoefficients().rows(), 3);
  VecDf gdT(piece_num);
  gdC.setZero();
  gdT.setZero();
  opt_vars_.penalty_log.setZero();

  if (!opt_vars_.block_energy_cost)
  {
    double energy = 0.0;
    typename SnapTraj::CoeffMat gdC_energy;
    VecDf gdT_energy;
    minco_traj_.getEnergyPartialGradByCoeffs(energy, gdC_energy);
    minco_traj_.getEnergyPartialGradByTimes(gdT_energy);
    cost += energy;
    gdC += gdC_energy;
    gdT += gdT_energy;
    opt_vars_.penalty_log(0) = energy;
  }

  std::vector<double> time_vec = toStdVector(times);
  cost += linear_time_cost_(time_vec, gdT);

  esdf_cost_manager_.reset(map_manager_.get(),
                           opt_vars_.safe_distance,
                           opt_vars_.smooth_eps,
                           opt_vars_.weight_esdf,
                           opt_vars_.magnitude_bounds,
                           opt_vars_.penalty_weights,
                           &opt_vars_.quadrotor_flatness,
                           swarm_config_,
                           swarm_trajs_,
                           swarm_current_wall_time_);
  double guide_integral_cost = 0.0;
  const bool use_guide_integral_cost = opt_vars_.weight_guide_integral > 0.0 &&
                                       opt_vars_.guide_path.size() >= 2;
  const bool use_guide_velocity_integral_cost =
      opt_vars_.weight_guide_vel_integral > 0.0 &&
      opt_vars_.guide_path.size() >= 2 &&
      opt_vars_.guide_velocities.size() == opt_vars_.guide_path.size();
  const auto &coeffs = minco_traj_.getCoefficients();
  double seg_start = 0.0;
  for (int i = 0; i < piece_num; ++i)
  {
    const double T = times(i);
    const double inv_K = 1.0 / static_cast<double>(opt_vars_.integral_res);
    const double step = T * inv_K;
    const int base = i * SnapTraj::COEFF_NUM;
    const auto coeff_block = coeffs.template block<SnapTraj::COEFF_NUM, TRAJ_DIM>(base, 0);
    for (int k = 0; k <= opt_vars_.integral_res; ++k)
    {
      const double alpha = static_cast<double>(k) * inv_K;
      const double t = alpha * T;
      const double node = (k == 0 || k == opt_vars_.integral_res) ? 0.5 : 1.0;
      const double common = node * step;
      SnapTraj::BasisRow bp, bv, ba, bj, bs;
      SnapTraj::computeBasisFunctions(t, bp, bv, ba, bj, bs);
      Vec3f p = coeff_block.transpose() * bp.transpose();
      Vec3f v = coeff_block.transpose() * bv.transpose();
      Vec3f a = coeff_block.transpose() * ba.transpose();
      Vec3f j = coeff_block.transpose() * bj.transpose();
      Vec3f s = coeff_block.transpose() * bs.transpose();
      Vec3f gp = Vec3f::Zero();
      Vec3f gv = Vec3f::Zero();
      Vec3f ga = Vec3f::Zero();
      Vec3f gj = Vec3f::Zero();
      Vec3f gs = Vec3f::Zero();
      double gt = 0.0;
      double sample_cost = esdf_cost_manager_(t, seg_start + t, i, k, p, v, a, j, s, gp, gv, ga, gj, gs, gt);
      if (use_guide_integral_cost || use_guide_velocity_integral_cost)
      {
        const ClosestGuidePV ref = closestPVOnPolyline(opt_vars_.guide_path, opt_vars_.guide_velocities, p);
        if (use_guide_integral_cost)
        {
          const Vec3f diff = p - ref.position;
          const double guide_sample_cost = 0.5 * opt_vars_.weight_guide_integral * diff.squaredNorm();
          sample_cost += guide_sample_cost;
          gp += opt_vars_.weight_guide_integral * diff;
          guide_integral_cost += common * guide_sample_cost;
        }
        if (use_guide_velocity_integral_cost)
        {
          const Vec3f diff_v = v - ref.velocity;
          const double guide_velocity_sample_cost =
              0.5 * opt_vars_.weight_guide_vel_integral * diff_v.squaredNorm();
          sample_cost += guide_velocity_sample_cost;
          gv += opt_vars_.weight_guide_vel_integral * diff_v;
          guide_integral_cost += common * guide_velocity_sample_cost;
        }
      }
      cost += common * sample_cost;
      gdC.template block<SnapTraj::COEFF_NUM, TRAJ_DIM>(base, 0).noalias() +=
          (bp.transpose() * gp.transpose() +
           bv.transpose() * gv.transpose() +
           ba.transpose() * ga.transpose() +
           bj.transpose() * gj.transpose()) * common;
      gdT(i) += node * inv_K * sample_cost;
      gdT(i) += (gp.dot(v) + gv.dot(a) + ga.dot(j) + gj.dot(s)) * alpha * common;
      if (std::abs(gt) > 1.0e-12)
      {
        if (i > 0)
        {
          gdT.head(i).array() += gt * common;
        }
        gdT(i) += gt * alpha * common;
      }
    }
    seg_start += T;
  }

  const auto grad_result = minco_traj_.propagateGradFull(gdC, gdT);
  for (int i = 0; i < piece_num; ++i)
  {
    g(i) += time_map_.backward(x(i), times(i), grad_result.grad_by_times(i));
  }
  offset = piece_num;
  double guide_cost = 0.0;
  const bool use_guide_cost = opt_vars_.weight_guide > 0.0 &&
                              opt_vars_.guide_points.cols() == piece_num - 1;
  for (int i = 0; i < piece_num - 1; ++i)
  {
    Vec3f grad_point = grad_result.grad_by_points.col(i);
    if (use_guide_cost)
    {
      const Vec3f diff = inner.col(i) - opt_vars_.guide_points.col(i);
      guide_cost += 0.5 * opt_vars_.weight_guide * diff.squaredNorm();
      grad_point += opt_vars_.weight_guide * diff;
    }
    g.segment<3>(offset) += grad_point;
    offset += 3;
  }
  cost += guide_cost;
  opt_vars_.max_violation = esdf_cost_manager_.getMaxViolation();
  opt_vars_.penalty_log.tail(7) = esdf_cost_manager_.getPenaltyLog().tail(7);
  opt_vars_.penalty_log(5) = guide_cost + guide_integral_cost;
  return cost;
}

void ESDFTrajOpt::decodeOptimizationVector(const VecDf &x, VecDf &times, Mat3Df &inner) const
{
  const int piece_num = static_cast<int>(opt_vars_.times.size());
  times.resize(piece_num);
  for (int i = 0; i < piece_num; ++i)
  {
    times(i) = time_map_.toTime(x(i));
  }

  inner.resize(3, piece_num - 1);
  int offset = piece_num;
  for (int i = 0; i < piece_num - 1; ++i)
  {
    inner.col(i) = x.segment<3>(offset);
    offset += 3;
  }
}

std::string ESDFTrajOpt::validationReportToString(const ValidationReport &report)
{
  std::ostringstream ss;
  ss << "reason=" << report.reason
     << ", duration=" << report.duration
     << ", max_vel=" << report.max_vel
     << ", max_acc=" << report.max_acc
     << ", min_esdf_dist=" << report.min_esdf_dist
     << ", t=" << report.time
     << ", p=[" << report.position.transpose() << "]"
     << ", grid=" << gridTypeName(report.grid_type);
  return ss.str();
}

void ESDFTrajOpt::logValidationReport(const std::string &stage,
                                      const ValidationReport &report,
                                      double cost) const
{
  const std::string msg = " -- [" + label_ + "] " + stage + " validation: " + validationReportToString(report);
  if (report.valid)
  {
    if (cfg_.print_optimizer_log)
    {
      std::cout << GREEN << msg << RESET << std::endl;
    }
  }
  else
  {
    if (cfg_.print_optimizer_log ||
        (stage.rfind("initial", 0) != 0 && stage != "optimized_recoverable"))
    {
      std::cout << YELLOW << msg << RESET << std::endl;
    }
  }

  if (esdf_debug_log_.is_open())
  {
    esdf_debug_log_ << ros_ptr_->getSimTime() << ","
                    << stage << ","
                    << (report.valid ? 1 : 0) << ","
                    << report.reason << ","
                    << cost << ","
                    << report.duration << ","
                    << report.max_vel << ","
                    << report.max_acc << ","
                    << report.min_esdf_dist << ","
                    << report.time << ","
                    << report.position.x() << ","
                    << report.position.y() << ","
                    << report.position.z() << ","
                    << gridTypeName(report.grid_type)
                    << std::endl;
  }
}

double ESDFTrajOpt::optimize(Trajectory &traj, double rel_cost_tol)
{
  const int piece_num = static_cast<int>(opt_vars_.times.size());
  VecDf x(piece_num + 3 * (piece_num - 1));
  for (int i = 0; i < piece_num; ++i)
  {
    x(i) = time_map_.toTau(opt_vars_.times(i));
  }
  int offset = piece_num;
  for (int i = 0; i < piece_num - 1; ++i)
  {
    x.segment<3>(offset) = opt_vars_.points.col(i);
    offset += 3;
  }

  auto buildTrajectory = [&](const Mat3Df &inner, const VecDf &times) {
    minco_traj_.generate(inner,
                         toSnapBoundary(opt_vars_.head_pvaj),
                         toSnapBoundary(opt_vars_.tail_pvaj),
                         times);
    return toGeometryTrajectory(minco_traj_);
  };

  auto initialScaleFromReport = [&](const ValidationReport &report) {
    double scale = 1.25;
    if (report.reason == "MAX_VEL" && cfg_.max_vel > 1.0e-3)
    {
      scale = std::max(scale, report.max_vel / std::max(1.0e-3, 1.35 * cfg_.max_vel));
    }
    if (report.reason == "MAX_ACC" && cfg_.max_acc > 1.0e-3)
    {
      scale = std::max(scale, std::sqrt(report.max_acc / std::max(1.0e-3, 1.35 * cfg_.max_acc)));
    }
    return std::clamp(scale, 1.25, 5.0);
  };

  const Trajectory initial_traj = buildTrajectory(opt_vars_.points, opt_vars_.times);
  const ValidationReport initial_report = validateTrajectoryDetailed(initial_traj);
  logValidationReport("initial", initial_report, 0.0);
  Trajectory valid_initial_traj = initial_traj;
  ValidationReport valid_initial_report = initial_report;
  bool has_valid_initial = initial_report.valid;
  if (!has_valid_initial &&
      (initial_report.reason == "MAX_VEL" || initial_report.reason == "MAX_ACC"))
  {
    double scale = initialScaleFromReport(initial_report);
    for (int attempt = 1; attempt <= 5; ++attempt)
    {
      const VecDf scaled_times = opt_vars_.times * scale;
      Trajectory scaled_initial_traj = buildTrajectory(opt_vars_.points, scaled_times);
      ValidationReport scaled_initial_report = validateTrajectoryDetailed(scaled_initial_traj);
      logValidationReport("initial_time_scale_" + std::to_string(attempt), scaled_initial_report, 0.0);
      if (scaled_initial_report.valid)
      {
        valid_initial_traj = scaled_initial_traj;
        valid_initial_report = scaled_initial_report;
        has_valid_initial = true;
        break;
      }
      scale *= 1.25;
    }
  }

  opt_vars_.iter_num = 0;
  double min_cost = 0.0;
  lbfgs::lbfgs_parameter_t params;
  params.mem_size = 32;
  params.past = 3;
  params.min_step = 1.0e-32;
  params.g_epsilon = 0.0;
  params.delta = rel_cost_tol;
  params.max_iterations = 80;
  params.max_linesearch = 32;
  const int ret = lbfgs::lbfgs_optimize(x, min_cost, &ESDFTrajOpt::costFunctional, nullptr, nullptr, this, params);
  const bool recoverable_ret = ret == lbfgs::LBFGSERR_MAXIMUMITERATION ||
                               ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
                               ret == lbfgs::LBFGSERR_MINIMUMSTEP ||
                               ret == lbfgs::LBFGSERR_WIDTHTOOSMALL;
  if (ret < 0 && !recoverable_ret)
  {
    traj.clear();
    std::cout << YELLOW << " -- [" << label_ << "] Optimization failed: " << lbfgs::lbfgs_strerror(ret) << RESET << std::endl;
    return INFINITY;
  }
  if (ret < 0 && cfg_.print_optimizer_log)
  {
    std::cout << YELLOW << " -- [" << label_ << "] Optimization stopped with recoverable status: "
              << lbfgs::lbfgs_strerror(ret)
              << ", validate last accepted iterate." << RESET << std::endl;
  }

  VecDf grad = VecDf::Zero(x.size());
  min_cost = evaluateCurrentCost(x, grad);
  VecDf optimized_times;
  Mat3Df optimized_inner;
  decodeOptimizationVector(x, optimized_times, optimized_inner);
  minco_traj_.generate(optimized_inner,
                       toSnapBoundary(opt_vars_.head_pvaj),
                       toSnapBoundary(opt_vars_.tail_pvaj),
                       optimized_times);
  traj = toGeometryTrajectory(minco_traj_);
  ValidationReport report = validateTrajectoryDetailed(traj);
  logValidationReport(has_valid_initial ? "optimized_recoverable" : "optimized", report, min_cost);
  if (!report.valid)
  {
    const bool dynamic_violation = report.reason == "MAX_VEL" || report.reason == "MAX_ACC";
    if (dynamic_violation)
    {
      double scale = 1.25;
      if (report.reason == "MAX_VEL" && cfg_.max_vel > 1.0e-3)
      {
        scale = std::max(scale, report.max_vel / std::max(1.0e-3, 1.35 * cfg_.max_vel));
      }
      if (report.reason == "MAX_ACC" && cfg_.max_acc > 1.0e-3)
      {
        scale = std::max(scale, std::sqrt(report.max_acc / std::max(1.0e-3, 1.35 * cfg_.max_acc)));
      }
      scale = std::clamp(scale, 1.25, 4.0);

      for (int attempt = 1; attempt <= 4; ++attempt)
      {
        const VecDf scaled_times = optimized_times * scale;
        minco_traj_.generate(optimized_inner,
                             toSnapBoundary(opt_vars_.head_pvaj),
                             toSnapBoundary(opt_vars_.tail_pvaj),
                             scaled_times);
        Trajectory scaled_traj = toGeometryTrajectory(minco_traj_);
        ValidationReport scaled_report = validateTrajectoryDetailed(scaled_traj);
        logValidationReport("time_scale_" + std::to_string(attempt), scaled_report, min_cost);
        if (scaled_report.valid)
        {
          traj = scaled_traj;
          return min_cost;
        }
        scale *= 1.25;
      }
    }

    if (has_valid_initial)
    {
      traj = valid_initial_traj;
      logValidationReport("initial_reuse", valid_initial_report, min_cost);
      return min_cost;
    }

    traj.clear();
    std::cout << YELLOW << " -- [" << label_ << "] Optimized trajectory is not valid: "
              << validationReportToString(report) << RESET << std::endl;
    return INFINITY;
  }
  return min_cost;
}

ESDFTrajOpt::ValidationReport ESDFTrajOpt::validateTrajectoryDetailed(const Trajectory &traj) const
{
  ValidationReport report;
  report.position.setZero();
  report.grid_type = static_cast<int>(super_utils::GridType::UNDEFINED);
  if (traj.empty())
  {
    report.reason = "EMPTY_TRAJ";
    return report;
  }

  const double duration = traj.getTotalDuration();
  report.duration = duration;
  if (!std::isfinite(duration) || duration < 1.0e-3)
  {
    report.reason = "BAD_DURATION";
    return report;
  }
  report.max_vel = traj.getMaxVelRate();
  report.max_acc = traj.getMaxAccRate();
  if (cfg_.penna_vel > 0.0 && report.max_vel > 1.5 * cfg_.max_vel)
  {
    report.reason = "MAX_VEL";
    return report;
  }
  if (cfg_.penna_acc > 0.0 && report.max_acc > 1.5 * cfg_.max_acc)
  {
    report.reason = "MAX_ACC";
    return report;
  }

  if (map_manager_ == nullptr)
  {
    report.valid = true;
    report.reason = "OK_NO_MAP";
    return report;
  }

  const double dt = std::max(0.02, map_manager_->getResolution() / std::max(1.0, cfg_.max_vel));
  report.min_esdf_dist = std::numeric_limits<double>::infinity();
  for (double t = 0.0; t <= duration + 1.0e-6; t += dt)
  {
    const double eval_t = std::min(t, duration);
    const Vec3f p = traj.getPos(eval_t);
    if (!p.allFinite())
    {
      report.reason = "NONFINITE_POS";
      report.time = eval_t;
      report.position = p;
      return report;
    }

    double dist = 0.0;
    Vec3f grad = Vec3f::Zero();
    const bool esdf_ready = map_manager_->evaluateESDF(p, dist, grad);
    if (esdf_ready)
    {
      if (dist < report.min_esdf_dist)
      {
        report.min_esdf_dist = dist;
        report.time = eval_t;
        report.position = p;
      }
      if (dist < 0.5 * opt_vars_.safe_distance)
      {
        report.reason = "ESDF_TOO_CLOSE";
        return report;
      }
    }

    const auto grid_type = map_manager_->getInfGridType(p);
    report.grid_type = static_cast<int>(grid_type);
    if (grid_type == rog_map::GridType::OUT_OF_MAP)
    {
      report.reason = "OUT_OF_MAP";
      report.time = eval_t;
      report.position = p;
      return report;
    }
    if (!esdf_ready && grid_type == rog_map::GridType::OCCUPIED)
    {
      report.reason = "INF_OCCUPIED";
      report.time = eval_t;
      report.position = p;
      return report;
    }
  }
  report.valid = true;
  report.reason = "OK";
  return report;
}

bool ESDFTrajOpt::validateTrajectory(const Trajectory &traj) const
{
  return validateTrajectoryDetailed(traj).valid;
}

bool ESDFTrajOpt::optimize(const StatePVAJ &headPVAJ,
                           const StatePVAJ &tailPVAJ,
                           const vec_E<Vec3f> &guide_path,
                           const std::vector<double> &guide_t,
                           Trajectory &out_traj)
{
  if (map_manager_ == nullptr || !map_manager_->hasESDF())
  {
    return false;
  }
  opt_vars_.head_pvaj = headPVAJ;
  opt_vars_.tail_pvaj = tailPVAJ;
  if (!initializeFromGuide(guide_path, guide_t))
  {
    return false;
  }
  out_traj.clear();
  const bool success = !std::isinf(optimize(out_traj, cfg_.opt_accuracy));
  if (success)
  {
    out_traj.start_WT = ros_ptr_->getSimTime();
  }
  return success;
}

PlainTrajOpt::PlainTrajOpt(const traj_opt::Config &cfg,
                           const ros_interface::RosInterface::Ptr &ros_ptr)
    : cfg_(cfg), ros_ptr_(ros_ptr)
{
  opt_vars_.rho = cfg_.penna_t;
  opt_vars_.block_energy_cost = cfg_.block_energy_cost;
  opt_vars_.smooth_eps = cfg_.smooth_eps;
  opt_vars_.integral_res = std::max(1, cfg_.integral_reso);
  opt_vars_.weight_pv = cfg_.penna_pos > 0.0 ? cfg_.penna_pos : 1.0;
  constexpr double kDefaultPlainGuideWeight = 2.0e+3;
  constexpr double kDefaultPlainGuideVelWeight = 1.0e+2;
  opt_vars_.weight_guide = cfg_.penna_attract > 0.0 ? cfg_.penna_attract : kDefaultPlainGuideWeight;
  opt_vars_.weight_guide_integral = opt_vars_.weight_guide;
  opt_vars_.weight_guide_vel_integral =
      cfg_.penna_guide_vel > 0.0 ? cfg_.penna_guide_vel : kDefaultPlainGuideVelWeight;
  opt_vars_.weight_guide_tube = 0.0;
  opt_vars_.magnitude_bounds.resize(6);
  opt_vars_.penalty_weights.resize(7);
  opt_vars_.magnitude_bounds << cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk,
      cfg_.max_omg, cfg_.min_acc_thr * cfg_.mass, cfg_.max_acc_thr * cfg_.mass;
  opt_vars_.penalty_weights << 0.0,
      std::max(0.0, cfg_.penna_vel),
      std::max(0.0, cfg_.penna_acc),
      std::max(0.0, cfg_.penna_jerk),
      0.0,
      std::max(0.0, cfg_.penna_omg),
      std::max(0.0, cfg_.penna_thr);
  opt_vars_.penalty_log = VecDf::Zero(8);
  opt_vars_.quadrotor_flatness = cfg_.quadrotot_flatness;
  linear_time_cost_.weight = opt_vars_.rho;

  if (cfg_.save_log_en)
  {
    plain_debug_log_.open(DEBUG_FILE_DIR("plain_opt_debug.csv"), std::ios::out | std::ios::trunc);
    if (plain_debug_log_.is_open())
    {
      plain_debug_log_ << "time,stage,valid,reason,cost,duration,max_vel,max_acc,min_clearance,fail_t,fail_x,fail_y,fail_z,grid_type\n";
    }
  }
}

PlainTrajOpt::~PlainTrajOpt()
{
  if (plain_debug_log_.is_open())
  {
    plain_debug_log_.close();
  }
}

void PlainTrajOpt::setMapManager(const general_planner::MapManager::Ptr &map_manager)
{
  map_manager_ = map_manager;
}

void PlainTrajOpt::setLocalAstar(const std::shared_ptr<path_search::Astar> &astar)
{
  local_astar_ = astar;
}

void PlainTrajOpt::setSafeDistance(double safe_distance)
{
  opt_vars_.safe_distance = std::max(0.0, safe_distance);
  opt_vars_.guide_tube_radius = std::clamp(1.35 * opt_vars_.safe_distance, 0.35, 0.85);
  opt_vars_.guide_tube_radius_sqr = opt_vars_.guide_tube_radius * opt_vars_.guide_tube_radius;
}

void PlainTrajOpt::setShortcutGuide(bool shortcut_guide)
{
  opt_vars_.shortcut_guide = shortcut_guide;
}

void PlainTrajOpt::setSwarmConfig(const SwarmPenaltyConfig &config)
{
  swarm_config_ = config;
}

void PlainTrajOpt::setSwarmTrajectories(const SwarmTrajectoriesConstPtr &trajectories)
{
  swarm_trajs_ = trajectories;
}

void PlainTrajOpt::setSwarmCurrentWallTime(double wall_time)
{
  swarm_current_wall_time_ = wall_time;
}

SnapBoundaryState PlainTrajOpt::toSnapBoundary(const StatePVAJ &state)
{
  SnapBoundaryState out;
  out.col(0) = state.col(0);
  out.col(1) = state.col(1);
  out.col(2) = state.col(2);
  out.col(3) = state.col(3);
  return out;
}

Trajectory PlainTrajOpt::toGeometryTrajectory(const SnapTraj &traj)
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

bool PlainTrajOpt::findPVPairForPoint(const Vec3f &query,
                                      const Vec3f &reference,
                                      Vec3f &base_point,
                                      Vec3f &direction) const
{
  base_point.setZero();
  direction.setZero();
  if (map_manager_ == nullptr || !query.allFinite() || !map_manager_->insideLocalMap(query))
  {
    return false;
  }

  const double search_radius = std::clamp(3.0 * opt_vars_.safe_distance, 0.6, 2.5);
  const double map_res = std::max(0.05, map_manager_->getResolution());
  Vec3f ref_dir = reference - query;
  const double ref_len = ref_dir.norm();
  if (reference.allFinite() && std::isfinite(ref_len) && ref_len > map_res)
  {
    ref_dir /= ref_len;
    bool hit_occupied = false;
    double boundary_s = 0.0;
    for (double s = ref_len; s >= 0.0; s -= map_res)
    {
      const Vec3f p = query + s * ref_dir;
      if (!map_manager_->insideLocalMap(p))
      {
        continue;
      }
      if (map_manager_->getInfGridType(p) == rog_map::GridType::OCCUPIED)
      {
        hit_occupied = true;
        boundary_s = std::min(ref_len, s + map_res);
        break;
      }
    }
    if (hit_occupied)
    {
      base_point = query + boundary_s * ref_dir;
      direction = ref_dir;
      return true;
    }
  }

  const auto inf_type = map_manager_->getInfGridType(query);
  if (inf_type == rog_map::GridType::OCCUPIED)
  {
    Vec3f nearest_free = Vec3f::Zero();
    if (map_manager_->getNearestInfCellNot(rog_map::GridType::OCCUPIED, query, nearest_free, search_radius))
    {
      Vec3f escape_dir = nearest_free - query;
      const double escape_norm = escape_dir.norm();
      if (std::isfinite(escape_norm) && escape_norm > 1.0e-4)
      {
        direction = escape_dir / escape_norm;
        base_point = nearest_free;
        return true;
      }
    }
  }

  Vec3f box_min = query - Vec3f::Constant(search_radius);
  Vec3f box_max = query + Vec3f::Constant(search_radius);
  map_manager_->boundBoxByLocalMap(box_min, box_max);
  rog_map::vec_E<rog_map::Vec3f> occupied_points;
  map_manager_->boxSearchInflate(box_min, box_max, rog_map::GridType::OCCUPIED, occupied_points);
  if (occupied_points.empty())
  {
    return false;
  }

  double best_sq = std::numeric_limits<double>::infinity();
  Vec3f best_occupied = Vec3f::Zero();
  for (const auto &occupied : occupied_points)
  {
    const double sq = (query - occupied).squaredNorm();
    if (sq < best_sq)
    {
      best_sq = sq;
      best_occupied = occupied;
    }
  }

  Vec3f normal = query - best_occupied;
  const double normal_norm = normal.norm();
  if (!std::isfinite(normal_norm) || normal_norm < 1.0e-4)
  {
    return false;
  }

  base_point = best_occupied;
  direction = normal / normal_norm;
  return true;
}

bool PlainTrajOpt::plainSampleOccupied(const Vec3f &position) const
{
  if (map_manager_ == nullptr || !position.allFinite() || !map_manager_->insideLocalMap(position))
  {
    return true;
  }
  const auto inf_type = map_manager_->getInfGridType(position);
  return inf_type == rog_map::GridType::OCCUPIED ||
         inf_type == rog_map::GridType::OUT_OF_MAP;
}

bool PlainTrajOpt::plainSampleNeedsPVPair(const Vec3f &position) const
{
  if (map_manager_ == nullptr || !position.allFinite() || !map_manager_->insideLocalMap(position))
  {
    return false;
  }
  if (plainSampleOccupied(position))
  {
    return true;
  }

  const double map_res = std::max(0.05, map_manager_->getResolution());
  const double trigger_radius = std::max(opt_vars_.safe_distance + 2.0 * map_res,
                                         3.0 * map_res);
  Vec3f box_min = position - Vec3f::Constant(trigger_radius);
  Vec3f box_max = position + Vec3f::Constant(trigger_radius);
  map_manager_->boundBoxByLocalMap(box_min, box_max);

  rog_map::vec_E<rog_map::Vec3f> occupied_points;
  map_manager_->boxSearchInflate(box_min, box_max, rog_map::GridType::OCCUPIED, occupied_points);
  if (occupied_points.empty())
  {
    return false;
  }

  const double trigger_sqr = trigger_radius * trigger_radius;
  for (const auto &occupied : occupied_points)
  {
    if ((position - occupied).squaredNorm() <= trigger_sqr)
    {
      return true;
    }
  }
  return false;
}

void PlainTrajOpt::resetPVPairBuckets(int sample_count)
{
  sample_count = std::max(0, sample_count);
  opt_vars_.pv_pairs.clear();
  opt_vars_.pv_pairs.resize(sample_count);
  opt_vars_.local_astar_segments = 0;
  opt_vars_.local_astar_success = 0;
  opt_vars_.local_astar_pairs = 0;
  opt_vars_.fallback_pv_pairs = 0;
}

bool PlainTrajOpt::appendPVPair(int sample_idx,
                                const Vec3f &base_point,
                                const Vec3f &direction,
                                std::vector<unsigned char> &pv_filled,
                                int &active_pv_pairs)
{
  if (sample_idx < 0 ||
      sample_idx >= static_cast<int>(opt_vars_.pv_pairs.size()) ||
      sample_idx >= static_cast<int>(pv_filled.size()) ||
      !base_point.allFinite() ||
      !direction.allFinite())
  {
    return false;
  }

  const double dir_norm = direction.norm();
  if (!std::isfinite(dir_norm) || dir_norm < 1.0e-6)
  {
    return false;
  }

  cost_functional_manager::PlainPVPair pair;
  pair.base_point = base_point;
  pair.direction = direction / dir_norm;

  auto &bucket = opt_vars_.pv_pairs[sample_idx];
  constexpr int kMaxPairsPerSample = 4;
  for (const auto &existing : bucket)
  {
    const double dir_dot = existing.direction.normalized().dot(pair.direction);
    if (dir_dot > 0.985 && (existing.base_point - pair.base_point).norm() < 0.15)
    {
      return false;
    }
  }
  if (static_cast<int>(bucket.size()) >= kMaxPairsPerSample)
  {
    return false;
  }

  bucket.push_back(pair);
  pv_filled[sample_idx] = 1;
  ++active_pv_pairs;
  return true;
}

bool PlainTrajOpt::buildPVPairFromLocalPath(const std::vector<Vec3f> &sample_positions,
                                            int sample_idx,
                                            const vec_E<Vec3f> &local_path,
                                            Vec3f &base_point,
                                            Vec3f &direction) const
{
  base_point.setZero();
  direction.setZero();
  if (map_manager_ == nullptr ||
      sample_idx < 0 ||
      sample_idx >= static_cast<int>(sample_positions.size()) ||
      local_path.size() < 2)
  {
    return false;
  }

  const Vec3f sample = sample_positions[sample_idx];
  const int prev_idx = std::max(0, sample_idx - 1);
  const int next_idx = std::min(static_cast<int>(sample_positions.size()) - 1, sample_idx + 1);
  Vec3f tangent = sample_positions[next_idx] - sample_positions[prev_idx];
  double tangent_norm = tangent.norm();
  if (!std::isfinite(tangent_norm) || tangent_norm < 1.0e-4)
  {
    tangent = local_path.back() - local_path.front();
    tangent_norm = tangent.norm();
  }
  if (!std::isfinite(tangent_norm) || tangent_norm < 1.0e-4)
  {
    return false;
  }

  Vec3f intersection = Vec3f::Zero();
  double best_intersection_sq = std::numeric_limits<double>::infinity();
  bool found_intersection = false;
  for (int i = 1; i < static_cast<int>(local_path.size()); ++i)
  {
    const Vec3f p0 = local_path[i - 1];
    const Vec3f p1 = local_path[i];
    const double v0 = (p0 - sample).dot(tangent);
    const double v1 = (p1 - sample).dot(tangent);
    if (v0 * v1 > 0.0 || (std::abs(v0) < 1.0e-9 && std::abs(v1) < 1.0e-9))
    {
      continue;
    }

    const double denom = tangent.dot(p1 - p0);
    if (std::abs(denom) < 1.0e-9)
    {
      continue;
    }

    const double ratio = std::clamp(-v0 / denom, 0.0, 1.0);
    const Vec3f candidate = p0 + ratio * (p1 - p0);
    const double sq = (candidate - sample).squaredNorm();
    if (sq < best_intersection_sq)
    {
      best_intersection_sq = sq;
      intersection = candidate;
      found_intersection = true;
    }
  }

  if (!found_intersection)
  {
    for (const auto &path_pt : local_path)
    {
      const double sq = (path_pt - sample).squaredNorm();
      if (sq < best_intersection_sq)
      {
        best_intersection_sq = sq;
        intersection = path_pt;
        found_intersection = true;
      }
    }
  }

  if (!found_intersection)
  {
    return false;
  }

  Vec3f local_direction = intersection - sample;
  const double length = local_direction.norm();
  if (!std::isfinite(length) || length < 1.0e-4)
  {
    return false;
  }
  local_direction /= length;

  const double map_res = std::max(0.05, map_manager_->getResolution());
  for (double a = length; a >= 0.0; a -= map_res)
  {
    Vec3f test_point = (a / length) * intersection + (1.0 - a / length) * sample;
    if (plainSampleOccupied(test_point) || a < map_res)
    {
      if (plainSampleOccupied(test_point))
      {
        a = std::min(length, a + map_res);
      }
      base_point = (a / length) * intersection + (1.0 - a / length) * sample;
      direction = local_direction;
      return true;
    }
  }

  base_point = sample;
  direction = local_direction;
  return true;
}

void PlainTrajOpt::generateLocalAstarPVPairs(const std::vector<Vec3f> &sample_positions,
                                             std::vector<unsigned char> &pv_filled,
                                             int &active_pv_pairs)
{
  if (local_astar_ == nullptr ||
      map_manager_ == nullptr ||
      sample_positions.empty() ||
      sample_positions.size() != pv_filled.size())
  {
    return;
  }

  std::vector<std::pair<int, int>> collision_segments;
  constexpr int enough_interval = 2;
  bool last_occ = false;
  bool got_start = false;
  bool maybe_got_end = false;
  int same_occ_state_times = enough_interval + 1;
  int in_id = -1;
  int out_id = -1;
  for (int i = 0; i < static_cast<int>(sample_positions.size()); ++i)
  {
    const bool occ = plainSampleOccupied(sample_positions[i]);
    if (occ && !last_occ)
    {
      if (same_occ_state_times > enough_interval || i == 0)
      {
        in_id = std::max(0, i - 1);
        got_start = true;
      }
      same_occ_state_times = 0;
      maybe_got_end = false;
    }
    else if (!occ && last_occ)
    {
      out_id = std::min(static_cast<int>(sample_positions.size()) - 1, i + 1);
      maybe_got_end = true;
      same_occ_state_times = 0;
    }
    else
    {
      ++same_occ_state_times;
    }

    if (got_start && maybe_got_end &&
        (same_occ_state_times > enough_interval ||
         i == static_cast<int>(sample_positions.size()) - 1))
    {
      if (in_id >= 0 && out_id > in_id + 1)
      {
        collision_segments.emplace_back(in_id, out_id);
      }
      got_start = false;
      maybe_got_end = false;
      in_id = -1;
      out_id = -1;
    }
    last_occ = occ;
  }
  if (got_start && in_id >= 0 &&
      static_cast<int>(sample_positions.size()) - 1 > in_id + 1)
  {
    collision_segments.emplace_back(in_id, static_cast<int>(sample_positions.size()) - 1);
  }
  opt_vars_.local_astar_segments = static_cast<int>(collision_segments.size());

  if (collision_segments.empty())
  {
    return;
  }

  std::vector<std::pair<int, int>> segment_bounds(collision_segments.size());
  const int sample_count = static_cast<int>(sample_positions.size());
  for (int i = 0; i < static_cast<int>(collision_segments.size()); ++i)
  {
    int low = 1;
    int high = sample_count - 2;
    if (i > 0)
    {
      low = (collision_segments[i].first + collision_segments[i - 1].second + 1) / 2;
    }
    if (i + 1 < static_cast<int>(collision_segments.size()))
    {
      high = (collision_segments[i].second + collision_segments[i + 1].first - 1) / 2;
    }
    low = std::clamp(low, 1, std::max(1, sample_count - 2));
    high = std::clamp(high, 1, std::max(1, sample_count - 2));
    if (low > high)
    {
      const int mid = std::clamp((low + high) / 2, 1, std::max(1, sample_count - 2));
      low = mid;
      high = mid;
    }
    segment_bounds[i] = {low, high};
  }

  const int expand_samples = std::clamp(opt_vars_.integral_res / 3, 1, 6);

  const int inf_flag = path_search::ON_INF_MAP |
                       path_search::UNKNOWN_AS_FREE |
                       path_search::DONT_USE_INF_NEIGHBOR;
  const int prob_flag = path_search::ON_PROB_MAP |
                        path_search::UNKNOWN_AS_FREE |
                        path_search::USE_INF_NEIGHBOR;

  for (int seg_id = 0; seg_id < static_cast<int>(collision_segments.size()); ++seg_id)
  {
    const auto &segment = collision_segments[seg_id];
    Vec3f start_pt = sample_positions[segment.first];
    Vec3f end_pt = sample_positions[segment.second];
    if (plainSampleOccupied(start_pt) &&
        !map_manager_->getNearestInfCellNot(rog_map::GridType::OCCUPIED, start_pt, start_pt, 1.5))
    {
      continue;
    }
    if (plainSampleOccupied(end_pt) &&
        !map_manager_->getNearestInfCellNot(rog_map::GridType::OCCUPIED, end_pt, end_pt, 1.5))
    {
      continue;
    }

    vec_Vec3f local_path;
    const double chord = (end_pt - start_pt).norm();
    const double horizon = std::clamp(chord + 4.0, 3.0, std::max(6.0, 2.0 * cfg_.max_vel));
    auto ret = local_astar_->pointToPointPathSearch(end_pt, start_pt, inf_flag, horizon, local_path, 0.08);
    if ((ret != super_utils::SUCCESS && ret != super_utils::REACH_GOAL && ret != super_utils::REACH_HORIZON) ||
        local_path.size() < 2)
    {
      local_path.clear();
      ret = local_astar_->pointToPointPathSearch(end_pt, start_pt, prob_flag, horizon, local_path, 0.08);
    }
    if ((ret != super_utils::SUCCESS && ret != super_utils::REACH_GOAL && ret != super_utils::REACH_HORIZON) ||
        local_path.size() < 2)
    {
      continue;
    }
    ++opt_vars_.local_astar_success;

    const int adjusted_first = std::clamp(segment.first - expand_samples,
                                          segment_bounds[seg_id].first,
                                          segment_bounds[seg_id].second);
    const int adjusted_second = std::clamp(segment.second + expand_samples,
                                           segment_bounds[seg_id].first,
                                           segment_bounds[seg_id].second);
    if (adjusted_second < adjusted_first)
    {
      continue;
    }

    std::vector<unsigned char> local_has_pair(sample_positions.size(), 0);
    std::vector<Vec3f> local_base_points(sample_positions.size(), Vec3f::Zero());
    std::vector<Vec3f> local_directions(sample_positions.size(), Vec3f::Zero());
    std::vector<int> explicit_pair_indices;

    for (int sample_idx = segment.first + 1; sample_idx < segment.second; ++sample_idx)
    {
      if (sample_idx < 0 || sample_idx >= static_cast<int>(pv_filled.size()))
      {
        continue;
      }
      Vec3f base_point = Vec3f::Zero();
      Vec3f direction = Vec3f::Zero();
      if (buildPVPairFromLocalPath(sample_positions, sample_idx, local_path, base_point, direction))
      {
        local_has_pair[sample_idx] = 1;
        local_base_points[sample_idx] = base_point;
        local_directions[sample_idx] = direction;
        explicit_pair_indices.push_back(sample_idx);
      }
    }

    if (explicit_pair_indices.empty() && segment.second - segment.first <= 2)
    {
      const int sample_idx = std::clamp((segment.first + segment.second) / 2,
                                        adjusted_first,
                                        adjusted_second);
      Vec3f base_point = Vec3f::Zero();
      Vec3f direction = Vec3f::Zero();
      if (buildPVPairFromLocalPath(sample_positions, sample_idx, local_path, base_point, direction))
      {
        local_has_pair[sample_idx] = 1;
        local_base_points[sample_idx] = base_point;
        local_directions[sample_idx] = direction;
        explicit_pair_indices.push_back(sample_idx);
      }
    }

    if (explicit_pair_indices.empty())
    {
      continue;
    }

    for (int sample_idx = adjusted_first; sample_idx <= adjusted_second; ++sample_idx)
    {
      int source_idx = -1;
      if (local_has_pair[sample_idx])
      {
        source_idx = sample_idx;
      }
      else
      {
        int best_delta = std::numeric_limits<int>::max();
        for (const int candidate_idx : explicit_pair_indices)
        {
          const int delta = std::abs(candidate_idx - sample_idx);
          if (delta < best_delta)
          {
            best_delta = delta;
            source_idx = candidate_idx;
          }
        }
      }

      if (source_idx >= 0 &&
          appendPVPair(sample_idx,
                       local_base_points[source_idx],
                       local_directions[source_idx],
                       pv_filled,
                       active_pv_pairs))
      {
        ++opt_vars_.local_astar_pairs;
      }
    }
  }
}

void PlainTrajOpt::collectCurrentTrajectorySamples(std::vector<Vec3f> &sample_positions) const
{
  const int piece_num = static_cast<int>(minco_traj_.getPieceNum());
  sample_positions.assign(piece_num * opt_vars_.pv_samples_per_piece, Vec3f::Zero());
  if (piece_num <= 0 || opt_vars_.pv_samples_per_piece <= 0)
  {
    return;
  }

  const auto &coeffs = minco_traj_.getCoefficients();
  const auto &times = minco_traj_.getDurations();
  for (int i = 0; i < piece_num; ++i)
  {
    const double T = times(i);
    const int base = i * SnapTraj::COEFF_NUM;
    const auto coeff_block = coeffs.template block<SnapTraj::COEFF_NUM, TRAJ_DIM>(base, 0);
    for (int k = 0; k <= opt_vars_.integral_res; ++k)
    {
      const double alpha = static_cast<double>(k) / static_cast<double>(opt_vars_.integral_res);
      SnapTraj::BasisRow bp, bv, ba, bj, bs;
      SnapTraj::computeBasisFunctions(alpha * T, bp, bv, ba, bj, bs);
      const int pv_idx = i * opt_vars_.pv_samples_per_piece + k;
      sample_positions[pv_idx] = coeff_block.transpose() * bp.transpose();
    }
  }
}

bool PlainTrajOpt::sampleNeedsNewPVPair(int sample_idx,
                                        const Vec3f &position) const
{
  if (!plainSampleNeedsPVPair(position))
  {
    return false;
  }
  if (sample_idx < 0 || sample_idx >= static_cast<int>(opt_vars_.pv_pairs.size()))
  {
    return true;
  }

  const auto &bucket = opt_vars_.pv_pairs[sample_idx];
  if (bucket.empty())
  {
    return true;
  }

  const double map_res = map_manager_ != nullptr ? std::max(0.05, map_manager_->getResolution()) : 0.1;
  const double active_distance = opt_vars_.safe_distance + 1.5 * map_res;
  for (const auto &pair : bucket)
  {
    const double dir_norm = pair.direction.norm();
    if (!std::isfinite(dir_norm) || dir_norm < 1.0e-6)
    {
      continue;
    }
    const double signed_distance = (position - pair.base_point).dot(pair.direction / dir_norm);
    if (signed_distance < active_distance)
    {
      return false;
    }
  }
  return true;
}

bool PlainTrajOpt::maybeRefreshPVPairsForRebound(const VecDf &x, int iteration)
{
  if (iteration < 3 || local_astar_ == nullptr || map_manager_ == nullptr)
  {
    return false;
  }

  VecDf times;
  Mat3Df inner;
  decodeOptimizationVector(x, times, inner);
  minco_traj_.generate(inner,
                       toSnapBoundary(opt_vars_.head_pvaj),
                       toSnapBoundary(opt_vars_.tail_pvaj),
                       times);

  std::vector<Vec3f> sample_positions;
  collectCurrentTrajectorySamples(sample_positions);
  bool has_new_collision = false;
  for (int i = 1; i + 1 < static_cast<int>(sample_positions.size()); ++i)
  {
    if (sampleNeedsNewPVPair(i, sample_positions[i]))
    {
      has_new_collision = true;
      break;
    }
  }
  if (!has_new_collision)
  {
    return false;
  }

  return refreshPVPairsFromCurrentTrajectory() > 0;
}

int PlainTrajOpt::refreshPVPairsFromCurrentTrajectory()
{
  const int piece_num = static_cast<int>(minco_traj_.getPieceNum());
  if (piece_num <= 0 || opt_vars_.integral_res <= 0)
  {
    return 0;
  }

  opt_vars_.pv_samples_per_piece = opt_vars_.integral_res + 1;
  std::vector<Vec3f> pv_sample_positions;
  collectCurrentTrajectorySamples(pv_sample_positions);
  resetPVPairBuckets(static_cast<int>(pv_sample_positions.size()));

  int active_pv_pairs = 0;
  std::vector<unsigned char> pv_filled(pv_sample_positions.size(), 0);
  generateLocalAstarPVPairs(pv_sample_positions, pv_filled, active_pv_pairs);
  for (int i = 0; i < piece_num; ++i)
  {
    for (int k = 0; k <= opt_vars_.integral_res; ++k)
    {
      const int pv_idx = i * opt_vars_.pv_samples_per_piece + k;
      if (pv_filled[pv_idx])
      {
        continue;
      }
      const Vec3f query = pv_sample_positions[pv_idx];
      if (!plainSampleNeedsPVPair(query))
      {
        continue;
      }
      const ClosestGuidePV ref = closestPVOnPolyline(opt_vars_.guide_path,
                                                     opt_vars_.guide_velocities,
                                                     query);
      Vec3f base_point = Vec3f::Zero();
      Vec3f direction = Vec3f::Zero();
      if (findPVPairForPoint(query, ref.position, base_point, direction))
      {
        if (appendPVPair(pv_idx, base_point, direction, pv_filled, active_pv_pairs))
        {
          ++opt_vars_.fallback_pv_pairs;
        }
      }
    }
  }
  return active_pv_pairs;
}

bool PlainTrajOpt::initializeFromGuide(const vec_E<Vec3f> &guide_path,
                                       const std::vector<double> &guide_t)
{
  if (guide_path.size() < 2 || guide_path.size() != guide_t.size())
  {
    return false;
  }

  vec_E<Vec3f> filtered_path;
  std::vector<double> filtered_time;
  filtered_path.reserve(guide_path.size());
  filtered_time.reserve(guide_t.size());

  const double max_vel = std::max(1.0e-3, cfg_.max_vel);
  const double t0 = guide_t.front();
  for (int i = 0; i < static_cast<int>(guide_path.size()); ++i)
  {
    if (!guide_path[i].allFinite() || !std::isfinite(guide_t[i]))
    {
      continue;
    }

    double t = std::max(0.0, guide_t[i] - t0);
    if (!filtered_path.empty())
    {
      const double dis = (guide_path[i] - filtered_path.back()).norm();
      if (dis < 1.0e-4)
      {
        continue;
      }
      t = std::max(t, filtered_time.back() + 0.5 * dis / max_vel);
    }
    filtered_path.emplace_back(guide_path[i]);
    filtered_time.emplace_back(t);
  }

  if (filtered_path.size() < 2)
  {
    return false;
  }

  const double map_res = map_manager_ != nullptr ? map_manager_->getResolution() : 0.1;
  double tube_radius = std::max(opt_vars_.guide_tube_radius,
                                1.35 * opt_vars_.safe_distance);
  tube_radius = std::max(tube_radius, 4.0 * std::max(0.05, map_res));
  opt_vars_.guide_tube_radius = std::clamp(tube_radius, 0.35, 0.85);
  opt_vars_.guide_tube_radius_sqr =
      opt_vars_.guide_tube_radius * opt_vars_.guide_tube_radius;
  const int filtered_count = static_cast<int>(filtered_path.size());

  auto segmentSafeForShortcut = [&](const Vec3f &start, const Vec3f &end) {
    if (map_manager_ == nullptr)
    {
      return false;
    }
    if (!map_manager_->isLineFree(start, end, true, false))
    {
      return false;
    }

    const double len = (end - start).norm();
    const int sample_num = std::max(1, static_cast<int>(std::ceil(len / std::max(0.05, map_res))));
    for (int k = 0; k <= sample_num; ++k)
    {
      const double ratio = static_cast<double>(k) / static_cast<double>(sample_num);
      const Vec3f p = start + ratio * (end - start);
      if (!map_manager_->insideLocalMap(p))
      {
        return false;
      }
      const auto inf_type = map_manager_->getInfGridType(p);
      if (inf_type == rog_map::GridType::OCCUPIED ||
          inf_type == rog_map::GridType::OUT_OF_MAP)
      {
        return false;
      }
    }
    return true;
  };

  vec_E<Vec3f> shortcut_path;
  shortcut_path.reserve(filtered_path.size());
  if (filtered_path.size() <= 2 || map_manager_ == nullptr || !opt_vars_.shortcut_guide)
  {
    shortcut_path = filtered_path;
  }
  else
  {
    size_t anchor = 0;
    shortcut_path.emplace_back(filtered_path.front());
    while (anchor + 1 < filtered_path.size())
    {
      size_t best = anchor + 1;
      for (size_t candidate = filtered_path.size() - 1; candidate > anchor + 1; --candidate)
      {
        if (segmentSafeForShortcut(filtered_path[anchor], filtered_path[candidate]))
        {
          best = candidate;
          break;
        }
      }
      shortcut_path.emplace_back(filtered_path[best]);
      anchor = best;
    }
  }
  if (shortcut_path.size() >= 2)
  {
    filtered_path = shortcut_path;
    filtered_time.assign(filtered_path.size(), 0.0);
    for (int i = 1; i < static_cast<int>(filtered_path.size()); ++i)
    {
      const double dt = (filtered_path[i] - filtered_path[i - 1]).norm() / max_vel;
      filtered_time[i] = filtered_time[i - 1] + std::max(0.05, dt);
    }
  }
  const int shortcut_count = static_cast<int>(filtered_path.size());
  opt_vars_.guide_path = filtered_path;
  opt_vars_.guide_velocities = estimateGuideVelocities(filtered_path,
                                                       filtered_time,
                                                       opt_vars_.head_pvaj.col(1),
                                                       opt_vars_.tail_pvaj.col(1),
                                                       max_vel);

  std::vector<double> arc_lengths(filtered_path.size(), 0.0);
  for (int i = 1; i < static_cast<int>(filtered_path.size()); ++i)
  {
    arc_lengths[i] = arc_lengths[i - 1] + (filtered_path[i] - filtered_path[i - 1]).norm();
  }
  const double total_len = arc_lengths.back();
  if (total_len < 1.0e-4)
  {
    return false;
  }

  const double target_piece_len = std::clamp(6.0 * map_res, 0.55, 0.85);
  const int exact_piece_limit = 12;
  const bool use_exact_guide = static_cast<int>(filtered_path.size()) - 1 <= exact_piece_limit;
  const int piece_num = use_exact_guide
                            ? static_cast<int>(filtered_path.size()) - 1
                            : std::clamp(static_cast<int>(std::ceil(total_len / target_piece_len)), 1, 28);

  vec_E<Vec3f> sampled_path;
  std::vector<double> sampled_time;
  sampled_path.reserve(piece_num + 1);
  sampled_time.reserve(piece_num + 1);
  if (use_exact_guide)
  {
    sampled_path = filtered_path;
    sampled_time = filtered_time;
  }
  else
  {
    for (int i = 0; i <= piece_num; ++i)
    {
      const double target_arc = total_len * static_cast<double>(i) / static_cast<double>(piece_num);
      double t = 0.0;
      sampled_path.emplace_back(interpolateGuideByArc(filtered_path, filtered_time, arc_lengths, target_arc, t));
      sampled_time.emplace_back(t);
    }
  }
  sampled_path.front() = filtered_path.front();
  sampled_path.back() = filtered_path.back();
  sampled_time.front() = filtered_time.front();
  sampled_time.back() = filtered_time.back();

  opt_vars_.points.resize(3, piece_num - 1);
  for (int i = 1; i < piece_num; ++i)
  {
    opt_vars_.points.col(i - 1) = sampled_path[i];
  }
  opt_vars_.guide_points = opt_vars_.points;

  std::vector<double> segment_lengths(piece_num, 0.0);
  double segment_length_sum = 0.0;
  for (int i = 1; i <= piece_num; ++i)
  {
    segment_lengths[i - 1] = (sampled_path[i] - sampled_path[i - 1]).norm();
    segment_length_sum += segment_lengths[i - 1];
  }
  segment_length_sum = std::max(1.0e-6, segment_length_sum);

  const double start_vel = opt_vars_.head_pvaj.col(1).norm();
  const double end_vel = opt_vars_.tail_pvaj.col(1).norm();
  const double profile_vel_ratio = std::clamp(cfg_.init_profile_vel_ratio, 0.1, 1.0);
  const double duration_scale = std::clamp(cfg_.init_duration_scale, 1.0, 3.0);
  const double profile_max_vel = std::max(1.0e-3, profile_vel_ratio * max_vel);
  const double dynamic_duration = estimateTrapezoidalDuration(segment_length_sum,
                                                              start_vel,
                                                              end_vel,
                                                              profile_max_vel,
                                                              cfg_.max_acc);
  const double cruise_duration = segment_length_sum / profile_max_vel;
  const double guide_duration = sampled_time.back() - sampled_time.front();
  double target_duration = std::max(dynamic_duration, cruise_duration);
  const double max_reasonable_guide_duration = std::max(1.0, 3.0 * target_duration);
  if (std::isfinite(guide_duration) &&
      guide_duration > 0.05 &&
      guide_duration < max_reasonable_guide_duration)
  {
    target_duration = std::max(target_duration, guide_duration);
  }
  target_duration = std::max(0.1, duration_scale * target_duration);

  opt_vars_.times.resize(piece_num);
  for (int i = 1; i <= piece_num; ++i)
  {
    const double min_dt = std::max(0.08, 0.75 * segment_lengths[i - 1] / max_vel);
    opt_vars_.times(i - 1) = std::max(min_dt,
                                      target_duration * segment_lengths[i - 1] / segment_length_sum);
  }

  opt_vars_.pv_samples_per_piece = opt_vars_.integral_res + 1;
  minco_traj_.generate(opt_vars_.points,
                       toSnapBoundary(opt_vars_.head_pvaj),
                       toSnapBoundary(opt_vars_.tail_pvaj),
                       opt_vars_.times);
  std::vector<Vec3f> pv_sample_positions;
  collectCurrentTrajectorySamples(pv_sample_positions);
  resetPVPairBuckets(static_cast<int>(pv_sample_positions.size()));

  int active_pv_pairs = 0;
  std::vector<unsigned char> pv_filled(pv_sample_positions.size(), 0);
  generateLocalAstarPVPairs(pv_sample_positions, pv_filled, active_pv_pairs);
  for (int i = 0; i < piece_num; ++i)
  {
    for (int k = 0; k <= opt_vars_.integral_res; ++k)
    {
      const int pv_idx = i * opt_vars_.pv_samples_per_piece + k;
      if (pv_filled[pv_idx])
      {
        continue;
      }
      const Vec3f query = pv_sample_positions[pv_idx];
      if (!plainSampleNeedsPVPair(query))
      {
        continue;
      }
      const ClosestGuidePV ref = closestPVOnPolyline(opt_vars_.guide_path,
                                                     opt_vars_.guide_velocities,
                                                     query);
      Vec3f base_point = Vec3f::Zero();
      Vec3f direction = Vec3f::Zero();
      if (findPVPairForPoint(query, ref.position, base_point, direction))
      {
        if (appendPVPair(pv_idx, base_point, direction, pv_filled, active_pv_pairs))
        {
          ++opt_vars_.fallback_pv_pairs;
        }
      }
    }
  }

  if (cfg_.print_optimizer_log)
  {
    std::cout << " -- [" << label_ << "] Guide points: " << guide_path.size()
              << " -> filtered: " << filtered_count
              << " -> shortcut: " << shortcut_count
              << " -> pieces: " << piece_num
              << ", length: " << total_len
              << ", pv_pairs: " << active_pv_pairs
              << ", tube radius: " << opt_vars_.guide_tube_radius
              << ", profile duration: " << target_duration
              << ", duration: " << opt_vars_.times.sum() << std::endl;
  }
  if (plain_debug_log_.is_open())
  {
    plain_debug_log_ << ros_ptr_->getSimTime()
                     << ",initialize,1,OK,0,"
                     << opt_vars_.times.sum() << ",0,0,inf,0,"
                     << sampled_path.front().x() << ","
                     << sampled_path.front().y() << ","
                     << sampled_path.front().z()
                     << ",raw_" << guide_path.size()
                     << "_filtered_" << filtered_count
                     << "_shortcut_" << shortcut_count
                     << "_pieces_" << piece_num
                     << "_pv_" << active_pv_pairs
                     << "_lseg_" << opt_vars_.local_astar_segments
                     << "_lastar_" << opt_vars_.local_astar_success
                     << "_lpv_" << opt_vars_.local_astar_pairs
                     << "_fpv_" << opt_vars_.fallback_pv_pairs
                     << "_tubeR_" << opt_vars_.guide_tube_radius
                     << std::endl;
  }
  return opt_vars_.times.allFinite() && opt_vars_.times.minCoeff() > 0.0;
}

double PlainTrajOpt::costFunctional(void *ptr, const VecDf &x, VecDf &g)
{
  return static_cast<PlainTrajOpt *>(ptr)->evaluateCurrentCost(x, g);
}

double PlainTrajOpt::evaluateCurrentCost(const VecDf &x, VecDf &g)
{
  opt_vars_.iter_num++;
  g.setZero();
  const int piece_num = static_cast<int>(opt_vars_.times.size());
  VecDf times(piece_num);
  for (int i = 0; i < piece_num; ++i)
  {
    times(i) = time_map_.toTime(x(i));
  }

  Mat3Df inner(3, piece_num - 1);
  int offset = piece_num;
  for (int i = 0; i < piece_num - 1; ++i)
  {
    inner.col(i) = x.segment<3>(offset);
    offset += 3;
  }

  minco_traj_.generate(inner, toSnapBoundary(opt_vars_.head_pvaj), toSnapBoundary(opt_vars_.tail_pvaj), times);

  double cost = 0.0;
  typename SnapTraj::CoeffMat gdC(minco_traj_.getCoefficients().rows(), 3);
  VecDf gdT(piece_num);
  gdC.setZero();
  gdT.setZero();
  opt_vars_.penalty_log.setZero();

  if (!opt_vars_.block_energy_cost)
  {
    double energy = 0.0;
    typename SnapTraj::CoeffMat gdC_energy;
    VecDf gdT_energy;
    minco_traj_.getEnergyPartialGradByCoeffs(energy, gdC_energy);
    minco_traj_.getEnergyPartialGradByTimes(gdT_energy);
    cost += energy;
    gdC += gdC_energy;
    gdT += gdT_energy;
    opt_vars_.penalty_log(0) = energy;
  }

  std::vector<double> time_vec = toStdVector(times);
  cost += linear_time_cost_(time_vec, gdT);

  plain_cost_manager_.reset(&opt_vars_.pv_pairs,
                            opt_vars_.safe_distance,
                            opt_vars_.weight_pv,
                            opt_vars_.smooth_eps,
                            opt_vars_.magnitude_bounds,
                            opt_vars_.penalty_weights,
                            &opt_vars_.quadrotor_flatness,
                            swarm_config_,
                            swarm_trajs_,
                            swarm_current_wall_time_,
                            opt_vars_.pv_samples_per_piece);

  double guide_integral_cost = 0.0;
  double guide_tube_cost = 0.0;
  opt_vars_.guide_tube_violation = 0.0;
  const bool use_guide_integral_cost = opt_vars_.weight_guide_integral > 0.0 &&
                                       opt_vars_.guide_path.size() >= 2;
  const bool use_guide_velocity_integral_cost =
      opt_vars_.weight_guide_vel_integral > 0.0 &&
      opt_vars_.guide_path.size() >= 2 &&
      opt_vars_.guide_velocities.size() == opt_vars_.guide_path.size();
  const bool use_guide_tube_cost =
      opt_vars_.weight_guide_tube > 0.0 &&
      opt_vars_.guide_tube_radius_sqr > 0.0 &&
      opt_vars_.guide_path.size() >= 2;
  const auto &coeffs = minco_traj_.getCoefficients();
  double seg_start = 0.0;
  for (int i = 0; i < piece_num; ++i)
  {
    const double T = times(i);
    const double inv_K = 1.0 / static_cast<double>(opt_vars_.integral_res);
    const double step = T * inv_K;
    const int base = i * SnapTraj::COEFF_NUM;
    const auto coeff_block = coeffs.template block<SnapTraj::COEFF_NUM, TRAJ_DIM>(base, 0);
    for (int k = 0; k <= opt_vars_.integral_res; ++k)
    {
      const double alpha = static_cast<double>(k) * inv_K;
      const double t = alpha * T;
      const double node = (k == 0 || k == opt_vars_.integral_res) ? 0.5 : 1.0;
      const double common = node * step;
      SnapTraj::BasisRow bp, bv, ba, bj, bs;
      SnapTraj::computeBasisFunctions(t, bp, bv, ba, bj, bs);
      Vec3f p = coeff_block.transpose() * bp.transpose();
      Vec3f v = coeff_block.transpose() * bv.transpose();
      Vec3f a = coeff_block.transpose() * ba.transpose();
      Vec3f j = coeff_block.transpose() * bj.transpose();
      Vec3f s = coeff_block.transpose() * bs.transpose();
      Vec3f gp = Vec3f::Zero();
      Vec3f gv = Vec3f::Zero();
      Vec3f ga = Vec3f::Zero();
      Vec3f gj = Vec3f::Zero();
      Vec3f gs = Vec3f::Zero();
      double gt = 0.0;
      double sample_cost = plain_cost_manager_(t, seg_start + t, i, k, p, v, a, j, s, gp, gv, ga, gj, gs, gt);
      if (use_guide_integral_cost || use_guide_velocity_integral_cost || use_guide_tube_cost)
      {
        const ClosestGuidePV ref = closestPVOnPolyline(opt_vars_.guide_path, opt_vars_.guide_velocities, p);
        const Vec3f diff = p - ref.position;
        if (use_guide_integral_cost)
        {
          const double guide_sample_cost = 0.5 * opt_vars_.weight_guide_integral * diff.squaredNorm();
          sample_cost += guide_sample_cost;
          gp += opt_vars_.weight_guide_integral * diff;
          guide_integral_cost += common * guide_sample_cost;
        }
        if (use_guide_tube_cost)
        {
          const double dist_sqr = diff.squaredNorm();
          const double violation = dist_sqr - opt_vars_.guide_tube_radius_sqr;
          if (violation > 0.0)
          {
            const double tube_sample_cost =
                0.5 * opt_vars_.weight_guide_tube * violation * violation;
            sample_cost += tube_sample_cost;
            gp += 2.0 * opt_vars_.weight_guide_tube * violation * diff;
            guide_tube_cost += common * tube_sample_cost;
            opt_vars_.guide_tube_violation =
                std::max(opt_vars_.guide_tube_violation,
                         std::sqrt(dist_sqr) - opt_vars_.guide_tube_radius);
          }
        }
        if (use_guide_velocity_integral_cost)
        {
          const Vec3f diff_v = v - ref.velocity;
          const double guide_velocity_sample_cost =
              0.5 * opt_vars_.weight_guide_vel_integral * diff_v.squaredNorm();
          sample_cost += guide_velocity_sample_cost;
          gv += opt_vars_.weight_guide_vel_integral * diff_v;
          guide_integral_cost += common * guide_velocity_sample_cost;
        }
      }
      cost += common * sample_cost;
      gdC.template block<SnapTraj::COEFF_NUM, TRAJ_DIM>(base, 0).noalias() +=
          (bp.transpose() * gp.transpose() +
           bv.transpose() * gv.transpose() +
           ba.transpose() * ga.transpose() +
           bj.transpose() * gj.transpose()) * common;
      gdT(i) += node * inv_K * sample_cost;
      gdT(i) += (gp.dot(v) + gv.dot(a) + ga.dot(j) + gj.dot(s)) * alpha * common;
      if (std::abs(gt) > 1.0e-12)
      {
        if (i > 0)
        {
          gdT.head(i).array() += gt * common;
        }
        gdT(i) += gt * alpha * common;
      }
    }
    seg_start += T;
  }

  const auto grad_result = minco_traj_.propagateGradFull(gdC, gdT);
  for (int i = 0; i < piece_num; ++i)
  {
    g(i) += time_map_.backward(x(i), times(i), grad_result.grad_by_times(i));
  }
  offset = piece_num;
  double guide_cost = 0.0;
  const bool use_guide_cost = opt_vars_.weight_guide > 0.0 &&
                              opt_vars_.guide_points.cols() == piece_num - 1;
  for (int i = 0; i < piece_num - 1; ++i)
  {
    Vec3f grad_point = grad_result.grad_by_points.col(i);
    if (use_guide_cost)
    {
      const Vec3f diff = inner.col(i) - opt_vars_.guide_points.col(i);
      guide_cost += 0.5 * opt_vars_.weight_guide * diff.squaredNorm();
      grad_point += opt_vars_.weight_guide * diff;
    }
    g.segment<3>(offset) += grad_point;
    offset += 3;
  }
  cost += guide_cost;
  opt_vars_.max_violation =
      std::max(plain_cost_manager_.getMaxCollisionViolation(),
               opt_vars_.guide_tube_violation);
  opt_vars_.penalty_log.tail(7) = plain_cost_manager_.getPenaltyLog().tail(7);
  opt_vars_.penalty_log(5) = guide_cost + guide_integral_cost + guide_tube_cost;
  return cost;
}

void PlainTrajOpt::decodeOptimizationVector(const VecDf &x, VecDf &times, Mat3Df &inner) const
{
  const int piece_num = static_cast<int>(opt_vars_.times.size());
  times.resize(piece_num);
  for (int i = 0; i < piece_num; ++i)
  {
    times(i) = time_map_.toTime(x(i));
  }

  inner.resize(3, piece_num - 1);
  int offset = piece_num;
  for (int i = 0; i < piece_num - 1; ++i)
  {
    inner.col(i) = x.segment<3>(offset);
    offset += 3;
  }
}

std::string PlainTrajOpt::validationReportToString(const ValidationReport &report)
{
  std::ostringstream ss;
  ss << "reason=" << report.reason
     << ", duration=" << report.duration
     << ", max_vel=" << report.max_vel
     << ", max_acc=" << report.max_acc
     << ", min_clearance=" << report.min_clearance
     << ", t=" << report.time
     << ", p=[" << report.position.transpose() << "]"
     << ", grid=" << gridTypeName(report.grid_type);
  return ss.str();
}

void PlainTrajOpt::logValidationReport(const std::string &stage,
                                       const ValidationReport &report,
                                       double cost) const
{
  const std::string msg = " -- [" + label_ + "] " + stage + " validation: " + validationReportToString(report);
  if (report.valid)
  {
    if (cfg_.print_optimizer_log)
    {
      std::cout << GREEN << msg << RESET << std::endl;
    }
  }
  else
  {
    if (cfg_.print_optimizer_log ||
        (stage.rfind("initial", 0) != 0 && stage != "optimized_recoverable"))
    {
      std::cout << YELLOW << msg << RESET << std::endl;
    }
  }

  if (plain_debug_log_.is_open())
  {
    plain_debug_log_ << ros_ptr_->getSimTime() << ","
                     << stage << ","
                     << (report.valid ? 1 : 0) << ","
                     << report.reason << ","
                     << cost << ","
                     << report.duration << ","
                     << report.max_vel << ","
                     << report.max_acc << ","
                     << report.min_clearance << ","
                     << report.time << ","
                     << report.position.x() << ","
                     << report.position.y() << ","
                     << report.position.z() << ","
                     << gridTypeName(report.grid_type)
                     << std::endl;
  }
}

int PlainTrajOpt::reboundProgress(void *ptr,
                                  const VecDf &x,
                                  const VecDf & /*g*/,
                                  double /*fx*/,
                                  double /*step*/,
                                  int k,
                                  int /*ls*/)
{
  auto *self = static_cast<PlainTrajOpt *>(ptr);
  return self->maybeRefreshPVPairsForRebound(x, k) ? 1 : 0;
}

double PlainTrajOpt::optimize(Trajectory &traj, double rel_cost_tol)
{
  const int piece_num = static_cast<int>(opt_vars_.times.size());
  VecDf x(piece_num + 3 * (piece_num - 1));
  for (int i = 0; i < piece_num; ++i)
  {
    x(i) = time_map_.toTau(opt_vars_.times(i));
  }
  int offset = piece_num;
  for (int i = 0; i < piece_num - 1; ++i)
  {
    x.segment<3>(offset) = opt_vars_.points.col(i);
    offset += 3;
  }

  auto buildTrajectory = [&](const Mat3Df &inner, const VecDf &times) {
    minco_traj_.generate(inner,
                         toSnapBoundary(opt_vars_.head_pvaj),
                         toSnapBoundary(opt_vars_.tail_pvaj),
                         times);
    return toGeometryTrajectory(minco_traj_);
  };

  auto initialScaleFromReport = [&](const ValidationReport &report) {
    double scale = 1.25;
    if (report.reason == "MAX_VEL" && cfg_.max_vel > 1.0e-3)
    {
      scale = std::max(scale, report.max_vel / std::max(1.0e-3, 1.35 * cfg_.max_vel));
    }
    if (report.reason == "MAX_ACC" && cfg_.max_acc > 1.0e-3)
    {
      scale = std::max(scale, std::sqrt(report.max_acc / std::max(1.0e-3, 1.35 * cfg_.max_acc)));
    }
    return std::clamp(scale, 1.25, 5.0);
  };

  const Trajectory initial_traj = buildTrajectory(opt_vars_.points, opt_vars_.times);
  const ValidationReport initial_report = validateTrajectoryDetailed(initial_traj);
  logValidationReport("initial", initial_report, 0.0);
  Trajectory valid_initial_traj = initial_traj;
  ValidationReport valid_initial_report = initial_report;
  bool has_valid_initial = initial_report.valid;
  if (!has_valid_initial &&
      (initial_report.reason == "MAX_VEL" || initial_report.reason == "MAX_ACC"))
  {
    double scale = initialScaleFromReport(initial_report);
    for (int attempt = 1; attempt <= 5; ++attempt)
    {
      const VecDf scaled_times = opt_vars_.times * scale;
      Trajectory scaled_initial_traj = buildTrajectory(opt_vars_.points, scaled_times);
      ValidationReport scaled_initial_report = validateTrajectoryDetailed(scaled_initial_traj);
      logValidationReport("initial_time_scale_" + std::to_string(attempt), scaled_initial_report, 0.0);
      if (scaled_initial_report.valid)
      {
        valid_initial_traj = scaled_initial_traj;
        valid_initial_report = scaled_initial_report;
        has_valid_initial = true;
        break;
      }
      scale *= 1.25;
    }
  }

  double min_cost = 0.0;
  lbfgs::lbfgs_parameter_t params;
  params.mem_size = 32;
  params.past = 3;
  params.min_step = 1.0e-32;
  params.g_epsilon = 0.0;
  params.delta = rel_cost_tol;
  params.max_iterations = 80;
  params.max_linesearch = 32;

  VecDf grad = VecDf::Zero(x.size());
  VecDf optimized_times;
  Mat3Df optimized_inner;
  ValidationReport report;
  bool optimized_once = false;
  int ret = lbfgs::LBFGSERR_UNKNOWNERROR;
  constexpr int max_rebound_rounds = 12;
  for (int round = 0; round <= max_rebound_rounds; ++round)
  {
    opt_vars_.iter_num = 0;
    ret = lbfgs::lbfgs_optimize(x,
                                min_cost,
                                &PlainTrajOpt::costFunctional,
                                nullptr,
                                &PlainTrajOpt::reboundProgress,
                                this,
                                params);
    const bool recoverable_ret = ret == lbfgs::LBFGSERR_MAXIMUMITERATION ||
                                 ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
                                 ret == lbfgs::LBFGSERR_MINIMUMSTEP ||
                                 ret == lbfgs::LBFGSERR_WIDTHTOOSMALL ||
                                 ret == lbfgs::LBFGS_CANCELED;
    if (ret < 0 && !recoverable_ret)
    {
      traj.clear();
      last_opt_report_ = ValidationReport();
      last_opt_report_.reason = "LBFGS_FAILED";
      std::cout << YELLOW << " -- [" << label_ << "] Optimization failed: " << lbfgs::lbfgs_strerror(ret) << RESET << std::endl;
      return INFINITY;
    }
    if (ret < 0 && cfg_.print_optimizer_log)
    {
      std::cout << YELLOW << " -- [" << label_ << "] Optimization stopped with recoverable status: "
                << lbfgs::lbfgs_strerror(ret)
                << ", validate last accepted iterate." << RESET << std::endl;
    }

    min_cost = evaluateCurrentCost(x, grad);
    decodeOptimizationVector(x, optimized_times, optimized_inner);
    minco_traj_.generate(optimized_inner,
                         toSnapBoundary(opt_vars_.head_pvaj),
                         toSnapBoundary(opt_vars_.tail_pvaj),
                         optimized_times);
    traj = toGeometryTrajectory(minco_traj_);
    report = validateTrajectoryDetailed(traj);
    last_opt_report_ = report;
    optimized_once = true;
    logValidationReport((ret == lbfgs::LBFGS_CANCELED ? "rebound_prepare_" : "optimized_round_") +
                            std::to_string(round),
                        report,
                        min_cost);

    if (report.valid)
    {
      break;
    }

    if (ret != lbfgs::LBFGS_CANCELED &&
        (report.reason != "INF_OCCUPIED" && report.reason != "OUT_OF_MAP"))
    {
      break;
    }

    const int active_pairs = refreshPVPairsFromCurrentTrajectory();
    if (active_pairs <= 0)
    {
      break;
    }
  }
  if (!optimized_once)
  {
    traj.clear();
    last_opt_report_ = ValidationReport();
    last_opt_report_.reason = "NO_OPTIMIZED_ITERATE";
    return INFINITY;
  }

  if (!report.valid)
  {
    const bool dynamic_violation = report.reason == "MAX_VEL" || report.reason == "MAX_ACC";
    if (dynamic_violation)
    {
      double scale = 1.25;
      if (report.reason == "MAX_VEL" && cfg_.max_vel > 1.0e-3)
      {
        scale = std::max(scale, report.max_vel / std::max(1.0e-3, 1.35 * cfg_.max_vel));
      }
      if (report.reason == "MAX_ACC" && cfg_.max_acc > 1.0e-3)
      {
        scale = std::max(scale, std::sqrt(report.max_acc / std::max(1.0e-3, 1.35 * cfg_.max_acc)));
      }
      scale = std::clamp(scale, 1.25, 4.0);

      for (int attempt = 1; attempt <= 4; ++attempt)
      {
        const VecDf scaled_times = optimized_times * scale;
        minco_traj_.generate(optimized_inner,
                             toSnapBoundary(opt_vars_.head_pvaj),
                             toSnapBoundary(opt_vars_.tail_pvaj),
                             scaled_times);
        Trajectory scaled_traj = toGeometryTrajectory(minco_traj_);
        ValidationReport scaled_report = validateTrajectoryDetailed(scaled_traj);
        last_opt_report_ = scaled_report;
        logValidationReport("time_scale_" + std::to_string(attempt), scaled_report, min_cost);
        if (scaled_report.valid)
        {
          traj = scaled_traj;
          return min_cost;
        }
        scale *= 1.25;
      }
    }

    if (has_valid_initial)
    {
      traj = valid_initial_traj;
      last_opt_report_ = valid_initial_report;
      logValidationReport("initial_reuse", valid_initial_report, min_cost);
      return min_cost;
    }

    traj.clear();
    last_opt_report_ = report;
    std::cout << YELLOW << " -- [" << label_ << "] Optimized trajectory is not valid: "
              << validationReportToString(report) << RESET << std::endl;
    return INFINITY;
  }
  last_opt_report_ = report;
  return min_cost;
}

PlainTrajOpt::ValidationReport PlainTrajOpt::validateTrajectoryDetailed(const Trajectory &traj) const
{
  ValidationReport report;
  report.position.setZero();
  report.grid_type = static_cast<int>(super_utils::GridType::UNDEFINED);
  if (traj.empty())
  {
    report.reason = "EMPTY_TRAJ";
    return report;
  }

  const double duration = traj.getTotalDuration();
  report.duration = duration;
  if (!std::isfinite(duration) || duration < 1.0e-3)
  {
    report.reason = "BAD_DURATION";
    return report;
  }
  report.max_vel = traj.getMaxVelRate();
  report.max_acc = traj.getMaxAccRate();

  auto fillDynamicFailureState = [&](const std::string &reason, bool use_acceleration) {
    report.reason = reason;
    double best_norm = -std::numeric_limits<double>::infinity();
    double best_t = 0.0;
    Vec3f best_position = traj.getPos(0.0);
    const double base_dt = map_manager_ != nullptr
                               ? map_manager_->getResolution() / std::max(1.0, cfg_.max_vel)
                               : duration / 200.0;
    const double dt = std::clamp(base_dt, 0.005, 0.05);
    for (double t = 0.0; t <= duration + 1.0e-9; t += dt)
    {
      const double eval_t = std::min(t, duration);
      const Vec3f value = use_acceleration ? traj.getAcc(eval_t) : traj.getVel(eval_t);
      const double norm = value.norm();
      if (std::isfinite(norm) && norm > best_norm)
      {
        best_norm = norm;
        best_t = eval_t;
        best_position = traj.getPos(eval_t);
      }
    }
    report.time = best_t;
    report.position = best_position;
    if (map_manager_ != nullptr && best_position.allFinite())
    {
      report.grid_type = static_cast<int>(map_manager_->getInfGridType(best_position));
    }
  };

  if (cfg_.penna_vel > 0.0 && report.max_vel > 1.5 * cfg_.max_vel)
  {
    fillDynamicFailureState("MAX_VEL", false);
    return report;
  }
  if (cfg_.penna_acc > 0.0 && report.max_acc > 1.5 * cfg_.max_acc)
  {
    fillDynamicFailureState("MAX_ACC", true);
    return report;
  }

  if (map_manager_ == nullptr)
  {
    report.valid = true;
    report.reason = "OK_NO_MAP";
    return report;
  }

  auto updateApproxClearance = [&](const Vec3f &p) {
    const double map_res = std::max(0.05, map_manager_->getResolution());
    const double search_radius = std::max(2.0 * opt_vars_.safe_distance,
                                          5.0 * map_res);
    Vec3f box_min = p - Vec3f::Constant(search_radius);
    Vec3f box_max = p + Vec3f::Constant(search_radius);
    map_manager_->boundBoxByLocalMap(box_min, box_max);
    rog_map::vec_E<rog_map::Vec3f> occupied_points;
    map_manager_->boxSearchInflate(box_min, box_max, rog_map::GridType::OCCUPIED, occupied_points);
    for (const auto &occupied : occupied_points)
    {
      report.min_clearance = std::min(report.min_clearance,
                                      static_cast<double>((p - occupied).norm()));
    }
  };

  const double dt = std::max(0.02, map_manager_->getResolution() / std::max(1.0, cfg_.max_vel));
  for (double t = 0.0; t <= duration + 1.0e-6; t += dt)
  {
    const double eval_t = std::min(t, duration);
    const Vec3f p = traj.getPos(eval_t);
    if (!p.allFinite())
    {
      report.reason = "NONFINITE_POS";
      report.time = eval_t;
      report.position = p;
      return report;
    }

    updateApproxClearance(p);
    const auto grid_type = map_manager_->getInfGridType(p);
    report.grid_type = static_cast<int>(grid_type);
    if (grid_type == rog_map::GridType::OUT_OF_MAP)
    {
      report.reason = "OUT_OF_MAP";
      report.time = eval_t;
      report.position = p;
      return report;
    }
    if (grid_type == rog_map::GridType::OCCUPIED)
    {
      report.reason = "INF_OCCUPIED";
      report.time = eval_t;
      report.position = p;
      return report;
    }
  }

  report.valid = true;
  report.reason = "OK_INF_MAP";
  return report;
}

bool PlainTrajOpt::validateTrajectory(const Trajectory &traj) const
{
  return validateTrajectoryDetailed(traj).valid;
}

bool PlainTrajOpt::optimize(const StatePVAJ &headPVAJ,
                            const StatePVAJ &tailPVAJ,
                            const vec_E<Vec3f> &guide_path,
                            const std::vector<double> &guide_t,
                            Trajectory &out_traj)
{
  auto runWithTailState = [&](const StatePVAJ &tail_state, const std::string &tag) {
    opt_vars_.head_pvaj = headPVAJ;
    opt_vars_.tail_pvaj = tail_state;
    if (!initializeFromGuide(guide_path, guide_t))
    {
      last_opt_report_ = ValidationReport();
      last_opt_report_.reason = "INIT_FAILED";
      return false;
    }
    out_traj.clear();
    const bool success = !std::isinf(optimize(out_traj, cfg_.opt_accuracy));
    if (success)
    {
      out_traj.start_WT = ros_ptr_->getSimTime();
    }
    else if (cfg_.print_optimizer_log && !tag.empty())
    {
      std::cout << YELLOW << " -- [" << label_ << "] " << tag
                << " failed: " << validationReportToString(last_opt_report_) << RESET << std::endl;
    }
    return success;
  };

  if (runWithTailState(tailPVAJ, "nominal terminal state"))
  {
    return true;
  }

  const bool dynamic_failure = last_opt_report_.reason == "MAX_VEL" ||
                               last_opt_report_.reason == "MAX_ACC";
  const double tail_speed = tailPVAJ.col(1).norm();
  if (dynamic_failure && tail_speed > 1.0e-3)
  {
    const std::array<double, 3> velocity_scales{{0.5, 0.2, 0.0}};
    for (const double scale : velocity_scales)
    {
      StatePVAJ relaxed_tail = tailPVAJ;
      relaxed_tail.col(1) = scale * tailPVAJ.col(1);
      relaxed_tail.col(2).setZero();
      relaxed_tail.col(3).setZero();
      if (cfg_.print_optimizer_log)
      {
        std::cout << YELLOW << " -- [" << label_ << "] Retry with relaxed terminal velocity scale="
                  << scale << ", speed=" << relaxed_tail.col(1).norm() << RESET << std::endl;
      }
      if (runWithTailState(relaxed_tail, "relaxed terminal state"))
      {
        return true;
      }
    }
  }

  return false;
}

TrajManager::TrajManager(const traj_opt::Config &exp_cfg,
                         const traj_opt::Config &esdf_cfg,
                         const traj_opt::Config &plain_cfg,
                         const traj_opt::Config &backup_cfg,
                         double yaw_dot_max,
                         double esdf_safe_distance,
                         const ros_interface::RosInterface::Ptr &ros_ptr,
                         const general_planner::MapManager::Ptr &map_manager)
{
  exp_traj_opt_ = std::make_shared<ExpTrajOpt>(exp_cfg, ros_ptr);
  esdf_traj_opt_ = std::make_shared<ESDFTrajOpt>(esdf_cfg, ros_ptr);
  esdf_traj_opt_->setMapManager(map_manager);
  esdf_traj_opt_->setSafeDistance(esdf_safe_distance);
  plain_traj_opt_ = std::make_shared<PlainTrajOpt>(plain_cfg, ros_ptr);
  plain_traj_opt_->setMapManager(map_manager);
  plain_traj_opt_->setSafeDistance(esdf_safe_distance);
  plain_traj_opt_->setShortcutGuide(true);
  backup_traj_opt_ = std::make_shared<BackupTrajOpt>(backup_cfg, ros_ptr);
  yaw_traj_opt_ = std::make_shared<YawTrajOpt>(yaw_dot_max);
  tracking_jerk_traj_opt_ = std::make_shared<TrackingJerkTrajOpt>(esdf_cfg, ros_ptr);
  tracking_snap_traj_opt_ = std::make_shared<TrackingSnapTrajOpt>(esdf_cfg, ros_ptr);
  perching_snap_traj_opt_ = std::make_shared<PerchingSnapTrajOpt>(esdf_cfg, ros_ptr);
  tracking_jerk_traj_opt_->setMapManager(map_manager);
  tracking_snap_traj_opt_->setMapManager(map_manager);
  perching_snap_traj_opt_->setMapManager(map_manager);
  tracking_jerk_traj_opt_->setSafeDistance(esdf_safe_distance);
  tracking_snap_traj_opt_->setSafeDistance(esdf_safe_distance);
  perching_snap_traj_opt_->setSafeDistance(esdf_safe_distance);
}

void TrajManager::setMapManager(const general_planner::MapManager::Ptr &map_manager)
{
  if (esdf_traj_opt_)
  {
    esdf_traj_opt_->setMapManager(map_manager);
  }
  if (plain_traj_opt_)
  {
    plain_traj_opt_->setMapManager(map_manager);
  }
  if (tracking_jerk_traj_opt_)
  {
    tracking_jerk_traj_opt_->setMapManager(map_manager);
  }
  if (tracking_snap_traj_opt_)
  {
    tracking_snap_traj_opt_->setMapManager(map_manager);
  }
  if (perching_snap_traj_opt_)
  {
    perching_snap_traj_opt_->setMapManager(map_manager);
  }
}

void TrajManager::setESDFSafeDistance(double safe_distance)
{
  if (esdf_traj_opt_)
  {
    esdf_traj_opt_->setSafeDistance(safe_distance);
  }
  if (plain_traj_opt_)
  {
    plain_traj_opt_->setSafeDistance(safe_distance);
  }
  if (tracking_jerk_traj_opt_)
  {
    tracking_jerk_traj_opt_->setSafeDistance(safe_distance);
  }
  if (tracking_snap_traj_opt_)
  {
    tracking_snap_traj_opt_->setSafeDistance(safe_distance);
  }
  if (perching_snap_traj_opt_)
  {
    perching_snap_traj_opt_->setSafeDistance(safe_distance);
  }
}

void TrajManager::setSwarmConfig(const SwarmPenaltyConfig &config)
{
  if (exp_traj_opt_)
  {
    exp_traj_opt_->setSwarmConfig(config);
  }
  if (esdf_traj_opt_)
  {
    esdf_traj_opt_->setSwarmConfig(config);
  }
  if (plain_traj_opt_)
  {
    plain_traj_opt_->setSwarmConfig(config);
  }
}

void TrajManager::setSwarmTrajectories(const SwarmTrajectoriesConstPtr &trajectories)
{
  if (exp_traj_opt_)
  {
    exp_traj_opt_->setSwarmTrajectories(trajectories);
  }
  if (esdf_traj_opt_)
  {
    esdf_traj_opt_->setSwarmTrajectories(trajectories);
  }
  if (plain_traj_opt_)
  {
    plain_traj_opt_->setSwarmTrajectories(trajectories);
  }
}

void TrajManager::setSwarmCurrentWallTime(double wall_time)
{
  if (exp_traj_opt_)
  {
    exp_traj_opt_->setSwarmCurrentWallTime(wall_time);
  }
  if (esdf_traj_opt_)
  {
    esdf_traj_opt_->setSwarmCurrentWallTime(wall_time);
  }
  if (plain_traj_opt_)
  {
    plain_traj_opt_->setSwarmCurrentWallTime(wall_time);
  }
}
