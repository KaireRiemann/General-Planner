#include "traj_opt/traj_manager.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

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

Vec3f closestPointOnPolyline(const vec_E<Vec3f> &path, const Vec3f &query)
{
  if (path.empty())
  {
    return query;
  }
  if (path.size() == 1)
  {
    return path.front();
  }

  double best_sq = std::numeric_limits<double>::infinity();
  Vec3f best = path.front();
  for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i)
  {
    const Vec3f a = path[i];
    const Vec3f b = path[i + 1];
    const Vec3f ab = b - a;
    const double denom = ab.squaredNorm();
    const double s = denom > 1.0e-9 ? std::clamp((query - a).dot(ab) / denom, 0.0, 1.0) : 0.0;
    const Vec3f candidate = a + s * ab;
    const double sq = (query - candidate).squaredNorm();
    if (sq < best_sq)
    {
      best_sq = sq;
      best = candidate;
    }
  }
  return best;
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
                                          const Trajectory &pos_traj)
{
  double eval_t = 0.0;
  double last_yaw = init_state(0);
  way_pts.resize(std::max(0, static_cast<int>(times.size()) - 1));
  const double pos_traj_duration = pos_traj.getTotalDuration();
  for (int i = 0; i < way_pts.size(); ++i)
  {
    eval_t += times(i);
    double cur_yaw = last_yaw;
    Vec3f pt_i = pos_traj.getPos(eval_t);
    Vec3f pt_g;
    if (eval_t + 0.5 >= pos_traj_duration)
    {
      pt_g = pos_traj.getPos(pos_traj_duration);
      pt_i = pos_traj.getPos(pos_traj_duration - 0.5 > 0.0 ? pos_traj_duration - 0.5 : 0.0);
    }
    else
    {
      pt_g = pos_traj.getPos(eval_t + 0.5);
    }

    const Vec3f dir = pt_g - pt_i;
    if (dir.norm() > 0.1)
    {
      cur_yaw = std::atan2(dir.y(), dir.x());
      geometry_utils::normalizeNextYaw(last_yaw, cur_yaw);
    }
    way_pts(i) = cur_yaw;
    last_yaw = cur_yaw;
  }

  if (way_pts.size() == 0)
  {
    geometry_utils::normalizeNextYaw(init_state[0], goal_state[0]);
  }
  else
  {
    geometry_utils::normalizeNextYaw(way_pts(way_pts.size() - 1), goal_state[0]);
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
  getYawWaypointAllocation(init_state, goal_state, way_pts, times, pos_traj);

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
  const int shortcut_count = static_cast<int>(filtered_path.size());
  opt_vars_.guide_path = filtered_path;

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
  const double profile_max_vel = std::max(1.0e-3, 0.65 * max_vel);
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
  target_duration = std::max(0.1, 1.25 * target_duration);

  opt_vars_.times.resize(piece_num);
  for (int i = 1; i <= piece_num; ++i)
  {
    const double min_dt = std::max(0.08, 0.75 * segment_lengths[i - 1] / max_vel);
    opt_vars_.times(i - 1) = std::max(min_dt,
                                      target_duration * segment_lengths[i - 1] / segment_length_sum);
  }

  if (cfg_.print_optimizer_log)
  {
    std::cout << " -- [ESDFTrajOpt] Guide points: " << guide_path.size()
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
      if (use_guide_integral_cost)
      {
        const Vec3f ref = closestPointOnPolyline(opt_vars_.guide_path, p);
        const Vec3f diff = p - ref;
        const double guide_sample_cost = 0.5 * opt_vars_.weight_guide_integral * diff.squaredNorm();
        sample_cost += guide_sample_cost;
        gp += opt_vars_.weight_guide_integral * diff;
        guide_integral_cost += common * guide_sample_cost;
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
  const std::string msg = " -- [ESDFTrajOpt] " + stage + " validation: " + validationReportToString(report);
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
    std::cout << YELLOW << " -- [ESDFTrajOpt] Optimization failed: " << lbfgs::lbfgs_strerror(ret) << RESET << std::endl;
    return INFINITY;
  }
  if (ret < 0 && cfg_.print_optimizer_log)
  {
    std::cout << YELLOW << " -- [ESDFTrajOpt] Optimization stopped with recoverable status: "
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
    std::cout << YELLOW << " -- [ESDFTrajOpt] Optimized trajectory is not valid: "
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

TrajManager::TrajManager(const traj_opt::Config &exp_cfg,
                         const traj_opt::Config &esdf_cfg,
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
}
