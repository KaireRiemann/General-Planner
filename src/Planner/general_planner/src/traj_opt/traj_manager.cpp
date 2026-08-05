#include "traj_opt/traj_manager.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <path_search/astar.h>
#include <utils/header/color_msg_utils.hpp>
#include <utils/optimization/optimization_utils.h>
#include <utils/optimization/phr_alm.hpp>

using namespace traj_opt;
using namespace color_text;
using namespace general_utils;
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
  if (grid_type >= 0 && grid_type < static_cast<int>(general_utils::GridTypeStr.size()))
  {
    return general_utils::GridTypeStr[grid_type];
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

SnapOptimizer::WaypointsType toOptimizerWaypoints(const Mat3Df &waypoints)
{
  SnapOptimizer::WaypointsType out(waypoints.cols(), TRAJ_DIM);
  out = waypoints.transpose();
  return out;
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

struct GuideOverlapSample
{
  Vec3f position{Vec3f::Zero()};
  double time{0.0};
  double clearance{0.0};
  bool valid{false};
};

double minNormalizedClearanceToHPoly(const PolyhedronH &h_poly, const Vec3f &point)
{
  if (h_poly.rows() <= 0 || !point.allFinite() || !std::isfinite(h_poly.sum()))
  {
    return -std::numeric_limits<double>::infinity();
  }

  double min_clearance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < h_poly.rows(); ++i)
  {
    const Vec3f normal = h_poly.row(i).head<3>().transpose();
    const double normal_norm = normal.norm();
    if (normal_norm <= 1.0e-12 || !std::isfinite(normal_norm))
    {
      continue;
    }
    const double signed_dist = normal.dot(point) + h_poly(i, 3);
    min_clearance = std::min(min_clearance, -signed_dist / normal_norm);
  }
  return min_clearance;
}

bool clipGuideSegmentToHPoly(const Vec3f &start,
                             const Vec3f &end,
                             const PolyhedronH &h_poly,
                             double boundary_margin,
                             double &s_min,
                             double &s_max)
{
  if (!start.allFinite() || !end.allFinite() ||
      h_poly.rows() <= 0 || !std::isfinite(h_poly.sum()))
  {
    return false;
  }

  const Vec3f delta = end - start;
  s_min = 0.0;
  s_max = 1.0;
  for (int i = 0; i < h_poly.rows(); ++i)
  {
    const Vec3f normal = h_poly.row(i).head<3>().transpose();
    const double normal_norm = normal.norm();
    if (normal_norm <= 1.0e-12 || !std::isfinite(normal_norm))
    {
      continue;
    }

    const double signed_start = normal.dot(start) + h_poly(i, 3) +
                                std::max(0.0, boundary_margin) * normal_norm;
    const double signed_delta = normal.dot(delta);
    if (std::abs(signed_delta) <= 1.0e-12)
    {
      if (signed_start > 1.0e-9)
      {
        return false;
      }
      continue;
    }

    const double s_bound = -signed_start / signed_delta;
    if (signed_delta > 0.0)
    {
      s_max = std::min(s_max, s_bound);
    }
    else
    {
      s_min = std::max(s_min, s_bound);
    }

    if (s_min > s_max + 1.0e-9)
    {
      return false;
    }
  }

  s_min = std::clamp(s_min, 0.0, 1.0);
  s_max = std::clamp(s_max, 0.0, 1.0);
  return s_min <= s_max + 1.0e-9;
}

GuideOverlapSample closestGuideProjectionToPoint(const vec_E<Vec3f> &path,
                                                 const std::vector<double> &times,
                                                 const Vec3f &query)
{
  GuideOverlapSample best;
  if (path.size() != times.size() || path.empty() || !query.allFinite())
  {
    return best;
  }

  double best_sq = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(path.size()); ++i)
  {
    if (!path[i].allFinite() || !std::isfinite(times[i]))
    {
      continue;
    }

    const double sq = (path[i] - query).squaredNorm();
    if (sq < best_sq)
    {
      best_sq = sq;
      best.position = path[i];
      best.time = times[i];
      best.valid = true;
    }

    if (i + 1 >= static_cast<int>(path.size()) ||
        !path[i + 1].allFinite() ||
        !std::isfinite(times[i + 1]))
    {
      continue;
    }
    const Vec3f segment = path[i + 1] - path[i];
    const double segment_sq = segment.squaredNorm();
    if (segment_sq <= 1.0e-12)
    {
      continue;
    }
    const double ratio = std::clamp((query - path[i]).dot(segment) / segment_sq, 0.0, 1.0);
    const Vec3f candidate = path[i] + ratio * segment;
    const double candidate_sq = (candidate - query).squaredNorm();
    if (candidate_sq < best_sq)
    {
      best_sq = candidate_sq;
      best.position = candidate;
      best.time = times[i] + ratio * (times[i + 1] - times[i]);
      best.valid = true;
    }
  }
  return best;
}

bool findGuidePointInOverlap(const vec_E<Vec3f> &path,
                             const std::vector<double> &times,
                             const PolyhedronH &overlap,
                             const Vec3f &time_reference_point,
                             const double min_time,
                             GuideOverlapSample &sample)
{
  sample = GuideOverlapSample{};
  if (path.size() < 2 || path.size() != times.size() ||
      overlap.rows() <= 0 || !std::isfinite(overlap.sum()))
  {
    return false;
  }

  const GuideOverlapSample reference =
      closestGuideProjectionToPoint(path, times, time_reference_point);
  const double target_time = reference.valid && std::isfinite(reference.time)
                                 ? std::max(min_time, reference.time)
                                 : min_time;

  double best_score = std::numeric_limits<double>::infinity();
  double best_clearance = -std::numeric_limits<double>::infinity();
  for (int i = 0; i + 1 < static_cast<int>(path.size()); ++i)
  {
    if (!path[i].allFinite() || !path[i + 1].allFinite() ||
        !std::isfinite(times[i]) || !std::isfinite(times[i + 1]))
    {
      continue;
    }
    if (times[i + 1] + 1.0e-8 < min_time)
    {
      continue;
    }

    double s_min = 0.0;
    double s_max = 1.0;
    if (!clipGuideSegmentToHPoly(path[i], path[i + 1], overlap, 0.0, s_min, s_max))
    {
      continue;
    }

    const double dt = times[i + 1] - times[i];
    if (dt > 1.0e-9)
    {
      s_min = std::max(s_min, (min_time - times[i]) / dt);
    }
    else if (times[i] + 1.0e-8 < min_time)
    {
      continue;
    }
    s_min = std::clamp(s_min, 0.0, 1.0);
    if (s_min > s_max + 1.0e-9)
    {
      continue;
    }

    double ratio = 0.5 * (s_min + s_max);
    if (dt > 1.0e-9 && std::isfinite(target_time))
    {
      ratio = std::clamp((target_time - times[i]) / dt, s_min, s_max);
    }
    const Vec3f position = path[i] + ratio * (path[i + 1] - path[i]);
    const double time = times[i] + ratio * dt;
    const double clearance = minNormalizedClearanceToHPoly(overlap, position);
    if (!position.allFinite() || !std::isfinite(time) ||
        !std::isfinite(clearance) || clearance < -1.0e-6)
    {
      continue;
    }

    const double score = std::abs(time - target_time);
    if (score < best_score - 1.0e-9 ||
        (std::abs(score - best_score) <= 1.0e-9 && clearance > best_clearance))
    {
      best_score = score;
      best_clearance = clearance;
      sample.position = position;
      sample.time = time;
      sample.clearance = std::max(0.0, clearance);
      sample.valid = true;
    }
  }

  return sample.valid;
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
  const auto active_penalties = cfg_.activePenaltyWeights();
  if (cfg_.save_log_en)
  {
    failed_traj_log_.open(DEBUG_FILE_DIR("exp_opt_log.csv"), std::ios::out | std::ios::trunc);
    penalty_log_.open(DEBUG_FILE_DIR("exp_opt_penna.csv"), std::ios::out | std::ios::trunc);
  }

  opt_vars_.magnitude_bounds.resize(6);
  opt_vars_.penalty_weights.resize(7);
  opt_vars_.magnitude_bounds << cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk,
      cfg_.max_omg, cfg_.min_acc_thr * cfg_.mass, cfg_.max_acc_thr * cfg_.mass;
  opt_vars_.penalty_weights << active_penalties.position,
      active_penalties.velocity,
      active_penalties.acceleration,
      active_penalties.jerk,
      active_penalties.attractor,
      active_penalties.angular_rate,
      active_penalties.thrust;
  opt_vars_.rho = active_penalties.time;
  opt_vars_.pos_constraint_type = cfg_.pos_constraint_type;
  opt_vars_.block_energy_cost = cfg_.block_energy_cost;
  opt_vars_.smooth_eps = cfg_.smooth_eps;
  opt_vars_.integral_res = std::max(1, cfg_.integral_reso);
  opt_vars_.lbfgs_fast_enabled = cfg_.lbfgs_fast_en;
  opt_vars_.lbfgs_mem_size = std::max(3, cfg_.lbfgs_mem_size);
  opt_vars_.lbfgs_step_bound_enabled = cfg_.lbfgs_step_bound_en;
  opt_vars_.lbfgs_time_ratio_min =
      std::clamp(cfg_.lbfgs_time_ratio_min, 1.0e-3, 1.0);
  opt_vars_.lbfgs_time_ratio_max =
      std::max(1.0, cfg_.lbfgs_time_ratio_max);
  opt_vars_.lbfgs_fast_window = std::max(1, cfg_.lbfgs_fast_window);
  opt_vars_.lbfgs_fast_min_iterations =
      std::max(1, cfg_.lbfgs_fast_min_iterations);
  opt_vars_.lbfgs_fast_consecutive =
      std::max(1, cfg_.lbfgs_fast_consecutive);
  opt_vars_.lbfgs_fast_rel_cost =
      std::max(0.0, cfg_.lbfgs_fast_rel_cost);
  opt_vars_.lbfgs_fast_rel_step =
      std::max(0.0, cfg_.lbfgs_fast_rel_step);
  opt_vars_.lbfgs_fast_rel_penalty =
      std::max(0.0, cfg_.lbfgs_fast_rel_penalty);
  opt_vars_.lbfgs_fast_phase0_guards_en = cfg_.lbfgs_fast_phase0_guards_en;
  opt_vars_.lbfgs_fast_rel_time =
      std::max(0.0, cfg_.lbfgs_fast_rel_time);
  opt_vars_.lbfgs_fast_rel_waypoint =
      std::max(0.0, cfg_.lbfgs_fast_rel_waypoint);
  opt_vars_.lbfgs_fast_scaled_grad =
      std::max(0.0, cfg_.lbfgs_fast_scaled_grad);
  opt_vars_.lbfgs_fast_min_step =
      std::max(0.0, cfg_.lbfgs_fast_min_step);
  opt_vars_.lbfgs_fast_penalty_tol =
      std::max(0.0, cfg_.lbfgs_fast_penalty_tol);
  opt_vars_.lbfgs_fast_small_step_limit =
      std::max(1, cfg_.lbfgs_fast_small_step_limit);
  opt_vars_.lbfgs_warm_start_enabled = cfg_.lbfgs_warm_start_en;
  opt_vars_.lbfgs_warm_start_max_endpoint_shift =
      std::max(0.0, cfg_.lbfgs_warm_start_max_endpoint_shift);
  opt_vars_.lbfgs_warm_start_max_waypoint_shift =
      std::max(0.0, cfg_.lbfgs_warm_start_max_waypoint_shift);
  opt_vars_.lbfgs_warm_start_duration_blend =
      std::clamp(cfg_.lbfgs_warm_start_duration_blend, 0.0, 1.0);
  opt_vars_.lbfgs_warm_start_cost_ratio =
      std::max(0.0, cfg_.lbfgs_warm_start_cost_ratio);
  opt_vars_.lbfgs_warm_start_gradient_ratio =
      std::max(0.0, cfg_.lbfgs_warm_start_gradient_ratio);
  opt_vars_.lbfgs_warm_start_penalty_ratio =
      std::max(0.0, cfg_.lbfgs_warm_start_penalty_ratio);
  opt_vars_.guide_initial_time_scale =
      std::max(0.1, cfg_.guide_initial_time_scale);
  opt_vars_.convex_hull_enabled = cfg_.convex_hull_en;
  opt_vars_.convex_hull_basis =
      cfg_.convex_hull_basis == 1
          ? traj_opt::convex_hull::Basis::MINVO
          : traj_opt::convex_hull::Basis::Bezier;
  opt_vars_.convex_hull_require_certification =
      cfg_.convex_hull_require_certification;
  opt_vars_.convex_hull_phase2_objective_active = false;
  opt_vars_.phase2_triggered = false;
  opt_vars_.phase2_packed_constraints = 0;
  opt_vars_.stage_a_fine_segments = 0;
  opt_vars_.convex_hull_phase2_top_k =
      std::max(1, cfg_.convex_hull_polish_top_k);
  opt_vars_.convex_hull_phase2_max_constraints =
      std::max(opt_vars_.convex_hull_phase2_top_k,
               cfg_.convex_hull_polish_max_constraints);
  opt_vars_.convex_hull_phase2_max_outer =
      std::max(1, cfg_.convex_hull_polish_max_outer);
  opt_vars_.convex_hull_phase2_inner_tol_init =
      std::max(1.0e-12, cfg_.convex_hull_polish_inner_tol_init);
  opt_vars_.convex_hull_phase2_inner_tol_final =
      std::max(1.0e-12, cfg_.convex_hull_polish_inner_tol_final);
  if (opt_vars_.convex_hull_phase2_inner_tol_final >
      opt_vars_.convex_hull_phase2_inner_tol_init)
  {
    opt_vars_.convex_hull_phase2_inner_tol_final =
        opt_vars_.convex_hull_phase2_inner_tol_init;
  }
  opt_vars_.convex_hull_phase2_append_en =
      cfg_.convex_hull_polish_append_en;
  opt_vars_.convex_hull_phase2_multiplier_reuse_en =
      cfg_.convex_hull_polish_multiplier_reuse_en;
  opt_vars_.convex_hull_flatness_enabled =
      cfg_.convex_hull_en && cfg_.convex_hull_flatness_en;
  opt_vars_.quadrotor_flatness = cfg_.quadrotot_flatness;
  opt_vars_.guide_z_tube_radius = std::max(0.0, cfg_.guide_z_tube_radius);
  // The generic lateral guide integral remains disabled.  The dedicated z
  // lower-bound term below is independently wired into every ExpTrajOpt
  // evaluator and therefore cannot silently disappear with this legacy path.
  opt_vars_.weight_guide_integral = 0.0;
  opt_vars_.guide_path_tube_radius = std::max(0.0, cfg_.guide_path_tube_radius);
  opt_vars_.guide_path_z_tube_radius = std::max(0.0, cfg_.guide_path_z_tube_radius);
  opt_vars_.guide_path_huber_delta = std::max(0.0, cfg_.guide_path_huber_delta);
  opt_vars_.guide_path_time_gradient_en = cfg_.guide_path_time_gradient_en;
  opt_vars_.weight_guide_z_tube =
      opt_vars_.guide_z_tube_radius > 0.0
          ? std::max(0.0, active_penalties.guide_z_tube)
          : 0.0;

  linear_time_cost_.weight = opt_vars_.rho;
  optimizer_.setTimeMap(&time_map_);
  optimizer_.setSpatialMap(&spatial_map_);
  optimizer_.setEnergyWeight(opt_vars_.block_energy_cost ? 0.0 : 1.0);
  // Stable state2state route: the depth-2 V2 hull owns polynomial P/V/A,
  // while the exact nonlinear FlatnessMap remains in the real objective.
  // The current flatness envelope is shadow-only until its non-convex
  // angular-rate bound is replaced by a rigorous Bernstein certificate.
  {
    // P/V/A are removed from this grid by the hull manager. Keep enough true
    // FlatnessMap nodes to expose angular/thrust peaks to the primary solve;
    // two nodes were too sparse and left the certificate-triggered polish
    // with violations too large to recover reliably.
    const int dense_samples =
        opt_vars_.convex_hull_enabled
            ? (opt_vars_.convex_hull_flatness_enabled ? 8 : 2)
            : opt_vars_.integral_res;
    optimizer_.setSamplesPerPiece(dense_samples);
  }
  optimizer_.setTimingEnabled(true);
  {
    exp_convex_cost_manager_.configure(
        opt_vars_.convex_hull_basis,
        opt_vars_.convex_hull_subdivision_depth,
        cfg_.convex_hull_position_scale,
        cfg_.convex_hull_robust_certificate_margin);
    {
      traj_opt::convex_hull::FlatnessConvexHullCost::Config flat_cfg;
      flat_cfg.mass = cfg_.mass;
      flat_cfg.gravity = cfg_.grav;
      flat_cfg.horizontal_drag = cfg_.dh;
      flat_cfg.vertical_drag = cfg_.dv;
      flat_cfg.parasitic_drag = cfg_.cp;
      flat_cfg.speed_smoothing = cfg_.v_eps;
      flat_cfg.max_angular_rate = cfg_.max_omg;
      flat_cfg.max_tilt_angle =
          std::clamp(cfg_.max_tilt, 1.0e-3, 0.5 * M_PI - 1.0e-3);
      flat_cfg.min_thrust = cfg_.min_acc_thr * cfg_.mass;
      flat_cfg.max_thrust = cfg_.max_acc_thr * cfg_.mass;
      flat_cfg.smooth_epsilon = cfg_.smooth_eps;
      flat_cfg.weights.angular_rate =
          std::max(0.0, active_penalties.angular_rate);
      flat_cfg.weights.thrust =
          std::max(0.0, active_penalties.thrust);
      flat_cfg.weights.tilt = std::max(flat_cfg.weights.thrust,
                                       flat_cfg.weights.angular_rate);
      flat_cfg.weights.force_projection = flat_cfg.weights.thrust;
      flat_cfg.weights.velocity_trust_region =
          10.0 * std::max(flat_cfg.weights.angular_rate,
                          flat_cfg.weights.thrust);
      exp_convex_cost_manager_.configureFlatness(
          opt_vars_.convex_hull_flatness_enabled &&
              opt_vars_.convex_hull_basis ==
                  traj_opt::convex_hull::Basis::Bezier,
          flat_cfg,
          /*advisory_only=*/true);
      opt_vars_.convex_hull_flatness_enabled =
          exp_convex_cost_manager_.usesFlatnessHull();
    }
  }
  {
    cost_functional_manager::ExpPackedCorrectorCostManager::Options
        packed_options;
    packed_options.position_scale = cfg_.convex_hull_position_scale;
    packed_options.max_constraints =
        opt_vars_.convex_hull_phase2_max_constraints;
    packed_options.flatness_enabled =
        opt_vars_.convex_hull_flatness_enabled &&
        !exp_convex_cost_manager_.flatnessAdvisoryOnly();
    packed_options.flatness = exp_convex_cost_manager_.flatnessConfig();
    exp_packed_corrector_cost_manager_.configure(packed_options);
  }
  last_timing_report_.mode =
      opt_vars_.convex_hull_enabled
          ? (opt_vars_.convex_hull_basis ==
                     traj_opt::convex_hull::Basis::MINVO
                 ? (opt_vars_.convex_hull_require_certification
                        ? "convex_minvo_d2_cert"
                        : "convex_minvo_d2_stable_monitor")
                 : opt_vars_.convex_hull_flatness_enabled
                       ? (opt_vars_.convex_hull_require_certification
                              ? "convex_bezier_v2_d2_cert_flatness_shadow"
                              : "convex_bezier_v2_d2_stable_flatness_shadow")
                       : (opt_vars_.convex_hull_require_certification
                              ? "convex_bezier_v2_d2_cert"
                              : "convex_bezier_v2_d2_stable_monitor"))
          : "dense";
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

void ExpTrajOpt::setGuideZFloorReference(double altitude)
{
  opt_vars_.guide_z_floor_reference = std::isfinite(altitude)
                                     ? altitude
                                     : std::numeric_limits<double>::quiet_NaN();
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
  int guide_overlap_fallback_count = 0;
  for (int j = 0; j < opt_vars_.waypoint_attractor.cols(); ++j)
  {
    const Vec3f chebyshev_center = opt_vars_.waypoint_attractor.col(j);
    const double chebyshev_dead_d = opt_vars_.waypoint_attractor_dead_d(j);
    GuideOverlapSample guide_sample;
    if (findGuidePointInOverlap(opt_vars_.guide_path,
                                opt_vars_.guide_t,
                                opt_vars_.h_overlap_polytopes[j],
                                chebyshev_center,
                                time_stamps(j),
                                guide_sample))
    {
      opt_vars_.waypoint_attractor.col(j) = guide_sample.position;
      opt_vars_.points.col(j) = guide_sample.position;
      opt_vars_.waypoint_attractor_dead_d(j) =
          std::min(chebyshev_dead_d, std::max(1.0e-3, 0.5 * guide_sample.clearance));
      time_stamps(j + 1) = std::clamp(guide_sample.time,
                                      time_stamps(0),
                                      time_stamps(time_stamps.size() - 1));
    }
    else
    {
      double min_dis = std::numeric_limits<double>::max();
      int min_id = 0;
      for (int i = 0; i < static_cast<int>(opt_vars_.guide_path.size()); ++i)
      {
        const double dis = (opt_vars_.guide_path[i] - chebyshev_center).norm();
        if (dis < min_dis)
        {
          min_dis = dis;
          min_id = i;
        }
      }
      opt_vars_.points.col(j) = chebyshev_center;
      time_stamps(j + 1) = opt_vars_.guide_t[min_id];
      ++guide_overlap_fallback_count;
    }
  }

  for (int i = 1; i < time_stamps.size(); ++i)
  {
    opt_vars_.times(i - 1) = std::max(0.01, time_stamps(i) - time_stamps(i - 1));
  }
  if (cfg_.print_optimizer_log && guide_overlap_fallback_count > 0)
  {
    std::cout << YELLOW << " -- [ExpTrajOpt] Guide-overlap waypoint fallback count: "
              << guide_overlap_fallback_count << RESET << std::endl;
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
    // The guide trajectory is already the best available timing warm start.
    // A fixed 0.8 compression amplified velocity/acceleration/jerk by roughly
    // 1.25/1.56/1.95 before every solve and forced LBFGS to undo artificial
    // dynamic violations. Keep the scale configurable for reproducible A/B
    // tests, with 1.0 as the fast-path default.
    opt_vars_.times *= opt_vars_.guide_initial_time_scale;
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
  return static_cast<ExpTrajOpt *>(ptr)->evaluateMincoCost(x, g);
}

math_utils::FastLbfgs::PhysicalSnapshot ExpTrajOpt::fastLbfgsSnapshot(
    void *ptr, const Eigen::VectorXd &x)
{
  auto *self = static_cast<ExpTrajOpt *>(ptr);
  const auto &vars = self->opt_vars_;
  math_utils::FastLbfgs::PhysicalSnapshot snap;
  snap.penalty_log = vars.penalty_log;
  snap.corridor_scale =
      std::max(1.0e-3, self->cfg_.convex_hull_position_scale);

  const int time_dim =
      std::min<int>(vars.piece_num, static_cast<int>(x.size()));
  snap.durations.resize(time_dim);
  for (int i = 0; i < time_dim; ++i)
  {
    snap.durations(i) = self->time_map_.toTime(x(i));
  }

  snap.waypoints.resize(3, std::max(0, vars.piece_num - 1));
  int offset = time_dim;
  for (int i = 1; i < vars.piece_num; ++i)
  {
    const int dof = self->spatial_map_.getUnconstrainedDim(i);
    if (offset + dof > x.size())
    {
      break;
    }
    snap.waypoints.col(i - 1) =
        self->spatial_map_.toPhysical(x.segment(offset, dof), i);
    offset += dof;
  }
  return snap;
}

void ExpTrajOpt::configureFastLbfgs(double rel_cost_tol,
                                    bool early_stop_enabled,
                                    int decision_dim)
{
  math_utils::FastLbfgs::Options options;
  options.early_stop_enabled = early_stop_enabled;
  // Match legacy: fast path caps mem by problem dim; classical uses 256.
  options.mem_size =
      opt_vars_.lbfgs_fast_enabled
          ? std::min(opt_vars_.lbfgs_mem_size,
                     std::max(3, decision_dim))
          : 256;
  options.fallback_mem_size = 256;
  options.fallback_on_failure = early_stop_enabled;
  options.step_bound_enabled = opt_vars_.lbfgs_step_bound_enabled;
  options.window = opt_vars_.lbfgs_fast_window;
  options.min_iterations = opt_vars_.lbfgs_fast_min_iterations;
  options.consecutive = opt_vars_.lbfgs_fast_consecutive;
  options.rel_cost = opt_vars_.lbfgs_fast_rel_cost;
  options.rel_step = opt_vars_.lbfgs_fast_rel_step;
  options.rel_penalty = opt_vars_.lbfgs_fast_rel_penalty;
  options.phase0_guards_en = opt_vars_.lbfgs_fast_phase0_guards_en;
  options.rel_time = opt_vars_.lbfgs_fast_rel_time;
  options.rel_waypoint = opt_vars_.lbfgs_fast_rel_waypoint;
  options.scaled_grad = opt_vars_.lbfgs_fast_scaled_grad;
  options.min_step_for_stall = opt_vars_.lbfgs_fast_min_step;
  options.penalty_tol = opt_vars_.lbfgs_fast_penalty_tol;
  options.small_step_limit = opt_vars_.lbfgs_fast_small_step_limit;
  options.past = 3;
  options.delta = rel_cost_tol;
  options.g_epsilon = 0.0;
  options.min_step = 1.0e-32;
  options.max_linesearch = 64;
  options.fallback_max_linesearch = 128;
  options.fallback_min_step = 1.0e-20;
  fast_lbfgs_.setOptions(options);
}

void ExpTrajOpt::syncFastLbfgsReport()
{
  const auto &report = fast_lbfgs_.report();
  opt_vars_.lbfgs_iterations = report.iterations;
  opt_vars_.line_search_evaluations = report.line_search_evaluations;
  opt_vars_.max_line_search_evaluations =
      report.max_line_search_evaluations;
  opt_vars_.accepted_step_sum = report.accepted_step_sum;
  opt_vars_.min_accepted_step = report.min_accepted_step;
  opt_vars_.fast_stop_satisfied = report.fast_stop_satisfied;
  opt_vars_.fast_stop_iteration = report.fast_stop_iteration;
  opt_vars_.fast_fallback_used =
      opt_vars_.fast_fallback_used || report.fallback_used;
  opt_vars_.quality.relative_cost_change = report.relative_cost_change;
  opt_vars_.quality.relative_decision_step = report.relative_decision_step;
  opt_vars_.quality.relative_physical_time_change =
      report.relative_physical_time_change;
  opt_vars_.quality.relative_waypoint_step = report.relative_waypoint_step;
  opt_vars_.quality.scaled_gradient_inf = report.scaled_gradient_inf;
  opt_vars_.quality.min_accepted_step = report.min_accepted_step;
  opt_vars_.quality.max_sampled_violation = report.max_sampled_violation;
  opt_vars_.quality.trajectory_stable = report.trajectory_stable;
}

bool ExpTrajOpt::buildWarmStartCandidate(VecDf &candidate)
{
  candidate.resize(0);
  opt_vars_.warm_start_status = 0;
  opt_vars_.warm_start_max_waypoint_shift = 0.0;
  opt_vars_.warm_start_topology_resampled = false;

  if (!opt_vars_.lbfgs_warm_start_enabled ||
      opt_vars_.given_init_ts_and_ps || !warm_start_cache_.valid)
  {
    return false;
  }
  if (warm_start_cache_.piece_num <= 0 ||
      warm_start_cache_.durations.size() != warm_start_cache_.piece_num ||
      warm_start_cache_.trajectory.empty() ||
      warm_start_cache_.trajectory.getPieceNum() !=
          warm_start_cache_.piece_num)
  {
    opt_vars_.warm_start_status = 2;
    return false;
  }
  if (warm_start_cache_.piece_num != opt_vars_.piece_num ||
      warm_start_cache_.inner_points.cols() != opt_vars_.piece_num - 1)
  {
    opt_vars_.warm_start_status = 2;
    return false;
  }

  const double head_shift =
      (warm_start_cache_.head - opt_vars_.head_pvaj.col(0)).norm();
  const double tail_shift =
      (warm_start_cache_.tail - opt_vars_.tail_pvaj.col(0)).norm();
  if (!std::isfinite(head_shift) || !std::isfinite(tail_shift) ||
      head_shift > opt_vars_.lbfgs_warm_start_max_endpoint_shift ||
      tail_shift > opt_vars_.lbfgs_warm_start_max_endpoint_shift)
  {
    opt_vars_.warm_start_status = 3;
    return false;
  }

  Mat3Df warm_points(3, std::max(0, opt_vars_.piece_num - 1));
  opt_vars_.warm_start_topology_resampled = false;
  const double guide_total = opt_vars_.times.sum();
  const double cached_total = warm_start_cache_.trajectory.getTotalDuration();
  if (!std::isfinite(guide_total) || !std::isfinite(cached_total) ||
      guide_total <= 1.0e-3 || cached_total <= 1.0e-3)
  {
    opt_vars_.warm_start_status = 2;
    return false;
  }
  for (int i = 0; i < warm_points.cols(); ++i)
  {
    const Vec3f cached_point = warm_start_cache_.inner_points.col(i);
    const Vec3f guide_point = opt_vars_.points.col(i);
    if (!cached_point.allFinite() || !guide_point.allFinite() ||
        i >= static_cast<int>(opt_vars_.h_overlap_polytopes.size()))
    {
      opt_vars_.warm_start_status = 4;
      return false;
    }

    Vec3f delta = cached_point - guide_point;
    const double raw_shift = delta.norm();
    if (!std::isfinite(raw_shift))
    {
      opt_vars_.warm_start_status = 5;
      return false;
    }
    double blend_limit = 1.0;
    if (raw_shift > opt_vars_.lbfgs_warm_start_max_waypoint_shift &&
        raw_shift > 1.0e-12)
    {
      blend_limit =
          opt_vars_.lbfgs_warm_start_max_waypoint_shift / raw_shift;
    }
    double s_min = 0.0;
    double s_max = 1.0;
    if (!clipGuideSegmentToHPoly(
            guide_point,
            cached_point,
            opt_vars_.h_overlap_polytopes[static_cast<std::size_t>(i)],
            1.0e-4,
            s_min,
            s_max))
    {
      opt_vars_.warm_start_status = 4;
      return false;
    }
    const double blend =
        std::clamp(std::min(blend_limit, 0.95 * s_max), 0.0, 1.0);
    const Vec3f point = guide_point + blend * delta;
    if (!point.allFinite() ||
        !geometry_utils::pointInsidePolytope(
            point,
            opt_vars_.h_overlap_polytopes[static_cast<std::size_t>(i)]))
    {
      opt_vars_.warm_start_status = 4;
      return false;
    }
    const double shift = (point - guide_point).norm();
    opt_vars_.warm_start_max_waypoint_shift =
        std::max(opt_vars_.warm_start_max_waypoint_shift, shift);
    if (!std::isfinite(shift))
    {
      opt_vars_.warm_start_status = 5;
      return false;
    }
    warm_points.col(i) = point;
  }

  VecDf warm_times(opt_vars_.piece_num);
  const double blend = opt_vars_.lbfgs_warm_start_duration_blend;
  for (int i = 0; i < opt_vars_.piece_num; ++i)
  {
    const double guide_time = opt_vars_.times(i);
    const double cached_time = warm_start_cache_.durations(i);
    if (!std::isfinite(guide_time) || !std::isfinite(cached_time) ||
        guide_time <= 1.0e-3 || cached_time <= 1.0e-3)
    {
      opt_vars_.warm_start_status = 6;
      return false;
    }
    const double lower =
        opt_vars_.lbfgs_time_ratio_min * guide_time;
    const double upper =
        opt_vars_.lbfgs_time_ratio_max * guide_time;
    warm_times(i) = std::clamp(
        (1.0 - blend) * guide_time + blend * cached_time,
        lower,
        upper);
  }

  const Mat3Df physical_waypoints =
      waypointsToMatrix(opt_vars_.head_pvaj,
                        warm_points,
                        opt_vars_.tail_pvaj);
  candidate = optimizer_.encodeDecisionVector(
      toStdVector(warm_times),
      toOptimizerWaypoints(physical_waypoints));
  if (candidate.size() <= 0 || !candidate.allFinite())
  {
    candidate.resize(0);
    opt_vars_.warm_start_status = 6;
    return false;
  }
  return true;
}

void ExpTrajOpt::updateWarmStartCache(const Trajectory &traj)
{
  if (!opt_vars_.lbfgs_warm_start_enabled || traj.empty() ||
      traj.getPieceNum() <= 0)
  {
    return;
  }

  const VecDf durations = traj.getDurations();
  if (durations.size() != traj.getPieceNum() || !durations.allFinite() ||
      durations.minCoeff() <= 1.0e-3)
  {
    return;
  }

  WarmStartCache next;
  next.valid = true;
  next.piece_num = traj.getPieceNum();
  next.durations = durations;
  next.inner_points.resize(3, std::max(0, next.piece_num - 1));
  double time = 0.0;
  for (int i = 0; i < next.piece_num - 1; ++i)
  {
    time += durations(i);
    next.inner_points.col(i) = traj.getPos(time);
  }
  next.head = traj.getPos(0.0);
  next.tail = traj.getPos(traj.getTotalDuration());
  next.trajectory = traj;
  if (!next.inner_points.allFinite() || !next.head.allFinite() ||
      !next.tail.allFinite())
  {
    return;
  }
  warm_start_cache_ = std::move(next);
}

double ExpTrajOpt::stepBoundFunctional(void *ptr,
                                       const VecDf &x,
                                       const VecDf &direction)
{
  auto *self = static_cast<ExpTrajOpt *>(ptr);
  const auto &vars = self->opt_vars_;
  if (!vars.lbfgs_step_bound_enabled || x.size() != direction.size())
  {
    return 1.0e20;
  }

  const int time_dim =
      std::min<int>(vars.piece_num, static_cast<int>(x.size()));
  double step_bound = 1.0e20;
  for (int i = 0; i < time_dim; ++i)
  {
    const double d = direction(i);
    if (!std::isfinite(d) || std::abs(d) <= 1.0e-16)
    {
      continue;
    }

    const double duration = self->time_map_.toTime(x(i));
    if (!std::isfinite(duration) || duration <= 0.0)
    {
      continue;
    }

    if (d > 0.0)
    {
      const double upper_tau = self->time_map_.toTau(
          duration * vars.lbfgs_time_ratio_max);
      step_bound = std::min(step_bound, (upper_tau - x(i)) / d);
    }
    else
    {
      const double lower_tau = self->time_map_.toTau(
          duration * vars.lbfgs_time_ratio_min);
      step_bound = std::min(step_bound, (lower_tau - x(i)) / d);
    }
  }

  if (!std::isfinite(step_bound) || step_bound >= 1.0e20)
  {
    return 1.0e20;
  }
  return std::max(1.0e-16, 0.999 * step_bound);
}

double ExpTrajOpt::evaluateMincoCost(const VecDf &x, VecDf &g)
{
  opt_vars_.iter_num++;
  double cost = 0.0;
  if (opt_vars_.convex_hull_phase2_objective_active)
  {
    cost = optimizer_.evaluate(
        x, g, linear_time_cost_, exp_packed_corrector_cost_manager_);
    opt_vars_.guide_integral_violation =
        exp_packed_corrector_cost_manager_.guideIntegralViolation();
    opt_vars_.guide_path_cost_log =
        exp_packed_corrector_cost_manager_.guideCostLog();
    opt_vars_.guide_path_max_abs_time_grad =
        exp_packed_corrector_cost_manager_.guideMaxAbsTimeGrad();
    opt_vars_.guide_path_out_of_time_range_samples =
        exp_packed_corrector_cost_manager_.guideOutOfTimeRangeSamples();
    opt_vars_.penalty_log.tail(7) =
        exp_packed_corrector_cost_manager_.getPenaltyLog().segment(1, 7);
    opt_vars_.guide_z_tube_violation =
        exp_packed_corrector_cost_manager_.guideZLowerViolation();
  }
  else if (opt_vars_.convex_hull_enabled)
  {
    cost = optimizer_.evaluate(
        x, g, linear_time_cost_, exp_convex_cost_manager_);
    opt_vars_.guide_integral_violation =
        exp_convex_cost_manager_.guideIntegralViolation();
    opt_vars_.guide_path_cost_log =
        exp_convex_cost_manager_.guideCostLog();
    opt_vars_.guide_path_max_abs_time_grad =
        exp_convex_cost_manager_.guideMaxAbsTimeGrad();
    opt_vars_.guide_path_out_of_time_range_samples =
        exp_convex_cost_manager_.guideOutOfTimeRangeSamples();
    opt_vars_.penalty_log.tail(7) =
        exp_convex_cost_manager_.getPenaltyLog().segment(1, 7);
    opt_vars_.guide_z_tube_violation =
        exp_convex_cost_manager_.guideZLowerViolation();
  }
  else
  {
    cost = optimizer_.evaluate(x, g, linear_time_cost_, exp_cost_manager_);
    opt_vars_.guide_integral_violation = exp_cost_manager_.guideIntegralViolation();
    opt_vars_.guide_path_cost_log = exp_cost_manager_.guideCostLog();
    opt_vars_.guide_path_max_abs_time_grad = exp_cost_manager_.guideMaxAbsTimeGrad();
    opt_vars_.guide_path_out_of_time_range_samples =
        exp_cost_manager_.guideOutOfTimeRangeSamples();
    opt_vars_.penalty_log.tail(7) =
        exp_cost_manager_.getPenaltyLog().segment(1, 7);
    opt_vars_.guide_z_tube_violation = exp_cost_manager_.guideZLowerViolation();
  }
  opt_vars_.penalty_log(0) = optimizer_.lastEnergyCost();
  opt_vars_.penalty_log(5) = std::max({opt_vars_.penalty_log(5),
                                       opt_vars_.guide_integral_violation});
  return cost;
}

void ExpTrajOpt::maybeUpdateCertifiedIncumbent(const VecDf &x, double cost)
{
  if (!opt_vars_.convex_hull_enabled || !x.allFinite() ||
      !std::isfinite(cost))
  {
    return;
  }
  if (!optimizer_.updateTrajectoryFromDecisionVector(x))
  {
    return;
  }
  const auto certificate =
      exp_convex_cost_manager_.computeContinuousCertificate(
          optimizer_.getTrajectory(),
          /*evaluate_flatness_advisory=*/false);
  opt_vars_.last_certificate = certificate;
  if (!certificate.continuous_feasible)
  {
    return;
  }
  if (!opt_vars_.has_certified_incumbent ||
      cost < opt_vars_.certified_incumbent_cost)
  {
    opt_vars_.has_certified_incumbent = true;
    opt_vars_.certified_incumbent_x = x;
    opt_vars_.certified_incumbent_cost = cost;
  }
}

void ExpTrajOpt::fillPostSolveQualityReport(const VecDf &x,
                                            const VecDf &grad)
{
  const SolverQualityReport progress_snapshot = opt_vars_.quality;
  SolverQualityReport &report = opt_vars_.quality;
  report = SolverQualityReport{};
  report.relative_cost_change = progress_snapshot.relative_cost_change;
  report.relative_decision_step = progress_snapshot.relative_decision_step;
  report.relative_physical_time_change =
      progress_snapshot.relative_physical_time_change;
  report.relative_waypoint_step = progress_snapshot.relative_waypoint_step;
  report.scaled_gradient_inf = progress_snapshot.scaled_gradient_inf;
  report.min_accepted_step = opt_vars_.min_accepted_step;
  report.trajectory_stable = opt_vars_.fast_stop_satisfied;
  report.has_certified_incumbent = opt_vars_.has_certified_incumbent;

  if (opt_vars_.penalty_log.size() > 1 && opt_vars_.penalty_log.allFinite())
  {
    report.max_sampled_violation =
        opt_vars_.penalty_log.tail(opt_vars_.penalty_log.size() - 1)
            .lpNorm<Eigen::Infinity>();
    report.sampled_feasible =
        report.max_sampled_violation <= opt_vars_.lbfgs_fast_penalty_tol;
  }

  if (opt_vars_.convex_hull_enabled &&
      optimizer_.updateTrajectoryFromDecisionVector(x))
  {
    const auto certificate =
        exp_convex_cost_manager_.computeContinuousCertificate(
            optimizer_.getTrajectory());
    opt_vars_.last_certificate = certificate;
    report.continuous_feasible = certificate.continuous_feasible;
    report.robustly_certified = certificate.robustly_certified;
    report.max_position_violation = certificate.max_position_violation;
    report.max_derivative_violation = std::max(
        {certificate.max_velocity_violation,
         certificate.max_acceleration_violation,
         certificate.max_jerk_violation});
    report.max_normalized_violation = certificate.max_normalized_violation;
    report.min_position_margin = certificate.min_position_margin;
    report.min_derivative_margin = std::min(
        {certificate.min_velocity_margin,
         certificate.min_acceleration_margin,
         certificate.min_jerk_margin});
    report.scalar_constraint_checks = certificate.scalar_constraint_checks;
    report.max_depth_used = certificate.max_depth_used;
    report.unresolved_leaves = certificate.unresolved_leaves;
  }

  if (grad.size() == x.size() && grad.allFinite() && x.allFinite())
  {
    double scaled = 0.0;
    for (Eigen::Index i = 0; i < x.size(); ++i)
    {
      scaled = std::max(scaled,
                        std::abs(grad(i)) / std::max(1.0, std::abs(x(i))));
    }
    report.scaled_gradient_inf = scaled;
    report.stationarity_residual = scaled;
    report.approximately_kkt =
        report.continuous_feasible &&
        scaled <= opt_vars_.lbfgs_fast_scaled_grad;
  }

  report.strictly_polished = false;
  last_quality_report_ = report;
}

bool ExpTrajOpt::runPhase2PackedCorrection(
    VecDf &x,
    double &min_cost,
    double rel_cost_tol,
    int lbfgs_mem_size,
    std::size_t &outer_iterations,
    std::size_t &inner_solves,
    std::size_t &topology_changes,
    double &max_violation,
    bool &correction_certified)
{
  correction_certified = false;
  max_violation = 0.0;
  opt_vars_.phase2_triggered = false;
  opt_vars_.phase2_packed_constraints = 0;
  if (!opt_vars_.convex_hull_enabled ||
      !optimizer_.updateTrajectoryFromDecisionVector(x))
  {
    return false;
  }

  auto certificate =
      exp_convex_cost_manager_.computeContinuousCertificate(
          optimizer_.getTrajectory(),
          /*evaluate_flatness_advisory=*/false);
  opt_vars_.last_certificate = certificate;
  max_violation = certificate.max_normalized_violation;
  if (certificate.continuous_feasible)
  {
    correction_certified = true;
    maybeUpdateCertifiedIncumbent(x, min_cost);
    return true;
  }
  // Only enter packed correction when the oracle reports concrete violations.
  if (certificate.violated.empty())
  {
    std::cout << YELLOW
              << " -- [ExpTrajOpt] Phase2 skip: infeasible but no packable "
                 "violations (unresolved="
              << certificate.unresolved_leaves
              << ", nviol=" << certificate.max_normalized_violation << ")"
              << RESET << std::endl;
    return false;
  }

  const bool reuse = opt_vars_.convex_hull_phase2_multiplier_reuse_en &&
                     phase2_cached_signature_ != 0 &&
                     phase2_cached_multipliers_.size() > 0;
  const auto *previous_set = reuse ? &phase2_cached_pack_ : nullptr;
  const Eigen::VectorXd *previous_multipliers =
      reuse ? &phase2_cached_multipliers_ : nullptr;
  auto packed = traj_opt::convex_hull::packConstraintCandidates(
      certificate.violated,
      opt_vars_.convex_hull_phase2_top_k,
      previous_multipliers,
      previous_set);
  traj_opt::convex_hull::appendFlatnessTrustRegionGuards(
      packed,
      /*velocity_controls_per_leaf=*/SNAP_TRAJ_ORDER,
      opt_vars_.convex_hull_phase2_max_constraints);
  if (packed.constraints.empty())
  {
    return false;
  }

  opt_vars_.phase2_triggered = true;
  opt_vars_.phase2_packed_constraints = packed.constraints.size();
  std::cout << " -- [ExpTrajOpt] Phase2 trigger: packed_constraints="
            << opt_vars_.phase2_packed_constraints
            << ", oracle_nviol=" << max_violation
            << ", jerk_cert="
            << (certificate.jerk_certificate_enabled ? 1 : 0) << std::endl;

  Eigen::VectorXd multipliers =
      traj_opt::convex_hull::seedMultipliers(packed);
  double penalty = cfg_.convex_hull_polish_initial_penalty;
  if (reuse &&
      packed.topology_signature == phase2_cached_signature_ &&
      phase2_cached_penalty_ > 0.0)
  {
    penalty = phase2_cached_penalty_;
  }

  if (!exp_packed_corrector_cost_manager_.initializeFromPacked(
          packed, multipliers, penalty))
  {
    return false;
  }

  opt_vars_.convex_hull_phase2_objective_active = true;
  // Keep cumulative LBFGS counters; only clear early-stop window state.
  fast_lbfgs_.resetStopHistory();
  {
    auto options = fast_lbfgs_.options();
    options.mem_size = std::max(3, lbfgs_mem_size);
    options.min_step = 1.0e-20;
    options.max_linesearch = 64;
    // Phase-2 runs under the same early-stop policy as the outer solve.
    fast_lbfgs_.setOptions(options);
  }

  const int max_outer = opt_vars_.convex_hull_phase2_max_outer;
  const int max_attempts = std::max(8, max_outer * 4);
  double previous_violation = std::numeric_limits<double>::infinity();
  int outer = 0;
  for (int attempt = 0; attempt < max_attempts && outer < max_outer;
       ++attempt)
  {
    const double t =
        max_outer <= 1 ? 1.0
                       : static_cast<double>(outer) /
                             static_cast<double>(max_outer - 1);
    const double log_init =
        std::log(opt_vars_.convex_hull_phase2_inner_tol_init);
    const double log_final =
        std::log(opt_vars_.convex_hull_phase2_inner_tol_final);
    const double inner_tol = std::exp((1.0 - t) * log_init + t * log_final);

    {
      auto options = fast_lbfgs_.options();
      options.delta = std::max(rel_cost_tol, inner_tol);
      fast_lbfgs_.setOptions(options);
    }
    exp_packed_corrector_cost_manager_.setPhrState(multipliers, penalty);
    const VecDf accepted_x = x;
    const double violation_before =
        std::max(0.0, certificate.max_normalized_violation);
    VecDf model_gradient = VecDf::Zero(x.size());
    const double model_before = evaluateMincoCost(x, model_gradient);
    opt_vars_.convex_hull_phase2_objective_active = false;
    VecDf real_gradient = VecDf::Zero(x.size());
    const double real_before = evaluateMincoCost(x, real_gradient);
    opt_vars_.convex_hull_phase2_objective_active = true;
    const double real_merit_before =
        real_before + 0.5 * penalty * violation_before * violation_before;

    ++inner_solves;
    const int status = fast_lbfgs_.run(x,
                                       min_cost,
                                       &ExpTrajOpt::costFunctional,
                                       nullptr,
                                       this,
                                       &ExpTrajOpt::fastLbfgsSnapshot,
                                       /*allow_fallback=*/false);
    syncFastLbfgsReport();
    const bool accepted_fast = fast_lbfgs_.acceptedFastStop();
    if (status < 0 && !accepted_fast)
    {
      break;
    }

    if (!optimizer_.updateTrajectoryFromDecisionVector(x))
    {
      break;
    }
    certificate =
        exp_convex_cost_manager_.computeContinuousCertificate(
            optimizer_.getTrajectory(),
            /*evaluate_flatness_advisory=*/false);
    opt_vars_.last_certificate = certificate;
    max_violation = certificate.max_normalized_violation;

    if (exp_packed_corrector_cost_manager_.hasFlatnessConstraints())
    {
      opt_vars_.convex_hull_phase2_objective_active = false;
      VecDf trial_gradient = VecDf::Zero(x.size());
      const double real_after = evaluateMincoCost(x, trial_gradient);
      opt_vars_.convex_hull_phase2_objective_active = true;
      const double violation_after =
          std::max(0.0, certificate.max_normalized_violation);
      const double real_merit_after =
          real_after + 0.5 * penalty * violation_after * violation_after;
      const double predicted_reduction = model_before - min_cost;
      const double actual_reduction =
          real_merit_before - real_merit_after;
      const double ratio =
          actual_reduction /
          std::max(1.0e-12, predicted_reduction);
      opt_vars_.predicted_reduction = predicted_reduction;
      opt_vars_.actual_reduction = actual_reduction;
      opt_vars_.trust_region_ratio = ratio;

      if (!std::isfinite(real_after) ||
          !std::isfinite(actual_reduction) ||
          !std::isfinite(ratio) ||
          !(predicted_reduction > 0.0) ||
          ratio < 0.1)
      {
        x = accepted_x;
        min_cost = model_before;
        ++opt_vars_.trust_region_rejections;
        exp_packed_corrector_cost_manager_.shrinkFlatnessTrustRegions(0.5);
        fast_lbfgs_.resetStopHistory();
        optimizer_.updateTrajectoryFromDecisionVector(x);
        certificate =
            exp_convex_cost_manager_.computeContinuousCertificate(
                optimizer_.getTrajectory(),
                /*evaluate_flatness_advisory=*/false);
        opt_vars_.last_certificate = certificate;
        max_violation = certificate.max_normalized_violation;
        continue;
      }
    }

    const auto report =
        exp_packed_corrector_cost_manager_.inspectAndMaybeAppend(
            optimizer_.getTrajectory(),
            certificate.violated,
            opt_vars_.convex_hull_phase2_append_en);
    if (report.topology_changed)
    {
      ++topology_changes;
      multipliers = exp_packed_corrector_cost_manager_.multipliers();
      packed = exp_packed_corrector_cost_manager_.packedSet();
      continue;
    }

    ++outer;
    ++outer_iterations;
    multipliers = exp_packed_corrector_cost_manager_.multipliers();
    const Eigen::VectorXd &values =
        exp_packed_corrector_cost_manager_.constraintValues();
    const double packed_violation =
        values.size() > 0 ? std::max(0.0, values.maxCoeff()) : 0.0;
    max_violation = std::max(max_violation, packed_violation);

    if (certificate.continuous_feasible)
    {
      correction_certified = true;
      maybeUpdateCertifiedIncumbent(x, min_cost);
      break;
    }

    if (values.size() == multipliers.size() && values.size() > 0)
    {
      multipliers = (multipliers + penalty * values).cwiseMax(0.0);
    }
    if (std::isfinite(previous_violation) &&
        packed_violation >
            cfg_.convex_hull_polish_progress_ratio * previous_violation)
    {
      penalty *= cfg_.convex_hull_polish_penalty_growth;
    }
    previous_violation = packed_violation;
    exp_packed_corrector_cost_manager_.setPhrState(multipliers, penalty);
  }

  opt_vars_.convex_hull_phase2_objective_active = false;

  if (correction_certified &&
      opt_vars_.convex_hull_phase2_multiplier_reuse_en &&
      exp_packed_corrector_cost_manager_.constraintCount() > 0)
  {
    phase2_cached_pack_ = exp_packed_corrector_cost_manager_.packedSet();
    phase2_cached_multipliers_ =
        exp_packed_corrector_cost_manager_.multipliers();
    phase2_cached_penalty_ = penalty;
    phase2_cached_signature_ = phase2_cached_pack_.topology_signature;
  }
  else if (!correction_certified)
  {
    // A failed correction is not a valid rolling warm start. Reusing its
    // escalated penalty/multipliers across FSM retries progressively destroys
    // conditioning while repeating the same rejected trajectory.
    phase2_cached_pack_ = traj_opt::convex_hull::PackedConstraintSet{};
    phase2_cached_multipliers_.resize(0);
    phase2_cached_penalty_ = 0.0;
    phase2_cached_signature_ = 0;
  }

  if (!correction_certified &&
      optimizer_.updateTrajectoryFromDecisionVector(x))
  {
    certificate =
        exp_convex_cost_manager_.computeContinuousCertificate(
            optimizer_.getTrajectory(),
            /*evaluate_flatness_advisory=*/false);
    opt_vars_.last_certificate = certificate;
    max_violation = certificate.max_normalized_violation;
    correction_certified = certificate.continuous_feasible;
    if (correction_certified)
    {
      maybeUpdateCertifiedIncumbent(x, min_cost);
    }
  }
  if (!correction_certified && opt_vars_.has_certified_incumbent &&
      opt_vars_.certified_incumbent_x.size() == x.size())
  {
    // Prefer a previously certified incumbent over an uncertified correction.
    x = opt_vars_.certified_incumbent_x;
    min_cost = opt_vars_.certified_incumbent_cost;
    if (optimizer_.updateTrajectoryFromDecisionVector(x))
    {
      certificate =
          exp_convex_cost_manager_.computeContinuousCertificate(
              optimizer_.getTrajectory(),
              /*evaluate_flatness_advisory=*/false);
      opt_vars_.last_certificate = certificate;
      max_violation = certificate.max_normalized_violation;
      correction_certified = certificate.continuous_feasible;
    }
  }
  opt_vars_.phase2_packed_constraints = std::max(
      opt_vars_.phase2_packed_constraints,
      exp_packed_corrector_cost_manager_.constraintCount());
  return correction_certified ||
         exp_packed_corrector_cost_manager_.constraintCount() > 0;
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

  optimizer_.setUniformTimeMode(false);
  optimizer_.setEnergyWeight(opt_vars_.block_energy_cost ? 0.0 : 1.0);
  // Hull covers pos/vel/acc continuously; keep residual dense grid coarse.
  optimizer_.setSamplesPerPiece(
      opt_vars_.convex_hull_enabled
          ? (opt_vars_.convex_hull_flatness_enabled ? 8 : 2)
          : opt_vars_.integral_res);
  if (!optimizer_.setInitState(toStdVector(opt_vars_.times),
                               toOptimizerWaypoints(waypoints),
                               toSnapBoundary(opt_vars_.head_pvaj),
                               toSnapBoundary(opt_vars_.tail_pvaj)))
  {
    return INFINITY;
  }
  VecDf x = optimizer_.generateInitialGuess();
  if (x.size() <= 0 || !x.allFinite())
  {
    return INFINITY;
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
                          swarm_current_wall_time_,
                          &opt_vars_.guide_path,
                          &opt_vars_.guide_t,
                          opt_vars_.weight_guide_integral,
                          opt_vars_.guide_path_tube_radius,
                          opt_vars_.guide_path_z_tube_radius,
                          opt_vars_.guide_path_huber_delta,
                          opt_vars_.guide_path_time_gradient_en,
                          opt_vars_.weight_guide_z_tube,
                          opt_vars_.guide_z_tube_radius,
                          opt_vars_.guide_z_floor_reference);

  exp_convex_cost_manager_.reset(&opt_vars_.h_polytopes,
                                 &opt_vars_.h_poly_idx,
                                 &opt_vars_.waypoint_attractor,
                                 &opt_vars_.waypoint_attractor_dead_d,
                                 opt_vars_.smooth_eps,
                                 opt_vars_.magnitude_bounds,
                                 opt_vars_.penalty_weights,
                                 &opt_vars_.quadrotor_flatness,
                                 swarm_config_,
                                 swarm_trajs_,
                                 swarm_current_wall_time_,
                                 &opt_vars_.guide_path,
                                 &opt_vars_.guide_t,
                                 opt_vars_.weight_guide_integral,
                                 opt_vars_.guide_path_tube_radius,
                                 opt_vars_.guide_path_z_tube_radius,
                                 opt_vars_.guide_path_huber_delta,
                                 opt_vars_.guide_path_time_gradient_en,
                                 opt_vars_.weight_guide_z_tube,
                                 opt_vars_.guide_z_tube_radius,
                                 opt_vars_.guide_z_floor_reference);
  exp_packed_corrector_cost_manager_.reset(
      &opt_vars_.h_polytopes,
      &opt_vars_.h_poly_idx,
      &opt_vars_.waypoint_attractor,
      &opt_vars_.waypoint_attractor_dead_d,
      opt_vars_.smooth_eps,
      opt_vars_.magnitude_bounds,
      opt_vars_.penalty_weights,
      &opt_vars_.quadrotor_flatness,
      swarm_config_,
      swarm_trajs_,
      swarm_current_wall_time_,
      &opt_vars_.guide_path,
      &opt_vars_.guide_t,
      opt_vars_.weight_guide_integral,
      opt_vars_.guide_path_tube_radius,
      opt_vars_.guide_path_z_tube_radius,
      opt_vars_.guide_path_huber_delta,
      opt_vars_.guide_path_time_gradient_en,
      opt_vars_.weight_guide_z_tube,
      opt_vars_.guide_z_tube_radius,
      opt_vars_.guide_z_floor_reference);
  opt_vars_.convex_hull_phase2_objective_active = false;
  opt_vars_.phase2_triggered = false;
  opt_vars_.phase2_packed_constraints = 0;
  opt_vars_.stage_a_fine_segments = 0;

  opt_vars_.iter_num = 0;
  opt_vars_.lbfgs_iterations = 0;
  opt_vars_.line_search_evaluations = 0;
  opt_vars_.max_line_search_evaluations = 0;
  opt_vars_.accepted_step_sum = 0.0;
  opt_vars_.min_accepted_step = 0.0;
  opt_vars_.fast_stop_satisfied = false;
  opt_vars_.fast_stop_iteration = 0;
  opt_vars_.warm_start_status = 0;
  opt_vars_.warm_start_attempted = false;
  opt_vars_.warm_start_accepted = false;
  opt_vars_.warm_start_topology_resampled = false;
  opt_vars_.warm_start_comparison_evaluations = 0;
  opt_vars_.warm_start_seconds = 0.0;
  opt_vars_.warm_start_baseline_cost = 0.0;
  opt_vars_.warm_start_candidate_cost = 0.0;
  opt_vars_.warm_start_baseline_gradient = 0.0;
  opt_vars_.warm_start_candidate_gradient = 0.0;
  opt_vars_.warm_start_baseline_penalty = 0.0;
  opt_vars_.warm_start_candidate_penalty = 0.0;
  opt_vars_.warm_start_max_waypoint_shift = 0.0;
  opt_vars_.trust_region_rejections = 0;
  opt_vars_.trust_region_ratio = 0.0;
  opt_vars_.actual_reduction = 0.0;
  opt_vars_.predicted_reduction = 0.0;
  opt_vars_.has_certified_incumbent = false;
  opt_vars_.certified_incumbent_x.resize(0);
  opt_vars_.certified_incumbent_cost =
      std::numeric_limits<double>::infinity();
  opt_vars_.last_certificate = ContinuousCertificateReport{};
  opt_vars_.quality = SolverQualityReport{};
  last_quality_report_ = SolverQualityReport{};
  opt_vars_.fast_fallback_used = false;
  double min_cost = 0.0;

  optimizer_.resetTimingStatistics();
  const auto optimization_begin = std::chrono::steady_clock::now();
  {
    const auto warm_begin = std::chrono::steady_clock::now();
    VecDf warm_candidate;
    if (buildWarmStartCandidate(warm_candidate) &&
        warm_candidate.size() == x.size())
    {
      opt_vars_.warm_start_attempted = true;
      VecDf baseline_grad = VecDf::Zero(x.size());
      VecDf candidate_grad = VecDf::Zero(warm_candidate.size());
      const double baseline_cost = evaluateMincoCost(x, baseline_grad);
      const double baseline_penalty =
          opt_vars_.penalty_log.size() > 1 &&
                  opt_vars_.penalty_log.allFinite()
              ? opt_vars_.penalty_log.tail(
                    opt_vars_.penalty_log.size() - 1)
                    .lpNorm<Eigen::Infinity>()
              : std::numeric_limits<double>::infinity();
      const double candidate_cost =
          evaluateMincoCost(warm_candidate, candidate_grad);
      const double candidate_penalty =
          opt_vars_.penalty_log.size() > 1 &&
                  opt_vars_.penalty_log.allFinite()
              ? opt_vars_.penalty_log.tail(
                    opt_vars_.penalty_log.size() - 1)
                    .lpNorm<Eigen::Infinity>()
              : std::numeric_limits<double>::infinity();
      const double baseline_gradient =
          baseline_grad.allFinite()
              ? baseline_grad.lpNorm<Eigen::Infinity>()
              : std::numeric_limits<double>::infinity();
      const double candidate_gradient =
          candidate_grad.allFinite()
              ? candidate_grad.lpNorm<Eigen::Infinity>()
              : std::numeric_limits<double>::infinity();
      opt_vars_.warm_start_comparison_evaluations = 2;
      opt_vars_.warm_start_baseline_cost = baseline_cost;
      opt_vars_.warm_start_candidate_cost = candidate_cost;
      opt_vars_.warm_start_baseline_gradient = baseline_gradient;
      opt_vars_.warm_start_candidate_gradient = candidate_gradient;
      opt_vars_.warm_start_baseline_penalty = baseline_penalty;
      opt_vars_.warm_start_candidate_penalty = candidate_penalty;
      const bool cost_better =
          candidate_cost <=
          opt_vars_.lbfgs_warm_start_cost_ratio * baseline_cost;
      const bool gradient_not_worse =
          candidate_gradient <=
          opt_vars_.lbfgs_warm_start_gradient_ratio *
              std::max(1.0e-12, baseline_gradient);
      const bool penalty_not_worse =
          candidate_penalty <=
          std::max(opt_vars_.lbfgs_fast_penalty_tol,
                   opt_vars_.lbfgs_warm_start_penalty_ratio *
                       baseline_penalty);
      const bool accept =
          std::isfinite(baseline_cost) && std::isfinite(candidate_cost) &&
          std::isfinite(baseline_gradient) &&
          std::isfinite(candidate_gradient) &&
          std::isfinite(baseline_penalty) &&
          std::isfinite(candidate_penalty) && cost_better &&
          gradient_not_worse && penalty_not_worse;
      if (accept)
      {
        x = std::move(warm_candidate);
        opt_vars_.warm_start_accepted = true;
        opt_vars_.warm_start_status = 1;
      }
      else
      {
        opt_vars_.warm_start_status = 7;
      }
    }
    opt_vars_.warm_start_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      warm_begin)
            .count();
  }
  const VecDf initial_decision = x;

  const bool early_stop_enabled = opt_vars_.lbfgs_fast_enabled;
  configureFastLbfgs(rel_cost_tol,
                     early_stop_enabled,
                     static_cast<int>(x.size()));
  fast_lbfgs_.reset();

  int ret = 0;
  std::size_t alm_outer_iterations = 0;
  std::size_t alm_inner_solves = 0;
  std::size_t alm_topology_changes = 0;
  double alm_warm_start_seconds = 0.0;
  cost_functional_manager::ExpPackedCorrectorCostManager::UpdateReport
      alm_report;

  // One and only one hot-path major. It uses the real objective (including
  // exact FlatnessMap residuals). Conservative models are evaluated after it.
  ret = fast_lbfgs_.run(x,
                        min_cost,
                        &ExpTrajOpt::costFunctional,
                        &ExpTrajOpt::stepBoundFunctional,
                        this,
                        &ExpTrajOpt::fastLbfgsSnapshot,
                        /*allow_fallback=*/true,
                        &initial_decision);
  syncFastLbfgsReport();

  // The stable production line keeps the continuous oracle read-only. The
  // experimental hard-certification line may opt in to a certificate-triggered
  // second solve; a certified hot-path result returns without constructing a
  // pack.
  if (opt_vars_.convex_hull_enabled &&
      opt_vars_.convex_hull_require_certification &&
      (ret >= 0 || fast_lbfgs_.acceptedFastStop()))
  {
    maybeUpdateCertifiedIncumbent(x, min_cost);
    bool phase2_certified = false;
    double phase2_violation = 0.0;
    runPhase2PackedCorrection(x,
                              min_cost,
                              rel_cost_tol,
                              fast_lbfgs_.options().mem_size,
                              alm_outer_iterations,
                              alm_inner_solves,
                              alm_topology_changes,
                              phase2_violation,
                              phase2_certified);
    alm_report.certified = phase2_certified;
    alm_report.max_normalized_violation = phase2_violation;
    alm_report.constraints =
        exp_packed_corrector_cost_manager_.constraintCount();
  }
  const double optimization_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    optimization_begin)
          .count();

  // Prefer a certified incumbent when the final solve fails numerically.
  const bool accepted_fast_stop_early =
      ret == lbfgs::LBFGS_CANCELED &&
      opt_vars_.fast_stop_satisfied;
  if (opt_vars_.convex_hull_require_certification &&
      (ret < 0 && !accepted_fast_stop_early) &&
      opt_vars_.has_certified_incumbent &&
      opt_vars_.certified_incumbent_x.size() == x.size())
  {
    x = opt_vars_.certified_incumbent_x;
    min_cost = opt_vars_.certified_incumbent_cost;
    ret = lbfgs::LBFGS_CONVERGENCE;
  }
  else if (opt_vars_.convex_hull_require_certification &&
           (ret >= 0 || accepted_fast_stop_early))
  {
    maybeUpdateCertifiedIncumbent(x, min_cost);
  }

  {
    VecDf provisional_grad = VecDf::Zero(x.size());
    fillPostSolveQualityReport(x, provisional_grad);
  }

  const auto &timing = optimizer_.cumulativeTimingStatistics();
  const double dense_share_of_optimization =
      optimization_seconds > 0.0
          ? timing.dense_integral_seconds / optimization_seconds
          : 0.0;
  last_timing_report_.evaluations = timing.evaluations;
  last_timing_report_.iterations = opt_vars_.lbfgs_iterations;
  last_timing_report_.line_search_evaluations =
      opt_vars_.line_search_evaluations;
  last_timing_report_.max_line_search_evaluations =
      opt_vars_.max_line_search_evaluations;
  last_timing_report_.accepted_step_sum =
      opt_vars_.accepted_step_sum;
  last_timing_report_.min_accepted_step =
      opt_vars_.lbfgs_iterations > 0 ? opt_vars_.min_accepted_step : 0.0;
  last_timing_report_.polynomial_pieces =
      static_cast<std::size_t>(optimizer_.getPieceNum());
  const bool dense_sampling_active =
      !opt_vars_.convex_hull_enabled ||
      exp_convex_cost_manager_.usesDenseSampling();
  last_timing_report_.dense_nodes_per_evaluation =
      dense_sampling_active
          ? static_cast<std::size_t>(optimizer_.getPieceNum()) *
                static_cast<std::size_t>(optimizer_.getSamplesPerPiece() + 1)
          : 0;
  last_timing_report_.hull_control_checks_per_evaluation =
      opt_vars_.convex_hull_enabled
          ? std::max(exp_convex_cost_manager_
                         .activeControlPointChecksPerEvaluation(),
                     exp_packed_corrector_cost_manager_
                         .activeControlPointChecksPerEvaluation())
          : 0;
  last_timing_report_.scalar_constraint_checks =
      opt_vars_.convex_hull_enabled
          ? std::max(exp_convex_cost_manager_.lastScalarConstraintChecks(),
                     exp_packed_corrector_cost_manager_.lastScalarChecks())
          : 0;
  {
    const auto &ev = exp_convex_cost_manager_.lastEvaluatorTiming();
    last_timing_report_.hull_transform_seconds = ev.transform_seconds;
    last_timing_report_.hull_hodograph_seconds = ev.hodograph_seconds;
    last_timing_report_.hull_position_residual_seconds =
        ev.position_residual_seconds;
    last_timing_report_.hull_derivative_residual_seconds =
        ev.derivative_residual_seconds;
    last_timing_report_.hull_reverse_hodograph_seconds =
        ev.reverse_hodograph_seconds;
    last_timing_report_.hull_backward_add_seconds = ev.backward_add_seconds;
    last_timing_report_.hull_discrete_attractor_seconds =
        ev.discrete_attractor_seconds;
  }
  last_timing_report_.phase2_triggered = opt_vars_.phase2_triggered;
  last_timing_report_.phase2_packed_constraints =
      opt_vars_.phase2_packed_constraints;
  last_timing_report_.jerk_certificate_enabled =
      opt_vars_.last_certificate.jerk_certificate_enabled;
  last_timing_report_.trust_region_rejections =
      opt_vars_.trust_region_rejections;
  last_timing_report_.trust_region_ratio =
      opt_vars_.trust_region_ratio;
  last_timing_report_.actual_reduction =
      opt_vars_.actual_reduction;
  last_timing_report_.predicted_reduction =
      opt_vars_.predicted_reduction;
  last_timing_report_.alm_constraints =
      std::max(opt_vars_.phase2_packed_constraints,
               exp_packed_corrector_cost_manager_.constraintCount());
  last_timing_report_.alm_outer_iterations = alm_outer_iterations;
  last_timing_report_.alm_inner_solves = alm_inner_solves;
  last_timing_report_.alm_topology_changes = alm_topology_changes;
  last_timing_report_.adaptive_coarse_segments = 0;
  last_timing_report_.adaptive_fine_segments = 0;
  last_timing_report_.alm_max_violation =
      opt_vars_.convex_hull_enabled ? alm_report.max_normalized_violation
                                    : 0.0;
  // Packed polish certification mirrors the continuous-time oracle.
  last_timing_report_.alm_certified =
      opt_vars_.convex_hull_enabled &&
      (alm_report.certified || opt_vars_.quality.continuous_feasible);
  last_timing_report_.alm_warm_start_seconds = alm_warm_start_seconds;
  last_timing_report_.mode =
      opt_vars_.convex_hull_enabled
          ? (opt_vars_.convex_hull_basis ==
                     traj_opt::convex_hull::Basis::MINVO
                 ? (opt_vars_.convex_hull_require_certification
                        ? "convex_minvo_d2_cert"
                        : "convex_minvo_d2_stable_monitor")
                 : opt_vars_.convex_hull_flatness_enabled
                       ? (opt_vars_.convex_hull_require_certification
                              ? "convex_bezier_v2_d2_cert_flatness_shadow"
                              : "convex_bezier_v2_d2_stable_flatness_shadow")
                       : (opt_vars_.convex_hull_require_certification
                              ? "convex_bezier_v2_d2_cert"
                              : "convex_bezier_v2_d2_stable_monitor"))
          : "dense";
  last_timing_report_.dense_integral_seconds =
      timing.dense_integral_seconds;
  last_timing_report_.control_point_seconds =
      timing.coefficient_seconds;
  last_timing_report_.minco_evaluation_seconds =
      timing.evaluation_seconds;
  last_timing_report_.optimization_seconds = optimization_seconds;
  last_timing_report_.dense_share_of_minco_evaluation =
      timing.denseIntegralShareOfEvaluation();
  last_timing_report_.control_point_share_of_minco_evaluation =
      timing.coefficientShareOfEvaluation();
  last_timing_report_.dense_share_of_optimization =
      dense_share_of_optimization;
  last_timing_report_.fast_stop_satisfied =
      opt_vars_.fast_stop_satisfied;
  last_timing_report_.fast_stop_iteration =
      opt_vars_.fast_stop_iteration;
  last_timing_report_.fast_fallback_used =
      opt_vars_.fast_fallback_used;
  {
    const auto &fast_report = fast_lbfgs_.report();
    last_timing_report_.fast_stop_candidate_checks =
        fast_report.stop_candidate_checks;
    last_timing_report_.fast_stop_cost_passes =
        fast_report.cost_passes;
    last_timing_report_.fast_stop_decision_step_passes =
        fast_report.decision_step_passes;
    last_timing_report_.fast_stop_penalty_change_passes =
        fast_report.penalty_change_passes;
    last_timing_report_.fast_stop_physical_time_passes =
        fast_report.physical_time_passes;
    last_timing_report_.fast_stop_waypoint_passes =
        fast_report.waypoint_passes;
    last_timing_report_.fast_stop_gradient_passes =
        fast_report.gradient_passes;
    last_timing_report_.fast_stop_violation_passes =
        fast_report.violation_passes;
    last_timing_report_.fast_stop_nonstall_passes =
        fast_report.nonstall_passes;
    last_timing_report_.fast_stop_base_rule_passes =
        fast_report.base_rule_passes;
    last_timing_report_.fast_stop_guarded_rule_passes =
        fast_report.guarded_rule_passes;
  }
  last_timing_report_.warm_start_status =
      opt_vars_.warm_start_status;
  last_timing_report_.warm_start_attempted =
      opt_vars_.warm_start_attempted;
  last_timing_report_.warm_start_accepted =
      opt_vars_.warm_start_accepted;
  last_timing_report_.warm_start_topology_resampled =
      opt_vars_.warm_start_topology_resampled;
  last_timing_report_.warm_start_comparison_evaluations =
      opt_vars_.warm_start_comparison_evaluations;
  last_timing_report_.warm_start_seconds =
      opt_vars_.warm_start_seconds;
  last_timing_report_.warm_start_baseline_cost =
      opt_vars_.warm_start_baseline_cost;
  last_timing_report_.warm_start_candidate_cost =
      opt_vars_.warm_start_candidate_cost;
  last_timing_report_.warm_start_baseline_gradient =
      opt_vars_.warm_start_baseline_gradient;
  last_timing_report_.warm_start_candidate_gradient =
      opt_vars_.warm_start_candidate_gradient;
  last_timing_report_.warm_start_baseline_penalty =
      opt_vars_.warm_start_baseline_penalty;
  last_timing_report_.warm_start_candidate_penalty =
      opt_vars_.warm_start_candidate_penalty;
  last_timing_report_.warm_start_max_waypoint_shift =
      opt_vars_.warm_start_max_waypoint_shift;
  last_timing_report_.continuous_feasible =
      opt_vars_.quality.continuous_feasible;
  last_timing_report_.robustly_certified =
      opt_vars_.quality.robustly_certified;
  last_timing_report_.has_certified_incumbent =
      opt_vars_.has_certified_incumbent;
  last_timing_report_.max_normalized_violation =
      opt_vars_.quality.max_normalized_violation;
  last_timing_report_.min_position_margin =
      opt_vars_.quality.min_position_margin;
  last_timing_report_.primal_residual = opt_vars_.quality.primal_residual;
  last_timing_report_.dual_residual = opt_vars_.quality.dual_residual;
  last_timing_report_.complementarity_residual =
      opt_vars_.quality.complementarity_residual;
  last_timing_report_.stationarity_residual =
      opt_vars_.quality.stationarity_residual;

  cumulative_timing_report_.mode = last_timing_report_.mode;
  cumulative_timing_report_.evaluations += last_timing_report_.evaluations;
  cumulative_timing_report_.iterations += last_timing_report_.iterations;
  cumulative_timing_report_.line_search_evaluations +=
      last_timing_report_.line_search_evaluations;
  cumulative_timing_report_.max_line_search_evaluations =
      std::max(cumulative_timing_report_.max_line_search_evaluations,
               last_timing_report_.max_line_search_evaluations);
  cumulative_timing_report_.accepted_step_sum +=
      last_timing_report_.accepted_step_sum;
  if (last_timing_report_.iterations > 0)
  {
    cumulative_timing_report_.min_accepted_step =
        cumulative_timing_report_.iterations == last_timing_report_.iterations
            ? last_timing_report_.min_accepted_step
            : std::min(cumulative_timing_report_.min_accepted_step,
                       last_timing_report_.min_accepted_step);
  }
  cumulative_timing_report_.polynomial_pieces +=
      last_timing_report_.polynomial_pieces;
  cumulative_timing_report_.dense_nodes_per_evaluation =
      last_timing_report_.dense_nodes_per_evaluation;
  cumulative_timing_report_.hull_control_checks_per_evaluation =
      last_timing_report_.hull_control_checks_per_evaluation;
  cumulative_timing_report_.alm_constraints =
      last_timing_report_.alm_constraints;
  cumulative_timing_report_.alm_outer_iterations +=
      last_timing_report_.alm_outer_iterations;
  cumulative_timing_report_.alm_inner_solves +=
      last_timing_report_.alm_inner_solves;
  cumulative_timing_report_.alm_topology_changes +=
      last_timing_report_.alm_topology_changes;
  cumulative_timing_report_.adaptive_coarse_segments +=
      last_timing_report_.adaptive_coarse_segments;
  cumulative_timing_report_.adaptive_fine_segments +=
      last_timing_report_.adaptive_fine_segments;
  cumulative_timing_report_.alm_max_violation =
      std::max(cumulative_timing_report_.alm_max_violation,
               last_timing_report_.alm_max_violation);
  cumulative_timing_report_.alm_certified =
      cumulative_timing_report_.alm_certified ||
      last_timing_report_.alm_certified;
  cumulative_timing_report_.phase2_triggered =
      cumulative_timing_report_.phase2_triggered ||
      last_timing_report_.phase2_triggered;
  cumulative_timing_report_.phase2_packed_constraints +=
      last_timing_report_.phase2_packed_constraints;
  cumulative_timing_report_.jerk_certificate_enabled =
      last_timing_report_.jerk_certificate_enabled;
  cumulative_timing_report_.trust_region_rejections +=
      last_timing_report_.trust_region_rejections;
  cumulative_timing_report_.trust_region_ratio =
      last_timing_report_.trust_region_ratio;
  cumulative_timing_report_.actual_reduction +=
      last_timing_report_.actual_reduction;
  cumulative_timing_report_.predicted_reduction +=
      last_timing_report_.predicted_reduction;
  cumulative_timing_report_.alm_warm_start_seconds +=
      last_timing_report_.alm_warm_start_seconds;
  cumulative_timing_report_.dense_integral_seconds +=
      last_timing_report_.dense_integral_seconds;
  cumulative_timing_report_.control_point_seconds +=
      last_timing_report_.control_point_seconds;
  cumulative_timing_report_.minco_evaluation_seconds +=
      last_timing_report_.minco_evaluation_seconds;
  cumulative_timing_report_.optimization_seconds +=
      last_timing_report_.optimization_seconds;
  cumulative_timing_report_.fast_stop_satisfied =
      cumulative_timing_report_.fast_stop_satisfied ||
      last_timing_report_.fast_stop_satisfied;
  cumulative_timing_report_.fast_stop_iteration +=
      last_timing_report_.fast_stop_iteration;
  cumulative_timing_report_.fast_fallback_used =
      cumulative_timing_report_.fast_fallback_used ||
      last_timing_report_.fast_fallback_used;
  cumulative_timing_report_.fast_stop_candidate_checks +=
      last_timing_report_.fast_stop_candidate_checks;
  cumulative_timing_report_.fast_stop_cost_passes +=
      last_timing_report_.fast_stop_cost_passes;
  cumulative_timing_report_.fast_stop_decision_step_passes +=
      last_timing_report_.fast_stop_decision_step_passes;
  cumulative_timing_report_.fast_stop_penalty_change_passes +=
      last_timing_report_.fast_stop_penalty_change_passes;
  cumulative_timing_report_.fast_stop_physical_time_passes +=
      last_timing_report_.fast_stop_physical_time_passes;
  cumulative_timing_report_.fast_stop_waypoint_passes +=
      last_timing_report_.fast_stop_waypoint_passes;
  cumulative_timing_report_.fast_stop_gradient_passes +=
      last_timing_report_.fast_stop_gradient_passes;
  cumulative_timing_report_.fast_stop_violation_passes +=
      last_timing_report_.fast_stop_violation_passes;
  cumulative_timing_report_.fast_stop_nonstall_passes +=
      last_timing_report_.fast_stop_nonstall_passes;
  cumulative_timing_report_.fast_stop_base_rule_passes +=
      last_timing_report_.fast_stop_base_rule_passes;
  cumulative_timing_report_.fast_stop_guarded_rule_passes +=
      last_timing_report_.fast_stop_guarded_rule_passes;
  cumulative_timing_report_.warm_start_status =
      last_timing_report_.warm_start_status;
  cumulative_timing_report_.warm_start_attempted =
      cumulative_timing_report_.warm_start_attempted ||
      last_timing_report_.warm_start_attempted;
  cumulative_timing_report_.warm_start_accepted =
      cumulative_timing_report_.warm_start_accepted ||
      last_timing_report_.warm_start_accepted;
  cumulative_timing_report_.warm_start_topology_resampled =
      cumulative_timing_report_.warm_start_topology_resampled ||
      last_timing_report_.warm_start_topology_resampled;
  cumulative_timing_report_.warm_start_comparison_evaluations +=
      last_timing_report_.warm_start_comparison_evaluations;
  cumulative_timing_report_.warm_start_seconds +=
      last_timing_report_.warm_start_seconds;
  cumulative_timing_report_.warm_start_baseline_cost =
      last_timing_report_.warm_start_baseline_cost;
  cumulative_timing_report_.warm_start_candidate_cost =
      last_timing_report_.warm_start_candidate_cost;
  cumulative_timing_report_.warm_start_baseline_gradient =
      last_timing_report_.warm_start_baseline_gradient;
  cumulative_timing_report_.warm_start_candidate_gradient =
      last_timing_report_.warm_start_candidate_gradient;
  cumulative_timing_report_.warm_start_baseline_penalty =
      last_timing_report_.warm_start_baseline_penalty;
  cumulative_timing_report_.warm_start_candidate_penalty =
      last_timing_report_.warm_start_candidate_penalty;
  cumulative_timing_report_.warm_start_max_waypoint_shift =
      std::max(cumulative_timing_report_.warm_start_max_waypoint_shift,
               last_timing_report_.warm_start_max_waypoint_shift);
  if (cumulative_timing_report_.minco_evaluation_seconds > 0.0)
  {
    cumulative_timing_report_.dense_share_of_minco_evaluation =
        cumulative_timing_report_.dense_integral_seconds /
        cumulative_timing_report_.minco_evaluation_seconds;
    cumulative_timing_report_.control_point_share_of_minco_evaluation =
        cumulative_timing_report_.control_point_seconds /
        cumulative_timing_report_.minco_evaluation_seconds;
  }
  if (cumulative_timing_report_.optimization_seconds > 0.0)
  {
    cumulative_timing_report_.dense_share_of_optimization =
        cumulative_timing_report_.dense_integral_seconds /
        cumulative_timing_report_.optimization_seconds;
  }

  if (cfg_.print_optimizer_log)
  {
    std::cout << " -- [ExpTrajOpt] Opt finish, mode="
              << last_timing_report_.mode
              << ", iter: " << opt_vars_.iter_num << "\n"
              << "\tEnergy: " << opt_vars_.penalty_log(0) << "\n"
              << "\tPos: " << opt_vars_.penalty_log(1) << "\n"
              << "\tVel: " << opt_vars_.penalty_log(2) << "\n"
              << "\tAcc: " << opt_vars_.penalty_log(3) << "\n"
              << "\tJerk: " << opt_vars_.penalty_log(4) << "\n"
              << "\tAttract: " << opt_vars_.penalty_log(5) << "\n"
              << "\tOmg: " << opt_vars_.penalty_log(6) << "\n"
              << "\tThr: " << opt_vars_.penalty_log(7) << "\n"
              << "\tGuidePathCost(sample): " << opt_vars_.guide_path_cost_log << "\n"
              << "\tGuidePathMaxExcess: " << opt_vars_.guide_integral_violation << "\n"
              << "\tGuidePathMaxAbsTimeGrad: " << opt_vars_.guide_path_max_abs_time_grad << "\n"
              << "\tGuidePathOutOfTimeSamples: " << opt_vars_.guide_path_out_of_time_range_samples << "\n"
              << "\tTimingEvaluations: " << timing.evaluations << "\n"
              << "\tLBFGSIterations: " << last_timing_report_.iterations << "\n"
              << "\tLineSearchEvaluations: "
              << last_timing_report_.line_search_evaluations << "\n"
              << "\tAverageLineSearchEvaluations: "
              << (last_timing_report_.iterations > 0
                      ? static_cast<double>(last_timing_report_.line_search_evaluations) /
                            static_cast<double>(last_timing_report_.iterations)
                      : 0.0)
              << "\n"
              << "\tMaxLineSearchEvaluations: "
              << last_timing_report_.max_line_search_evaluations << "\n"
              << "\tFastStopSatisfied: "
              << (last_timing_report_.fast_stop_satisfied ? 1 : 0) << "\n"
              << "\tFastStopIteration: "
              << last_timing_report_.fast_stop_iteration << "\n"
              << "\tFastFallbackUsed: "
              << (last_timing_report_.fast_fallback_used ? 1 : 0) << "\n"
              << "\tPolynomialPieces: " << last_timing_report_.polynomial_pieces << "\n"
              << "\tDenseNodesPerEvaluation: " << last_timing_report_.dense_nodes_per_evaluation << "\n"
              << "\tHullControlChecksPerEvaluation: " << last_timing_report_.hull_control_checks_per_evaluation << "\n"
              << "\tDenseIntegralMs: " << timing.dense_integral_seconds * 1.0e3 << "\n"
              << "\tControlPointFunctionalMs: " << timing.coefficient_seconds * 1.0e3 << "\n"
              << "\tMincoEvaluationMs: " << timing.evaluation_seconds * 1.0e3 << "\n"
              << "\tLBFGSOptimizationMs: " << optimization_seconds * 1.0e3 << "\n"
              << "\tDenseIntegralShareOfMincoEvaluation: "
              << timing.denseIntegralShareOfEvaluation() * 100.0 << "%\n"
              << "\tControlPointShareOfMincoEvaluation: "
              << timing.coefficientShareOfEvaluation() * 100.0 << "%\n"
              << "\tDenseIntegralShareOfOptimization: "
              << dense_share_of_optimization * 100.0 << "%" << std::endl;
  }

  const bool certification_failed =
      opt_vars_.convex_hull_enabled &&
      opt_vars_.convex_hull_require_certification &&
      !alm_report.certified &&
      !opt_vars_.quality.continuous_feasible &&
      !opt_vars_.has_certified_incumbent;
  const bool accepted_fast_stop =
      ret == lbfgs::LBFGS_CANCELED &&
      opt_vars_.fast_stop_satisfied;
  if ((ret < 0 && !accepted_fast_stop) || certification_failed)
  {
    traj.clear();
    std::cout << YELLOW << " -- [ExpTrajOpt] Optimization failed: "
              << (certification_failed
                      ? "continuous certificate not satisfied"
                      : lbfgs::lbfgs_strerror(ret))
              << ", alm_violation=" << alm_report.max_normalized_violation
              << ", alm_position_violation="
              << opt_vars_.quality.max_position_violation
              << ", alm_derivative_violation="
              << opt_vars_.quality.max_derivative_violation
              << ", quality=" << opt_vars_.quality.summary()
              << ", guide_excess=" << opt_vars_.guide_integral_violation
              << ", guide_cost_sample=" << opt_vars_.guide_path_cost_log
              << ", guide_max_abs_gt=" << opt_vars_.guide_path_max_abs_time_grad
              << ", guide_oob_samples=" << opt_vars_.guide_path_out_of_time_range_samples
              << RESET << std::endl;
    return INFINITY;
  }

  VecDf grad = VecDf::Zero(x.size());
  min_cost = evaluateMincoCost(x, grad);
  fillPostSolveQualityReport(x, grad);
  last_timing_report_.continuous_feasible =
      opt_vars_.quality.continuous_feasible;
  last_timing_report_.robustly_certified =
      opt_vars_.quality.robustly_certified;
  last_timing_report_.has_certified_incumbent =
      opt_vars_.has_certified_incumbent;
  last_timing_report_.max_normalized_violation =
      opt_vars_.quality.max_normalized_violation;
  last_timing_report_.min_position_margin =
      opt_vars_.quality.min_position_margin;
  last_timing_report_.primal_residual = opt_vars_.quality.primal_residual;
  last_timing_report_.dual_residual = opt_vars_.quality.dual_residual;
  last_timing_report_.complementarity_residual =
      opt_vars_.quality.complementarity_residual;
  last_timing_report_.stationarity_residual =
      opt_vars_.quality.stationarity_residual;

  traj = toGeometryTrajectory(optimizer_.getTrajectory());
  updateWarmStartCache(traj);
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

ExplorationTrajOpt::ExplorationTrajOpt(const traj_opt::Config &cfg,
                       const ros_interface::RosInterface::Ptr &ros_ptr)
    : cfg_(cfg), ros_ptr_(ros_ptr)
{
  if (cfg_.save_log_en)
  {
    failed_traj_log_.open(DEBUG_FILE_DIR("exp_opt_log.csv"), std::ios::out | std::ios::trunc);
    penalty_log_.open(DEBUG_FILE_DIR("exp_opt_penna.csv"), std::ios::out | std::ios::trunc);
  }

  opt_vars_.magnitude_bounds.resize(6);
  opt_vars_.penalty_weights.resize(6);
  opt_vars_.magnitude_bounds << cfg_.max_vel, cfg_.max_acc,
      cfg_.max_omg, std::numeric_limits<double>::infinity(),
      cfg_.min_acc_thr * cfg_.mass, cfg_.max_acc_thr * cfg_.mass;
  opt_vars_.penalty_weights << cfg_.penna_pos, cfg_.penna_vel,
      cfg_.penna_acc, cfg_.penna_omg, cfg_.penna_theta, cfg_.penna_thr;
  opt_vars_.rho = cfg_.penna_t;
  opt_vars_.pos_constraint_type = cfg_.pos_constraint_type;
  opt_vars_.block_energy_cost = cfg_.block_energy_cost;
  opt_vars_.smooth_eps = cfg_.smooth_eps;
  opt_vars_.integral_res = std::max(1, cfg_.integral_reso);
  opt_vars_.quadrotor_flatness = cfg_.quadrotot_flatness;
  opt_vars_.guide_z_tube_radius = std::max(0.0, cfg_.guide_z_tube_radius);
  // Guide-path integral cost is disabled until its planner-level semantics are redesigned.
  opt_vars_.weight_guide_integral = 0.0;
  opt_vars_.guide_path_tube_radius = std::max(0.0, cfg_.guide_path_tube_radius);
  opt_vars_.guide_path_z_tube_radius = std::max(0.0, cfg_.guide_path_z_tube_radius);
  opt_vars_.guide_path_huber_delta = std::max(0.0, cfg_.guide_path_huber_delta);
  opt_vars_.guide_path_time_gradient_en = cfg_.guide_path_time_gradient_en;
  opt_vars_.weight_guide_z_tube =
      opt_vars_.guide_z_tube_radius > 0.0 ? std::max(0.0, cfg_.penna_guide_z_tube) : 0.0;

  linear_time_cost_.weight = opt_vars_.rho;
  optimizer_.setTimeMap(&time_map_);
  optimizer_.setSpatialMap(&spatial_map_);
  optimizer_.setEnergyWeight(opt_vars_.block_energy_cost ? 0.0 : 1.0);
  optimizer_.setSamplesPerPiece(opt_vars_.integral_res);
}

ExplorationTrajOpt::~ExplorationTrajOpt()
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

void ExplorationTrajOpt::clearPieceVelocityBounds()
{
  opt_vars_.piece_velocity_bounds.resize(0);
  opt_vars_.use_piece_velocity_bounds = false;
}

void ExplorationTrajOpt::setPieceVelocityBounds(const VecDf &piece_velocity_bounds)
{
  opt_vars_.piece_velocity_bounds = piece_velocity_bounds;
  opt_vars_.use_piece_velocity_bounds = piece_velocity_bounds.size() > 0;
}

void ExplorationTrajOpt::normalizePieceVelocityBounds()
{
  if (!opt_vars_.use_piece_velocity_bounds)
  {
    return;
  }
  if (opt_vars_.piece_num <= 0)
  {
    clearPieceVelocityBounds();
    return;
  }

  if (opt_vars_.piece_velocity_bounds.size() == 1)
  {
    const double bound = opt_vars_.piece_velocity_bounds(0);
    opt_vars_.piece_velocity_bounds = VecDf::Constant(opt_vars_.piece_num, bound);
  }
  else if (opt_vars_.piece_velocity_bounds.size() != opt_vars_.piece_num)
  {
    const VecDf old_bounds = opt_vars_.piece_velocity_bounds;
    opt_vars_.piece_velocity_bounds =
        VecDf::Constant(opt_vars_.piece_num, std::max(1.0e-3, cfg_.max_vel));
    const int copy_num = std::min<int>(old_bounds.size(), opt_vars_.piece_num);
    for (int i = 0; i < copy_num; ++i)
    {
      opt_vars_.piece_velocity_bounds(i) = old_bounds(i);
    }
  }

  for (int i = 0; i < opt_vars_.piece_velocity_bounds.size(); ++i)
  {
    double &bound = opt_vars_.piece_velocity_bounds(i);
    if (!std::isfinite(bound) || bound <= 1.0e-3)
    {
      bound = std::max(1.0e-3, cfg_.max_vel);
    }
  }
}

void ExplorationTrajOpt::setSwarmConfig(const SwarmPenaltyConfig &config)
{
  swarm_config_ = config;
}

void ExplorationTrajOpt::setSwarmTrajectories(const SwarmTrajectoriesConstPtr &trajectories)
{
  swarm_trajs_ = trajectories;
}

void ExplorationTrajOpt::setSwarmCurrentWallTime(double wall_time)
{
  swarm_current_wall_time_ = wall_time;
}

SnapBoundaryState ExplorationTrajOpt::toSnapBoundary(const StatePVAJ &state)
{
  SnapBoundaryState out;
  out.col(0) = state.col(0);
  out.col(1) = state.col(1);
  out.col(2) = state.col(2);
  out.col(3) = state.col(3);
  return out;
}

Trajectory ExplorationTrajOpt::toGeometryTrajectory(const SnapTraj &traj)
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

bool ExplorationTrajOpt::processCorridor()
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
      std::cout << YELLOW << " -- [ExplorationTrajOpt] Failed to enumerate corridor vertices." << RESET << std::endl;
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

bool ExplorationTrajOpt::processCorridorWithGuideTraj()
{
  if (!processCorridor())
  {
    return false;
  }

  VecDf time_stamps(opt_vars_.waypoint_attractor.cols() + 2);
  time_stamps(0) = 0.0;
  time_stamps(time_stamps.size() - 1) = opt_vars_.guide_t.back();
  int guide_overlap_fallback_count = 0;
  for (int j = 0; j < opt_vars_.waypoint_attractor.cols(); ++j)
  {
    const Vec3f chebyshev_center = opt_vars_.waypoint_attractor.col(j);
    const double chebyshev_dead_d = opt_vars_.waypoint_attractor_dead_d(j);
    GuideOverlapSample guide_sample;
    if (findGuidePointInOverlap(opt_vars_.guide_path,
                                opt_vars_.guide_t,
                                opt_vars_.h_overlap_polytopes[j],
                                chebyshev_center,
                                time_stamps(j),
                                guide_sample))
    {
      opt_vars_.waypoint_attractor.col(j) = guide_sample.position;
      opt_vars_.points.col(j) = guide_sample.position;
      opt_vars_.waypoint_attractor_dead_d(j) =
          std::min(chebyshev_dead_d, std::max(1.0e-3, 0.5 * guide_sample.clearance));
      time_stamps(j + 1) = std::clamp(guide_sample.time,
                                      time_stamps(0),
                                      time_stamps(time_stamps.size() - 1));
    }
    else
    {
      double min_dis = std::numeric_limits<double>::max();
      int min_id = 0;
      for (int i = 0; i < static_cast<int>(opt_vars_.guide_path.size()); ++i)
      {
        const double dis = (opt_vars_.guide_path[i] - chebyshev_center).norm();
        if (dis < min_dis)
        {
          min_dis = dis;
          min_id = i;
        }
      }
      opt_vars_.points.col(j) = chebyshev_center;
      time_stamps(j + 1) = opt_vars_.guide_t[min_id];
      ++guide_overlap_fallback_count;
    }
  }

  for (int i = 1; i < time_stamps.size(); ++i)
  {
    opt_vars_.times(i - 1) = std::max(0.01, time_stamps(i) - time_stamps(i - 1));
  }
  if (cfg_.print_optimizer_log && guide_overlap_fallback_count > 0)
  {
    std::cout << YELLOW << " -- [ExplorationTrajOpt] Guide-overlap waypoint fallback count: "
              << guide_overlap_fallback_count << RESET << std::endl;
  }
  return true;
}

void ExplorationTrajOpt::defaultInitialization()
{
  const VecDf dis = (opt_vars_.init_path.rightCols(opt_vars_.piece_num) -
                     opt_vars_.init_path.leftCols(opt_vars_.piece_num))
                        .colwise()
                        .norm();
  opt_vars_.times = (dis.array() / std::max(1.0e-3, cfg_.max_vel)).max(0.01);
  opt_vars_.points = opt_vars_.waypoint_attractor;
}

bool ExplorationTrajOpt::setupProblemAndCheck()
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

bool ExplorationTrajOpt::loadCorridors(PolytopeVec &sfcs)
{
  if (sfcs.empty())
  {
    std::cout << YELLOW << " -- [ExplorationTrajOpt] Empty SFC." << RESET << std::endl;
    return false;
  }

  if (!geometry_utils::SimplifySFC(opt_vars_.head_pvaj.col(0), opt_vars_.tail_pvaj.col(0), sfcs))
  {
    std::cout << YELLOW << " -- [ExplorationTrajOpt] Cannot simplify SFC." << RESET << std::endl;
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

double ExplorationTrajOpt::costFunctional(void *ptr, const VecDf &x, VecDf &g)
{
  return static_cast<ExplorationTrajOpt *>(ptr)->evaluateMincoCost(x, g);
}

double ExplorationTrajOpt::evaluateMincoCost(const VecDf &x, VecDf &g)
{
  opt_vars_.iter_num++;
  const double cost = optimizer_.evaluate(x, g, linear_time_cost_, exploration_cost_manager_);
  opt_vars_.guide_integral_violation = 0.0;
  opt_vars_.guide_path_cost_log = 0.0;
  opt_vars_.guide_path_max_abs_time_grad = 0.0;
  opt_vars_.guide_path_out_of_time_range_samples = 0;
  opt_vars_.guide_z_tube_violation = 0.0;
  opt_vars_.penalty_log(0) = optimizer_.lastEnergyCost();
  opt_vars_.penalty_log.tail(7) = exploration_cost_manager_.getPenaltyLog().tail(7);
  return cost;
}

double ExplorationTrajOpt::optimize(Trajectory &traj, double rel_cost_tol)
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

  optimizer_.setUniformTimeMode(false);
  optimizer_.setEnergyWeight(opt_vars_.block_energy_cost ? 0.0 : 1.0);
  optimizer_.setSamplesPerPiece(opt_vars_.integral_res);
  if (!optimizer_.setInitState(toStdVector(opt_vars_.times),
                               toOptimizerWaypoints(waypoints),
                               toSnapBoundary(opt_vars_.head_pvaj),
                               toSnapBoundary(opt_vars_.tail_pvaj)))
  {
    return INFINITY;
  }
  VecDf x = optimizer_.generateInitialGuess();
  if (x.size() <= 0 || !x.allFinite())
  {
    return INFINITY;
  }

  const VecDf *piece_velocity_bounds =
      opt_vars_.use_piece_velocity_bounds ? &opt_vars_.piece_velocity_bounds : nullptr;

  exploration_cost_manager_.reset(&opt_vars_.h_polytopes,
                                  &opt_vars_.h_poly_idx,
                                  piece_velocity_bounds,
                                  opt_vars_.smooth_eps,
                                  opt_vars_.magnitude_bounds,
                                  opt_vars_.penalty_weights,
                                  &opt_vars_.quadrotor_flatness);

  opt_vars_.iter_num = 0;
  double min_cost = 0.0;
  lbfgs::lbfgs_parameter_t params;
  params.mem_size = 256;
  params.past = 3;
  params.min_step = 1.0e-32;
  params.g_epsilon = 0.0;
  params.delta = rel_cost_tol;

  const int ret = lbfgs::lbfgs_optimize(x, min_cost, &ExplorationTrajOpt::costFunctional, nullptr, nullptr, this, params);

  if (cfg_.print_optimizer_log)
  {
    std::cout << " -- [ExplorationTrajOpt] Opt finish, iter: " << opt_vars_.iter_num << "\n"
              << "\tEnergy: " << opt_vars_.penalty_log(0) << "\n"
              << "\tPos: " << opt_vars_.penalty_log(1) << "\n"
              << "\tVel: " << opt_vars_.penalty_log(2) << "\n"
              << "\tAcc: " << opt_vars_.penalty_log(3) << "\n"
              << "\tJerk: " << opt_vars_.penalty_log(4) << "\n"
              << "\tAttract: " << opt_vars_.penalty_log(5) << "\n"
              << "\tOmg: " << opt_vars_.penalty_log(6) << "\n"
              << "\tThr: " << opt_vars_.penalty_log(7) << "\n"
              << "\tGuidePathCost(sample): " << opt_vars_.guide_path_cost_log << "\n"
              << "\tGuidePathMaxExcess: " << opt_vars_.guide_integral_violation << "\n"
              << "\tGuidePathMaxAbsTimeGrad: " << opt_vars_.guide_path_max_abs_time_grad << "\n"
              << "\tGuidePathOutOfTimeSamples: " << opt_vars_.guide_path_out_of_time_range_samples << std::endl;
  }

  if (ret < 0)
  {
    traj.clear();
    std::cout << YELLOW << " -- [ExplorationTrajOpt] Optimization failed: " << lbfgs::lbfgs_strerror(ret)
              << ", guide_excess=" << opt_vars_.guide_integral_violation
              << ", guide_cost_sample=" << opt_vars_.guide_path_cost_log
              << ", guide_max_abs_gt=" << opt_vars_.guide_path_max_abs_time_grad
              << ", guide_oob_samples=" << opt_vars_.guide_path_out_of_time_range_samples
              << RESET << std::endl;
    return INFINITY;
  }

  VecDf grad = VecDf::Zero(x.size());
  min_cost = evaluateMincoCost(x, grad);

  traj = toGeometryTrajectory(optimizer_.getTrajectory());
  return min_cost;
}

bool ExplorationTrajOpt::optimize(const StatePVAJ &headPVAJ,
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
  clearPieceVelocityBounds();
  if (!loadCorridors(sfcs) || !setupProblemAndCheck())
  {
    return false;
  }
  normalizePieceVelocityBounds();
  out_traj.clear();
  const bool success = !std::isinf(optimize(out_traj, cfg_.opt_accuracy));
  if (success)
  {
    out_traj.start_WT = ros_ptr_->getSimTime();
  }
  return success;
}

bool ExplorationTrajOpt::optimize(const StatePVAJ &headPVAJ,
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
  clearPieceVelocityBounds();
  if (!loadCorridors(sfcs) || !setupProblemAndCheck())
  {
    return false;
  }
  normalizePieceVelocityBounds();
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

bool ExplorationTrajOpt::optimize(const StatePVAJ &headPVAJ,
                                  const StatePVAJ &tailPVAJ,
                                  const vec_E<Vec3f> &guide_path,
                                  const std::vector<double> &guide_t,
                                  PolytopeVec &sfcs,
                                  const VecDf &piece_velocity_bounds,
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
  setPieceVelocityBounds(piece_velocity_bounds);
  if (!loadCorridors(sfcs) || !setupProblemAndCheck())
  {
    return false;
  }
  normalizePieceVelocityBounds();
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

bool ExplorationTrajOpt::optimize(const StatePVAJ &headPVAJ,
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
  opt_vars_.guide_z_tube_radius = std::max(0.0, cfg_.guide_z_tube_radius);
  opt_vars_.weight_guide_z_tube =
      opt_vars_.guide_z_tube_radius > 0.0 ? std::max(0.0, cfg_.penna_guide_z_tube) : 0.0;
  linear_time_cost_.weight = opt_vars_.rho;
  optimizer_.setTimeMap(&time_map_);
  optimizer_.setSpatialMap(&spatial_map_);
  optimizer_.setEnergyWeight(opt_vars_.block_energy_cost ? 0.0 : 1.0);
  optimizer_.setSamplesPerPiece(opt_vars_.integral_res);
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

double BackupTrajOpt::costFunctional(void *ptr, const VecDf &x, VecDf &g)
{
  return static_cast<BackupTrajOpt *>(ptr)->evaluateMincoCost(x, g);
}

double BackupTrajOpt::evaluateMincoCost(const VecDf &x, VecDf &g)
{
  opt_vars_.iter_num++;
  const double cost = optimizer_.evaluateWithBoundaryMapping(x,
                                                            g,
                                                            linear_time_cost_,
                                                            backup_cost_manager_,
                                                            &backup_boundary_mapping_);
  opt_vars_.penalty_log(0) = optimizer_.lastEnergyCost();
  opt_vars_.penalty_log.tail(7) = backup_cost_manager_.getPenaltyLog().tail(7);
  opt_vars_.guide_z_tube_violation = 0.0;
  opt_vars_.ts = backup_boundary_mapping_.lastTs();
  opt_vars_.times = optimizer_.getCurrentTimes();
  if (optimizer_.getTrajectory().getPieceNum() == opt_vars_.piece_num)
  {
    opt_vars_.points = optimizer_.getTrajectory().getPositions().middleCols(1, opt_vars_.piece_num);
  }
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

  spatial_map_.reset(&opt_vars_.v_polytope,
                     opt_vars_.piece_num,
                     opt_vars_.pos_constraint_type == 1);
  backup_boundary_mapping_.configure(&opt_vars_.exp_traj,
                                     opt_vars_.min_ts,
                                     opt_vars_.max_ts,
                                     opt_vars_.ts,
                                     opt_vars_.points.col(opt_vars_.piece_num - 1),
                                     &opt_vars_.v_polytope,
                                     opt_vars_.piece_num,
                                     opt_vars_.pos_constraint_type == 1,
                                     opt_vars_.weight_ts);

  Mat3Df inner_points(3, std::max(0, opt_vars_.piece_num - 1));
  if (opt_vars_.piece_num > 1)
  {
    inner_points = opt_vars_.points.leftCols(opt_vars_.piece_num - 1);
  }
  const Mat3Df waypoints = waypointsToMatrix(opt_vars_.head_pvaj, inner_points, opt_vars_.tail_pvaj);
  optimizer_.setUniformTimeMode(opt_vars_.uniform_time_en);
  optimizer_.setEnergyWeight(opt_vars_.block_energy_cost ? 0.0 : 1.0);
  optimizer_.setSamplesPerPiece(opt_vars_.integral_res);
  if (!optimizer_.setInitState(toStdVector(opt_vars_.times),
                               toOptimizerWaypoints(waypoints),
                               toSnapBoundary(opt_vars_.head_pvaj),
                               toSnapBoundary(opt_vars_.tail_pvaj)))
  {
    return INFINITY;
  }
  VecDf x = optimizer_.generateInitialGuess(&backup_boundary_mapping_);
  if (x.size() <= 0 || !x.allFinite())
  {
    return INFINITY;
  }

  backup_cost_manager_.reset(&opt_vars_.h_polytope,
                             opt_vars_.smooth_eps,
                             opt_vars_.magnitude_bounds,
                             opt_vars_.penalty_weights,
                             &opt_vars_.quadrotor_flatness);

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
  min_cost = evaluateMincoCost(x, grad);
  traj = toGeometryTrajectory(optimizer_.getTrajectory());
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
  optimizer_.setTimeMap(&time_map_);
  optimizer_.setEnergyWeight(opt_vars_.block_energy_cost ? 0.0 : 1.0);
  optimizer_.setSamplesPerPiece(opt_vars_.integral_res);

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
  return static_cast<ESDFTrajOpt *>(ptr)->evaluateMincoCost(x, g);
}

double ESDFTrajOpt::evaluateMincoCost(const VecDf &x, VecDf &g)
{
  opt_vars_.iter_num++;
  opt_vars_.penalty_log.setZero();
  const double cost = optimizer_.evaluate(x, g, linear_time_cost_, esdf_cost_manager_);
  opt_vars_.max_violation = esdf_cost_manager_.getMaxViolation();
  opt_vars_.penalty_log(0) = optimizer_.lastEnergyCost();
  opt_vars_.penalty_log.tail(7) = esdf_cost_manager_.getPenaltyLog().tail(7);
  opt_vars_.penalty_log(5) = esdf_cost_manager_.getGuideCostLog();
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

  const Mat3Df waypoints = waypointsToMatrix(opt_vars_.head_pvaj, opt_vars_.points, opt_vars_.tail_pvaj);
  optimizer_.setUniformTimeMode(false);
  optimizer_.setEnergyWeight(opt_vars_.block_energy_cost ? 0.0 : 1.0);
  optimizer_.setSamplesPerPiece(opt_vars_.integral_res);
  if (!optimizer_.setInitState(toStdVector(opt_vars_.times),
                               toOptimizerWaypoints(waypoints),
                               toSnapBoundary(opt_vars_.head_pvaj),
                               toSnapBoundary(opt_vars_.tail_pvaj)))
  {
    return INFINITY;
  }
  VecDf x = optimizer_.generateInitialGuess();
  if (x.size() <= 0 || !x.allFinite())
  {
    return INFINITY;
  }

  esdf_cost_manager_.reset(map_manager_.get(),
                           opt_vars_.safe_distance,
                           opt_vars_.smooth_eps,
                           opt_vars_.weight_esdf,
                           opt_vars_.magnitude_bounds,
                           opt_vars_.penalty_weights,
                           &opt_vars_.quadrotor_flatness,
                           swarm_config_,
                           swarm_trajs_,
                           swarm_current_wall_time_,
                           &opt_vars_.guide_path,
                           &opt_vars_.guide_velocities,
                           &opt_vars_.guide_points,
                           opt_vars_.weight_guide,
                           opt_vars_.weight_guide_integral,
                           opt_vars_.weight_guide_vel_integral,
                           opt_vars_.integral_res);

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
  min_cost = evaluateMincoCost(x, grad);
  VecDf optimized_times;
  Mat3Df optimized_inner;
  optimized_times = optimizer_.getCurrentTimes();
  optimized_inner.resize(3, std::max(0, piece_num - 1));
  if (piece_num > 1)
  {
    optimized_inner = optimizer_.getTrajectory().getPositions().middleCols(1, piece_num - 1);
  }
  minco_traj_ = optimizer_.getTrajectory();
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
  report.grid_type = static_cast<int>(general_utils::GridType::UNDEFINED);
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
  optimizer_.setTimeMap(&time_map_);
  optimizer_.setEnergyWeight(opt_vars_.block_energy_cost ? 0.0 : 1.0);
  optimizer_.setSamplesPerPiece(opt_vars_.integral_res);

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
    if ((ret != general_utils::SUCCESS && ret != general_utils::REACH_GOAL && ret != general_utils::REACH_HORIZON) ||
        local_path.size() < 2)
    {
      local_path.clear();
      ret = local_astar_->pointToPointPathSearch(end_pt, start_pt, prob_flag, horizon, local_path, 0.08);
    }
    if ((ret != general_utils::SUCCESS && ret != general_utils::REACH_GOAL && ret != general_utils::REACH_HORIZON) ||
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

  if (!optimizer_.updateTrajectoryFromDecisionVector(x))
  {
    return false;
  }
  minco_traj_ = optimizer_.getTrajectory();

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
  return static_cast<PlainTrajOpt *>(ptr)->evaluateMincoCost(x, g);
}

double PlainTrajOpt::evaluateMincoCost(const VecDf &x, VecDf &g)
{
  opt_vars_.iter_num++;
  opt_vars_.penalty_log.setZero();
  const double cost = optimizer_.evaluate(x, g, linear_time_cost_, plain_cost_manager_);
  opt_vars_.guide_tube_violation = plain_cost_manager_.getGuideTubeViolation();
  opt_vars_.max_violation =
      std::max(plain_cost_manager_.getMaxCollisionViolation(),
               opt_vars_.guide_tube_violation);
  opt_vars_.penalty_log(0) = optimizer_.lastEnergyCost();
  opt_vars_.penalty_log.tail(7) = plain_cost_manager_.getPenaltyLog().tail(7);
  opt_vars_.penalty_log(5) = plain_cost_manager_.getGuideCostLog();
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

  const Mat3Df waypoints = waypointsToMatrix(opt_vars_.head_pvaj, opt_vars_.points, opt_vars_.tail_pvaj);
  optimizer_.setUniformTimeMode(false);
  optimizer_.setEnergyWeight(opt_vars_.block_energy_cost ? 0.0 : 1.0);
  optimizer_.setSamplesPerPiece(opt_vars_.integral_res);
  if (!optimizer_.setInitState(toStdVector(opt_vars_.times),
                               toOptimizerWaypoints(waypoints),
                               toSnapBoundary(opt_vars_.head_pvaj),
                               toSnapBoundary(opt_vars_.tail_pvaj)))
  {
    return INFINITY;
  }
  VecDf x = optimizer_.generateInitialGuess();
  if (x.size() <= 0 || !x.allFinite())
  {
    return INFINITY;
  }

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
                            opt_vars_.pv_samples_per_piece,
                            &opt_vars_.guide_path,
                            &opt_vars_.guide_velocities,
                            &opt_vars_.guide_points,
                            opt_vars_.weight_guide,
                            opt_vars_.weight_guide_integral,
                            opt_vars_.weight_guide_vel_integral,
                            opt_vars_.weight_guide_tube,
                            opt_vars_.guide_tube_radius,
                            opt_vars_.guide_tube_radius_sqr,
                            opt_vars_.integral_res);

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

    min_cost = evaluateMincoCost(x, grad);
    optimized_times = optimizer_.getCurrentTimes();
    optimized_inner.resize(3, std::max(0, piece_num - 1));
    if (piece_num > 1)
    {
      optimized_inner = optimizer_.getTrajectory().getPositions().middleCols(1, piece_num - 1);
    }
    minco_traj_ = optimizer_.getTrajectory();
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
  report.grid_type = static_cast<int>(general_utils::GridType::UNDEFINED);
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

  constexpr double kPlainHardVelRejectRatio = 1.5;
  constexpr double kPlainHardAccRejectRatio = 2.1;
  if (cfg_.penna_vel > 0.0 && report.max_vel > kPlainHardVelRejectRatio * cfg_.max_vel)
  {
    fillDynamicFailureState("MAX_VEL", false);
    return report;
  }
  if (cfg_.penna_acc > 0.0 && report.max_acc > kPlainHardAccRejectRatio * cfg_.max_acc)
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
