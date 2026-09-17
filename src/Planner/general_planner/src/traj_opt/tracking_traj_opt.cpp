#include "traj_opt/tracking_traj_opt.hpp"
#include "traj_opt/tracking_objective.hpp"
#include "ros_interface/ros_interface.hpp"
#include "utils/geometry/geometry_utils.h"
#include "utils/optimization/lbfgs.h"
#include <chrono>
#include <numeric>

namespace traj_opt { namespace {
using geometry_utils::Trajectory;
using general_utils::StatePVAJ;
using general_utils::Vec3f;
constexpr double kTiny = 1.e-9;
template <int S> using TaskOptimizer = typename TrackingObjective<S>::PosOptimizer;
template <int S> using TaskTraj = typename TaskOptimizer<S>::TrajType;
double clampPositive(double value, double fallback)
{
  if (!std::isfinite(value) || value <= 0.0)
  {
    return fallback;
  }
  return value;
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

template <int SPos>
class ElasticTrackingRunner
{
public:
  using PosTraj = minco::MINCOTrajectory<3, SPos>;
  using YawTraj = minco::MINCOTrajectory<1, 3>;
  using YawBoundaryState = typename YawTraj::BoundaryState;
  using YawInnerMat = typename YawTraj::InnerPointsMat;

  ElasticTrackingRunner(const traj_opt::Config &cfg,
                      const std::shared_ptr<ros_interface::RosInterface> &)
      : cfg_(cfg)
  {
    samples_per_piece_ = std::max(1, cfg_.integral_reso);
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
                Trajectory *out_yaw_traj,
                std::string *failure_reason = nullptr)
  {
    out_traj.clear();
    if (out_yaw_traj) out_yaw_traj->clear();
    if (failure_reason) failure_reason->clear();
    const auto fail = [&](const std::string &message) {
      if (failure_reason) *failure_reason = message;
      return false;
    };
    problem_ = std::move(problem);
    if (problem_.safe_distance <= 0.0)
    {
      problem_.safe_distance = safe_distance_;
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
        return fail("corridor initialization failed: invalid planes, endpoints, or overlap");
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
      return fail("position initialization failed");
    }

    piece_num_ = static_cast<int>(init_times_.size());
    const double lower = std::max(problem_.min_total_duration,
        problem_.target_prediction.empty() ? 0.0 : problem_.target_prediction.back().t);
    if (problem_.max_total_duration <= lower + 1.e-3) return fail("empty duration interval");
    const double sum = std::accumulate(init_times_.begin(), init_times_.end(), 0.0);
    const double seed_total = std::clamp(sum, lower + 0.1 * (problem_.max_total_duration - lower),
                                       lower + 0.8 * (problem_.max_total_duration - lower));
    for (auto &t : init_times_) t *= seed_total / std::max(1.e-6, sum);
    init_yaw_inner_.resize(1, std::max(0, piece_num_ - 1));
    setupBoundaryStatesAndYawGuess();

    // Preserve the supplied moving head exactly; only the feasible future is optimized.
    typename TrackingObjective<SPos>::YawOptimizer::WaypointsType yaw_points(piece_num_ + 1, 1);
    yaw_points(0, 0) = yaw_head_state_(0, 0);
    yaw_points(piece_num_, 0) = yaw_tail_state_(0, 0);
    for (int i = 1; i < piece_num_; ++i) yaw_points(i, 0) = init_yaw_inner_(0, i - 1);
    objective_ = std::make_unique<TrackingObjective<SPos>>(cfg_, problem_, map_manager_, h_polytopes_);
    if (!objective_->initialize(problem_, init_times_, init_pos_waypoints_, yaw_points,
                               yaw_head_state_, yaw_tail_state_,
                               use_corridor_ ? &corridor_spatial_map_ : nullptr))
      return fail("invalid tracking duration bounds or initial state");
    Eigen::VectorXd x = objective_->initialGuess();
    math_utils::lbfgs::lbfgs_parameter_t params;
    params.mem_size = 16; params.past = 3; params.min_step = 1.e-24;
    params.g_epsilon = 0.0; params.delta = std::max(1.e-8, cfg_.opt_accuracy);
    params.max_iterations = std::max(1, problem_.max_iterations);
    params.max_linesearch = 20;
    double min_cost = 0.0;
    solve_start_ = std::chrono::steady_clock::now();
    if (problem_.should_stop && problem_.should_stop()) return fail("planning deadline expired");
    const int ret = math_utils::lbfgs::lbfgs_optimize(x, min_cost,
        &ElasticTrackingRunner::costFunctional, nullptr, &ElasticTrackingRunner::progress, this, params);
    // A stopped solve is usable only if the resulting trajectory passes checks.
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(x.size());
    min_cost = evaluate(x, gradient);
    if (!std::isfinite(min_cost) || !gradient.allFinite())
      return fail("non-finite tracking objective or gradient; solver=" + std::to_string(ret));
    Trajectory candidate = toGeometryTrajectoryGeneric(objective_->position());
    std::string reason;
    if (use_corridor_ && !validateTrajectoryInCorridor(candidate, &reason)) return fail(reason);
    if (problem_.should_stop && problem_.should_stop()) return fail("planning deadline expired");
    out_traj = std::move(candidate);
    out_traj.start_WT = problem_.target_prediction.empty() ? 0.0
        : problem_.target_prediction.front().reference_time;
    if (out_yaw_traj) {
      *out_yaw_traj = toGeometryTrajectoryGeneric(objective_->yaw());
      out_yaw_traj->start_WT = out_traj.start_WT;
    }
    return !out_traj.empty();
  }

private:
  static double costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g)
  {
    auto *runner = reinterpret_cast<ElasticTrackingRunner *>(ptr);
    return runner->evaluate(x, g);
  }

  bool setupCorridorInitialState()
  {
    if (problem_.sfcs.empty())
    {
      return false;
    }

    h_polytopes_.clear();
    h_polytopes_.reserve(2 * problem_.sfcs.size());
    for (const auto &sfc : problem_.sfcs)
    {
      spatial_map::PolyhedronH h_poly = sfc.GetPlanes();
      normalizeTrackingHPoly(h_poly);
      if (h_poly.rows() == 0 || !std::isfinite(h_poly.sum()))
      {
        return false;
      }
      h_polytopes_.push_back(h_poly);
      h_polytopes_.push_back(h_poly);
    }

    piece_num_ = static_cast<int>(h_polytopes_.size());
    if (piece_num_ > 24) return false;
    if (piece_num_ <= 0)
    {
      return false;
    }

    if (!geometry_utils::pointInsidePolytope(problem_.head_pvaj.col(0), h_polytopes_.front(), 0.02) ||
        !geometry_utils::pointInsidePolytope(problem_.tail_pvaj.col(0), h_polytopes_.back(), 0.02))
      return false;

    init_times_.assign(static_cast<std::size_t>(piece_num_), std::max(0.05, problem_.min_piece_duration));
    init_pos_waypoints_.resize(piece_num_ + 1, 3);
    init_pos_waypoints_.row(0) = problem_.head_pvaj.col(0).transpose();
    init_pos_waypoints_.row(piece_num_) = problem_.tail_pvaj.col(0).transpose();

    v_polytopes_.clear();
    v_polytopes_.reserve(std::max(1, 2 * (piece_num_ - 1) + 1));
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

    const double guide_duration = problem_.guide_t.empty() ? 0.0 : problem_.guide_t.back();
    const auto guide = sanitizeGuide(problem_.guide_path, problem_.head_pvaj.col(0), problem_.tail_pvaj.col(0));
    std::vector<double> arc(guide.size(), 0.0);
    for (std::size_t i = 1; i < guide.size(); ++i)
      arc[i] = arc[i - 1] + (guide[i] - guide[i - 1]).norm();
    std::size_t guide_cursor = 0;

    for (int i = 0; i < piece_num_ - 1; ++i)
    {
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
      Vec3f seed = interior;
      const Vec3f preferred = interpolateByArc(guide, arc, arc.back() * (i + 1) / piece_num_);
      double best_distance = std::numeric_limits<double>::infinity();
      std::size_t best_id = guide_cursor;
      for (std::size_t g = guide_cursor; g < problem_.guide_path.size(); ++g) {
        const Vec3f &point = problem_.guide_path[g];
        const double distance = (point - preferred).squaredNorm();
        if (distance < best_distance && geometry_utils::pointInsidePolytope(point, overlap, -0.02)) {
          best_distance = distance;
          best_id = g;
          seed = 0.9 * point + 0.1 * interior;
        }
      }
      guide_cursor = best_id;
      if (geometry_utils::pointInsidePolytope(preferred, overlap, -0.02)) seed = preferred;
      init_pos_waypoints_.row(i + 1) = seed.transpose();
    }
    if (!geometry_utils::enumerateVs(h_polytopes_.back(), cur_v) ||
        !pushLocalVPoly(cur_v))
    {
      return false;
    }

    std::vector<double> lengths(static_cast<std::size_t>(piece_num_));
    double length = 0.0;
    for (int i = 0; i < piece_num_; ++i) {
      lengths[i] = (init_pos_waypoints_.row(i + 1) - init_pos_waypoints_.row(i)).norm();
      length += lengths[i];
    }
    const double vmax = clampPositive(cfg_.max_vel, 2.0);
    const double amax = clampPositive(cfg_.max_acc, 2.0);
    const double jmax = clampPositive(cfg_.max_jerk, 6.0);
    const double duration = std::max({problem_.min_total_duration, guide_duration,
        estimateDuration(length, problem_.head_pvaj.col(1).norm(),
                         problem_.tail_pvaj.col(1).norm(), vmax, amax) + amax / jmax});
    for (int i = 0; i < piece_num_; ++i) {
      init_times_[i] = std::max(std::max(0.05, problem_.min_piece_duration),
          duration * (length > 1.e-6 ? lengths[i] / length : 1.0 / piece_num_));
    }

    corridor_spatial_map_.reset(&v_polytopes_, &v_poly_idx_, piece_num_ - 1, false);
    return true;
  }

  bool validateTrajectoryInCorridor(const Trajectory &traj, std::string *reason) const
  {
    if (traj.empty() || traj.getPieceNum() != static_cast<int>(h_polytopes_.size())) {
      if (reason) *reason = "SFC piece count mismatch";
      return false;
    }
    double worst = 0.0, worst_t = 0.0;
    int worst_piece = -1, worst_plane = -1;
    for (int i = 0; i < traj.getPieceNum(); ++i) {
      const double T = traj[i].getDuration();
      if (!std::isfinite(T) || T <= 0.0) {
        if (reason) *reason = "SFC non-finite or non-positive duration";
        return false;
      }
      const int samples = std::max({4, samples_per_piece_, static_cast<int>(std::ceil(T / 0.05))});
      const auto &planes = h_polytopes_[i];
      for (int k = 0; k <= samples; ++k) {
        const double t = T * static_cast<double>(k) / samples;
        const Vec3f p = traj[i].getPos(t);
        if (!p.allFinite()) {
          if (reason) *reason = "SFC non-finite position";
          return false;
        }
        for (int plane = 0; plane < planes.rows(); ++plane) {
          const double violation = planes.row(plane).head<3>().dot(p) + planes(plane, 3);
          if (violation > worst) {
            worst = violation; worst_t = t; worst_piece = i; worst_plane = plane;
          }
        }
      }
    }
    if (worst <= 0.02) return true;
    if (reason) *reason = "SFC violation: piece=" + std::to_string(worst_piece) +
        "; local_t=" + std::to_string(worst_t) + "; plane=" + std::to_string(worst_plane) +
        "; outside_m=" + std::to_string(worst) + "; tolerance_m=0.02";
    return false;
  }

  traj_opt::DynamicTargetState targetAt(double t) const
  {
    return traj_opt::sampleTrackingTarget(problem_.target_prediction, t);
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
    yaw_head_state_.setZero();
    yaw_tail_state_.setZero();
    yaw_head_state_(0, 0) = std::isfinite(problem_.head_yaw(0, 0))
                                ? problem_.head_yaw(0, 0)
                                : 0.0;
    yaw_head_state_(0, 1) = std::isfinite(problem_.head_yaw(0, 1))
                                ? problem_.head_yaw(0, 1)
                                : 0.0;

    yaw_head_state_(0, 2) = std::isfinite(problem_.head_yaw_acceleration)
                                ? problem_.head_yaw_acceleration : 0.0;

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
    yaw_tail_state_(0, 0) = tail_yaw;
    const Vec3f sight = tail_target.position - problem_.tail_pvaj.col(0);
    const Vec3f relative_velocity = tail_target.velocity - problem_.tail_pvaj.col(1);
    const double bearing_rate = sight.head<2>().squaredNorm() > 1.e-6
        ? (sight.x() * relative_velocity.y() - sight.y() * relative_velocity.x()) /
          sight.head<2>().squaredNorm() : 0.0;
    yaw_tail_state_(0, 1) = std::clamp(bearing_rate, -problem_.max_yaw_rate, problem_.max_yaw_rate);

  }

  double evaluate(const Eigen::VectorXd &x, Eigen::VectorXd &g) {
    return objective_->evaluate(x, g);
  }

  static int progress(void *ptr, const Eigen::VectorXd &, const Eigen::VectorXd &,
                      double, double, int, int) {
    const auto &self = *static_cast<ElasticTrackingRunner *>(ptr);
    return self.expired() ? 1 : 0;
  }

  bool expired() const {
    return (problem_.should_stop && problem_.should_stop()) ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() - solve_start_).count()
            >= std::max(0.001, problem_.solve_budget_seconds);
  }

private:
  traj_opt::Config cfg_;
  general_planner::MapManager::Ptr map_manager_;
  double safe_distance_{0.45};
  int piece_num_{0};
  int samples_per_piece_{5};
  bool use_corridor_{false};

  TrackingProblem problem_;
  std::vector<double> init_times_;
  typename TaskOptimizer<SPos>::WaypointsType init_pos_waypoints_;
  YawInnerMat init_yaw_inner_;
  spatial_map::PolyhedraH h_polytopes_;
  spatial_map::PolyhedraV v_polytopes_;
  Eigen::VectorXi v_poly_idx_;
  spatial_map::PolytopeSpatialMap corridor_spatial_map_;

  std::unique_ptr<TrackingObjective<SPos>> objective_;
  std::chrono::steady_clock::time_point solve_start_;
  YawBoundaryState yaw_head_state_;
  YawBoundaryState yaw_tail_state_;
};


} // namespace
struct TrackingJerkTrajOpt::Impl final : public ElasticTrackingRunner<3>
{
  Impl(const traj_opt::Config &cfg,
       const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
      : ElasticTrackingRunner<3>(cfg, ros_ptr)
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
                                   Trajectory *out_yaw_traj,
                                   std::string *failure_reason)
{
  return impl_->optimize(problem, out_traj, out_yaw_traj, failure_reason);
}

struct TrackingSnapTrajOpt::Impl final : public ElasticTrackingRunner<4>
{
  Impl(const traj_opt::Config &cfg,
       const std::shared_ptr<ros_interface::RosInterface> &ros_ptr)
      : ElasticTrackingRunner<4>(cfg, ros_ptr)
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
                                   Trajectory *out_yaw_traj,
                                   std::string *failure_reason)
{
  return impl_->optimize(problem, out_traj, out_yaw_traj, failure_reason);
}


} // namespace traj_opt
