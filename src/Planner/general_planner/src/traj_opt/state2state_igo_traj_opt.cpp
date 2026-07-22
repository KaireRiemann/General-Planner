#include <traj_opt/state2state_igo_traj_opt.hpp>

#include <traj_opt/convex_hull/convex_hull.hpp>
#include <traj_opt/zero_order/gaussian_igo.hpp>
#include <utils/geometry/geometry_utils.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>

namespace traj_opt
{
namespace
{

using SnapTrajIgo = minco::MINCO_S4<3>;
using BoundaryState = SnapTrajIgo::BoundaryState;
using Vec3 = Eigen::Vector3d;
using HPoly = Eigen::Matrix<double, Eigen::Dynamic, 4>;

struct OccupancySnapshot
{
  enum Cell : std::uint8_t
  {
    FREE = 0,
    UNKNOWN = 1,
    OCCUPIED = 2,
    OUTSIDE = 3
  };

  Vec3 origin{Vec3::Zero()};
  Eigen::Vector3i dimensions{Eigen::Vector3i::Zero()};
  double resolution{0.0};
  std::vector<std::uint8_t> cells;

  Cell query(const Vec3 &position) const
  {
    if (!position.allFinite() || !(resolution > 0.0))
    {
      return OUTSIDE;
    }
    const Eigen::Vector3i index = ((position - origin) / resolution).array().floor().cast<int>();
    if ((index.array() < 0).any() || (index.array() >= dimensions.array()).any())
    {
      return OUTSIDE;
    }
    const std::size_t linear = static_cast<std::size_t>(index.x()) +
                               static_cast<std::size_t>(dimensions.x()) *
                                   (static_cast<std::size_t>(index.y()) +
                                    static_cast<std::size_t>(dimensions.y()) *
                                        static_cast<std::size_t>(index.z()));
    return static_cast<Cell>(cells[linear]);
  }
};

struct CandidateScore
{
  bool numerical_valid{false};
  bool feasible{false};
  int occupied_samples{0};
  int outside_samples{0};
  int unknown_hard_samples{0};
  int unknown_samples{0};
  double sfc_max_violation{std::numeric_limits<double>::infinity()};
  double sfc_sum_violation{std::numeric_limits<double>::infinity()};
  double dynamic_max_violation{std::numeric_limits<double>::infinity()};
  double dynamic_sum_violation{std::numeric_limits<double>::infinity()};
  double quality{std::numeric_limits<double>::infinity()};
};

struct Candidate
{
  CandidateScore score;
  Eigen::Matrix3Xd points;
  Eigen::VectorXd durations;
};

HPoly normalizePlanes(const HPoly &planes)
{
  HPoly normalized = planes;
  for (int row = 0; row < normalized.rows(); ++row)
  {
    const double norm = normalized.row(row).head<3>().norm();
    if (norm > 1.0e-12)
    {
      normalized.row(row) /= norm;
    }
  }
  return normalized;
}

BoundaryState toBoundary(const general_utils::StatePVAJ &state)
{
  BoundaryState boundary;
  boundary.col(0) = state.col(0);
  boundary.col(1) = state.col(1);
  boundary.col(2) = state.col(2);
  boundary.col(3) = state.col(3);
  return boundary;
}

geometry_utils::Trajectory toTrajectory(const SnapTrajIgo &minco)
{
  geometry_utils::Trajectory trajectory;
  const auto &durations = minco.getDurations();
  trajectory.reserve(static_cast<int>(durations.size()));
  for (int piece = 0; piece < durations.size(); ++piece)
  {
    trajectory.emplace_back(durations(piece), minco.getPieceCoeffMat(piece));
  }
  return trajectory;
}

bool scoreBetter(const CandidateScore &lhs, const CandidateScore &rhs)
{
  if (lhs.numerical_valid != rhs.numerical_valid)
  {
    return lhs.numerical_valid;
  }
  if (!lhs.numerical_valid)
  {
    return lhs.quality < rhs.quality;
  }
  const auto lhs_hard = std::tie(lhs.occupied_samples,
                                 lhs.outside_samples,
                                 lhs.unknown_hard_samples);
  const auto rhs_hard = std::tie(rhs.occupied_samples,
                                 rhs.outside_samples,
                                 rhs.unknown_hard_samples);
  if (lhs_hard != rhs_hard)
  {
    return lhs_hard < rhs_hard;
  }
  const auto lhs_constraints = std::tie(lhs.sfc_max_violation,
                                        lhs.sfc_sum_violation,
                                        lhs.dynamic_max_violation,
                                        lhs.dynamic_sum_violation);
  const auto rhs_constraints = std::tie(rhs.sfc_max_violation,
                                        rhs.sfc_sum_violation,
                                        rhs.dynamic_max_violation,
                                        rhs.dynamic_sum_violation);
  if (lhs_constraints != rhs_constraints)
  {
    return lhs_constraints < rhs_constraints;
  }
  return lhs.quality < rhs.quality;
}

double rayLimit(const HPoly &polytope, const Vec3 &center, const Vec3 &direction)
{
  double limit = std::numeric_limits<double>::infinity();
  for (int row = 0; row < polytope.rows(); ++row)
  {
    const double denominator = polytope.row(row).head<3>().dot(direction);
    if (denominator > 1.0e-12)
    {
      const double numerator = -(polytope.row(row).head<3>().dot(center) + polytope(row, 3));
      limit = std::min(limit, numerator / denominator);
    }
  }
  return limit;
}

bool findGuideAnchor(const general_utils::vec_E<general_utils::Vec3f> &guide_path,
                     const std::vector<double> &guide_stamps,
                     const HPoly &polytope,
                     const Vec3 &reference,
                     double minimum_time,
                     double margin,
                     Vec3 &anchor,
                     double &anchor_time)
{
  if (guide_path.size() < 2 || guide_path.size() != guide_stamps.size())
  {
    return false;
  }
  double best_score = std::numeric_limits<double>::infinity();
  bool found = false;
  for (std::size_t segment_index = 0; segment_index + 1 < guide_path.size(); ++segment_index)
  {
    const Vec3 start = guide_path[segment_index];
    const Vec3 delta = guide_path[segment_index + 1] - start;
    const double start_time = guide_stamps[segment_index];
    const double end_time = guide_stamps[segment_index + 1];
    if (!start.allFinite() || !delta.allFinite() || !std::isfinite(start_time) ||
        !std::isfinite(end_time) || end_time + 1.0e-9 < minimum_time)
    {
      continue;
    }
    double lower = 0.0;
    double upper = 1.0;
    for (int row = 0; row < polytope.rows() && lower <= upper; ++row)
    {
      const double value = polytope.row(row).head<3>().dot(start) +
                           polytope(row, 3) + margin;
      const double slope = polytope.row(row).head<3>().dot(delta);
      if (std::abs(slope) <= 1.0e-12)
      {
        if (value > 0.0)
        {
          lower = 1.0;
          upper = 0.0;
        }
      }
      else if (slope > 0.0)
      {
        upper = std::min(upper, -value / slope);
      }
      else
      {
        lower = std::max(lower, -value / slope);
      }
    }
    const double segment_time = end_time - start_time;
    if (segment_time > 1.0e-9)
    {
      lower = std::max(lower, (minimum_time - start_time) / segment_time);
    }
    lower = std::clamp(lower, 0.0, 1.0);
    upper = std::clamp(upper, 0.0, 1.0);
    if (lower > upper)
    {
      continue;
    }
    const double projection = delta.squaredNorm() > 1.0e-12
                                  ? (reference - start).dot(delta) / delta.squaredNorm()
                                  : 0.5;
    const double ratio = std::clamp(projection, lower, upper);
    const Vec3 candidate = start + ratio * delta;
    const double candidate_time = start_time + ratio * segment_time;
    const double score = (candidate - reference).squaredNorm();
    if (score < best_score)
    {
      best_score = score;
      anchor = candidate;
      anchor_time = candidate_time;
      found = true;
    }
  }
  return found;
}

Vec3 decodePoint(const Vec3 &unconstrained,
                 const Vec3 &center,
                 const HPoly &polytope,
                 double spatial_margin)
{
  const double magnitude = unconstrained.norm();
  if (magnitude < 1.0e-12)
  {
    return center;
  }
  const Vec3 direction = unconstrained / magnitude;
  const double raw_limit = rayLimit(polytope, center, direction);
  if (!std::isfinite(raw_limit) || raw_limit <= spatial_margin)
  {
    return center;
  }
  const double safe_limit = std::max(0.0, raw_limit - std::max(0.0, spatial_margin));
  return center + std::tanh(magnitude) * safe_limit * direction;
}

double sigmoid(double value)
{
  if (value >= 0.0)
  {
    const double exp_negative = std::exp(-value);
    return 1.0 / (1.0 + exp_negative);
  }
  const double exp_positive = std::exp(value);
  return exp_positive / (1.0 + exp_positive);
}

double decodeDuration(double variable,
                      double reference,
                      double min_scale,
                      double max_scale)
{
  const double lower = std::max(0.02, reference * std::max(0.05, min_scale));
  const double upper = std::max(lower + 0.02, reference * std::max(min_scale + 0.05, max_scale));
  const double ratio = std::clamp((reference - lower) / (upper - lower), 1.0e-6, 1.0 - 1.0e-6);
  const double bias = std::log(ratio / (1.0 - ratio));
  return lower + (upper - lower) * sigmoid(variable + bias);
}

double encodeDuration(double duration,
                      double reference,
                      double min_scale,
                      double max_scale)
{
  const double lower = std::max(0.02, reference * std::max(0.05, min_scale));
  const double upper = std::max(lower + 0.02, reference * std::max(min_scale + 0.05, max_scale));
  const double reference_ratio = std::clamp((reference - lower) / (upper - lower), 1.0e-6, 1.0 - 1.0e-6);
  const double ratio = std::clamp((duration - lower) / (upper - lower), 1.0e-6, 1.0 - 1.0e-6);
  return std::log(ratio / (1.0 - ratio)) -
         std::log(reference_ratio / (1.0 - reference_ratio));
}

Eigen::MatrixXd smoothInitialCovariance(int point_count,
                                        int piece_count,
                                        double sigma)
{
  const int dimension = point_count * 3 + piece_count;
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(dimension, dimension);
  const double variance = std::max(1.0e-4, sigma * sigma);
  if (point_count > 0)
  {
    Eigen::MatrixXd difference = Eigen::MatrixXd::Zero(std::max(0, point_count - 2), point_count);
    for (int row = 0; row < difference.rows(); ++row)
    {
      difference(row, row) = 1.0;
      difference(row, row + 1) = -2.0;
      difference(row, row + 2) = 1.0;
    }
    const Eigen::MatrixXd point_covariance =
        variance * (Eigen::MatrixXd::Identity(point_count, point_count) +
                    4.0 * difference.transpose() * difference)
                       .inverse();
    for (int i = 0; i < point_count; ++i)
    {
      for (int j = 0; j < point_count; ++j)
      {
        covariance.block<3, 3>(3 * i, 3 * j) =
            point_covariance(i, j) * Eigen::Matrix3d::Identity();
      }
    }
  }
  Eigen::MatrixXd first_difference = Eigen::MatrixXd::Zero(std::max(0, piece_count - 1), piece_count);
  for (int row = 0; row < first_difference.rows(); ++row)
  {
    first_difference(row, row) = -1.0;
    first_difference(row, row + 1) = 1.0;
  }
  const Eigen::MatrixXd time_covariance =
      variance * (Eigen::MatrixXd::Identity(piece_count, piece_count) +
                  2.0 * first_difference.transpose() * first_difference)
                     .inverse();
  covariance.bottomRightCorner(piece_count, piece_count) = time_covariance;
  return covariance;
}

bool buildSnapshot(const geometry_utils::PolytopeVec &corridor,
                   const general_planner::MapManager::Ptr &map_manager,
                   bool use_inflated_map,
                   int max_voxels,
                   OccupancySnapshot &snapshot,
                   std::string &reason)
{
  if (!map_manager || !map_manager->ready())
  {
    reason = "MAP_NOT_READY";
    return false;
  }
  Vec3 lower = Vec3::Constant(std::numeric_limits<double>::infinity());
  Vec3 upper = Vec3::Constant(-std::numeric_limits<double>::infinity());
  for (const auto &polytope : corridor)
  {
    Eigen::Matrix3Xd vertices;
    if (!geometry_utils::enumerateVs(polytope.GetPlanes(), vertices) || vertices.cols() == 0)
    {
      reason = "CORRIDOR_VERTEX_ENUMERATION_FAILED";
      return false;
    }
    lower = lower.cwiseMin(vertices.rowwise().minCoeff());
    upper = upper.cwiseMax(vertices.rowwise().maxCoeff());
  }
  snapshot.resolution = std::max(
      0.02,
      use_inflated_map ? map_manager->getInfResolution() : map_manager->getResolution());
  const double padding = 2.0 * snapshot.resolution;
  snapshot.origin = ((lower.array() - padding) / snapshot.resolution).floor().matrix() *
                    snapshot.resolution;
  snapshot.dimensions = (((upper.array() + padding - snapshot.origin.array()) /
                          snapshot.resolution)
                             .ceil()
                             .cast<int>() +
                         1)
                            .matrix();
  const std::size_t voxel_count = static_cast<std::size_t>(snapshot.dimensions.x()) *
                                  static_cast<std::size_t>(snapshot.dimensions.y()) *
                                  static_cast<std::size_t>(snapshot.dimensions.z());
  if (voxel_count == 0 || voxel_count > static_cast<std::size_t>(std::max(1, max_voxels)))
  {
    reason = "MAP_SNAPSHOT_TOO_LARGE";
    return false;
  }
  snapshot.cells.resize(voxel_count, OccupancySnapshot::OUTSIDE);
  for (int z = 0; z < snapshot.dimensions.z(); ++z)
  {
    for (int y = 0; y < snapshot.dimensions.y(); ++y)
    {
      for (int x = 0; x < snapshot.dimensions.x(); ++x)
      {
        const Vec3 position = snapshot.origin +
                              snapshot.resolution *
                                  (Eigen::Vector3d(x, y, z).array() + 0.5).matrix();
        OccupancySnapshot::Cell cell = OccupancySnapshot::FREE;
        if (!map_manager->insideLocalMap(position))
        {
          cell = OccupancySnapshot::OUTSIDE;
        }
        else
        {
          const auto raw = map_manager->getGridType(position);
          const auto selected = use_inflated_map ? map_manager->getInfGridType(position) : raw;
          if (selected == rog_map::GridType::OCCUPIED)
          {
            cell = OccupancySnapshot::OCCUPIED;
          }
          else if (selected == rog_map::GridType::OUT_OF_MAP ||
                   raw == rog_map::GridType::OUT_OF_MAP)
          {
            cell = OccupancySnapshot::OUTSIDE;
          }
          else if (raw == rog_map::GridType::UNKNOWN)
          {
            cell = OccupancySnapshot::UNKNOWN;
          }
        }
        const std::size_t linear = static_cast<std::size_t>(x) +
                                   static_cast<std::size_t>(snapshot.dimensions.x()) *
                                       (static_cast<std::size_t>(y) +
                                        static_cast<std::size_t>(snapshot.dimensions.y()) *
                                            static_cast<std::size_t>(z));
        snapshot.cells[linear] = cell;
      }
    }
  }
  return true;
}

double nearestGuideDistance(const Vec3 &position,
                            const general_utils::vec_E<general_utils::Vec3f> &guide)
{
  double distance = std::numeric_limits<double>::infinity();
  for (const auto &point : guide)
  {
    distance = std::min(distance, (position - point).norm());
  }
  return std::isfinite(distance) ? distance : 0.0;
}

void evaluateHull(const SnapTrajIgo &minco,
                  const std::vector<HPoly> &corridor,
                  const Config &config,
                  CandidateScore &score)
{
  using Representation = convex_hull::Representation<3>;
  const int pieces = static_cast<int>(corridor.size());
  Representation::Matrix coefficients = minco.getCoefficients();
  for (int derivative = 0; derivative <= 3; ++derivative)
  {
    Representation representation;
    representation.resetTopology(pieces,
                                 8,
                                 convex_hull::Basis::Bezier,
                                 derivative,
                                 std::max(0, config.igo_hull_subdivision_depth));
    representation.update(coefficients, minco.getDurations());
    for (int hull_piece = 0; hull_piece < representation.numPieces(); ++hull_piece)
    {
      const int source_piece = representation.pieceInfo(hull_piece).source_segment;
      const auto controls = representation.pieceControls(hull_piece);
      for (int row = 0; row < controls.rows(); ++row)
      {
        if (derivative == 0)
        {
          const Eigen::VectorXd violations =
              corridor[static_cast<std::size_t>(source_piece)].leftCols<3>() *
                  controls.row(row).transpose() +
              corridor[static_cast<std::size_t>(source_piece)].rightCols<1>();
          const double violation = std::max(0.0, violations.maxCoeff());
          score.sfc_max_violation = std::max(score.sfc_max_violation, violation);
          score.sfc_sum_violation += violations.cwiseMax(0.0).sum();
        }
        else
        {
          const double limit = derivative == 1 ? config.max_vel
                                : derivative == 2 ? config.max_acc
                                                  : config.max_jerk;
          if (limit > 0.0)
          {
            const double violation = std::max(0.0, controls.row(row).norm() / limit - 1.0);
            score.dynamic_max_violation = std::max(score.dynamic_max_violation, violation);
            score.dynamic_sum_violation += violation;
          }
        }
      }
    }
  }
}

}  // namespace

StateToStateIgoTrajOpt::StateToStateIgoTrajOpt(
    const Config &config,
    const ros_interface::RosInterface::Ptr &ros_ptr,
    const general_planner::MapManager::Ptr &map_manager)
    : config_(config), ros_ptr_(ros_ptr), map_manager_(map_manager)
{
}

void StateToStateIgoTrajOpt::setMapManager(
    const general_planner::MapManager::Ptr &map_manager)
{
  map_manager_ = map_manager;
}

bool StateToStateIgoTrajOpt::optimize(
    const general_utils::StatePVAJ &head,
    const general_utils::StatePVAJ &tail,
    const general_utils::vec_E<general_utils::Vec3f> &guide_path,
    const std::vector<double> &guide_stamps,
    geometry_utils::PolytopeVec &corridor,
    double wall_time_budget,
    geometry_utils::Trajectory &trajectory)
{
  const auto begin = std::chrono::steady_clock::now();
  const auto deadline = begin + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(std::max(1.0e-3, wall_time_budget)));
  last_report_ = Report{};
  last_report_.population = std::max(4, config_.igo_population);
  trajectory.clear();

  if (corridor.empty() ||
      !geometry_utils::SimplifySFC(head.col(0), tail.col(0), corridor) ||
      corridor.empty())
  {
    last_report_.reason = "INVALID_CORRIDOR";
    return false;
  }
  const int piece_count = static_cast<int>(corridor.size());
  const int point_count = piece_count - 1;
  const int dimension = 3 * point_count + piece_count;
  last_report_.decision_dimension = dimension;

  std::vector<HPoly> h_polytopes;
  h_polytopes.reserve(corridor.size());
  for (const auto &polytope : corridor)
  {
    h_polytopes.push_back(normalizePlanes(polytope.GetPlanes()));
  }
  std::vector<HPoly> overlaps;
  std::vector<Vec3> centers;
  std::vector<Vec3> strict_centers;
  std::vector<double> anchor_times;
  overlaps.reserve(static_cast<std::size_t>(point_count));
  centers.reserve(static_cast<std::size_t>(point_count));
  strict_centers.reserve(static_cast<std::size_t>(point_count));
  anchor_times.reserve(static_cast<std::size_t>(point_count));
  double previous_anchor_time = guide_stamps.empty() ? 0.0 : guide_stamps.front();
  for (int point = 0; point < point_count; ++point)
  {
    HPoly overlap(h_polytopes[static_cast<std::size_t>(point)].rows() +
                      h_polytopes[static_cast<std::size_t>(point + 1)].rows(),
                  4);
    overlap << h_polytopes[static_cast<std::size_t>(point)],
        h_polytopes[static_cast<std::size_t>(point + 1)];
    Vec3 center;
    const double depth = geometry_utils::findInteriorDist(overlap, center);
    if (!std::isfinite(depth) || depth <= std::max(1.0e-5, config_.igo_spatial_margin))
    {
      last_report_.reason = "CORRIDOR_OVERLAP_NOT_STRICT";
      return false;
    }
    Vec3 anchor = center;
    double anchor_time = previous_anchor_time;
    findGuideAnchor(guide_path,
                    guide_stamps,
                    overlap,
                    center,
                    previous_anchor_time,
                    std::max(1.0e-5, config_.igo_spatial_margin),
                    anchor,
                    anchor_time);
    overlaps.push_back(overlap);
    centers.push_back(anchor);
    strict_centers.push_back(center);
    anchor_times.push_back(anchor_time);
    previous_anchor_time = anchor_time;
  }

  Eigen::VectorXd reference_durations(piece_count);
  std::vector<Vec3> chain;
  chain.reserve(static_cast<std::size_t>(piece_count + 1));
  chain.push_back(head.col(0));
  chain.insert(chain.end(), centers.begin(), centers.end());
  chain.push_back(tail.col(0));
  const double profile_velocity = std::max(0.2, config_.max_vel *
                                                    std::max(0.1, config_.init_profile_vel_ratio));
  for (int piece = 0; piece < piece_count; ++piece)
  {
    const double distance_duration =
        (chain[static_cast<std::size_t>(piece + 1)] -
         chain[static_cast<std::size_t>(piece)])
            .norm() /
        profile_velocity * std::max(0.2, config_.init_duration_scale);
    double guide_duration = 0.0;
    if (guide_stamps.size() == guide_path.size() && guide_stamps.size() >= 2)
    {
      const double start_time = piece == 0 ? guide_stamps.front()
                                           : anchor_times[static_cast<std::size_t>(piece - 1)];
      const double end_time = piece == piece_count - 1
                                  ? guide_stamps.back()
                                  : anchor_times[static_cast<std::size_t>(piece)];
      guide_duration = std::max(0.0, end_time - start_time);
    }
    reference_durations(piece) = std::max({0.05, distance_duration, guide_duration});
  }

  OccupancySnapshot snapshot;
  std::string snapshot_reason;
  if (!buildSnapshot(corridor,
                     map_manager_,
                     config_.igo_use_inflated_map,
                     config_.igo_snapshot_max_voxels,
                     snapshot,
                     snapshot_reason))
  {
    last_report_.reason = snapshot_reason;
    return false;
  }

  const BoundaryState head_boundary = toBoundary(head);
  const BoundaryState tail_boundary = toBoundary(tail);
  auto decode = [&](const Eigen::VectorXd &decision,
                    Eigen::Matrix3Xd &points,
                    Eigen::VectorXd &durations) {
    points.resize(3, point_count);
    for (int point = 0; point < point_count; ++point)
    {
      points.col(point) = decodePoint(decision.segment<3>(3 * point),
                                      centers[static_cast<std::size_t>(point)],
                                      overlaps[static_cast<std::size_t>(point)],
                                      config_.igo_spatial_margin);
    }
    durations.resize(piece_count);
    for (int piece = 0; piece < piece_count; ++piece)
    {
      durations(piece) = decodeDuration(decision(3 * point_count + piece),
                                        reference_durations(piece),
                                        config_.igo_time_min_scale,
                                        config_.igo_time_max_scale);
    }
  };

  auto evaluator = [&](const Eigen::VectorXd &decision) {
    Candidate candidate;
    CandidateScore &score = candidate.score;
    score.sfc_max_violation = 0.0;
    score.sfc_sum_violation = 0.0;
    score.dynamic_max_violation = 0.0;
    score.dynamic_sum_violation = 0.0;
    try
    {
      decode(decision, candidate.points, candidate.durations);
      SnapTrajIgo minco;
      if (!minco.generate(candidate.points,
                          head_boundary,
                          tail_boundary,
                          candidate.durations) ||
          !minco.getCoefficients().allFinite())
      {
        score.quality = 1.0;
        return candidate;
      }
      score.numerical_valid = true;
      evaluateHull(minco, h_polytopes, config_, score);
      const double total_duration = candidate.durations.sum();
      const double sample_dt = std::max(
          0.005,
          std::min(config_.igo_sample_dt,
                   0.5 * snapshot.resolution / std::max(0.2, config_.max_vel)));
      double guide_distance = 0.0;
      int sample_count = 0;
      for (double time = 0.0; time < total_duration + 0.5 * sample_dt; time += sample_dt)
      {
        const Vec3 position = minco.getPos(std::min(time, total_duration));
        const auto cell = snapshot.query(position);
        score.occupied_samples += cell == OccupancySnapshot::OCCUPIED ? 1 : 0;
        score.outside_samples += cell == OccupancySnapshot::OUTSIDE ? 1 : 0;
        score.unknown_samples += cell == OccupancySnapshot::UNKNOWN ? 1 : 0;
        score.unknown_hard_samples +=
            config_.igo_unknown_as_occupied && cell == OccupancySnapshot::UNKNOWN ? 1 : 0;
        guide_distance += nearestGuideDistance(position, guide_path);
        ++sample_count;
      }
      const double dynamic_tolerance = std::max(0.0, config_.igo_dynamic_tolerance);
      score.feasible = score.occupied_samples == 0 && score.outside_samples == 0 &&
                       score.unknown_hard_samples == 0 &&
                       score.sfc_max_violation <= std::max(0.0, config_.igo_sfc_tolerance) &&
                       score.dynamic_max_violation <= dynamic_tolerance;
      score.quality = total_duration +
                      config_.igo_energy_weight * std::log1p(std::max(0.0, minco.getEnergy())) +
                      config_.igo_guide_weight * guide_distance / std::max(1, sample_count) +
                      config_.igo_unknown_weight * score.unknown_samples /
                          static_cast<double>(std::max(1, sample_count));
    }
    catch (const std::exception &)
    {
      score = CandidateScore{};
      score.quality = 1.0;
    }
    return candidate;
  };

  Eigen::VectorXd initial_mean = Eigen::VectorXd::Zero(dimension);
  std::vector<Eigen::VectorXd> injections;
  injections.push_back(Eigen::VectorXd::Zero(dimension));
  // Feasibility seeds are part of the population, not a separate optimizer.
  // They are essential for short/high-speed braking horizons where a random
  // temporal perturbation is unlikely to discover a dynamically valid scale.
  for (const double time_scale : {1.5, 2.0, 2.6})
  {
    Eigen::VectorXd slowed = Eigen::VectorXd::Zero(dimension);
    for (int piece = 0; piece < piece_count; ++piece)
    {
      slowed(3 * point_count + piece) =
          encodeDuration(reference_durations(piece) * time_scale,
                         reference_durations(piece),
                         config_.igo_time_min_scale,
                         config_.igo_time_max_scale);
    }
    injections.push_back(slowed);
  }
  for (const double spatial_blend : {0.35, 0.70, 1.0})
  {
    Eigen::VectorXd interior_seed = Eigen::VectorXd::Zero(dimension);
    bool encodable = true;
    for (int point = 0; point < point_count; ++point)
    {
      const Vec3 target = (1.0 - spatial_blend) * centers[static_cast<std::size_t>(point)] +
                          spatial_blend * strict_centers[static_cast<std::size_t>(point)];
      const Vec3 delta = target - centers[static_cast<std::size_t>(point)];
      const double distance = delta.norm();
      if (distance <= 1.0e-12)
      {
        continue;
      }
      const Vec3 direction = delta / distance;
      const double limit = rayLimit(overlaps[static_cast<std::size_t>(point)],
                                    centers[static_cast<std::size_t>(point)],
                                    direction) -
                           std::max(0.0, config_.igo_spatial_margin);
      if (!(limit > distance))
      {
        encodable = false;
        break;
      }
      interior_seed.segment<3>(3 * point) =
          std::atanh(std::clamp(distance / limit, 0.0, 1.0 - 1.0e-6)) * direction;
    }
    for (int piece = 0; encodable && piece < piece_count; ++piece)
    {
      interior_seed(3 * point_count + piece) =
          encodeDuration(reference_durations(piece) * 2.2,
                         reference_durations(piece),
                         config_.igo_time_min_scale,
                         config_.igo_time_max_scale);
    }
    if (encodable && interior_seed.allFinite())
    {
      injections.push_back(interior_seed);
    }
  }
  if (previous_piece_count_ == piece_count &&
      static_cast<int>(last_points_.size()) == point_count &&
      last_durations_.size() == piece_count)
  {
    Eigen::VectorXd warm = Eigen::VectorXd::Zero(dimension);
    bool valid_warm = true;
    for (int point = 0; point < point_count; ++point)
    {
      const Vec3 delta = last_points_[static_cast<std::size_t>(point)] -
                         centers[static_cast<std::size_t>(point)];
      const double distance = delta.norm();
      if (distance < 1.0e-12)
      {
        continue;
      }
      const Vec3 direction = delta / distance;
      const double limit = rayLimit(overlaps[static_cast<std::size_t>(point)],
                                    centers[static_cast<std::size_t>(point)],
                                    direction) -
                           std::max(0.0, config_.igo_spatial_margin);
      if (!(limit > distance))
      {
        valid_warm = false;
        break;
      }
      const double ratio = std::clamp(distance / limit, 0.0, 1.0 - 1.0e-6);
      warm.segment<3>(3 * point) = std::atanh(ratio) * direction;
    }
    for (int piece = 0; valid_warm && piece < piece_count; ++piece)
    {
      warm(3 * point_count + piece) = encodeDuration(last_durations_(piece),
                                                     reference_durations(piece),
                                                     config_.igo_time_min_scale,
                                                     config_.igo_time_max_scale);
    }
    if (valid_warm && warm.allFinite())
    {
      injections.push_back(warm);
    }
  }

  zero_order::GaussianIgoOptions options;
  options.population = config_.igo_population;
  options.generations = config_.igo_generations;
  options.threads = config_.igo_threads;
  options.no_feasible_expand_generations = config_.igo_no_feasible_expand_generations;
  options.elite_ratio = config_.igo_elite_ratio;
  options.mean_learning_rate = config_.igo_mean_learning_rate;
  options.covariance_learning_rate = config_.igo_covariance_learning_rate;
  options.min_eigenvalue = config_.igo_min_eigenvalue;
  options.max_condition_number = config_.igo_max_condition_number;
  options.covariance_expand_factor = config_.igo_covariance_expand_factor;
  options.antithetic = config_.igo_antithetic;
  options.seed = static_cast<std::uint64_t>(std::max(0, config_.igo_seed));

  zero_order::GaussianIgoSolver solver;
  const auto result = solver.optimize<Candidate>(
      initial_mean,
      smoothInitialCovariance(point_count, piece_count, config_.igo_initial_sigma),
      injections,
      options,
      deadline,
      evaluator,
      [](const Candidate &lhs, const Candidate &rhs) { return scoreBetter(lhs.score, rhs.score); },
      [](const Candidate &candidate) { return candidate.score.feasible; });

  last_report_.timed_out = result.timed_out;
  last_report_.generations = result.generations;
  last_report_.evaluations = result.evaluations;
  last_report_.feasible_ratio = result.final_feasible_ratio;
  last_report_.elapsed_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  if (!result.has_feasible || !result.best_feasible)
  {
    if (result.best_infeasible)
    {
      const auto &score = result.best_infeasible->score;
      last_report_.best_occupied_samples = score.occupied_samples;
      last_report_.best_outside_samples = score.outside_samples;
      last_report_.best_unknown_samples = score.unknown_samples;
      last_report_.best_sfc_violation = score.sfc_max_violation;
      last_report_.best_dynamic_violation = score.dynamic_max_violation;
      last_report_.best_quality = score.quality;
      last_report_.best_duration = result.best_infeasible->durations.sum();
    }
    last_report_.reason = result.timed_out ? "DEADLINE_NO_FEASIBLE" : "NO_FEASIBLE_CANDIDATE";
    return false;
  }

  const Candidate &best = *result.best_feasible;
  SnapTrajIgo best_minco;
  if (!best_minco.generate(best.points, head_boundary, tail_boundary, best.durations))
  {
    last_report_.reason = "ARCHIVE_RECOVERY_FAILED";
    return false;
  }
  trajectory = toTrajectory(best_minco);
  last_durations_ = best.durations;
  last_points_.clear();
  for (int point = 0; point < best.points.cols(); ++point)
  {
    last_points_.push_back(best.points.col(point));
  }
  previous_decision_ = result.best_feasible_decision;
  previous_piece_count_ = piece_count;
  last_report_.success = true;
  last_report_.reason = "FEASIBLE_ARCHIVE";
  last_report_.best_duration = best.durations.sum();
  last_report_.best_quality = best.score.quality;
  last_report_.best_occupied_samples = best.score.occupied_samples;
  last_report_.best_outside_samples = best.score.outside_samples;
  last_report_.best_unknown_samples = best.score.unknown_samples;
  last_report_.best_sfc_violation = best.score.sfc_max_violation;
  last_report_.best_dynamic_violation = best.score.dynamic_max_violation;
  return true;
}

bool StateToStateIgoTrajOpt::validateForCommit(
    const geometry_utils::Trajectory &trajectory,
    const geometry_utils::PolytopeVec &corridor,
    std::string &reason) const
{
  if (!map_manager_ || !map_manager_->ready() || trajectory.empty() ||
      trajectory.getPieceNum() != static_cast<int>(corridor.size()))
  {
    reason = "COMMIT_INPUT_INVALID";
    return false;
  }
  const double duration = trajectory.getTotalDuration();
  const Eigen::VectorXd durations = trajectory.getDurations();
  const double dt = std::max(
      0.005,
      std::min(config_.igo_sample_dt,
               0.5 * (config_.igo_use_inflated_map ? map_manager_->getInfResolution()
                                                   : map_manager_->getResolution()) /
                   std::max(0.2, config_.max_vel)));
  for (double time = 0.0; time < duration + 0.5 * dt; time += dt)
  {
    const Vec3 position = trajectory.getPos(std::min(time, duration));
    if (!position.allFinite() || !map_manager_->insideLocalMap(position))
    {
      reason = "COMMIT_OUT_OF_LOCAL_MAP";
      return false;
    }
    const auto raw = map_manager_->getGridType(position);
    const auto selected =
        config_.igo_use_inflated_map ? map_manager_->getInfGridType(position) : raw;
    if (selected == rog_map::GridType::OCCUPIED ||
        selected == rog_map::GridType::OUT_OF_MAP || raw == rog_map::GridType::OUT_OF_MAP)
    {
      reason = "COMMIT_MAP_COLLISION";
      return false;
    }
    if (config_.igo_unknown_as_occupied && raw == rog_map::GridType::UNKNOWN)
    {
      reason = "COMMIT_UNKNOWN_SPACE";
      return false;
    }
    double piece_end = 0.0;
    int piece = 0;
    for (; piece + 1 < durations.size(); ++piece)
    {
      piece_end += durations(piece);
      if (time <= piece_end)
      {
        break;
      }
    }
    const HPoly planes = normalizePlanes(corridor[static_cast<std::size_t>(piece)].GetPlanes());
    if ((planes.leftCols<3>() * position + planes.rightCols<1>()).maxCoeff() >
        std::max(0.0, config_.igo_sfc_tolerance))
    {
      reason = "COMMIT_SFC_VIOLATION";
      return false;
    }
    if (trajectory.getJer(std::min(time, duration)).norm() >
        config_.max_jerk * (1.0 + config_.igo_dynamic_tolerance))
    {
      reason = "COMMIT_JERK_LIMIT";
      return false;
    }
  }
  if (trajectory.getMaxVelRate() > config_.max_vel * (1.0 + config_.igo_dynamic_tolerance))
  {
    reason = "COMMIT_VELOCITY_LIMIT";
    return false;
  }
  if (trajectory.getMaxAccRate() > config_.max_acc * (1.0 + config_.igo_dynamic_tolerance))
  {
    reason = "COMMIT_ACCELERATION_LIMIT";
    return false;
  }
  reason = "COMMIT_ACCEPTED";
  return true;
}

void StateToStateIgoTrajOpt::setWarmStart(
    const geometry_utils::Trajectory &trajectory)
{
  if (trajectory.empty())
  {
    return;
  }
  last_durations_ = trajectory.getDurations();
  last_points_.clear();
  double junction_time = 0.0;
  for (int piece = 0; piece + 1 < trajectory.getPieceNum(); ++piece)
  {
    junction_time += last_durations_(piece);
    last_points_.push_back(trajectory.getPos(junction_time));
  }
  previous_piece_count_ = trajectory.getPieceNum();
}

void StateToStateIgoTrajOpt::getInitValue(general_utils::VecDf &durations,
                                          general_utils::vec_Vec3f &points) const
{
  durations = last_durations_;
  points = last_points_;
}

}  // namespace traj_opt
