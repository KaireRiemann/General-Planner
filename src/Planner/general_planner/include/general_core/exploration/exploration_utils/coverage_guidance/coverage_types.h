#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fast_planner {

enum class CoverageVoxelState : std::uint8_t {
  UNKNOWN = 0,
  KNOWN_FREE = 1,
  OCCUPIED = 2,
  UNSAFE_FREE = 3,
  INVALID = 4
};

enum class CoverageTargetType : std::uint8_t {
  CURRENT = 0,
  ACTIVE_FREE = 1,
  REACHABLE_UNKNOWN = 2
};

struct CoverageBox {
  Eigen::Vector3d min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max{Eigen::Vector3d::Zero()};
};

struct CoverageMapSpec {
  Eigen::Vector3d min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max{Eigen::Vector3d::Zero()};
  Eigen::Vector3i dims{Eigen::Vector3i::Zero()};
  double resolution{0.6};
  std::vector<CoverageBox> valid_boxes;
  std::vector<CoverageBox> dead_boxes;

  bool valid() const {
    return resolution > 0.0 && (max.array() > min.array()).all() &&
           (dims.array() > 0).all();
  }

  int flatten(const Eigen::Vector3i &index) const {
    return (index.z() * dims.y() + index.y()) * dims.x() + index.x();
  }

  bool contains(const Eigen::Vector3i &index) const {
    return (index.array() >= 0).all() && (index.array() < dims.array()).all();
  }

  Eigen::Vector3i positionToIndex(const Eigen::Vector3d &position) const {
    return ((position - min) / resolution).array().floor().cast<int>();
  }

  Eigen::Vector3d indexToPosition(const Eigen::Vector3i &index) const {
    return min + (index.cast<double>().array() + 0.5).matrix() * resolution;
  }
};

struct CoverageSample {
  int linear_index{-1};
  CoverageVoxelState state{CoverageVoxelState::UNKNOWN};
};

struct CoverageMapDelta {
  std::uint64_t version{0};
  std::vector<CoverageSample> samples;
};

struct CoverageFrontier {
  int cluster_id{-1};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double information_gain{0.0};
  double wait_age{0.0};
  double pass_debt{0.0};
};

struct CoverageTarget {
  CoverageTargetType type{CoverageTargetType::REACHABLE_UNKNOWN};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  int zone_id{-1};
  int voxel_count{0};
  int route_rank{-1};
  // For an unknown target, this is a known-free zone adjacent to the unknown
  // component.  It turns the persistent coverage plan into an executable
  // observation action without ever commanding a trajectory into unknown.
  Eigen::Vector3d approach_position{Eigen::Vector3d::Zero()};
  bool has_approach{false};
};

struct CoveragePlan {
  using Ptr = std::shared_ptr<const CoveragePlan>;

  bool valid{false};
  std::uint64_t map_version{0};
  std::uint64_t frontier_version{0};
  double compute_ms{0.0};
  double wall_stamp_sec{0.0};
  int free_zone_count{0};
  int unknown_zone_count{0};
  int valid_voxel_count{0};
  int observed_voxel_count{0};
  int known_free_voxel_count{0};
  int occupied_voxel_count{0};
  int unsafe_free_voxel_count{0};
  double coverage_ratio{0.0};
  int reachable_unknown_count{0};
  int active_free_count{0};
  std::vector<CoverageTarget> ordered_targets;
  std::unordered_map<int, double> cluster_priority;
  std::unordered_set<int> preferred_cluster_ids;
};

}  // namespace fast_planner
